/**
 * @file    tmc2209.c
 * @brief   Driver TMC2209 qua UART cho STM32 HAL.
 *
 * Frame (datasheet TMC2209 §4):
 *   Write      : [0x05][ADDR][REG|0x80][D31..24][D23..16][D15..8][D7..0][CRC]
 *   Read req   : [0x05][ADDR][REG][CRC]
 *   Read reply : [0x05][0xFF][REG][D31..24][D23..16][D15..8][D7..0][CRC]
 */
#include "tmc2209.h"
#include <string.h>

/* ========================================================================== */
/*  Helper                                                                     */
/* ========================================================================== */

#define TMC_TRY(expr)  do { tmc_status_t _s = (expr); if (_s != TMC_OK) return _s; } while (0)
#define TMC_XFER_BUF   (TMC_WRITE_FRAME_LEN + TMC_READ_REPLY_LEN)

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* Arduino map() cho số nguyên dương */
static uint32_t map_u32(uint32_t x, uint32_t in_min, uint32_t in_max,
                        uint32_t out_min, uint32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t pct_to_current(uint8_t pct)    { return (uint8_t)map_u32(clamp_u32(pct, 0, 100), 0, 100, 0, 31); }
static uint8_t current_to_pct(uint8_t cs)     { return (uint8_t)map_u32(cs, 0, 31, 0, 100); }
static uint8_t pct_to_hold_delay(uint8_t pct) { return (uint8_t)map_u32(clamp_u32(pct, 0, 100), 0, 100, 0, 15); }
static uint8_t hold_delay_to_pct(uint8_t hd)  { return (uint8_t)map_u32(hd, 0, 15, 0, 100); }

/** CRC8 theo datasheet (poly 0x07, xử lý bit LSB-first của mỗi byte). */
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (((crc >> 7) ^ (b & 0x01U)) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            b >>= 1;
        }
    }
    return crc;
}

static void bus_lock(tmc_bus_t *bus)   { if (bus->lock)   { bus->lock(bus->lock_ctx); } }
static void bus_unlock(tmc_bus_t *bus) { if (bus->unlock) { bus->unlock(bus->lock_ctx); } }

static tmc_status_t bus_error(tmc_bus_t *bus, tmc_status_t s)
{
    if (s != TMC_OK) {
        bus->err_count++;
        bus->last_err = s;
    }
    return s;
}

/* ========================================================================== */
/*  Lớp vận chuyển UART                                                        */
/* ========================================================================== */

static void uart_flush_rx(UART_HandleTypeDef *h)
{
    (void)HAL_UART_AbortReceive(h);
    __HAL_UART_CLEAR_OREFLAG(h);          /* F1/F4: đọc SR+DR; F7/H7/G4/L4: ICR */
#ifdef UART_RXDATA_FLUSH_REQUEST
    __HAL_UART_SEND_REQ(h, UART_RXDATA_FLUSH_REQUEST);
#endif
}

static tmc_status_t uart_wait_rx_it(UART_HandleTypeDef *h, uint32_t timeout_ms)
{
    uint32_t t0 = TMC_GET_TICK_MS();
    while (h->RxState != HAL_UART_STATE_READY) {
        if ((TMC_GET_TICK_MS() - t0) > timeout_ms) {
            (void)HAL_UART_AbortReceive(h);
            return TMC_ERR_TIMEOUT;
        }
    }
    if (h->ErrorCode != HAL_UART_ERROR_NONE) {
        return TMC_ERR_UART;
    }
    if (h->RxXferCount != 0U) {
        return TMC_ERR_TIMEOUT;
    }
    return TMC_OK;
}

/**
 * Gửi tx[0..txlen) và (nếu rxlen > 0) nhận rxlen byte reply.
 * TMC_UART_TX_RX: TX và RX nối chung -> nhận txlen byte echo + rxlen byte reply.
 * Echo được so khớp để phát hiện va chạm/nhiễu trên bus.
 */
static tmc_status_t bus_xfer(tmc_bus_t *bus, const uint8_t *tx, uint8_t txlen,
                             uint8_t *rx, uint8_t rxlen)
{
    UART_HandleTypeDef *h = bus->huart;

    switch (bus->mode) {
    case TMC_UART_TX_ONLY:
        if (rxlen != 0U) {
            return TMC_ERR_NO_RX;
        }
        return (HAL_UART_Transmit(h, (uint8_t *)tx, txlen, bus->timeout_ms) == HAL_OK)
               ? TMC_OK : TMC_ERR_TX;

    case TMC_UART_TX_RX: {
        uint8_t  buf[TMC_XFER_BUF];
        uint16_t total = (uint16_t)txlen + rxlen;
        tmc_status_t st;

        if (total > sizeof(buf)) {
            return TMC_ERR_PARAM;
        }
        uart_flush_rx(h);
        /* Bật nhận bằng ngắt TRƯỚC khi phát, để không mất byte echo */
        if (HAL_UART_Receive_IT(h, buf, total) != HAL_OK) {
            return TMC_ERR_UART;
        }
        if (HAL_UART_Transmit(h, (uint8_t *)tx, txlen, bus->timeout_ms) != HAL_OK) {
            (void)HAL_UART_AbortReceive(h);
            return TMC_ERR_TX;
        }
        st = uart_wait_rx_it(h, bus->timeout_ms);
        if (st != TMC_OK) {
            return st;
        }
        if (memcmp(buf, tx, txlen) != 0) {
            return TMC_ERR_ECHO;
        }
        if (rxlen != 0U) {
            memcpy(rx, &buf[txlen], rxlen);
        }
        return TMC_OK;
    }

    case TMC_UART_HALF_DUPLEX: {
        HAL_StatusTypeDef hs;
        (void)HAL_HalfDuplex_EnableTransmitter(h);
        hs = HAL_UART_Transmit(h, (uint8_t *)tx, txlen, bus->timeout_ms);
        (void)HAL_HalfDuplex_EnableReceiver(h);
        if (hs != HAL_OK) {
            return TMC_ERR_TX;
        }
        if (rxlen == 0U) {
            return TMC_OK;
        }
        __HAL_UART_CLEAR_OREFLAG(h);
        hs = HAL_UART_Receive(h, rx, rxlen, bus->timeout_ms);
        if (hs == HAL_TIMEOUT) {
            return TMC_ERR_TIMEOUT;
        }
        return (hs == HAL_OK) ? TMC_OK : TMC_ERR_UART;
    }

    default:
        return TMC_ERR_PARAM;
    }
}

/* ========================================================================== */
/*  Bus API                                                                    */
/* ========================================================================== */

tmc_status_t tmc_bus_init(tmc_bus_t *bus, UART_HandleTypeDef *huart, tmc_uart_mode_t mode)
{
    if ((bus == NULL) || (huart == NULL) || (mode > TMC_UART_HALF_DUPLEX)) {
        return TMC_ERR_PARAM;
    }
    memset(bus, 0, sizeof(*bus));
    bus->huart      = huart;
    bus->mode       = mode;
    bus->timeout_ms = TMC_DEFAULT_TIMEOUT_MS;
    bus->retries    = TMC_DEFAULT_RETRIES;
    if (mode == TMC_UART_HALF_DUPLEX) {
        (void)HAL_HalfDuplex_EnableReceiver(huart);   /* thả line khi rảnh */
    }
    return TMC_OK;
}

void tmc_bus_set_lock(tmc_bus_t *bus, void (*lock)(void *), void (*unlock)(void *), void *ctx)
{
    bus->lock     = lock;
    bus->unlock   = unlock;
    bus->lock_ctx = ctx;
}

bool tmc_bus_can_read(const tmc_bus_t *bus)
{
    return (bus != NULL) && (bus->mode != TMC_UART_TX_ONLY);
}

/* ========================================================================== */
/*  Register low-level                                                         */
/* ========================================================================== */

tmc_status_t tmc2209_write_reg(tmc2209_t *dev, uint8_t reg, uint32_t value)
{
    uint8_t f[TMC_WRITE_FRAME_LEN];
    tmc_status_t st = TMC_ERR_PARAM;

    if ((dev == NULL) || (dev->bus == NULL)) {
        return TMC_ERR_PARAM;
    }
    f[0] = TMC_SYNC;
    f[1] = dev->addr;
    f[2] = (uint8_t)(reg | TMC_RW_WRITE);
    f[3] = (uint8_t)(value >> 24);
    f[4] = (uint8_t)(value >> 16);
    f[5] = (uint8_t)(value >> 8);
    f[6] = (uint8_t)(value);
    f[7] = tmc_crc8(f, 7);

    bus_lock(dev->bus);
    for (uint8_t i = 0; i <= dev->bus->retries; i++) {
        st = bus_xfer(dev->bus, f, sizeof(f), NULL, 0);
        if ((st == TMC_OK) || (st == TMC_ERR_PARAM)) {
            break;
        }
    }
    bus_unlock(dev->bus);
    return bus_error(dev->bus, st);
}

tmc_status_t tmc2209_read_reg(tmc2209_t *dev, uint8_t reg, uint32_t *value)
{
    uint8_t req[TMC_READ_REQ_LEN];
    uint8_t rep[TMC_READ_REPLY_LEN];
    tmc_status_t st = TMC_ERR_PARAM;

    if ((dev == NULL) || (dev->bus == NULL) || (value == NULL)) {
        return TMC_ERR_PARAM;
    }
    if (!tmc_bus_can_read(dev->bus)) {
        return TMC_ERR_NO_RX;
    }
    req[0] = TMC_SYNC;
    req[1] = dev->addr;
    req[2] = (uint8_t)(reg & 0x7FU);
    req[3] = tmc_crc8(req, 3);

    bus_lock(dev->bus);
    for (uint8_t i = 0; i <= dev->bus->retries; i++) {
        st = bus_xfer(dev->bus, req, sizeof(req), rep, sizeof(rep));
        if (st == TMC_OK) {
            if (tmc_crc8(rep, 7) != rep[7]) {
                st = TMC_ERR_CRC;
            } else if (((rep[0] & 0x0FU) != TMC_SYNC) || (rep[1] != TMC_REPLY_ADDR) ||
                       (rep[2] != req[2])) {
                st = TMC_ERR_REPLY;
            } else {
                *value = ((uint32_t)rep[3] << 24) | ((uint32_t)rep[4] << 16) |
                         ((uint32_t)rep[5] << 8)  |  (uint32_t)rep[6];
                break;
            }
        }
        if (st == TMC_ERR_PARAM) {
            break;
        }
        TMC_DELAY_MS(1);   /* để bus về idle trước khi thử lại */
    }
    bus_unlock(dev->bus);
    return bus_error(dev->bus, st);
}

/* ---- ghi shadow ----------------------------------------------------------- */
static tmc_status_t write_gconf(tmc2209_t *d)    { return tmc2209_write_reg(d, TMC_REG_GCONF, d->gconf); }
static tmc_status_t write_chopconf(tmc2209_t *d) { return tmc2209_write_reg(d, TMC_REG_CHOPCONF, d->chopconf); }
static tmc_status_t write_pwmconf(tmc2209_t *d)  { return tmc2209_write_reg(d, TMC_REG_PWMCONF, d->pwmconf); }
static tmc_status_t write_coolconf(tmc2209_t *d) { return tmc2209_write_reg(d, TMC_REG_COOLCONF, d->coolconf); }

/* Giống writeStoredDriverCurrent(): SEIMIN tự chọn theo IRUN */
static tmc_status_t write_driver_current(tmc2209_t *d)
{
    uint8_t irun = (uint8_t)TMC_FIELD_GET(d->ihold_irun, IRUN_Msk, IRUN_Pos);
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_IHOLD_IRUN, d->ihold_irun));
    TMC_BIT_WRITE(d->coolconf, COOL_SEIMIN_Msk, irun >= TMC_SEIMIN_IRUN_LIMIT);
    if (d->cool_step_enabled) {
        return write_coolconf(d);
    }
    return TMC_OK;
}

/* ========================================================================== */
/*  Khởi tạo                                                                   */
/* ========================================================================== */

static tmc_status_t set_registers_to_defaults(tmc2209_t *d)
{
    d->ihold_irun = 0;
    TMC_FIELD_SET(d->ihold_irun, IHOLD_Msk, IHOLD_Pos, TMC_IHOLD_DEFAULT);
    TMC_FIELD_SET(d->ihold_irun, IRUN_Msk, IRUN_Pos, TMC_IRUN_DEFAULT);
    TMC_FIELD_SET(d->ihold_irun, IHOLDDELAY_Msk, IHOLDDELAY_Pos, TMC_IHOLDDELAY_DEFAULT);
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_IHOLD_IRUN, d->ihold_irun));

    d->chopconf = TMC_CHOPCONF_DEFAULT;
    TMC_FIELD_SET(d->chopconf, CHOP_TBL_Msk, CHOP_TBL_Pos, TMC_TBL_DEFAULT);
    TMC_FIELD_SET(d->chopconf, CHOP_HEND_Msk, CHOP_HEND_Pos, TMC_HEND_DEFAULT);
    TMC_FIELD_SET(d->chopconf, CHOP_HSTRT_Msk, CHOP_HSTRT_Pos, TMC_HSTRT_DEFAULT);
    TMC_FIELD_SET(d->chopconf, CHOP_TOFF_Msk, CHOP_TOFF_Pos, TMC_TOFF_DEFAULT);
    d->toff = TMC_TOFF_DEFAULT;
    TMC_TRY(write_chopconf(d));

    d->pwmconf = TMC_PWMCONF_DEFAULT;
    TMC_TRY(write_pwmconf(d));

    d->coolconf = 0;
    d->cool_step_enabled = false;
    TMC_TRY(write_coolconf(d));

    d->tpowerdown = TMC_TPOWERDOWN_DEFAULT;
    d->tpwmthrs   = 0;
    d->vactual    = 0;
    d->tcoolthrs  = 0;
    d->sgthrs     = 0;
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_TPOWERDOWN, d->tpowerdown));
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_TPWMTHRS, d->tpwmthrs));
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_VACTUAL, 0));
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_TCOOLTHRS, d->tcoolthrs));
    TMC_TRY(tmc2209_write_reg(d, TMC_REG_SGTHRS, d->sgthrs));
    return TMC_OK;
}

tmc_status_t tmc2209_init(tmc2209_t *dev, tmc_bus_t *bus, uint8_t addr)
{
    if ((dev == NULL) || (bus == NULL) || (addr > TMC_MAX_ADDR)) {
        return TMC_ERR_PARAM;
    }
    memset(dev, 0, sizeof(*dev));
    dev->bus  = bus;
    dev->addr = addr;
    dev->toff = TMC_TOFF_DEFAULT;

    /* Bus 2 chiều: kiểm tra chip có trả lời trước (VS phải có điện!) */
    if (tmc_bus_can_read(bus) && !tmc2209_is_communicating(dev)) {
        return TMC_ERR_NOT_READY;
    }

    /* setOperationModeToSerial() */
    dev->gconf = GCONF_PDN_DISABLE_Msk | GCONF_MSTEP_REG_SELECT_Msk | GCONF_MULTISTEP_FILT_Msk;
    TMC_TRY(write_gconf(dev));

    /* Bus 2 chiều: SENDDELAY >= 2 (khuyến nghị khi nhiều địa chỉ) */
    if (tmc_bus_can_read(bus)) {
        TMC_TRY(tmc2209_set_reply_delay(dev, 2));
    }

    TMC_TRY(set_registers_to_defaults(dev));
    TMC_TRY(tmc2209_clear_drive_error(dev));

    /* minimizeMotorCurrent() */
    TMC_FIELD_SET(dev->ihold_irun, IRUN_Msk, IRUN_Pos, 0);
    TMC_FIELD_SET(dev->ihold_irun, IHOLD_Msk, IHOLD_Pos, 0);
    TMC_TRY(write_driver_current(dev));

    TMC_TRY(tmc2209_disable(dev));
    TMC_TRY(tmc2209_disable_automatic_current_scaling(dev));
    TMC_TRY(tmc2209_disable_automatic_gradient_adaptation(dev));
    return TMC_OK;
}

tmc_status_t tmc2209_sync_from_chip(tmc2209_t *dev)
{
    uint32_t v;
    uint8_t  toff;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_GCONF, &v));
    dev->gconf = v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_CHOPCONF, &v));
    dev->chopconf = v;
    toff = (uint8_t)TMC_FIELD_GET(v, CHOP_TOFF_Msk, CHOP_TOFF_Pos);
    if (toff != 0U) {
        dev->toff = toff;
    }
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_PWMCONF, &v));
    dev->pwmconf = v;
    return TMC_OK;
}

tmc_status_t tmc2209_restore_to_chip(tmc2209_t *dev)
{
    TMC_TRY(write_gconf(dev));
    if (tmc_bus_can_read(dev->bus)) {
        TMC_TRY(tmc2209_write_reg(dev, TMC_REG_SLAVECONF, dev->slaveconf));
    }
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_IHOLD_IRUN, dev->ihold_irun));
    TMC_TRY(write_chopconf(dev));
    TMC_TRY(write_pwmconf(dev));
    TMC_TRY(write_coolconf(dev));
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_TPOWERDOWN, dev->tpowerdown));
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_TPWMTHRS, dev->tpwmthrs));
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_TCOOLTHRS, dev->tcoolthrs));
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_SGTHRS, dev->sgthrs));
    TMC_TRY(tmc2209_write_reg(dev, TMC_REG_VACTUAL, (uint32_t)dev->vactual));
    return tmc2209_clear_reset(dev);
}

/* ========================================================================== */
/*  Enable / microstep / dòng                                                  */
/* ========================================================================== */

void tmc2209_set_hardware_enable_pin(tmc2209_t *dev, GPIO_TypeDef *port, uint16_t pin)
{
    dev->en_port = port;
    dev->en_pin  = pin;
    if (port != NULL) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);   /* EN active-low -> disable */
    }
}

tmc_status_t tmc2209_enable(tmc2209_t *dev)
{
    if (dev->en_port != NULL) {
        HAL_GPIO_WritePin(dev->en_port, dev->en_pin, GPIO_PIN_RESET);
    }
    TMC_FIELD_SET(dev->chopconf, CHOP_TOFF_Msk, CHOP_TOFF_Pos, dev->toff);
    return write_chopconf(dev);
}

tmc_status_t tmc2209_disable(tmc2209_t *dev)
{
    if (dev->en_port != NULL) {
        HAL_GPIO_WritePin(dev->en_port, dev->en_pin, GPIO_PIN_SET);
    }
    TMC_FIELD_SET(dev->chopconf, CHOP_TOFF_Msk, CHOP_TOFF_Pos, 0);
    return write_chopconf(dev);
}

tmc_status_t tmc2209_set_microsteps_per_step(tmc2209_t *dev, uint16_t microsteps)
{
    /* Sửa lỗi nhỏ của bản gốc: dùng giá trị đã constrain */
    uint16_t v = (uint16_t)clamp_u32(microsteps, 1, 256);
    uint8_t  exponent = 0;
    while (v > 1U) {
        v >>= 1;
        exponent++;
    }
    return tmc2209_set_microsteps_per_step_power_of_two(dev, exponent);
}

tmc_status_t tmc2209_set_microsteps_per_step_power_of_two(tmc2209_t *dev, uint8_t exponent)
{
    if (exponent > 8U) {
        exponent = 8U;
    }
    /* MRES: 0 = 256, 1 = 128, ..., 8 = fullstep  => mres = 8 - exponent */
    TMC_FIELD_SET(dev->chopconf, CHOP_MRES_Msk, CHOP_MRES_Pos, 8U - exponent);
    return write_chopconf(dev);
}

uint16_t tmc2209_get_microsteps_per_step(const tmc2209_t *dev)
{
    uint8_t mres = (uint8_t)TMC_FIELD_GET(dev->chopconf, CHOP_MRES_Msk, CHOP_MRES_Pos);
    if (mres > 8U) {
        mres = 0U;   /* giá trị không hợp lệ -> coi như 256 */
    }
    return (uint16_t)(1U << (8U - mres));
}

tmc_status_t tmc2209_set_run_current(tmc2209_t *dev, uint8_t percent)
{
    TMC_FIELD_SET(dev->ihold_irun, IRUN_Msk, IRUN_Pos, pct_to_current(percent));
    return write_driver_current(dev);
}

tmc_status_t tmc2209_set_hold_current(tmc2209_t *dev, uint8_t percent)
{
    TMC_FIELD_SET(dev->ihold_irun, IHOLD_Msk, IHOLD_Pos, pct_to_current(percent));
    return write_driver_current(dev);
}

tmc_status_t tmc2209_set_hold_delay(tmc2209_t *dev, uint8_t percent)
{
    TMC_FIELD_SET(dev->ihold_irun, IHOLDDELAY_Msk, IHOLDDELAY_Pos, pct_to_hold_delay(percent));
    return write_driver_current(dev);
}

tmc_status_t tmc2209_set_all_current_values(tmc2209_t *dev, uint8_t run_pct,
                                            uint8_t hold_pct, uint8_t hold_delay_pct)
{
    TMC_FIELD_SET(dev->ihold_irun, IRUN_Msk, IRUN_Pos, pct_to_current(run_pct));
    TMC_FIELD_SET(dev->ihold_irun, IHOLD_Msk, IHOLD_Pos, pct_to_current(hold_pct));
    TMC_FIELD_SET(dev->ihold_irun, IHOLDDELAY_Msk, IHOLDDELAY_Pos, pct_to_hold_delay(hold_delay_pct));
    return write_driver_current(dev);
}

tmc_status_t tmc2209_set_rms_current(tmc2209_t *dev, uint16_t mA, float r_sense, float hold_multiplier)
{
    /* Công thức từ TMCStepper (bản gốc cũng dùng) */
    float cs = 32.0f * 1.41421f * (float)mA / 1000.0f * (r_sense + 0.02f) / 0.325f - 1.0f;
    int32_t ics;

    if (cs < 16.0f) {
        dev->chopconf |= CHOP_VSENSE_Msk;
        cs = 32.0f * 1.41421f * (float)mA / 1000.0f * (r_sense + 0.02f) / 0.180f - 1.0f;
    } else {
        dev->chopconf &= ~CHOP_VSENSE_Msk;
    }
    TMC_TRY(write_chopconf(dev));

    ics = (int32_t)cs;
    ics = (ics < 0) ? 0 : ((ics > 31) ? 31 : ics);
    if (hold_multiplier < 0.0f) { hold_multiplier = 0.0f; }
    if (hold_multiplier > 1.0f) { hold_multiplier = 1.0f; }

    TMC_FIELD_SET(dev->ihold_irun, IRUN_Msk, IRUN_Pos, (uint32_t)ics);
    TMC_FIELD_SET(dev->ihold_irun, IHOLD_Msk, IHOLD_Pos, (uint32_t)((float)ics * hold_multiplier));
    return write_driver_current(dev);
}

/* ========================================================================== */
/*  Các bit cấu hình                                                           */
/* ========================================================================== */

#define CHOP_BIT_FN(name, msk, on) \
    tmc_status_t name(tmc2209_t *dev) { TMC_BIT_WRITE(dev->chopconf, msk, on); return write_chopconf(dev); }
#define GCONF_BIT_FN(name, msk, on) \
    tmc_status_t name(tmc2209_t *dev) { TMC_BIT_WRITE(dev->gconf, msk, on); return write_gconf(dev); }
#define PWM_BIT_FN(name, msk, on) \
    tmc_status_t name(tmc2209_t *dev) { TMC_BIT_WRITE(dev->pwmconf, msk, on); return write_pwmconf(dev); }

CHOP_BIT_FN(tmc2209_enable_double_edge,   CHOP_DEDGE_Msk,  1)
CHOP_BIT_FN(tmc2209_disable_double_edge,  CHOP_DEDGE_Msk,  0)
CHOP_BIT_FN(tmc2209_enable_vsense,        CHOP_VSENSE_Msk, 1)
CHOP_BIT_FN(tmc2209_disable_vsense,       CHOP_VSENSE_Msk, 0)

GCONF_BIT_FN(tmc2209_enable_inverse_motor_direction,  GCONF_SHAFT_Msk,           1)
GCONF_BIT_FN(tmc2209_disable_inverse_motor_direction, GCONF_SHAFT_Msk,           0)
GCONF_BIT_FN(tmc2209_enable_stealth_chop,             GCONF_EN_SPREADCYCLE_Msk,  0)
GCONF_BIT_FN(tmc2209_disable_stealth_chop,            GCONF_EN_SPREADCYCLE_Msk,  1)
GCONF_BIT_FN(tmc2209_enable_analog_current_scaling,   GCONF_I_SCALE_ANALOG_Msk,  1)
GCONF_BIT_FN(tmc2209_disable_analog_current_scaling,  GCONF_I_SCALE_ANALOG_Msk,  0)
GCONF_BIT_FN(tmc2209_use_external_sense_resistors,    GCONF_INTERNAL_RSENSE_Msk, 0)
GCONF_BIT_FN(tmc2209_use_internal_sense_resistors,    GCONF_INTERNAL_RSENSE_Msk, 1)

PWM_BIT_FN(tmc2209_enable_automatic_current_scaling,      PWM_AUTOSCALE_Msk, 1)
PWM_BIT_FN(tmc2209_disable_automatic_current_scaling,     PWM_AUTOSCALE_Msk, 0)
PWM_BIT_FN(tmc2209_enable_automatic_gradient_adaptation,  PWM_AUTOGRAD_Msk,  1)
PWM_BIT_FN(tmc2209_disable_automatic_gradient_adaptation, PWM_AUTOGRAD_Msk,  0)

bool tmc2209_double_edge_enabled(const tmc2209_t *dev)
{
    return (dev->chopconf & CHOP_DEDGE_Msk) != 0U;
}

tmc_status_t tmc2209_set_standstill_mode(tmc2209_t *dev, tmc_standstill_mode_t mode)
{
    TMC_FIELD_SET(dev->pwmconf, PWM_FREEWHEEL_Msk, PWM_FREEWHEEL_Pos, mode);
    return write_pwmconf(dev);
}

tmc_status_t tmc2209_set_pwm_offset(tmc2209_t *dev, uint8_t pwm_amplitude)
{
    TMC_FIELD_SET(dev->pwmconf, PWM_OFS_Msk, PWM_OFS_Pos, pwm_amplitude);
    return write_pwmconf(dev);
}

tmc_status_t tmc2209_set_pwm_gradient(tmc2209_t *dev, uint8_t pwm_amplitude)
{
    TMC_FIELD_SET(dev->pwmconf, PWM_GRAD_Msk, PWM_GRAD_Pos, pwm_amplitude);
    return write_pwmconf(dev);
}

tmc_status_t tmc2209_set_power_down_delay(tmc2209_t *dev, uint8_t delay)
{
    dev->tpowerdown = delay;
    return tmc2209_write_reg(dev, TMC_REG_TPOWERDOWN, delay);
}

tmc_status_t tmc2209_set_reply_delay(tmc2209_t *dev, uint8_t delay)
{
    if (delay > 15U) {
        delay = 15U;
    }
    dev->slaveconf = 0;
    TMC_FIELD_SET(dev->slaveconf, SLAVECONF_SENDDELAY_Msk, SLAVECONF_SENDDELAY_Pos, delay);
    return tmc2209_write_reg(dev, TMC_REG_SLAVECONF, dev->slaveconf);
}

tmc_status_t tmc2209_set_stealth_chop_duration_threshold(tmc2209_t *dev, uint32_t tpwmthrs)
{
    dev->tpwmthrs = tpwmthrs & 0xFFFFFUL;
    return tmc2209_write_reg(dev, TMC_REG_TPWMTHRS, dev->tpwmthrs);
}

tmc_status_t tmc2209_set_cool_step_duration_threshold(tmc2209_t *dev, uint32_t tcoolthrs)
{
    dev->tcoolthrs = tcoolthrs & 0xFFFFFUL;
    return tmc2209_write_reg(dev, TMC_REG_TCOOLTHRS, dev->tcoolthrs);
}

tmc_status_t tmc2209_set_stall_guard_threshold(tmc2209_t *dev, uint8_t sgthrs)
{
    dev->sgthrs = sgthrs;
    return tmc2209_write_reg(dev, TMC_REG_SGTHRS, sgthrs);
}

tmc_status_t tmc2209_enable_cool_step(tmc2209_t *dev, uint8_t lower_threshold, uint8_t upper_threshold)
{
    TMC_FIELD_SET(dev->coolconf, COOL_SEMIN_Msk, COOL_SEMIN_Pos, clamp_u32(lower_threshold, 1, 15));
    TMC_FIELD_SET(dev->coolconf, COOL_SEMAX_Msk, COOL_SEMAX_Pos, clamp_u32(upper_threshold, 0, 15));
    dev->cool_step_enabled = true;
    return write_coolconf(dev);
}

tmc_status_t tmc2209_disable_cool_step(tmc2209_t *dev)
{
    TMC_FIELD_SET(dev->coolconf, COOL_SEMIN_Msk, COOL_SEMIN_Pos, 0);
    dev->cool_step_enabled = false;
    return write_coolconf(dev);
}

tmc_status_t tmc2209_set_cool_step_current_increment(tmc2209_t *dev, tmc_cs_increment_t inc)
{
    TMC_FIELD_SET(dev->coolconf, COOL_SEUP_Msk, COOL_SEUP_Pos, inc);
    return write_coolconf(dev);
}

tmc_status_t tmc2209_set_cool_step_measurement_count(tmc2209_t *dev, tmc_sg_count_t count)
{
    TMC_FIELD_SET(dev->coolconf, COOL_SEDN_Msk, COOL_SEDN_Pos, count);
    return write_coolconf(dev);
}

/* ========================================================================== */
/*  Vận tốc qua UART (VACTUAL)                                                 */
/* ========================================================================== */

tmc_status_t tmc2209_move_at_velocity(tmc2209_t *dev, int32_t vactual)
{
    if (vactual > TMC_VACTUAL_MAX)  { vactual = TMC_VACTUAL_MAX; }
    if (vactual < -TMC_VACTUAL_MAX) { vactual = -TMC_VACTUAL_MAX; }
    dev->vactual = vactual;
    return tmc2209_write_reg(dev, TMC_REG_VACTUAL, (uint32_t)vactual);
}

tmc_status_t tmc2209_move_using_step_dir_interface(tmc2209_t *dev)
{
    dev->vr_active  = false;
    dev->vr_current = 0.0f;
    dev->vr_target  = 0.0f;
    return tmc2209_move_at_velocity(dev, 0);
}

int32_t tmc2209_usteps_per_s_to_vactual(float usteps_per_s)
{
    float v = usteps_per_s / TMC_VACTUAL_HZ_PER_LSB;
    if (v > (float)TMC_VACTUAL_MAX)  { v = (float)TMC_VACTUAL_MAX; }
    if (v < -(float)TMC_VACTUAL_MAX) { v = -(float)TMC_VACTUAL_MAX; }
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

float tmc2209_vactual_to_usteps_per_s(int32_t vactual)
{
    return (float)vactual * TMC_VACTUAL_HZ_PER_LSB;
}

tmc_status_t tmc2209_velocity_ramp_set(tmc2209_t *dev, float target_usteps_per_s, float accel)
{
    dev->vr_target  = target_usteps_per_s;
    dev->vr_accel   = (accel > 0.0f) ? accel : 0.0f;
    dev->vr_current = tmc2209_vactual_to_usteps_per_s(dev->vactual);
    dev->vr_last_ms = TMC_GET_TICK_MS();
    dev->vr_active  = true;
    if (dev->vr_accel == 0.0f) {
        return tmc2209_velocity_ramp_task(dev);   /* không ramp: ghi ngay */
    }
    return TMC_OK;
}

tmc_status_t tmc2209_velocity_ramp_task(tmc2209_t *dev)
{
    uint32_t now;
    float    dt;
    int32_t  vact;

    if (!dev->vr_active) {
        return TMC_OK;
    }
    now = TMC_GET_TICK_MS();
    dt  = (float)(now - dev->vr_last_ms) / 1000.0f;
    dev->vr_last_ms = now;

    if (dev->vr_accel == 0.0f) {
        dev->vr_current = dev->vr_target;
    } else {
        float dv = dev->vr_accel * dt;
        if (dev->vr_current < dev->vr_target) {
            dev->vr_current = (dev->vr_target - dev->vr_current <= dv) ? dev->vr_target : dev->vr_current + dv;
        } else if (dev->vr_current > dev->vr_target) {
            dev->vr_current = (dev->vr_current - dev->vr_target <= dv) ? dev->vr_target : dev->vr_current - dv;
        }
    }

    vact = tmc2209_usteps_per_s_to_vactual(dev->vr_current);
    if (dev->vr_current == dev->vr_target) {
        dev->vr_active = false;
    }
    if (vact != dev->vactual) {
        return tmc2209_move_at_velocity(dev, vact);
    }
    return TMC_OK;
}

bool tmc2209_velocity_ramp_reached(const tmc2209_t *dev)
{
    return !dev->vr_active;
}

/* ========================================================================== */
/*  API đọc                                                                    */
/* ========================================================================== */

tmc_status_t tmc2209_get_version(tmc2209_t *dev, uint8_t *version)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_IOIN, &v));
    *version = (uint8_t)TMC_FIELD_GET(v, IOIN_VERSION_Msk, IOIN_VERSION_Pos);
    return TMC_OK;
}

bool tmc2209_is_communicating(tmc2209_t *dev)
{
    uint8_t ver = 0;
    return (tmc2209_get_version(dev, &ver) == TMC_OK) && (ver == TMC2209_VERSION);
}

bool tmc2209_is_setup_and_communicating(tmc2209_t *dev)
{
    uint32_t v;
    return (tmc2209_read_reg(dev, TMC_REG_GCONF, &v) == TMC_OK) && ((v & GCONF_PDN_DISABLE_Msk) != 0U);
}

bool tmc2209_is_communicating_but_not_setup(tmc2209_t *dev)
{
    return tmc2209_is_communicating(dev) && !tmc2209_is_setup_and_communicating(dev);
}

bool tmc2209_hardware_disabled(tmc2209_t *dev)
{
    uint32_t v;
    return (tmc2209_read_reg(dev, TMC_REG_IOIN, &v) == TMC_OK) && ((v & IOIN_ENN_Msk) != 0U);
}

tmc_status_t tmc2209_get_settings(tmc2209_t *dev, tmc2209_settings_t *s)
{
    uint8_t irun, ihold, ihd;

    memset(s, 0, sizeof(*s));
    s->is_communicating = tmc2209_is_communicating(dev);
    if (!s->is_communicating) {
        s->standstill_mode = (uint8_t)TMC_FIELD_GET(dev->pwmconf, PWM_FREEWHEEL_Msk, PWM_FREEWHEEL_Pos);
        return TMC_ERR_NOT_READY;
    }
    TMC_TRY(tmc2209_sync_from_chip(dev));

    irun  = (uint8_t)TMC_FIELD_GET(dev->ihold_irun, IRUN_Msk, IRUN_Pos);
    ihold = (uint8_t)TMC_FIELD_GET(dev->ihold_irun, IHOLD_Msk, IHOLD_Pos);
    ihd   = (uint8_t)TMC_FIELD_GET(dev->ihold_irun, IHOLDDELAY_Msk, IHOLDDELAY_Pos);

    s->is_setup                        = (dev->gconf & GCONF_PDN_DISABLE_Msk) != 0U;
    s->software_enabled                = TMC_FIELD_GET(dev->chopconf, CHOP_TOFF_Msk, CHOP_TOFF_Pos) != 0U;
    s->microsteps_per_step             = tmc2209_get_microsteps_per_step(dev);
    s->inverse_motor_direction_enabled = (dev->gconf & GCONF_SHAFT_Msk) != 0U;
    s->stealth_chop_enabled            = (dev->gconf & GCONF_EN_SPREADCYCLE_Msk) == 0U;
    s->standstill_mode                 = (uint8_t)TMC_FIELD_GET(dev->pwmconf, PWM_FREEWHEEL_Msk, PWM_FREEWHEEL_Pos);
    s->irun_percent                    = current_to_pct(irun);
    s->irun_register_value             = irun;
    s->ihold_percent                   = current_to_pct(ihold);
    s->ihold_register_value            = ihold;
    s->iholddelay_percent              = hold_delay_to_pct(ihd);
    s->iholddelay_register_value       = ihd;
    s->automatic_current_scaling_enabled     = (dev->pwmconf & PWM_AUTOSCALE_Msk) != 0U;
    s->automatic_gradient_adaptation_enabled = (dev->pwmconf & PWM_AUTOGRAD_Msk) != 0U;
    s->pwm_offset                      = (uint8_t)TMC_FIELD_GET(dev->pwmconf, PWM_OFS_Msk, PWM_OFS_Pos);
    s->pwm_gradient                    = (uint8_t)TMC_FIELD_GET(dev->pwmconf, PWM_GRAD_Msk, PWM_GRAD_Pos);
    s->cool_step_enabled               = dev->cool_step_enabled;
    s->analog_current_scaling_enabled  = (dev->gconf & GCONF_I_SCALE_ANALOG_Msk) != 0U;
    s->internal_sense_resistors_enabled = (dev->gconf & GCONF_INTERNAL_RSENSE_Msk) != 0U;
    return TMC_OK;
}

tmc_status_t tmc2209_get_status(tmc2209_t *dev, tmc2209_status_t *st)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_DRV_STATUS, &v));
    st->raw                        = v;
    st->over_temperature_warning   = (v & DRV_OTPW_Msk)  != 0U;
    st->over_temperature_shutdown  = (v & DRV_OT_Msk)    != 0U;
    st->short_to_ground_a          = (v & DRV_S2GA_Msk)  != 0U;
    st->short_to_ground_b          = (v & DRV_S2GB_Msk)  != 0U;
    st->low_side_short_a           = (v & DRV_S2VSA_Msk) != 0U;
    st->low_side_short_b           = (v & DRV_S2VSB_Msk) != 0U;
    st->open_load_a                = (v & DRV_OLA_Msk)   != 0U;
    st->open_load_b                = (v & DRV_OLB_Msk)   != 0U;
    st->over_temperature_120c      = (v & DRV_T120_Msk)  != 0U;
    st->over_temperature_143c      = (v & DRV_T143_Msk)  != 0U;
    st->over_temperature_150c      = (v & DRV_T150_Msk)  != 0U;
    st->over_temperature_157c      = (v & DRV_T157_Msk)  != 0U;
    st->current_scaling            = (uint8_t)TMC_FIELD_GET(v, DRV_CS_ACTUAL_Msk, DRV_CS_ACTUAL_Pos);
    st->stealth_chop_mode          = (v & DRV_STEALTH_Msk) != 0U;
    st->standstill                 = (v & DRV_STST_Msk)    != 0U;
    return TMC_OK;
}

tmc_status_t tmc2209_get_global_status(tmc2209_t *dev, tmc2209_gstat_t *gs)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_GSTAT, &v));
    gs->reset   = (v & GSTAT_RESET_Msk)   != 0U;
    gs->drv_err = (v & GSTAT_DRV_ERR_Msk) != 0U;
    gs->uv_cp   = (v & GSTAT_UV_CP_Msk)   != 0U;
    return TMC_OK;
}

tmc_status_t tmc2209_clear_reset(tmc2209_t *dev)
{
    return tmc2209_write_reg(dev, TMC_REG_GSTAT, GSTAT_RESET_Msk);   /* write-1-to-clear */
}

tmc_status_t tmc2209_clear_drive_error(tmc2209_t *dev)
{
    return tmc2209_write_reg(dev, TMC_REG_GSTAT, GSTAT_DRV_ERR_Msk);
}

tmc_status_t tmc2209_get_interface_transmission_counter(tmc2209_t *dev, uint8_t *ifcnt)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_IFCNT, &v));
    *ifcnt = (uint8_t)v;
    return TMC_OK;
}

tmc_status_t tmc2209_get_interstep_duration(tmc2209_t *dev, uint32_t *tstep)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_TSTEP, &v));
    *tstep = v & 0xFFFFFUL;
    return TMC_OK;
}

tmc_status_t tmc2209_get_stall_guard_result(tmc2209_t *dev, uint16_t *sg)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_SG_RESULT, &v));
    *sg = (uint16_t)(v & 0x3FFU);
    return TMC_OK;
}

tmc_status_t tmc2209_get_pwm_scale_sum(tmc2209_t *dev, uint8_t *out)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_PWM_SCALE, &v));
    *out = (uint8_t)(v & PWM_SCALE_SUM_Msk);
    return TMC_OK;
}

tmc_status_t tmc2209_get_pwm_scale_auto(tmc2209_t *dev, int16_t *out)
{
    uint32_t v;
    int16_t  raw;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_PWM_SCALE, &v));
    raw = (int16_t)TMC_FIELD_GET(v, PWM_SCALE_AUTO_Msk, PWM_SCALE_AUTO_Pos);
    if ((raw & 0x100) != 0) {
        raw = (int16_t)(raw - 0x200);    /* sign-extend 9 bit */
    }
    *out = raw;
    return TMC_OK;
}

tmc_status_t tmc2209_get_pwm_offset_auto(tmc2209_t *dev, uint8_t *out)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_PWM_AUTO, &v));
    *out = (uint8_t)(v & PWM_OFS_AUTO_Msk);
    return TMC_OK;
}

tmc_status_t tmc2209_get_pwm_gradient_auto(tmc2209_t *dev, uint8_t *out)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_PWM_AUTO, &v));
    *out = (uint8_t)TMC_FIELD_GET(v, PWM_GRAD_AUTO_Msk, PWM_GRAD_AUTO_Pos);
    return TMC_OK;
}

tmc_status_t tmc2209_get_microstep_counter(tmc2209_t *dev, uint16_t *mscnt)
{
    uint32_t v;
    TMC_TRY(tmc2209_read_reg(dev, TMC_REG_MSCNT, &v));
    *mscnt = (uint16_t)(v & 0x3FFU);
    return TMC_OK;
}

uint8_t tmc2209_scan_bus(tmc_bus_t *bus)
{
    uint8_t   mask = 0;
    tmc2209_t probe;
    uint8_t   saved_retries;

    if (!tmc_bus_can_read(bus)) {
        return 0;
    }
    saved_retries = bus->retries;
    bus->retries  = 0;
    memset(&probe, 0, sizeof(probe));
    probe.bus = bus;
    for (uint8_t a = 0; a <= TMC_MAX_ADDR; a++) {
        probe.addr = a;
        if (tmc2209_is_communicating(&probe)) {
            mask |= (uint8_t)(1U << a);
        }
    }
    bus->retries = saved_retries;
    return mask;
}

const char *tmc2209_status_str(tmc_status_t s)
{
    switch (s) {
    case TMC_OK:            return "OK";
    case TMC_ERR_PARAM:     return "PARAM";
    case TMC_ERR_TX:        return "TX";
    case TMC_ERR_TIMEOUT:   return "TIMEOUT";
    case TMC_ERR_ECHO:      return "ECHO";
    case TMC_ERR_CRC:       return "CRC";
    case TMC_ERR_REPLY:     return "REPLY";
    case TMC_ERR_NO_RX:     return "NO_RX";
    case TMC_ERR_UART:      return "UART";
    case TMC_ERR_NOT_READY: return "NOT_READY";
    default:                return "?";
    }
}
