#include "bma456.h"

#include "bma456_an.h"
#include "bma4_defs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_spim.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bma456);

nrfx_spim_t *bma456_spim = NULL;
static struct k_fifo bma456_fifo;

volatile bool bma456_ready = false;

struct fifo_interrupt_data
{
    void *fifo_reserved;
    uint32_t interrupt;
};

static struct fifo_interrupt_data tx_data;

static const struct gpio_dt_spec bma456_ss =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), ss_bma456_gpios);
static const struct gpio_dt_spec bma456_int =
    GPIO_DT_SPEC_GET(DT_ALIAS(intbma456), gpios);
static struct gpio_callback bma456_int_cb_data;

extern const uint8_t bma456_an_config_file[];

void __attribute__((used)) bma456_isr(const struct device *dev,
                                      struct gpio_callback *cb,
                                      gpio_port_pins_t pins) {
    LOG_DBG("BMA456 interrupt");
    tx_data.interrupt = 1;
    k_fifo_put(&bma456_fifo, &tx_data);
}

int initialize_bma456(void) {
    k_fifo_init(&bma456_fifo);

    nrfx_err_t status;
    (void)status;

    // set up SPI interface

    // check and set up pins:
    // - slave select
    // - interrupt
    if (!gpio_is_ready_dt(&bma456_ss)) {
        LOG_ERR("BMA456 SS pin not usable!");
        return -1;
    }
    (void)gpio_pin_configure_dt(&bma456_ss, (GPIO_OUTPUT | GPIO_ACTIVE_LOW));

    if (!gpio_is_ready_dt(&bma456_int)) {
        LOG_ERR("BMA456 INT pin not usable!");
        return -1;
    }
    (void)gpio_pin_configure_dt(&bma456_int, (GPIO_INPUT | GPIO_ACTIVE_LOW));
    (void)gpio_pin_interrupt_configure_dt(&bma456_int, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&bma456_int_cb_data, bma456_isr, BIT(bma456_int.pin));
    (void)gpio_add_callback_dt(&bma456_int, &bma456_int_cb_data);

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(BMA456_SPIM_INSTANCE)),
                IRQ_PRIO_LOWEST,
                NRFX_SPIM_INST_HANDLER_GET(BMA456_SPIM_INSTANCE), 0, 0);
#endif // defined(__ZEPHYR__)

    static nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(BMA456_SPIM_INSTANCE);
    bma456_spim = &spim_inst;

    nrfx_spim_config_t spim_config =
        NRFX_SPIM_DEFAULT_CONFIG(BMA456_SCK_PIN, BMA456_MOSI_PIN,
                                 BMA456_MISO_PIN, NRF_SPIM_PIN_NOT_CONNECTED);
    spim_config.frequency = NRFX_MHZ_TO_HZ(2);
    // spim_config.skip_gpio_cfg = true;

    status = nrfx_spim_init(&spim_inst, &spim_config, NULL, NULL);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    int chip_id = 0;

    for (int remaining = 3; remaining > 0; remaining--) {
        chip_id = bma456_get_chip_id();
        if (chip_id == BMA456_AN_CHIP_ID_PRIM) {
            break;
        }
        k_busy_wait(5 * USEC_PER_MSEC);
    }

    if (chip_id != BMA456_AN_CHIP_ID_PRIM) {
        return -1;
    }

    // reset the device
    bma456_soft_reset();
    k_busy_wait(10 * USEC_PER_MSEC);
    for (int remaining = 3; remaining > 0; remaining--) {
        chip_id = bma456_get_chip_id();
        if (chip_id == BMA456_AN_CHIP_ID_PRIM) {
            break;
        }
        k_busy_wait(5 * USEC_PER_MSEC);
    }

    if (chip_id != BMA456_AN_CHIP_ID_PRIM) {
        return -1;
    }

    // upload config/FW
    bma456_write_config_file();

    int bma456_status = 0;
    for (int remaining = 3; remaining > 0; remaining--) {
        // check internal status of sensor for errors
        bma456_status = bma456_get_internal_status();
        if (bma456_status == BMA4_ASIC_INITIALIZED) {
            break;
        }
        k_busy_wait(100);
    }

    if (bma456_status != BMA4_ASIC_INITIALIZED) {
        uint8_t err_reg = 0;
        uint8_t status_reg = 0;
        bma456_reg_read_byte(BMA4_ERROR_ADDR, &err_reg);
        bma456_reg_read_byte(BMA4_STATUS_ADDR, &status_reg);
        LOG_INF("bma456 internal status: 0x%x", bma456_status);
        LOG_INF("bma456 error register: 0x%x", err_reg);
        LOG_INF("bma456 status register: 0x%x", status_reg);
        return -1;
    }

    // configure sensor for anymotion/nomotion interrupts
    bma456_set_int1_config();

    // enable accel
    bma456_set_accel_enable(true);
    k_busy_wait(25 * USEC_PER_MSEC);
    bma456_set_accel_conf();
    k_busy_wait(25 * USEC_PER_MSEC);
    bma456_map_feature_interrupts(
        BMA4_INTR1_MAP,
        (BMA456_AN_ERROR_INT | BMA456_AN_NO_MOT_INT | BMA456_AN_ANY_MOT_INT));
    k_busy_wait(25 * USEC_PER_MSEC);
    // bma456_map_hw_interrupts();
    // k_busy_wait(25 * USEC_PER_MSEC);
    bma456_configure_anymotion_nomotion();
    k_busy_wait(25 * USEC_PER_MSEC);
    bma456_set_power_conf(true);
    k_busy_wait(25 * USEC_PER_MSEC);

    bma456_status = bma456_get_internal_status();

    bma456_ready = true;

    return 0;
}

static void bma456_ss_set(bool enable) {
    if (enable) {
        gpio_pin_set_dt(&bma456_ss, 1);
        k_busy_wait(1 * USEC_PER_MSEC);
    } else {
        k_busy_wait(1 * USEC_PER_MSEC);
        gpio_pin_set_dt(&bma456_ss, 0);
    }
}

int bma456_reg_read_byte(uint8_t reg, uint8_t *data) {
    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;
    uint8_t buf[2];

    // set high bit to 1 to indicate read
    reg |= BMA4_SPI_RD_MASK;

    bma456_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = buf,
                                        .rx_length = 2};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    bma456_ss_set(false);

    *data = buf[1];

    return 0;
}

int bma456_reg_read_burst(uint8_t reg, uint8_t *buf, uint16_t len) {
    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // set high bit to 1 to indicate read
    reg |= BMA4_SPI_RD_MASK;

    bma456_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    uint8_t dummy = 0;
    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = &dummy,
                                        .rx_length = 1};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = buf,
                                        .rx_length = len};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    bma456_ss_set(false);

    return 0;
}

int bma456_reg_write_byte(uint8_t reg, uint8_t val) {
    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // clear high bit to indicate write
    reg &= BMA4_SPI_WR_MASK;

    bma456_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &val,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    bma456_ss_set(false);

    return 0;
}

int bma456_reg_write_burst(uint8_t reg, const uint8_t *buf, uint16_t len) {
    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // clear high bit to indicate write
    reg &= BMA4_SPI_WR_MASK;

    bma456_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = buf,
                                        .tx_length = len,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bma456_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
    }

    // uint32_t txd_amount = bma456_spim->p_reg->TXD.AMOUNT &
    // SPIM_TXD_AMOUNT_AMOUNT_Msk; maybe assert that len == txd_amount?

    bma456_ss_set(false);

    return 0;
}

int bma456_get_chip_id(void) {
    uint8_t data = 0;

    bma456_reg_read_byte(BMA4_CHIP_ID_ADDR, &data);

    return (int)data;
}

int bma456_get_int0_status(void) {
    uint8_t data = 0;

    bma456_reg_read_byte(BMA4_INT_STAT_0_ADDR, &data);

    return (int)data;
}

int bma456_get_int1_status(void) {
    uint8_t data = 0;

    bma456_reg_read_byte(BMA4_INT_STAT_1_ADDR, &data);

    return (int)data;
}

int bma456_get_internal_status(void) {
    uint8_t data = 0;

    bma456_reg_read_byte(BMA4_INTERNAL_STAT, &data);

    return (int)data;
}

int bma456_get_acc_conf(void) {
    uint8_t data = 0;

    bma456_reg_read_byte(BMA4_ACCEL_CONFIG_ADDR, &data);

    return (int)data;
}

int bma456_get_sensortime(void) {
    uint32_t data = 0;

    bma456_reg_read_burst(BMA4_SENSORTIME_0_ADDR, (uint8_t *)&data, 4);

    return (data & 0x00FFFFFF);
}

int bma456_get_temperature(void) {
    int8_t data = 0;
    bma456_reg_read_byte(BMA4_TEMPERATURE_ADDR, (uint8_t *)&data);

    if (data == 0x80) {
        // invalid
        return (int)data;
    }
    return (int)data + 23;
}

void bma456_write_config_file(void) {
    int status = 0;
    bma456_set_power_conf(false);

    k_busy_wait(1 * USEC_PER_MSEC);

    status = bma456_reg_write_byte(BMA4_INIT_CTRL_ADDR, 0x0);
    if (status != 0) {
        LOG_ERR("error setting init ctrl conf to 0");
    }
    k_busy_wait(5);

    const uint16_t CONFIG_FILE_CHUNK_SIZE = 240;
    for (uint16_t idx = 0; idx < BMA456_CONFIG_FILE_SIZE_BYTES;
         idx += CONFIG_FILE_CHUNK_SIZE) {
        uint16_t word_idx = idx / 2;
        uint8_t idx_msb = word_idx >> 4;
        uint8_t idx_lsb = word_idx & 0x0F;

        // https://github.com/boschsensortec/BMA456_SensorAPI/blob/master/bma4.c#L3969
        // This seems to write a data word index when a config file write is
        // split up over many individual writes
        (void)bma456_reg_write_byte(BMA4_RESERVED_REG_5B_ADDR, idx_lsb);

        k_busy_wait(5);

        (void)bma456_reg_write_byte(BMA4_RESERVED_REG_5C_ADDR, idx_msb);

        k_busy_wait(5);

        status = bma456_reg_write_burst(BMA4_FEATURE_CONFIG_ADDR,
                                        (bma456_an_config_file + idx),
                                        CONFIG_FILE_CHUNK_SIZE);

        k_busy_wait(5);
    }
    if (status != 0) {
        LOG_ERR("error writing config file");
    }
    k_busy_wait(5);

    status = bma456_reg_write_byte(BMA4_INIT_CTRL_ADDR, 0x1);
    if (status != 0) {
        LOG_ERR("error setting init ctrl conf to 1");
    }

    k_busy_wait(150 * USEC_PER_MSEC);
}

void bma456_soft_reset(void) {
    uint8_t cmd = BMA4_SOFT_RESET;

    bma456_reg_write_byte(BMA4_CMD_ADDR, cmd);
}

void bma456_set_power_conf(bool adv_pwr_save) {
    uint8_t cfg = (0x2 | // fifo_self_wakeup on
                   (adv_pwr_save ? 0x1 : 0x0));

    bma456_reg_write_byte(BMA4_POWER_CONF_ADDR, cfg);
}

void bma456_set_int1_config(void) {
    uint8_t cfg =
        (BMA4_OUTPUT_ENABLE << 3 | BMA4_PUSH_PULL << 2 | BMA4_ACTIVE_LOW << 1);

    bma456_reg_write_byte(BMA4_INT1_IO_CTRL_ADDR, cfg);

    k_busy_wait(5);

    uint8_t latch = BMA4_LATCH_MODE;
    bma456_reg_write_byte(BMA4_INTR_LATCH_ADDR, latch);
}

void bma456_set_accel_enable(bool enable) {
    uint8_t cfg = ((enable ? 0x1 : 0x0) << BMA4_ACCEL_ENABLE_POS);

    bma456_reg_write_byte(BMA4_POWER_CTRL_ADDR, cfg);
}

void bma456_map_feature_interrupts(uint8_t int_line, uint8_t interrupts) {
    if (int_line == BMA4_INTR1_MAP) {
        bma456_reg_write_byte(BMA4_INT_MAP_1_ADDR, interrupts);
    } else if (int_line == BMA4_INTR2_MAP) {
        bma456_reg_write_byte(BMA4_INT_MAP_2_ADDR, interrupts);
    }
}

void bma456_map_hw_interrupts(void) {
    uint8_t cfg = 0x4; // int1_drdy

    bma456_reg_write_byte(BMA4_INT_MAP_DATA_ADDR, cfg);
}

void bma456_set_accel_conf(void) {
    uint8_t cfg = ((BMA4_CONTINUOUS_MODE << BMA4_ACCEL_PERFMODE_POS) |
                   (BMA4_ACCEL_NORMAL_AVG4 << BMA4_ACCEL_BW_POS) |
                   (BMA4_OUTPUT_DATA_RATE_50HZ));

    bma456_reg_write_byte(BMA4_ACCEL_CONFIG_ADDR, cfg);
}

inline float accel_raw_to_float(uint8_t lsb, uint8_t msb) {
    return (float)((int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb)) / 8192.0f;
}

void bma456_get_accel_sample(struct accel_data *data) {
    uint8_t raw_accel[6];
    uint8_t raw_time[3];

    bma456_reg_read_burst(BMA4_DATA_8_ADDR, raw_accel, 6);
    bma456_reg_read_burst(BMA4_SENSORTIME_0_ADDR, raw_time, 3);

    // NOTE: default range is 4g and we have not changed it
    // 4g == 8192 LSB/g

    data->x = accel_raw_to_float(raw_accel[1], raw_accel[0]);
    data->y = accel_raw_to_float(raw_accel[3], raw_accel[2]);
    ;
    data->z = accel_raw_to_float(raw_accel[5], raw_accel[4]);
    ;

    data->ts =
        (uint32_t)(((uint32_t)(raw_time[0])) | ((uint32_t)(raw_time[1]) << 8) |
                   ((uint32_t)(raw_time[2]) << 16));
}

struct any_no_mo1
{
    uint16_t threshold : 10;
    uint16_t reserved : 6;
};

struct any_no_mo2
{
    uint16_t duration : 13;
    uint16_t x_en : 1;
    uint16_t y_en : 1;
    uint16_t z_en : 1;
};

struct any_no_motion_config
{
    struct any_no_mo1 conf_1;
    struct any_no_mo2 conf_2;
};

typedef union {
    uint16_t words[BMA456_AN_NO_MOT_RD_WR_LEN / 2];
    struct
    {
        struct any_no_motion_config any_motion_config;
        struct any_no_motion_config no_motion_config;
    } as_s;
} any_no_mot_config;

void bma456_configure_anymotion_nomotion(void) {
    // read existing config
    uint8_t raw_config[BMA456_AN_FEATURE_SIZE];
    bma456_reg_read_burst(BMA4_FEATURE_CONFIG_ADDR, raw_config,
                          BMA456_AN_NO_MOT_RD_WR_LEN);
    any_no_mot_config feature_config = {
        .words = {((uint16_t)raw_config[0] << 8) | (uint16_t)raw_config[1],
                  ((uint16_t)raw_config[2] << 8) | (uint16_t)raw_config[3],
                  ((uint16_t)raw_config[4] << 8) | (uint16_t)raw_config[5],
                  ((uint16_t)raw_config[6] << 8) | (uint16_t)raw_config[7]}};

    k_busy_wait(10);
    // modify config to match our needs

    // enable ANY_MOTION for each axis
    feature_config.as_s.any_motion_config.conf_1.threshold = 10;
    feature_config.as_s.any_motion_config.conf_2.duration = 4;
    feature_config.as_s.any_motion_config.conf_2.x_en = 1;
    feature_config.as_s.any_motion_config.conf_2.y_en = 1;
    feature_config.as_s.any_motion_config.conf_2.z_en = 1;

    // enable NO_MOTION for each axis
    feature_config.as_s.no_motion_config.conf_1.threshold = 10;
    feature_config.as_s.no_motion_config.conf_2.duration = 4;
    feature_config.as_s.no_motion_config.conf_2.x_en = 1;
    feature_config.as_s.no_motion_config.conf_2.y_en = 1;
    feature_config.as_s.no_motion_config.conf_2.z_en = 1;

    // serialize back into raw config
    raw_config[0] = (uint8_t)((feature_config.words[0] & 0xFF00) >> 8);
    raw_config[1] = (uint8_t)(feature_config.words[0] & 0x00FF);
    raw_config[2] = (uint8_t)((feature_config.words[1] & 0xFF00) >> 8);
    raw_config[3] = (uint8_t)(feature_config.words[1] & 0x00FF);
    raw_config[4] = (uint8_t)((feature_config.words[2] & 0xFF00) >> 8);
    raw_config[5] = (uint8_t)(feature_config.words[2] & 0x00FF);
    raw_config[6] = (uint8_t)((feature_config.words[3] & 0xFF00) >> 8);
    raw_config[7] = (uint8_t)(feature_config.words[3] & 0x00FF);

    bma456_reg_write_burst(BMA4_FEATURE_CONFIG_ADDR, raw_config,
                           BMA456_AN_NO_MOT_RD_WR_LEN);
}

SYS_INIT(initialize_bma456, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if BMA456_ENABLE_POLL_TASK == 1

K_THREAD_DEFINE(bma456_thread_id, 1024, bma456_task, NULL, NULL, NULL, 6,
                K_USER, 0);
K_THREAD_DEFINE(bma456_poll_thread_id, 1024, bma456_poll, NULL, NULL, NULL, 7,
                K_USER, 0);

void bma456_task(void *, void *, void *) {
    while (!bma456_ready) {
        k_msleep(10);
    }

    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");

    struct accel_data accel_sample;

    while (1) {
        struct fifo_interrupt_data *data = k_fifo_get(&bma456_fifo, K_FOREVER);
        if (!data->interrupt) {
            continue;
        }

        uint8_t int_status_0 = bma456_get_int0_status();
        uint8_t int_status_1 = bma456_get_int1_status();

        // LOG_INF("int status 0: 0x%x", data->int_status_0);
        // LOG_INF("int status 1: 0x%x", data->int_status_1);
        if (int_status_0 & BMA456_AN_ANY_MOT_INT) {
            LOG_INF("got any motion!");
        }
        if (int_status_0 & BMA456_AN_NO_MOT_INT) {
            LOG_INF("got no motion!");
        }
        if (int_status_0 & BMA456_AN_ERROR_INT) {
            LOG_INF("got error!");
        }

        if (int_status_1 & 0x80) {
            LOG_INF("got accel data ready!");

            bma456_get_accel_sample(&accel_sample);
            LOG_INF("acc: %f %f %f %d", accel_sample.x, accel_sample.y,
                    accel_sample.z, accel_sample.ts);
        }
    }
}

void bma456_poll(void *, void *, void *) {
    struct accel_data accel_sample;
    int bma456_status = 0;
    uint8_t err_reg = 0;
    uint8_t status_reg = 0;

    while (!bma456_ready) {
        k_msleep(1000);
    }

    __ASSERT(bma456_spim != NULL, "SPIM is NULL");
    __ASSERT(bma456_spim->p_reg != NULL, "SPIM->p_reg is NULL");

    while (1) {
        bma456_status = bma456_get_internal_status();
        bma456_reg_read_byte(BMA4_ERROR_ADDR, &err_reg);
        bma456_reg_read_byte(BMA4_STATUS_ADDR, &status_reg);
        bma456_get_accel_sample(&accel_sample);

        LOG_INF("bma456 internal status: 0x%x", bma456_status);
        LOG_INF("bma456 error status: 0x%x", err_reg);
        LOG_INF("bma456 status: 0x%x", status_reg);
        LOG_INF("acc: %f %f %f %d", accel_sample.x, accel_sample.y,
                accel_sample.z, accel_sample.ts);

        k_msleep(1000);
    }
}

#endif // BMA456_ENABLE_POLL_TASK == 1