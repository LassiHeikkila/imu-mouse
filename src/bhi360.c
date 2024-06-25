#include "bhi360.h"

#include "bhi3_defs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_spim.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bhi360, LOG_LEVEL_DBG);

nrfx_spim_t *bhi360_spim = NULL;
static struct k_fifo bhi360_fifo;

volatile bool bhi360_ready = false;

static const struct gpio_dt_spec bhi360_ss =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), ss_bhi360_gpios);
static const struct gpio_dt_spec bhi360_int =
    GPIO_DT_SPEC_GET(DT_ALIAS(intbhi360), gpios);
static const struct gpio_dt_spec bhi360_reset =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), reset_bhi360_gpios);
static struct gpio_callback bhi360_int_cb_data;

extern const uint8_t bhi360_config_file[];

uint8_t fifo_data_lengths[] = {[BHY2_SENSOR_ID_ACC_PASS] = 7,
                               [BHY2_SENSOR_ID_ACC_RAW] = 7,
                               [BHY2_SENSOR_ID_ACC] = 7,
                               [BHY2_SENSOR_ID_ACC_BIAS] = 7,
                               [BHY2_SENSOR_ID_ACC_WU] = 7,
                               [BHY2_SENSOR_ID_ACC_RAW_WU] = 7,
                               [BHY2_SENSOR_ID_SI_ACCEL] = 7,
                               [BHY2_SENSOR_ID_GYRO_PASS] = 7,
                               [BHY2_SENSOR_ID_GYRO_RAW] = 7,
                               [BHY2_SENSOR_ID_GYRO] = 7,
                               [BHY2_SENSOR_ID_GYRO_BIAS] = 7,
                               [BHY2_SENSOR_ID_GYRO_WU] = 7,
                               [BHY2_SENSOR_ID_GYRO_RAW_WU] = 7,
                               [BHY2_SENSOR_ID_SI_GYROS] = 7,
                               [BHY2_SENSOR_ID_MAG_PASS] = 7,
                               [BHY2_SENSOR_ID_MAG_RAW] = 7,
                               [BHY2_SENSOR_ID_MAG] = 7,
                               [BHY2_SENSOR_ID_MAG_BIAS] = 7,
                               [BHY2_SENSOR_ID_MAG_WU] = 7,
                               [BHY2_SENSOR_ID_MAG_RAW_WU] = 7,
                               [BHY2_SENSOR_ID_GRA] = 7,
                               [BHY2_SENSOR_ID_GRA_WU] = 7,
                               [BHY2_SENSOR_ID_LACC] = 7,
                               [BHY2_SENSOR_ID_LACC_WU] = 7,
                               [BHY2_SENSOR_ID_RV] = 11,
                               [BHY2_SENSOR_ID_RV_WU] = 11,
                               [BHY2_SENSOR_ID_GAMERV] = 11,
                               [BHY2_SENSOR_ID_GAMERV_WU] = 11,
                               [BHY2_SENSOR_ID_GEORV] = 11,
                               [BHY2_SENSOR_ID_GEORV_WU] = 11,
                               [BHY2_SENSOR_ID_ORI] = 7,
                               [BHY2_SENSOR_ID_ORI_WU] = 7,
                               [BHY2_SENSOR_ID_WRIST_TILT_GESTURE] = 3,
                               [BHY2_SENSOR_ID_DEVICE_ORI] = 7,
                               [BHY2_SENSOR_ID_DEVICE_ORI_WU] = 7,
                               [BHY2_SENSOR_ID_STATIONARY_DET] = 0,
                               [BHY2_SENSOR_ID_MOTION_DET] = 0,
                               [BHY2_SENSOR_ID_ACC_BIAS_WU] = 7,
                               [BHY2_SENSOR_ID_GYRO_BIAS_WU] = 7,
                               [BHY2_SENSOR_ID_MAG_BIAS_WU] = 7,
                               [BHY2_SENSOR_ID_STD_WU] = 0,
                               [BHY2_SENSOR_ID_TEMP] = 3,
                               [BHY2_SENSOR_ID_BARO] = 4,
                               [BHY2_SENSOR_ID_HUM] = 2,
                               [BHY2_SENSOR_ID_GAS] = 5,
                               [BHY2_SENSOR_ID_TEMP_WU] = 3,
                               [BHY2_SENSOR_ID_BARO_WU] = 4,
                               [BHY2_SENSOR_ID_HUM_WU] = 2,
                               [BHY2_SENSOR_ID_GAS_WU] = 5,
                               [BHY2_SENSOR_ID_STC_LP] = 5,
                               [BHY2_SENSOR_ID_STD_LP] = 1,
                               [BHY2_SENSOR_ID_SIG_LP] = 0,
                               [BHY2_SENSOR_ID_STC_LP_WU] = 5,
                               [BHY2_SENSOR_ID_STD_LP_WU] = 1,
                               [BHY2_SENSOR_ID_SIG_LP_WU] = 0,
                               [BHY2_SENSOR_ID_ANY_MOTION_LP] = 1,
                               [BHY2_SENSOR_ID_ANY_MOTION_LP_WU] = 1,
                               [BHY2_SYS_ID_PADDING] = 1,
                               [BHY2_SYS_ID_TS_SMALL_DELTA] = 2,
                               [BHY2_SYS_ID_TS_LARGE_DELTA] = 3,
                               [BHY2_SYS_ID_TS_FULL] = 6,
                               [BHY2_SYS_ID_META_EVENT] = 4,
                               [BHY2_SYS_ID_TS_SMALL_DELTA_WU] = 2,
                               [BHY2_SYS_ID_TS_LARGE_DELTA_WU] = 3,
                               [BHY2_SYS_ID_TS_FULL_WU] = 6,
                               [BHY2_SYS_ID_META_EVENT_WU] = 4,
                               [BHY2_SYS_ID_FILLER] = 1,
                               [BHY2_SYS_ID_DEBUG_MSG] = 18};

void __attribute__((used)) bhi360_isr(const struct device *dev,
                                      struct gpio_callback *cb,
                                      gpio_port_pins_t pins) {
    LOG_DBG("BHI360 interrupt");

    uint8_t data = 0;
    bhi360_reg_read_byte(BHY2_REG_INT_STATUS, &data);

    LOG_DBG("int status: 0x%x", data);
}

static void bhi360_read_int_line(void) {
    int v = gpio_pin_get(bhi360_int.port, bhi360_int.pin);
    int r = gpio_pin_get_raw(bhi360_int.port, bhi360_int.pin);

    LOG_INF("BHI360 interrupt line: logical: %d / raw: %d", v, r);
}

static void bhi360_dump_info(void) {
    LOG_INF("Chip ID: 0x%x", bhi360_get_chip_id());
    LOG_INF("Fuser2 ID: 0x%x", bhi360_get_fuser2_id());
    LOG_INF("Fuser2 revision: 0x%x", bhi360_get_fuser2_rev());
    LOG_INF("ROM version: 0x%x", bhi360_get_rom_version());
    LOG_INF("Kernel version: 0x%x", bhi360_get_kernel_version());
    LOG_INF("User version: 0x%x", bhi360_get_user_version());
    LOG_INF("Boot status: 0x%x", bhi360_get_boot_status());
    LOG_INF("Feature status: 0x%x", bhi360_get_feature_status());
    LOG_INF("Host status: 0x%x", bhi360_get_host_status());
    LOG_INF("Error value: 0x%x", bhi360_get_error_value());
    LOG_INF("Interrupt status: 0x%x", bhi360_get_interrupt_status());
    LOG_INF("Interrupt control: 0x%x", bhi360_get_interrupt_control());
}

int initialize_bhi360(void) {
    LOG_INF("initializing BHI360");
    nrfx_err_t status;
    (void)status;

    // set up SPI interface
    // check and set up pins:
    // - slave select
    // - interrupt
    // - reset
    if (!gpio_is_ready_dt(&bhi360_ss)) {
        LOG_ERR("BHI360 SS pin not usable!");
        return -1;
    }
    (void)gpio_pin_configure_dt(&bhi360_ss, (GPIO_OUTPUT | GPIO_ACTIVE_LOW));

    if (!gpio_is_ready_dt(&bhi360_reset)) {
        LOG_ERR("BHI360 RESET pin not usable!");
        return -1;
    }
    (void)gpio_pin_configure_dt(&bhi360_reset, (GPIO_OUTPUT | GPIO_ACTIVE_LOW));

    if (!gpio_is_ready_dt(&bhi360_int)) {
        LOG_ERR("BHI360 INT pin not usable!");
        return -1;
    }
    (void)gpio_pin_configure_dt(&bhi360_int,
                                (GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW));
    (void)gpio_pin_interrupt_configure_dt(&bhi360_int, GPIO_INT_LEVEL_ACTIVE);
    gpio_init_callback(&bhi360_int_cb_data, bhi360_isr, BIT(bhi360_int.pin));
    (void)gpio_add_callback_dt(&bhi360_int, &bhi360_int_cb_data);

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(BHI360_SPIM_INSTANCE)),
                IRQ_PRIO_LOWEST,
                NRFX_SPIM_INST_HANDLER_GET(BHI360_SPIM_INSTANCE), 0, 0);
#endif // defined(__ZEPHYR__)

    static nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(BHI360_SPIM_INSTANCE);
    bhi360_spim = &spim_inst;

    nrfx_spim_config_t spim_config =
        NRFX_SPIM_DEFAULT_CONFIG(BHI360_SCK_PIN, BHI360_MOSI_PIN,
                                 BHI360_MISO_PIN, NRF_SPIM_PIN_NOT_CONNECTED);
    spim_config.frequency = NRFX_MHZ_TO_HZ(8);
    // spim_config.skip_gpio_cfg = true;

    status = nrfx_spim_init(&spim_inst, &spim_config, NULL, NULL);
    if (status != NRFX_SUCCESS) {
        LOG_ERR("failed to init SPIM: %d", status);
        return -1;
    }

    LOG_INF("power cycle BHI360");
    bhi360_hard_reset();

    k_busy_wait(200 * USEC_PER_MSEC);

    int chip_id = 0;

    LOG_INF("poll BHI360 until host interface ready");
    while (!(bhi360_get_boot_status() & BHY2_BST_HOST_INTERFACE_READY)) {
        k_busy_wait(10 * USEC_PER_MSEC);
    }

    chip_id = bhi360_get_chip_id();
    if (chip_id != BHI3_PRODUCT_ID) {
        LOG_ERR("unexpected chip id: 0x%x", chip_id);
        return -1;
    }

    bhi360_read_int_line();

    // empty status & debug fifo
    bhi360_read_status_debug_fifo();

    // dump info
    // bhi360_dump_info();

    // upload config/FW
    LOG_INF("uploading BHI360 firmware");
    int err = bhi360_write_config_file();
    if (err != 0) {
        LOG_ERR("error writing BHI360 config file: %d", err);
    }
    k_busy_wait(200 * USEC_PER_MSEC);
    bhi360_read_response(NULL);
    LOG_INF("int status: 0x%x", bhi360_get_interrupt_status());

    // poll boot status for firmware verify done
    LOG_INF("poll BHI360 until firmware verified");
    while (true) {
        int boot_status = bhi360_get_boot_status();
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            LOG_INF("BHI360 firmware ok");
            break;
        }
        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            LOG_ERR("BHI360 firmware verification error!");
            return 1;
        }
    }

    // tell sensor to boot to RAM
    LOG_INF("boot BHI360 to RAM");
    bhi360_command_t boot_to_ram = {.id = BHY2_CMD_BOOT_PROGRAM_RAM, .len = 0};
    bhi360_write_command(&boot_to_ram);
    bhi360_read_response(NULL);

    // poll boot status for host interface ready
    LOG_INF("poll BHI360 until host interface ready");
    while (!(bhi360_get_boot_status() & BHY2_BST_HOST_INTERFACE_READY)) {
        k_busy_wait(10 * USEC_PER_MSEC);
    }

    LOG_INF("BHI360 up and running");

    // configure interrupt to active low
    bhi360_configure_interrupts();

    k_busy_wait(100 * USEC_PER_MSEC);
    bhi360_read_int_line();
    int int_status = bhi360_get_interrupt_status();
    LOG_INF("host interrupt asserted: %d", (int_status & 0x1));

    // configure sensor to produce desired data

    LOG_INF("Configure game rotation vector");
    bhi360_configure_game_rotation();

    bhi360_read_nonwakeup_fifo();
    bhi360_read_wakeup_fifo();
    // bhi360_read_status_debug_fifo();

    // dump info again
    // bhi360_dump_info();

    return 0;
}

static void bhi360_ss_set(bool enable) {
    if (enable) {
        gpio_pin_set_dt(&bhi360_ss, 1);
        k_busy_wait(2 * USEC_PER_MSEC);
    } else {
        k_busy_wait(2 * USEC_PER_MSEC);
        gpio_pin_set_dt(&bhi360_ss, 0);
    }
}

int bhi360_reg_read_byte(uint8_t reg, uint8_t *data) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // set high bit to 1 to indicate read
    reg |= BHI3_SPI_RD_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = data,
                                        .rx_length = sizeof(*data)};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        return -1;
        bhi360_ss_set(false);
    }

    bhi360_ss_set(false);

    return 0;
}

int bhi360_reg_read_u16(uint8_t reg, uint16_t *data) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // set high bit to 1 to indicate read
    reg |= BHI3_SPI_RD_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = (uint8_t *)data,
                                        .rx_length = sizeof(*data)};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    bhi360_ss_set(false);

    return 0;
}

int bhi360_reg_read_burst(uint8_t reg, uint8_t *buf, uint16_t len) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // set high bit to 1 to indicate read
    reg |= BHI3_SPI_RD_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = buf,
                                        .rx_length = len};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    bhi360_ss_set(false);

    return 0;
}

int bhi360_reg_write_byte(uint8_t reg, uint8_t val) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // clear high bit to indicate write
    reg &= BHI3_SPI_WR_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &val,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    bhi360_ss_set(false);

    return 0;
}

int bhi360_reg_write_burst(uint8_t reg, const uint8_t *buf, uint16_t len) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // clear high bit to indicate write
    reg &= BHI3_SPI_WR_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = buf,
                                        .tx_length = len,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    // uint32_t txd_amount = bhi360_spim->p_reg->TXD.AMOUNT &
    // SPIM_TXD_AMOUNT_AMOUNT_Msk; maybe assert that len == txd_amount?

    bhi360_ss_set(false);

    return 0;
}

int bhi360_write_command(const bhi360_command_t *cmd) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");
    __ASSERT(cmd != NULL, "command is NULL");

    LOG_HEXDUMP_INF(cmd, 4, "command header:");
    LOG_HEXDUMP_INF(cmd->payload, cmd->len, "command payload:");

    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // clear high bit to indicate write
    uint8_t reg = BHY2_REG_CHAN_CMD & BHI3_SPI_WR_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = sizeof(reg),
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = (uint8_t *)&cmd->id,
                                        .tx_length = sizeof(cmd->id),
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = (uint8_t *)&cmd->len,
                                        .tx_length = sizeof(cmd->len),
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};
    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    if (cmd->payload) {
        xfer_desc =
            (nrfx_spim_xfer_desc_t){.p_tx_buffer = (uint8_t *)cmd->payload,
                                    .tx_length = cmd->len,
                                    .p_rx_buffer = NULL,
                                    .rx_length = 0};
        status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
        if (status != NRFX_SUCCESS) {
            bhi360_ss_set(false);
            return -1;
        }
    }

    bhi360_ss_set(false);

    return 0;
}

int bhi360_read_response(bhi360_response_t *rsp) {
    __ASSERT(bhi360_spim != NULL, "SPIM is NULL");
    __ASSERT(bhi360_spim->p_reg != NULL, "SPIM->p_reg is NULL");

    bhi360_response_t local_rsp;
    uint8_t local_rsp_buffer[32];
    bool handleLocally = false;

    if (rsp == NULL) {
        handleLocally = true;
        rsp = &local_rsp;
        rsp->payload = local_rsp_buffer;
        rsp->payload_cap = sizeof(local_rsp_buffer);
    }

    nrfx_err_t status;
    nrfx_spim_xfer_desc_t xfer_desc;

    // set high bit to 1 to indicate read
    uint8_t reg = BHY2_REG_CHAN_STATUS | BHI3_SPI_RD_MASK;

    bhi360_ss_set(true);

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = &reg,
                                        .tx_length = 1,
                                        .p_rx_buffer = NULL,
                                        .rx_length = 0};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = (uint8_t *)&rsp->status,
                                        .rx_length = sizeof(rsp->status)};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    xfer_desc = (nrfx_spim_xfer_desc_t){.p_tx_buffer = NULL,
                                        .tx_length = 0,
                                        .p_rx_buffer = (uint8_t *)&rsp->len,
                                        .rx_length = sizeof(rsp->len)};

    status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
    if (status != NRFX_SUCCESS) {
        bhi360_ss_set(false);
        return -1;
    }

    if (rsp->payload) {
        xfer_desc = (nrfx_spim_xfer_desc_t){
            .p_tx_buffer = NULL,
            .tx_length = 0,
            .p_rx_buffer = (uint8_t *)rsp->payload,
            .rx_length = MIN(rsp->payload_cap, rsp->len)};

        status = nrfx_spim_xfer(bhi360_spim, &xfer_desc, 0);
        if (status != NRFX_SUCCESS) {
            bhi360_ss_set(false);
            return -1;
        }
    }

    bhi360_ss_set(false);

    if (handleLocally) {
        LOG_INF("response id: 0x%x, len: %d", rsp->status, rsp->len);
        LOG_HEXDUMP_INF(rsp->payload, MIN(rsp->len, rsp->payload_cap),
                        "response payload:");
    }

    return 0;
}

int bhi360_get_chip_id(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_CHIP_ID, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_CHIP_ID: %d", err);
    }

    return val;
}

int bhi360_get_host_status(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_HOST_STATUS, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_HOST_STATUS: %d", err);
    }

    return val;
}

int bhi360_get_fuser2_id(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_PRODUCT_ID, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_PRODUCT_ID: %d", err);
    }

    return val;
}

int bhi360_get_fuser2_rev(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_REVISION_ID, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_REVISION_ID: %d", err);
    }

    return val;
}

int bhi360_get_rom_version(void) {
    uint16_t val = 0;

    int err = bhi360_reg_read_u16(BHY2_REG_ROM_VERSION_0, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_ROM_VERSION_0: %d", err);
    }

    return val;
}

int bhi360_get_kernel_version(void) {
    uint16_t val = 0;

    int err = bhi360_reg_read_u16(BHY2_REG_KERNEL_VERSION_0, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_KERNEL_VERSION_0: %d", err);
    }

    return val;
}

int bhi360_get_user_version(void) {
    uint16_t val = 0;

    int err = bhi360_reg_read_u16(BHY2_REG_USER_VERSION_0, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_USER_VERSION_0: %d", err);
    }

    return val;
}

int bhi360_get_feature_status(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_FEATURE_STATUS, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_FEATURE_STATUS: %d", err);
    }

    return val;
}

int bhi360_get_boot_status(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_BOOT_STATUS, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_BOOT_STATUS: %d", err);
    }

    return val;
}

int bhi360_get_interrupt_status(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_INT_STATUS, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_INT_STATUS: %d", err);
    }

    return val;
}

int bhi360_get_error_value(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_ERROR_VALUE, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_ERROR_VALUE: %d", err);
    }

    return val;
}

void bhi360_soft_reset(void) {

    int err = bhi360_reg_write_byte(BHY2_REG_RESET_REQ, 0x1);

    if (err != 0) {
        LOG_ERR("error reading xxx: %d", err);
    }
}

void bhi360_hard_reset(void) {
    gpio_pin_set_dt(&bhi360_reset, 0);
    k_busy_wait(100 * USEC_PER_MSEC);
    gpio_pin_set_dt(&bhi360_reset, 1);
}

void bhi360_set_chip_control(bool turbo, bool clear_errors) {
    uint8_t cfg = 0;

    if (turbo) {
        cfg |= BHY2_CHIP_CTRL_TURBO_ENABLE;
    }

    if (clear_errors) {
        cfg |= BHY2_CHIP_CTRL_CLR_ERR_REG;
    }

    bhi360_reg_write_byte(BHY2_REG_CHIP_CTRL, cfg);
}

int bhi360_write_config_file(void) {
    __ASSERT(
        BHI360_CONFIG_FILE_SIZE_BYTES > (BHY2_COMMAND_PACKET_LEN - 4),
        "Config file smaller than one chunk! Update to program logic needed.");

    int err = 0;
    uint32_t pos = 0;

    bhi360_command_t cmd_header = {
        .id = BHY2_CMD_UPLOAD_TO_PROGRAM_RAM,
        .len = BHI360_CONFIG_FILE_SIZE_BYTES / sizeof(uint32_t),
    };

    // payload length must be divisible by four, pad it if needed
    uint8_t payload[BHY2_COMMAND_PACKET_LEN - 4];
    memcpy(&payload[0], &cmd_header, 4);

    memcpy(&payload[4], &bhi360_config_file[pos], sizeof(payload) - 4);
    pos += sizeof(payload) - 4;

    // write first packet including command and payload length
    err = bhi360_reg_write_burst(BHY2_REG_CHAN_CMD, payload, sizeof(payload));
    if (err != 0) {
        LOG_ERR("error writing first config file chunk: %d", err);
        return -1;
    }

    while (pos < BHI360_CONFIG_FILE_SIZE_BYTES) {
        int bytes_to_send =
            MIN(BHI360_CONFIG_FILE_SIZE_BYTES - pos, sizeof(payload));
        memcpy(payload, &bhi360_config_file[pos], bytes_to_send);

        err = bhi360_reg_write_burst(BHY2_REG_CHAN_CMD, payload, bytes_to_send);
        if (err != 0) {
            LOG_ERR("error writing config file chunk: %d", err);
            return -1;
        }

        pos += bytes_to_send;
    }

    return 0;
}

typedef union {
    uint8_t as_bytes[8];
    struct __attribute__((packed))
    {
        uint8_t id;
        float rate;
        uint8_t latency[3];
    } as_struct;
} bhi360_sensor_config_payload_t;

union u32_or_float {
    uint32_t u32;
    float f;
};

void bhi360_configure_game_rotation(void) {
    // IMU fusion data is virtual sensor ID: 37 for non-wakeup and 38 for wakeup
    // We will poll it so go with non-wakeup
    // Data is represented as quaternions
    // see https://eater.net/quaternions

    // bhi360_sensor_config_payload_t payload = {
    //     .as_struct = {
    //         .id = BHY2_SENSOR_ID_GAMERV_WU,
    //         .rate = 25.0f,
    //         .latency = {0, 0, 0}
    //     }
    // };

    union u32_or_float rate = {.f = 25.0f};

    uint8_t payload[8];
    payload[0] = BHY2_SENSOR_ID_GAMERV_WU;
    payload[1] = (uint8_t)(rate.u32 & 0xFF);
    payload[2] = (uint8_t)((rate.u32 >> 8) & 0xFF);
    payload[3] = (uint8_t)((rate.u32 >> 16) & 0xFF);
    payload[4] = (uint8_t)((rate.u32 >> 24) & 0xFF);
    payload[5] = 0;
    payload[6] = 0;
    payload[7] = 0;

    bhi360_command_t configure_game_rotation = {.id = BHY2_CMD_CONFIG_SENSOR,
                                                .len = sizeof(payload),
                                                .payload = (void *)payload};

    bhi360_write_command(&configure_game_rotation);
    bhi360_read_response(NULL);
}

static void bhi360_get_fifo_len(uint8_t channel, uint16_t *len) {
    bhi360_reg_read_u16(channel, len);
    LOG_DBG("FIFO %d has %d bytes available", channel, *len);
}

static int bhi360_read_fifo(uint8_t channel, uint8_t *buf, uint16_t cap) {
    // first two bytes or 16-bit word indicates how many bytes are available
    uint16_t len = 0;
    bhi360_get_fifo_len(channel, &len);

    if (len == 0) {
        LOG_DBG("FIFO %d empty", channel);
        return 0;
    }

    if (len > (cap - 2)) {
        len = cap - 2;
    }

    bhi360_reg_read_burst(channel, buf, len);
    return len;
}
typedef struct
{
    uint16_t transfer_length;
    uint16_t timestamp;
} bhi360_fifo_descriptor_t;

typedef __attribute__((packed)) struct
{
    uint16_t meta_event;
    uint8_t timestamp[5];
} bhi360_fifo_block_header_t;

typedef struct
{
    uint8_t sensor_id;
    uint8_t len;
    uint8_t *data;
} bhi360_fifo_block_content_t;

typedef __attribute__((packed)) struct
{
    bhi360_fifo_block_header_t header;
    bhi360_fifo_block_content_t content;
} bhi360_fifo_block_t;

static int bhi360_parse_fifo_descriptor(uint8_t *buf, uint16_t len,
                                        bhi360_fifo_descriptor_t *desc) {
    __ASSERT(len >= 4, "length should be at least 4");
    if (len < 4) {
        LOG_ERR(
            "cannot parse FIFO descriptor, less than 4 bytes of data given");
        return 0;
    }
    desc->transfer_length = (uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8);
    desc->timestamp = (uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8);

    // return how many bytes were processed
    return 4;
}

static int bhi360_parse_fifo_block(uint8_t *buf, uint16_t len,
                                   bhi360_fifo_block_t *block) {
    __ASSERT(len >= 8, "length should be at least 8");
    if (len < 8) {
        LOG_ERR("cannot parse FIFO block, less than 8 bytes of data given");
        return 0;
    }

    block->header.meta_event = (uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8);
    memcpy(&(block->header.timestamp), &buf[2], 5);

    int data_len = fifo_data_lengths[buf[7]];
    int pad_bytes = (4 - (data_len % 4)) % 4;

    block->content.sensor_id = buf[7];
    block->content.len = data_len - 1;
    block->content.data = &buf[8];

    // return how many bytes were processed
    return 7 + data_len + pad_bytes;
}

void bhi360_read_wakeup_fifo(void) {
    uint8_t buf[BHI360_FIFO_READ_BUFFER_LENGTH];
    bhi360_fifo_descriptor_t desc;
    bhi360_fifo_block_t block;
    int len = 0;
    int bytes_processed = 0;
    int idx = 0;

    len = bhi360_read_fifo(BHY2_REG_CHAN_FIFO_W, buf,
                           BHI360_FIFO_READ_BUFFER_LENGTH);
    if (len == 0) {
        LOG_DBG("empty fifo");
        return;
    }

    bytes_processed = bhi360_parse_fifo_descriptor(&buf[idx], len, &desc);
    idx += bytes_processed;
    len -= bytes_processed;

    while (len > 0) {
        bytes_processed = bhi360_parse_fifo_block(&buf[idx], len, &block);
        if (bytes_processed == 0) {
            break;
        }
        idx += bytes_processed;
        len -= bytes_processed;
        LOG_INF("FIFO block: sensor id %d", block.content.sensor_id);
    }
}

void bhi360_read_nonwakeup_fifo(void) {
    uint8_t buf[BHI360_FIFO_READ_BUFFER_LENGTH];
    bhi360_fifo_descriptor_t desc;
    bhi360_fifo_block_t block;
    int len = 0;
    int bytes_processed = 0;
    int idx = 0;

    len = bhi360_read_fifo(BHY2_REG_CHAN_FIFO_NW, buf,
                           BHI360_FIFO_READ_BUFFER_LENGTH);
    if (len == 0) {
        LOG_DBG("empty fifo");
        return;
    }

    bytes_processed = bhi360_parse_fifo_descriptor(&buf[idx], len, &desc);
    idx += bytes_processed;
    len -= bytes_processed;

    while (len > 0) {
        bytes_processed = bhi360_parse_fifo_block(&buf[idx], len, &block);
        if (bytes_processed == 0) {
            break;
        }
        idx += bytes_processed;
        len -= bytes_processed;
        LOG_INF("FIFO block: sensor id %d", block.content.sensor_id);
    }
}

void bhi360_read_status_debug_fifo(void) {
    uint8_t buf[BHI360_FIFO_READ_BUFFER_LENGTH];
    bhi360_fifo_descriptor_t desc;
    bhi360_fifo_block_t block;
    int len = 0;
    int bytes_processed = 0;
    int idx = 0;

    len = bhi360_read_fifo(BHY2_REG_CHAN_STATUS, buf,
                           BHI360_FIFO_READ_BUFFER_LENGTH);
    if (len == 0) {
        LOG_DBG("empty fifo");
        return;
    }

    LOG_INF("read %d bytes", len);
    // just dump the data for now...
    LOG_HEXDUMP_INF(buf, len, "Status and Debug FIFO data");
}

void bhi360_configure_interrupts(void) {
    // bits 0-4 == 0 to not mask any interrupts
    // bit 5 == 1 for active low
    // bit 6 == 0 for level output
    // bit 7 == 1 for open drain
    uint8_t cfg = ((1 << 5) | (1 << 7));
    bhi360_reg_write_byte(BHY2_REG_HOST_INTERRUPT_CTRL, cfg);
}

int bhi360_get_interrupt_control(void) {
    uint8_t val = 0;

    int err = bhi360_reg_read_byte(BHY2_REG_HOST_INTERRUPT_CTRL, &val);

    if (err != 0) {
        LOG_ERR("error reading BHY2_REG_HOST_INTERRUPT_CTRL: %d", err);
    }

    return val;
}

void bhi360_read_parameter(uint16_t id) {
    uint16_t cmd_id = 0x1000 | id;
    bhi360_command_t cmd = {.id = cmd_id, .len = 0x0000};

    bhi360_write_command(&cmd);

    k_busy_wait(10 * USEC_PER_MSEC);

    bhi360_read_response(NULL);
}

void bhi360_write_parameter(uint16_t id, uint16_t len, uint8_t *data) {
    bhi360_command_t cmd = {.id = (id & 0x7FFF), .len = len, .payload = data};

    bhi360_write_command(&cmd);
}

void bhi360_dump_rotation_config(void) { bhi360_read_parameter(0x126); }

SYS_INIT(initialize_bhi360, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if BHI360_ENABLE_POLL_TASK == 1
void bhi360_poll_task(void *, void *, void *) {

    // dump available virtual sensors
    bhi360_read_parameter(0x011F);

    // dump available physical sensors
    bhi360_read_parameter(0x0120);

    while (true) {
        uint8_t int_status = 0;
        bhi360_reg_read_byte(BHY2_REG_INT_STATUS, &int_status);

        if (int_status & BHY2_IST_MASK_ASSERTED) {
            LOG_INF("interrupt asserted");
        }

        if (BHY2_IS_INT_FIFO_W(int_status)) {
            LOG_INF("read wakeup fifo");
            bhi360_read_wakeup_fifo();
        }

        if (BHY2_IS_INT_FIFO_NW(int_status)) {
            LOG_INF("read non-wakeup fifo");
            bhi360_read_nonwakeup_fifo();
        }

        if (BHY2_IS_INT_STATUS(int_status)) {
            uint16_t remaining_len = 0;
            bhi360_get_fifo_len(BHY2_REG_CHAN_STATUS, &remaining_len);
            uint8_t payload[32];
            while (remaining_len > 0) {
                LOG_INF("read status and debug fifo");
                bhi360_response_t rsp = {.payload = payload,
                                         .payload_cap = sizeof(payload)};

                bhi360_read_response(&rsp);
                LOG_INF("response: status: %d len: %d", rsp.status, rsp.len);
                LOG_HEXDUMP_INF(rsp.payload, rsp.len, "response payload");

                if (rsp.status == 0x000F) {
                    // command error
                    uint16_t cmd = (((uint8_t *)rsp.payload)[0] |
                                    ((uint8_t *)rsp.payload)[1] << 8);
                    uint8_t err = ((uint8_t *)rsp.payload)[2];
                    LOG_ERR("command error: 0x%x: 0x%x", cmd, err);
                }
                // bhi360_read_status_debug_fifo();
                bhi360_get_fifo_len(BHY2_REG_CHAN_STATUS, &remaining_len);
            }
        }

        bhi360_dump_info();
        bhi360_dump_rotation_config();

        k_busy_wait(1000 * USEC_PER_MSEC);
        // k_msleep(1000);
    }
}

K_THREAD_DEFINE(bhi360_poll_thread_id, 1024, bhi360_poll_task, NULL, NULL, NULL,
                6, K_USER, 0);
#endif // BHI360_ENABLE_POLL_TASK == 1
