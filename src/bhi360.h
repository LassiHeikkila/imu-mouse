#ifndef APPLICATION_BHI360_H_
#define APPLICATION_BHI360_H_

#include <stdbool.h>
#include <stdint.h>

#define BHI360_SPIM_INSTANCE 1
#define BHI360_SS_PIN 31
#define BHI360_SCK_PIN 3
#define BHI360_MOSI_PIN 4
#define BHI360_MISO_PIN 5

// this just so happens to be divisible by four, so no extra padding is needed
#define BHI360_CONFIG_FILE_SIZE_BYTES 127228
#define BHI360_FIFO_READ_BUFFER_LENGTH 512

#define BHI3_SPI_RD_MASK 0x80 // addr | BHI3_SPI_RD_MASK
#define BHI3_SPI_WR_MASK 0x7F // addr & BHI3_SPI_WR_MASK

#define ROUND_TO_NEAREST_4_UP(x) (((x + 3) >> 2) << 2)
#define ROUND_TO_NEAREST_4_DOWN(x) (((x) >> 2) << 2)

#define BHI360_ENABLE_POLL_TASK 1

typedef struct
{
    uint16_t id;   // command id
    uint16_t len;  // payload length in bytes. must be multiple of 4.
    void *payload; // pointer to payload. can be null.
} bhi360_command_t;

typedef struct
{
    uint16_t status;      // status code
    uint16_t len;         // length of payload
    void *payload;        // pointer to payload buffer. can be null.
    uint16_t payload_cap; // capacity of payload buffer
} bhi360_response_t;

/* Init function for setting up MCU side of things and configuring sensor */
int initialize_bhi360(void);

/* SPI writes and reads */

int bhi360_reg_read_byte(uint8_t reg, uint8_t *data);
int bhi360_reg_read_u16(uint8_t reg, uint16_t *data);
int bhi360_reg_read_burst(uint8_t reg, uint8_t *data, uint16_t len);

int bhi360_reg_write_byte(uint8_t reg, uint8_t val);
int bhi360_reg_write_burst(uint8_t reg, const uint8_t *buf, uint16_t len);

int bhi360_write_command(const bhi360_command_t *cmd);

int bhi360_read_response(bhi360_response_t *rsp);

/* Reset methods */

void bhi360_soft_reset(void);

void bhi360_hard_reset(void);

/* Getters */

int bhi360_get_host_status(void);

int bhi360_get_fuser2_id(void);

int bhi360_get_fuser2_rev(void);

int bhi360_get_rom_version(void);

int bhi360_get_kernel_version(void);

int bhi360_get_user_version(void);

int bhi360_get_feature_status(void);

int bhi360_get_boot_status(void);

int bhi360_get_chip_id(void);

int bhi360_get_interrupt_status(void);

int bhi360_get_error_value(void);

int bhi360_get_interrupt_control(void);

void bhi360_read_parameter(uint16_t id);

/* FIFO handling */

void bhi360_read_wakeup_fifo(void);

void bhi360_read_nonwakeup_fifo(void);

void bhi360_read_status_debug_fifo(void);

/* Setters */

void bhi360_set_chip_control(bool turbo, bool clear_errors);

/* More elaborate sensor config functions */

void bhi360_write_parameter(uint16_t id, uint16_t len, uint8_t *data);

int bhi360_write_config_file(void);

void bhi360_configure_interrupts(void);

// Game Rotation Vector is orientation vector calculated from six axis of data:
// - 3 accelerometer axis (x, y, z)
// - 3 gyroscope axis (x, y, z)
void bhi360_configure_game_rotation(void);

// Rotation Vector is orientation vector calculated from nine axis of data:
// - 3 accelerometer axis (x, y, z)
// - 3 gyroscope axis (x, y, z)
// - 3 magnetometer axis (x, y, z)
void bhi360_configure_rotation(void);

void bhi360_dump_rotation_config(void);

#if BHI360_ENABLE_POLL_TASK == 1
void bhi360_poll_task(void *, void *, void *);
#endif // BHI360_ENABLE_POLL_TASK == 1

#endif // APPLICATION_BHI360_H_
