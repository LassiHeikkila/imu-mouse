#ifndef APPLICATION_BMA456_H_
#define APPLICATION_BMA456_H_

#include <stdbool.h>
#include <stdint.h>

#include "accel_data.h"

#define BMA456_SPIM_INSTANCE 0
#define BMA456_SS_PIN 11
#define BMA456_SCK_PIN 14
#define BMA456_MOSI_PIN 16
#define BMA456_MISO_PIN 15

#define BMA456_CONFIG_FILE_SIZE_BYTES 1200

#define BMA456_ENABLE_POLL_TASK 1

/* Init function for setting up MCU side of things and configuring sensor */
int initialize_bma456(void);

/* SPI writes and reads */
int bma456_reg_read_byte(uint8_t reg, uint8_t *data);
int bma456_reg_read_burst(uint8_t reg, uint8_t *data, uint16_t len);

int bma456_reg_write_byte(uint8_t reg, uint8_t val);
int bma456_reg_write_burst(uint8_t reg, const uint8_t *buf, uint16_t len);

/* Sensor getters */
int bma456_get_chip_id(void);

int bma456_get_int0_status(void);

int bma456_get_int1_status(void);

int bma456_get_internal_status(void);

int bma456_get_acc_conf(void);

int bma456_get_sensortime(void);

int bma456_get_temperature(void);

void bma456_get_accel_sample(struct accel_data *data);

/* Sensor setters */

void bma456_soft_reset(void);

void bma456_set_power_conf(bool adv_pwr_save);

void bma456_set_int1_config(void);

void bma456_set_accel_conf(void);

void bma456_set_accel_enable(bool enable);

void bma456_map_feature_interrupts(uint8_t int_line, uint8_t interrupts);

void bma456_map_hw_interrupts(void);

/* More elaborate sensor config functions */
void bma456_write_config_file(void);

void bma456_configure_anymotion_nomotion(void);

/* Threads */

#if BMA456_ENABLE_POLL_TASK == 1

void bma456_task(void *, void *, void *);

void bma456_poll(void *, void *, void *);

#endif // BMA456_ENABLE_POLL_TASK == 1

#endif // APPLICATION_BMA456_H_
