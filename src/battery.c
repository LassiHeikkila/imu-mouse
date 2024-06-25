/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Extended by Lassi Heikkilä

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nrf52.h>
#include <nrf52_bitfields.h>

#include "battery.h"

LOG_MODULE_REGISTER(battery);

#define VBATT DT_PATH(vbatt)
#define ZEPHYR_USER DT_PATH(zephyr_user)

#define BATTERY_ADC_GAIN ADC_GAIN_1_6

struct io_channel_config
{
    uint8_t channel;
};

struct divider_config
{
    struct io_channel_config io_channel;
    struct gpio_dt_spec power_gpios;
    /* output_ohm is used as a flag value: if it is nonzero then
     * the battery is measured through a voltage divider;
     * otherwise it is assumed to be directly connected to Vdd.
     */
    uint32_t output_ohm;
    uint32_t full_ohm;
};

static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
    .io_channel =
        {
            DT_IO_CHANNELS_INPUT(VBATT),
        },
    .power_gpios = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {}),
    .output_ohm = DT_PROP(VBATT, output_ohms),
    .full_ohm = DT_PROP(VBATT, full_ohms),
#else  /* /vbatt exists */
    .io_channel =
        {
            DT_IO_CHANNELS_INPUT(ZEPHYR_USER),
        },
#endif /* /vbatt exists */
};

struct divider_data
{
    const struct device *adc;
    struct adc_channel_cfg adc_cfg;
    struct adc_sequence adc_seq;
    int16_t raw;
};
static struct divider_data divider_data = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
    .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(VBATT)),
#else
    .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER)),
#endif
};

/** A discharge curve specific to the power source. */
static const struct battery_level_point levels[] = {
    /* "Curve" here eyeballed from captured data for the [Adafruit
     * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
     * under full load that started with a charge of 3.96 V and
     * dropped about linearly to 3.58 V over 15 hours.  It then
     * dropped rapidly to 3.10 V over one hour, at which point it
     * stopped transmitting.
     *
     * Based on eyeball comparisons we'll say that 15/16 of life
     * goes between 3.95 and 3.55 V, and 1/16 goes between 3.55 V
     * and 3.1 V.
     */

    {10000, 3950},
    {625, 3550},
    {0, 3100},
};

static uint8_t battery_level = 0;

static int divider_setup(void) {
    const struct divider_config *cfg = &divider_config;
    const struct io_channel_config *iocp = &cfg->io_channel;
    const struct gpio_dt_spec *gcp = &cfg->power_gpios;
    struct divider_data *ddp = &divider_data;
    struct adc_sequence *asp = &ddp->adc_seq;
    struct adc_channel_cfg *accp = &ddp->adc_cfg;
    int rc;

    if (!device_is_ready(ddp->adc)) {
        return -ENOENT;
    }

    if (gcp->port) {
        if (!device_is_ready(gcp->port)) {
            return -ENOENT;
        }
        rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) {
            return rc;
        }
    }

    *asp = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &ddp->raw,
        .buffer_size = sizeof(ddp->raw),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    *accp = (struct adc_channel_cfg){
        .gain = BATTERY_ADC_GAIN,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    };

    if (cfg->output_ohm != 0) {
        accp->input_positive =
            SAADC_CH_PSELP_PSELP_AnalogInput0 + iocp->channel;
    } else {
        accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
    }

    asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

    rc = adc_channel_setup(ddp->adc, accp);

    return rc;
}

static bool battery_ok;

static int battery_setup(void) {
    int rc = divider_setup();

    battery_ok = (rc == 0);
    return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_measure_enable(bool enable) {
    int rc = -ENOENT;

    if (battery_ok) {
        const struct gpio_dt_spec *gcp = &divider_config.power_gpios;

        rc = 0;
        if (gcp->port) {
            rc = gpio_pin_set_dt(gcp, enable);
        }
    }
    return rc;
}

int battery_sample(void) {
    int rc = -ENOENT;

    if (battery_ok) {
        struct divider_data *ddp = &divider_data;
        const struct divider_config *dcp = &divider_config;
        struct adc_sequence *sp = &ddp->adc_seq;

        rc = adc_read(ddp->adc, sp);
        sp->calibrate = false;
        if (rc == 0) {
            int32_t val = ddp->raw;

            adc_raw_to_millivolts(adc_ref_internal(ddp->adc), ddp->adc_cfg.gain,
                                  sp->resolution, &val);

            if (dcp->output_ohm != 0) {
                rc = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
            } else {
                rc = val;
            }
        }
    }

    return rc;
}

unsigned int battery_level_pct(unsigned int batt_mV,
                               const struct battery_level_point *curve) {
    const struct battery_level_point *pb = curve;

    if (batt_mV >= pb->lvl_mV) {
        /* Measured voltage above highest point, cap at maximum. */
        return pb->lvl_pptt;
    }
    /* Go down to the last point at or below the measured voltage. */
    while ((pb->lvl_pptt > 0) && (batt_mV < pb->lvl_mV)) {
        ++pb;
    }
    if (batt_mV < pb->lvl_mV) {
        /* Below lowest point, cap at minimum */
        return pb->lvl_pptt;
    }

    /* Linear interpolation between below and above points. */
    const struct battery_level_point *pa = pb - 1;

    return pb->lvl_pptt + ((pa->lvl_pptt - pb->lvl_pptt) *
                           (batt_mV - pb->lvl_mV) / (pa->lvl_mV - pb->lvl_mV));
}

uint8_t battery_percentage(void) { return battery_level; }

#if BATTERY_ENABLE_TASK == 1

void battery_loop(void *, void *, void *) {
    while (1) {
        battery_measure_enable(true);
        k_msleep(5000);

        volatile int battMV = battery_sample();
        volatile float battPct =
            ((float)battery_level_pct(battMV, levels)) / 100.0f;
        LOG_INF("battery voltage: %d mV, remaining charge: %f%%", battMV,
                battPct);

        battery_level = (uint8_t)(battPct);

        battery_measure_enable(false);
        k_msleep(MS_BETWEEN_BATTERY_READINGS);
    }
}

K_THREAD_DEFINE(battery_thread_id, 1024, battery_loop, NULL, NULL, NULL, 7,
                K_USER, 0);

#endif // BATTERY_ENABLE_TASK == 1