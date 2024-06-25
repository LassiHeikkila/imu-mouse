/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Extended by Lassi Heikkilä

#ifndef APPLICATION_BATTERY_H_
#define APPLICATION_BATTERY_H_

// Defines

// Measure battery level every 10 minutes
#define MS_BETWEEN_BATTERY_READINGS (10 * 60 * 1000)

#define BATTERY_ENABLE_TASK 0

/** Enable or disable measurement of the battery voltage.
 *
 * @param enable true to enable, false to disable
 *
 * @return zero on success, or a negative error code.
 */
int battery_measure_enable(bool enable);

/** Measure the battery voltage.
 *
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
int battery_sample(void);

/** Return latest battery percentage value
 *
 * @return the latest estimated battery percentage
 */
uint8_t battery_percentage(void);

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point
{
    /** Remaining life at #lvl_mV. */
    uint16_t lvl_pptt;

    /** Battery voltage at #lvl_pptt remaining life. */
    uint16_t lvl_mV;
};

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 *
 * @return the estimated remaining capacity in parts per ten
 * thousand.
 */
unsigned int battery_level_pct(unsigned int batt_mV,
                               const struct battery_level_point *curve);

#endif /* APPLICATION_BATTERY_H_ */