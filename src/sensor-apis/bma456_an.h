/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bma456_an.h
 * @date       2023-07-05
 * @version    V2.29.0
 *
 */

/**
 * \ingroup bma4xy
 * \defgroup bma456_an BMA456_AN
 * @brief Sensor driver for BMA456_AN sensor
 */

#ifndef BMA456_AN_H
#define BMA456_AN_H

#ifdef __cplusplus
extern "C" {
#endif

/**\name Chip ID of BMA456_AN sensor */
#define BMA456_AN_CHIP_ID_PRIM UINT8_C(0x16)
#define BMA456_AN_CHIP_ID_SEC UINT8_C(0x1A)

/**\name Sensor feature size */
#define BMA456_AN_FEATURE_SIZE UINT8_C(12)
#define BMA456_AN_ANY_MOT_LEN UINT8_C(4)

/**\name Feature offset address */
#define BMA456_AN_ANY_MOT_OFFSET UINT8_C(0x00)
#define BMA456_AN_NO_MOT_OFFSET UINT8_C(0x04)
#define BMA456_AN_CONFIG_ID_OFFSET UINT8_C(0x08)
#define BMA456_AN_AXES_REMAP_OFFSET UINT8_C(0x0A)

/**\name Read/Write Lengths */
#define BMA456_AN_RD_WR_MIN_LEN UINT8_C(2)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456_AN_RD_WR_MAX_LEN ((uint16_t)sizeof(bma456_an_config_file))

#define BMA456_AN_NO_MOT_RD_WR_LEN                                             \
    (BMA456_AN_ANY_MOT_LEN + BMA456_AN_NO_MOT_OFFSET)

/**************************************************************/
/**\name    Re-map Axes */
/**************************************************************/
#define BMA456_AN_X_AXIS_MASK UINT8_C(0x03)
#define BMA456_AN_X_AXIS_SIGN_MASK UINT8_C(0x04)
#define BMA456_AN_Y_AXIS_MASK UINT8_C(0x18)
#define BMA456_AN_Y_AXIS_SIGN_MASK UINT8_C(0x20)
#define BMA456_AN_Z_AXIS_MASK UINT8_C(0xC0)
#define BMA456_AN_Z_AXIS_SIGN_MASK UINT8_C(0x01)

/**************************************************************/
/**\name    Any/no Motion */
/**************************************************************/
/**\name Any/No motion threshold macros */
#define BMA456_AN_ANY_NO_MOT_THRES_MSK UINT16_C(0x07FF)

/**\name Any/No motion duration macros */
#define BMA456_AN_ANY_NO_MOT_DUR_MSK UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA456_AN_ANY_NO_MOT_AXIS_EN_POS UINT8_C(0x0D)
#define BMA456_AN_ANY_NO_MOT_AXIS_EN_MSK UINT16_C(0xE000)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA456_AN_X_AXIS_EN UINT8_C(0x01)
#define BMA456_AN_Y_AXIS_EN UINT8_C(0x02)
#define BMA456_AN_Z_AXIS_EN UINT8_C(0x04)
#define BMA456_AN_EN_ALL_AXIS UINT8_C(0x07)
#define BMA456_AN_DIS_ALL_AXIS UINT8_C(0x00)

/**\name Interrupt status macros */
#define BMA456_AN_ANY_MOT_INT UINT8_C(0x20)
#define BMA456_AN_NO_MOT_INT UINT8_C(0x40)
#define BMA456_AN_ERROR_INT UINT8_C(0x80)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No motion configuration
 */
struct bma456_an_any_no_mot_config
{
    /*! Expressed in 50 Hz samples (20 ms) */
    uint16_t duration;

    /*! Threshold value for Any-motion/No-motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    /*! To enable selected axes */
    uint8_t axes_en;
};

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */
