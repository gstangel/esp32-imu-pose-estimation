/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */



enum {
    IDX_SVC,

    IDX_CHAR_IMU_DATA,
    IDX_CHAR_VAL_IMU_DATA, // 26 bytes (Euler: X,Y,Z, Gyro: X,Y,Z, Accel: X,Y,Z, Quat: X,Y,Z,W)
    IDX_CHAR_CFG_IMU_DATA,

    IDX_CHAR_IMU_RATE,
    IDX_CHAR_VAL_IMU_RATE, // 2 bytes for the rate (uint16_t)

    HRS_IDX_NB,
};
