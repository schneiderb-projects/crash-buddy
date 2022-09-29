/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    SERVICE,
    CHAR_STATUS,
    CHAR_STATUS_VALUE,

    CHAR_DATA_AVAILABLE,
    CHAR_DATA_AVAILABLE_VALUE,

    CHAR_DATA_SIZE,
    CHAR_DATA_SIZE_VALUE,

    CHAR_SET_THRESHHOLD,
    CHAR_SET_THRESHHOLD_VALUE,

    CHAR_SET_ENABLE_DEBUG,
    CHAR_SET_ENABLE_DEBUG_VALUE,

    CHAR_CRASH_DATA_CHAR_SIZE,
    CHAR_CRASH_DATA_CHAR_SIZE_VALUE,

    CHAR_DATA_POINTS_INITIAL,
};
