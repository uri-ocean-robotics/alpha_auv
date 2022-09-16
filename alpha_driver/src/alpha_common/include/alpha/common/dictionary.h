/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#pragma once
#ifndef ALPHA_COMMON_DICTIONARY_H
#define ALPHA_COMMON_DICTIONARY_H


#include "types.h"

STATIC_STRING NMEA_PWM_CMD = "PWMC";
STATIC_STRING NMEA_PWM_INITIALIZE = "PWMI";
STATIC_STRING NMEA_PWM_REPORT = "PWMR";
STATIC_STRING NMEA_BAROMETER_REPORT = "BARR";
STATIC_STRING NMEA_MULTIMETER_REPORT = "MULR";
STATIC_STRING NMEA_MULTIMETER_ALRT_REPORT = "MLAR";
STATIC_STRING NMEA_BOOTSEL_CMD = "BTSC";
STATIC_STRING NMEA_SAFETY_CMD = "SAFC";
STATIC_STRING NMEA_SAFETY_REPORT = "SAFR";
STATIC_STRING NMEA_NACK = "NACK";
STATIC_STRING NMEA_ACK = "ACK";
STATIC_STRING NMEA_STROBE_CMD = "STBC";
STATIC_STRING NMEA_STROBE_REPORT = "STBR";
STATIC_STRING NMEA_SERIAL0_CMD = "SER0";
STATIC_STRING NMEA_SERIAL1_CMD = "SER1";
STATIC_STRING NMEA_SERIAL0_REPORT = "SER0";
STATIC_STRING NMEA_SERIAL1_REPORT = "SER1";

STATIC_STRING NMEA_FORMAT_PWM_CMD = "%s,%d,%.5f";
STATIC_STRING NMEA_FORMAT_PWM_REPORT = "%s,%d,%.4f,%d,%d"; // channel,signal,mode,enabled
STATIC_STRING NMEA_FORMAT_BAROMETER_REPORT = "%s,%.5f,%.5f,%.5f";
STATIC_STRING NMEA_FORMAT_MULTIMETER_REPORT = "%s,%.5f,%.2f,%.1f";
STATIC_STRING NMEA_FORMAT_SAFETY_CMD = "%s,%d";
STATIC_STRING NMEA_FORMAT_SAFETY_REPORT = "%s,%d";
STATIC_STRING NMEA_FORMAT_STROBE_CMD = "%s,%d";
STATIC_STRING NMEA_FORMAT_STROBE_REPORT = "%s,%d";
STATIC_STRING NMEA_FORMAT_PWM_INIT = "%s,%d,%d";
STATIC_STRING NMEA_FORMAT_NACK = "%s,%s";
STATIC_STRING NMEA_FORMAT_ACK = "%s,%s";


#define SAFETY_MAX_DURATION_UNDERWATER_MS (10 * 60 * 1000)
#define SAFETY_MAX_DURATION_DEADMAN_VOLTAGE_MS (15 * 1000)
#define SAFETY_DEADMAN_VOLTAGE 11

#endif