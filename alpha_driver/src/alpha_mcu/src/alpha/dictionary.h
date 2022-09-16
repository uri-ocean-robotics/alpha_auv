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

#ifndef ALPHA_DICTIONARY_H
#define ALPHA_DICTIONARY_H

#define PULSE_WIDTH_MAX 1900
#define PULSE_WIDTH_MIN 1100
#define PULSE_WIDTH_CTR (PULSE_WIDTH_MAX + PULSE_WIDTH_MIN) / 2.0

#define REPORT_PWM_PERIOD 150

#define REPORT_BAROMETER_PERIOD 200

#define REPORT_MULTIMETER_PERIOD 100
#define MULTIMETER_ALERT_PIN 31

#define ALPHA_I2C_SDA_PIN 16
#define ALPHA_I2C_SCL_PIN 17

#define DROP_WEIGHT_PIN 22
#define REPORT_SAFETY_PERIOD 1000


#define STROBE_PIN 21
#define REPORT_STROBE_PERIOD 1000

#define GLOB_PWM_CHANNEL_0_PIN 2
#define GLOB_PWM_CHANNEL_1_PIN 4
#define GLOB_PWM_CHANNEL_2_PIN 6
#define GLOB_PWM_CHANNEL_3_PIN 10
#define GLOB_PWM_CHANNEL_4_PIN 12

#endif