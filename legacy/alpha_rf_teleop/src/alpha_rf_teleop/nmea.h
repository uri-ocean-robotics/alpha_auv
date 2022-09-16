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

#ifndef NMEA_H_
#define NMEA_H_

#include "cstdio"
#include "cstdint"
#include "cstdarg"
#include "string"

#define PURGE(x) \
    if(x != nullptr) {free(x) ; x = nullptr;}


class NMEA {
private:
    char _raw[BUFSIZ];

    char _cmd[BUFSIZ];

    float _values[BUFSIZ];

    int _argc;

    uint8_t _checksum;

    uint8_t _crc;

    bool _valid;

    void _clean_up();

public:
    NMEA();

    NMEA(const char* msg);

    ~NMEA();

    bool crc(uint8_t &crc, uint8_t *buf, size_t size);

    void parse();

    void parse(const char *msg);

    void parse(std::string msg);

    void construct(const char* cmd, float* values, size_t size);

    void construct(const char* format, ...);

    char* get_raw();

    int get_argc();

    bool get_valid();

    char* get_cmd();

    float* get_values();

    void debug();

};

#endif
