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

#include <cstdio>
#include <cstdint>
#include <cstdarg>

#define PURGE(x) vals\
    if(x) {free(x) ; x = nullptr;}


class NMEA {
private:
    char m_raw[BUFSIZ];

    char m_data[BUFSIZ];

    char m_cmd[BUFSIZ];

    int m_argc;

    uint8_t m_checksum{};

    uint8_t m_crc{};

    bool m_valid;

public:
    NMEA();

    explicit NMEA(const char* msg);

    ~NMEA();

    static bool crc(uint8_t &crc, const uint8_t *buf, size_t size);

    void parse();

    void parse(const char *msg);

    void construct(const char* format, ...);

    char* get_raw();

    [[nodiscard]] int get_argc() const;

    [[nodiscard]] bool get_valid() const;

    char* get_cmd();

    char* get_data();

};

#endif
