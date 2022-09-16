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

#include "cstdio"
#include "cstdlib"
#include "cstring"
#include "nmea.h"

NMEA::NMEA() :
    _argc(0),
    _valid(false)
{

}

NMEA::NMEA(const char* msg) :
    _argc(0),
    _valid(false)
{

    // _raw = (char*) malloc(strlen(msg));
    _argc = 0;
    strcpy(_raw, msg);
}

NMEA::~NMEA() {
    // _clean_up();
}

bool NMEA::crc(uint8_t &crc, uint8_t *buf, size_t size) {
    //NMNA0183
    uint8_t in = crc;
    crc=0;
    for (int i = 0; i < size; i ++) {
        crc ^= buf[i];
    }

    return crc == in;
}

void NMEA::parse() {
    int offset = 0;
    char* cursor = _raw;
    sscanf(cursor, "$%*[^,]%n", &offset);
    // _cmd = (char*)malloc(offset * sizeof(char));

    sscanf(cursor, "$%[^,]s%n", _cmd, &offset);

    _argc = 0;
    for(int i = 0 ; i < strlen(_raw) ; i++) {
        if(_raw[i] == ',') {
            _argc++;
        }
    }

    if(_argc > 0) {
        // _values = (float *) malloc(_argc * sizeof(float));

        float value;
        cursor += offset;
        for (int i = 0; i < _argc && sscanf(cursor, ",%f%n", &value, &offset); i++) {
            cursor += offset;
            _values[i] = value;
        }
    }

    for(offset = 0; offset < strlen(cursor) && cursor[offset] != '*' ; offset++);
    cursor = cursor + offset + 1;

    sscanf(cursor, "%02X", (unsigned int*)&_checksum);

    char m[BUFSIZ];
    // char* m = (char *)malloc( BUFSIZ * sizeof(char));
    sscanf(_raw, "$%[^*]s*", m);
    _valid = crc(_checksum, (uint8_t*)m, strlen(m));
    // free(m);
}

void NMEA::parse(std::string msg) {
    // _clean_up();
    // _raw = (char *)malloc(sizeof(char) * msg.size());
    strcpy(_raw, msg.c_str());
    parse();
}

void NMEA::parse(const char *msg) {
    // _clean_up();
    strcpy(_raw, msg);
    parse();
}

void NMEA::_clean_up() {
    //PURGE(_raw)
    //PURGE(_values)
    //PURGE(_cmd)
    _argc = 0;
    _valid = false;
}

void NMEA::debug() {
    printf("[DEBUG][%d] Raw: %s,",_valid, _cmd);
    int i = 0;
    for(; i < _argc - 1 ; i++) {
        printf("%.3f,", _values[i]);
    }
    printf("%.3f", _values[i++]);
    printf("*%02X\r\n", _checksum);
}

void NMEA::construct(const char *cmd, float* values, size_t size) {
    char m[BUFSIZ];
    // char* m = (char*)malloc(BUFSIZ * sizeof(char));
    // _raw = (char*)malloc(BUFSIZ * sizeof(char));
    char* cursor = m;
    cursor += sprintf(cursor, "%s,", cmd);
    int i = 0;
    for(i = 0 ; i < size - 1 ; i++) {
        cursor += sprintf(cursor, "%.3f,", values[i]);
    }
    sprintf(cursor, "%.3f", values[i+1]);

    crc(_crc, (uint8_t*)m, strlen(m));
    sprintf(_raw,"$%s*%02X", m, _crc);
}

void NMEA::construct(const char *format, ...) {
    va_list args;
    va_start( args, format );
    char m[BUFSIZ];
    // char* m = (char*)malloc(BUFSIZ * sizeof(char));
    // _raw = (char*)malloc(BUFSIZ * sizeof(char));
    vsprintf(m, format, args);
    crc(_crc, (uint8_t*)m, strlen(m));
    sprintf(_raw,"$%s*%02X", m, _crc);
    va_end( args );
}


char* NMEA::get_raw() {
    return _raw;
}

int NMEA::get_argc() {
    return _argc;
}

bool NMEA::get_valid() {
    return _valid;
}

char* NMEA::get_cmd() {
    return _cmd;
}

float* NMEA::get_values() {
    return _values;
}