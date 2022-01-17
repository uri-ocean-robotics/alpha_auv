/*

#ifndef NMEA_H_
#define NMEA_H_

#include "cstdint"
#include "cstdarg"
#include "cstdio"
#include "cstdlib"
#include "cstring"


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

    bool crc(uint8_t &crc, uint8_t *buf, size_t size);

    void parse();

    void parse(const char *msg);

    void construct(const char* cmd, float* values, size_t size);

    void construct(const char* format, ...);

    char* get_raw();

    int get_argc();

    bool get_valid();

    char* get_cmd();

    float* get_values();
};


NMEA::NMEA()
{
    _argc = 0;
    _valid = false;
    memset((void*)_raw, 0x00, BUFSIZ);
    memset((void*)_cmd, 0x00, BUFSIZ);
    memset((void*)_values, 0x00, BUFSIZ);
    _checksum = 0x00;
}

NMEA::NMEA(const char* msg) :
    _argc(0),
    _valid(false)
{
    _argc = 0;
    strcpy(_raw, msg);
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

    sscanf(cursor, "$%[^,]s%n", _cmd, &offset);
    _argc = 0;
    for(int i = 0 ; i < strlen(_raw) ; i++) {
        if(_raw[i] == ',') {
            _argc++;
        }
    }

    if(_argc > 0) {
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

    char* m = (char *)malloc( BUFSIZ * sizeof(char));
    sscanf(_raw, "$%[^*]s*", m);
    _valid = crc(_checksum, (uint8_t*)m, strlen(m));
    free(m);
}

void NMEA::parse(const char *msg) {
    _clean_up();
    strcpy(_raw, msg);
    parse();
}

void NMEA::_clean_up() {
    memset((void*)_raw, 0x00, BUFSIZ);
    memset((void*)_values, 0x00, BUFSIZ);
    memset((void*)_cmd, 0x00, BUFSIZ);
    _argc = 0;
    _valid = false;
}

void NMEA::construct(const char *format, ...) {
    va_list args;
    va_start( args, format );
    char* m = new char[BUFSIZ];
    vsprintf(m, format, args);
    crc(_crc, (uint8_t*)m, strlen(m));
    sprintf(_raw,"$%s*%02X", m, _crc);
    va_end( args );
    delete m;
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

#endif
*/