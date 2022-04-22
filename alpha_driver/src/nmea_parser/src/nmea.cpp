#include "cstdio"
#include "cstdlib"
#include "cstring"
#include "nmea/nmea.h"

NMEA::NMEA() :
    // _raw(nullptr),
    _values(nullptr),
    _cmd(nullptr),
    _argc(0),
    _valid(false)
{

}

NMEA::NMEA(const char* msg) :
    // _raw(nullptr),
    _values(nullptr),
    _cmd(nullptr),
    _argc(0),
    _valid(false)
{
    // _raw = (char*) malloc(strlen(msg));
    _argc = 0;
    strcpy(_raw, msg);
}

NMEA::~NMEA() {
    _clean_up();
}

bool NMEA::crc(uint8_t &crc, const uint8_t *buf, size_t size) {
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
    _cmd = (char*)malloc(offset * sizeof(char));

    sscanf(cursor, "$%[^,]s%n", _cmd, &offset);

    _argc = 0;
    for(int i = 0 ; i < strlen(_raw) ; i++) {
        if(_raw[i] == ',') {
            _argc++;
        }
    }

    if(_argc > 0) {
        _values = (float *) malloc(_argc * sizeof(float));

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
    // _raw = (char *)malloc(sizeof(char) * strlen(msg));
    strcpy(_raw, msg);
    parse();
}

void NMEA::_clean_up() {
    // PURGE(_raw)
    PURGE(_values)
    PURGE(_cmd)
    PURGE(_values)
    _argc = 0;
    _valid = false;
}

void NMEA::construct(const char *cmd, float* values, size_t size) {
    char* m = (char*)malloc(BUFSIZ * sizeof(char));
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
    free(m);
}

void NMEA::construct(const char *format, ...) {
    va_list args;
    va_start( args, format );
    char* m = (char*)malloc(BUFSIZ * sizeof(char));
    // _raw = (char*)malloc(BUFSIZ * sizeof(char));
    vsprintf(m, format, args);
    crc(_crc, (uint8_t*)m, strlen(m));
    sprintf(_raw,"$%s*%02X", m, _crc);
    free(m);
    va_end( args );
}


char* NMEA::get_raw() {
    return _raw;
}

int NMEA::get_argc() const {
    return _argc;
}

bool NMEA::get_valid() const {
    return _valid;
}

char* NMEA::get_cmd() {
    return _cmd;
}

float* NMEA::get_values() {
    return _values;
}