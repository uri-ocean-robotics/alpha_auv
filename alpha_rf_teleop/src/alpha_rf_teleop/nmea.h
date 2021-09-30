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
