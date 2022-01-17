#ifndef NMEA_H_
#define NMEA_H_

#include <cstdio>
#include <cstdint>
#include <cstdarg>

#define PURGE(x) \
    if(x) {free(x) ; x = nullptr;}


class NMEA {
private:
    char* _raw;

    char* _cmd;

    float* _values;

    int _argc;

    uint8_t _checksum{};

    uint8_t _crc{};

    bool _valid;

    void _clean_up();

public:
    NMEA();
    
    explicit NMEA(const char* msg);
    
    ~NMEA();

    static bool crc(uint8_t &crc, const uint8_t *buf, size_t size);

    void parse();

    void parse(const char *msg);
    
    void construct(const char* cmd, float* values, size_t size);
    
    void construct(const char* format, ...);

    char* get_raw();
    
    int get_argc() const;
    
    bool get_valid() const;
    
    char* get_cmd();

    float* get_values();

    void debug();

};

#endif
