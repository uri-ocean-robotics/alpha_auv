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
