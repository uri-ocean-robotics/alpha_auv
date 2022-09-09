#include <iostream>
#include "cstdio"
#include "cstdlib"
#include "cstring"
#include "cctype"
#include "nmea/nmea.h"

NMEA::NMEA() :
    m_argc(0),
    m_valid(false)
{
    memset(m_raw, 0x00, BUFSIZ);
    memset(m_data, 0x00, BUFSIZ);
    memset(m_cmd, 0x00, BUFSIZ);
}

NMEA::NMEA(const char* msg) :
    m_argc(0),
    m_valid(false)
{
    memset(m_raw, 0x00, BUFSIZ);
    memset(m_data, 0x00, BUFSIZ);
    memset(m_cmd, 0x00, BUFSIZ);
    m_argc = 0;
    strcpy(m_raw, msg);
}

NMEA::~NMEA() {

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

    auto l = strlen(m_raw);
    memset(m_data, 0, sizeof m_data);

    uint8_t check = 0;
    char checksum[2];
    bool read_cmd = true;

    if(l == 0) {
        // message body is empty
        m_valid = false;
        return;
    }

    if(m_raw[0] != '$') {
        // if message is not starting with $ is not valid
        m_valid = false;
        return;
    }

    for(size_t i = l ; i > 0 ; i--){
        if(std::isxdigit(m_raw[i])) {
            l = i;
            break;
        }
        if(i == 1) {
            m_valid = false;
            return;
        }
    }



    if(m_raw[l - 2] != '*') {
        // doesn't have checksum indicator
        m_valid = false;
        return;
    }

    // check the last two chars of the message
    if(!std::isxdigit(m_raw[l]) || !std::isxdigit(m_raw[l - 1])) {
        // checksum is broken
        m_valid = false;
        return;
    }

    // get cmd
    m_argc = 0;
    for(int i = 0; i < l ; i++) {
        char c = m_raw[i + 1];

        if(c == ',') {
            // After first ',' we don't read command anymore
            read_cmd = false;

            // increment argc by one
            m_argc++;
        }

        if(read_cmd) {
            // construct the command
            m_cmd[i] = c;
        }
    }

    // read the checksum
    checksum[0] = m_raw[l - 1];
    checksum[1] = m_raw[l];
    sscanf(checksum, "%02X", (unsigned int*)&m_checksum);

    // cut the $ and *XX from beginning and the end
    for (int i = 1; i < l - 2; i ++) {
        check ^= m_raw[i];
    }
    m_valid = m_checksum == check;
    memcpy(m_data, m_raw + 1, sizeof(char) * (l - 3));
}

void NMEA::parse(const char *msg) {
    memset(m_raw, 0, BUFSIZ);
    memset(m_cmd, 0, BUFSIZ);
    memset(m_data, 0, BUFSIZ);
    strcpy(m_raw, msg);
    parse();
}

void NMEA::construct(const char *format, ...) {
    va_list args;
    va_start( args, format );
    char* m = (char*)malloc(BUFSIZ * sizeof(char));
    vsprintf(m, format, args);
    crc(m_crc, (uint8_t*)m, strlen(m));
    sprintf(m_raw, "$%s*%02X", m, m_crc);
    free(m);
    va_end( args );
}


char* NMEA::get_raw() {
    return m_raw;
}

int NMEA::get_argc() const {
    return m_argc;
}

bool NMEA::get_valid() const {
    return m_valid;
}

char* NMEA::get_cmd() {
    return m_cmd;
}

char* NMEA::get_data() {
    return m_data;
}
