#ifndef ALPHA_PARSER_H
#define ALPHA_PARSER_H

#include "cstdio"
#include "alpha/common.h"
#include "alpha/thrusters.h"
#include "alpha/safety.h"
#include "pico/stdlib.h"
#include "iostream"
#include "cstring"
#include "pico/bootrom.h"

#include "nmea/nmea.h"
#include "alpha_common/dictionary.h"

void listen_incoming_messages();

#endif