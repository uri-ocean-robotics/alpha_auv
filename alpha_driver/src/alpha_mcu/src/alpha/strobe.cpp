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

#include "alpha/common/dictionary.h"
#include "strobe.h"
#include "iostream"


Strobe::Strobe() {

}

bool Strobe::f_flash(struct repeating_timer *t) {
    Strobe* _this = (Strobe*)t->user_data;




    return true;
}

bool Strobe::f_reporter(struct repeating_timer* t) {
    Strobe* _this = (Strobe*)t->user_data;

    NMEA* msg = new NMEA();
    msg->construct(NMEA_FORMAT_STROBE_REPORT, NMEA_STROBE_REPORT, _this->is_activated());
    std::cout << msg->get_raw() << std::endl;
    delete msg;

    return true;

}

void Strobe::initialize() {
    gpio_init(STROBE_PIN);
    gpio_set_dir(STROBE_PIN, GPIO_OUT);

    // add_repeating_timer_ms(REPORT_STROBE_PERIOD, Strobe::f_flash, this, &m_strobe_flashing_timer);

    add_repeating_timer_ms(REPORT_STROBE_PERIOD, Strobe::f_reporter, this, &m_reporter_timer);

    enable();
}

void Strobe::enable() const {
    gpio_put(STROBE_PIN, 1);
}

void Strobe::disable() const {
    gpio_put(STROBE_PIN, 0);
}

bool Strobe::is_activated() {
    return gpio_get(STROBE_PIN);
}
