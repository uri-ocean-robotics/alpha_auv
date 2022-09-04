#ifndef ALPHA_COMMON_H
#define ALPHA_COMMON_H

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void initialize_i2c();

#endif