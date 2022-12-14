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

#ifndef ALPHA_SIMULATOR_UTILITIES_HXX
#define ALPHA_SIMULATOR_UTILITIES_HXX

#include "Eigen/Core"
#include "Eigen/Geometry"


Eigen::Matrix3d rotx(double deg) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(deg, Eigen::Vector3d::UnitX());
    return m;
}

Eigen::Matrix3d roty(double deg) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(deg, Eigen::Vector3d::UnitY());
    return m;
}

Eigen::Matrix3d rotz(double deg) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(deg, Eigen::Vector3d::UnitZ());
    return m;
}

Eigen::Quaterniond rot(double roll, double pitch, double yaw) {
    Eigen::Quaterniond m
        = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return m;
}

double pwm2thrust(double pwm) {
    double pwm_l = 1100;
    double pwm_r = 1900;

    if (pwm > pwm_r) {
        pwm = pwm_r;
    } else if (pwm < pwm_l) {
        pwm = pwm_l;
    }


    double p1 =   -4.08e-12  ;
    double p2 =   5.154e-08  ;
    double p3 =  -0.0001736 ;
    double p4 =      0.2326  ;
    double p5 =      -111.6 ;
    double T = (p1 * std::pow(pwm, 4) + p2* std::pow(pwm, 3) + p3 * std::pow(pwm, 2) + p4 * pwm + p5) * 9.81;

    if(pwm >1465 & pwm <1535) {
        T = 0;
    }

    return T;
}

double wrapTo2Pi(double angle) {
    angle -= M_PI * std::floor(angle * (1. / M_PI));
    return angle;
}

#endif //ALPHA_SIMULATOR_UTILITIES_HXX
