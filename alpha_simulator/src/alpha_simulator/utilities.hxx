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
    double pwm_l = 1300;
    double pwm_r = 1700;

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
