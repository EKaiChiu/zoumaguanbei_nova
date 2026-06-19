#ifndef FUZZY_HPP
#define FUZZY_HPP

#include "image.hpp"
#include "motor.hpp"
#include "zf_common_headfile.hpp"



struct {
    float P;
    float I;
    float D;
    float LastError;
    float Integral;
    float Derivative;
    float Output;
} SteerPIDdata;


void Data_Settings(void);           //参数赋值

#endif
