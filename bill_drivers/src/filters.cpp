#include <iostream>
#include "bill_drivers/filters.hpp"

LowPassFilter::LowPassFilter()
{
    _a = 1;
    _dt = 0;
    _f = 0;
    _y_prev = 0;
}

LowPassFilter::LowPassFilter(float freq)
{
    _dt = 0;
    _f = freq;
    _a = _dt/((1.0/_f) + _dt);
    _y_prev = 0;
}

float LowPassFilter::update(float u)
{
    _y_prev = (1-_a)*_y_prev + _a*u;
    return _y_prev;
}

void LowPassFilter::setFrequency(float freq)
{
    _f = freq;
    _a = _dt/((1.0/_f) + _dt);
}

void LowPassFilter::setSamplingTime(float ts)
{
    _dt = ts;
    _a = _dt/((1.0/_f) + _dt);
}

void LowPassFilter::setPrevOutput(float y)
{
    _y_prev = y;
}

ComplementaryFilter::ComplementaryFilter(float freq)
{
    setFrequency(freq);
}

float ComplementaryFilter::update(float u_high, float u_low)
{
    return _low_pass.update(u_low) + (u_high - _high_pass.update(u_high));
}

void ComplementaryFilter::setFrequency(float freq)
{
    _high_pass.setFrequency(freq);
    _low_pass.setFrequency(freq);
}

void ComplementaryFilter::setSamplingTime(float ts)
{
    _high_pass.setSamplingTime(ts);
    _low_pass.setSamplingTime(ts);
}

void ComplementaryFilter::setPrevOutput(float y_high, float y_low)
{
    _low_pass.setPrevOutput(y_low);
    _high_pass.setPrevOutput(y_high);
}