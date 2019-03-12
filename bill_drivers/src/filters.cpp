#include "bill_drivers/filters.hpp"

LowPassFilter::LowPassFilter()
{
    a = 1;
    dt = 0;
    f = 0;
    y_prev = 0;
}

LowPassFilter::LowPassFilter(float freq, float ts)
{
    dt = ts;
    f = freq;
    a = dt/((1.0/f) + dt);
    y_prev = 0;
}

float LowPassFilter::update(float u)
{
    y_prev = (1-a)*y_prev + a*u;
    return y_prev;
}

void LowPassFilter::setFrequency(float freq)
{
    f = freq;
    a = dt/((1.0/f) + dt);
}

void LowPassFilter::setSamplingTime(float ts)
{
    dt = ts;
    a = dt/((1.0/f) + dt);
}

ComplementaryFilter::ComplementaryFilter(float freq, float ts)
{
    setSamplingTime(ts);
    setFrequency(freq);
}

float ComplementaryFilter::update(float u_high, float u_low)
{
    return lowPass.update(u_low) + (u_high - highPass.update(u_high));
}

void ComplementaryFilter::setFrequency(float freq)
{
    highPass.setFrequency(freq);
    lowPass.setFrequency(freq);
}

void ComplementaryFilter::setSamplingTime(float ts)
{
    highPass.setSamplingTime(ts);
    lowPass.setSamplingTime(ts);
}