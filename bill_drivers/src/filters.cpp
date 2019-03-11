#include "bill_drivers/filters.hpp"

LowPassFilter::LowPassFilter()
{
    a = 1;
    dt = 0;
    y_prev = 0;
}

LowPassFilter::LowPassFilter(float f, float ts)
{
    dt = ts;
    a = dt/((1.0/f) + dt);
    y_prev = 0;
}

float LowPassFilter::update(float u)
{
    y_prev = (1-a)*y_prev + a*u;
    return y_prev;
}

void LowPassFilter::setFrequency(float f)
{
    a = dt/((1.0/f) + dt);
}

void LowPassFilter::setSamplingTime(float ts)
{
    dt = ts;
    a = dt/((1.0/f) + dt);
}

ComplementaryFilter::ComplementaryFilter(float f, float ts)
{
    setSamplingTime(ts);
    setFrequency(f);
}

float ComplementaryFilter::update(float u_high, float u_low)
{
    return lowPass.update(u_low) + (u_high - highPass.update(u_high));
}

void ComplementaryFilter::setFrequency(float f)
{
    highPass.setFrequency(f);
    lowPass.setFrequency(f);
}

void ComplementaryFilter::setSamplingTime(float ts)
{
    highPass.setSamplingTime(ts);
    lowPass.setSampingTime(ts);
}