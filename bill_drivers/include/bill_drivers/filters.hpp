#ifndef FILTERS_HPP
#define FILTERS_HPP

class LowPassFilter
{
public:
    LowPassFilter();
    LowPassFilter(float freq);
    float update(float u);
    void setFrequency(float freq);
    void setSamplingTime(float ts);
    void setPrevOutput(float y);

private:
    float _a;
    float _f;
    float _dt;
    float _y_prev;
};

class ComplementaryFilter
{
public:
    ComplementaryFilter(float freq);
    float update(float u_high, float u_low);
    void setFrequency(float freq);
    void setSamplingTime(float ts);
    void setPrevOutput(float y_high, float y_low);

private:
    LowPassFilter _low_pass;
    LowPassFilter _high_pass;
};

#endif