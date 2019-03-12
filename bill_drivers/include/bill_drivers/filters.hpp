#ifndef FILTERS_HPP
#define FILTERS_HPP

class LowPassFilter
{
public:
    LowPassFilter();
    LowPassFilter(float freq, float ts);
    float update(float u);
    void setFrequency(float freq);
    void setSamplingTime(float ts);

private:
    float a;
    float f;
    float dt;
    float y_prev;
};

class ComplementaryFilter
{
public:
    ComplementaryFilter(float freq, float ts);
    float update(float u_high, float u_low);
    void setFrequency(float freq);
    void setSamplingTime(float ts);

private:
    float a;
    float dt;
    LowPassFilter lowPass;
    LowPassFilter highPass;

};

#endif