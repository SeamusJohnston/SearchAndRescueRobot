#ifndef FILTERS_HPP
#define FILTERS_HPP

class LowPassFilter
{
public:
    LowPassFilter();
    LowPassFilter(float f, float ts);
    float update(float u);
    void setFrequency(float f);
    void setSamplingTime(float ts);

private:
    float a;
    float dt;
    float y_prev;
};

class ComplementaryFilter
{
public:
    ComplementaryFilter(float f, float ts);
    float update(float u_high, float u_low);
    void setFrequency(float f);
    void setSamplingTime(float ts);

private:
    float a;
    float dt;
    LowPassFilter lowPass;
    LowPassFilter highPass;

};

#endif