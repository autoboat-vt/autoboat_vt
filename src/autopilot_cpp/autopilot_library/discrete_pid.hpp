#pragma once


class DiscretePID {
public:

    DiscretePID();
    DiscretePID(float sample_period_, float Kp_, float Ki_, float Kd_, float n_);

    void set_gains(float sample_period_ = -1.0, float Kp_ = -1.0, float Ki_ = -1.0, float Kd_ = -1.0, float n_ = -1.0);
    void reset();
    float step(float error);
    float operator()(float error);


private:
    float sample_period;
    float Kp, Ki, Kd, n;

    float prev_output1;
    float prev_output2;

    float prev_error1;
    float prev_error2;
};