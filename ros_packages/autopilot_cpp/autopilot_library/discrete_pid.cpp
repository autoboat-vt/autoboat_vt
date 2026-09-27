#include "discrete_pid.hpp"


DiscretePID::DiscretePID() {
    *this = DiscretePID(1.0, 0.0, 0.0, 0.0, 1.0);
}


DiscretePID::DiscretePID(float sample_period_, float Kp_, float Ki_, float Kd_, float n_) {
    sample_period = sample_period_;
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    n = n_;
    prev_output1 = 0.0;
    prev_output2 = 0.0;
    prev_error1 = 0.0;
    prev_error2 = 0.0;
}


void DiscretePID::set_gains(float sample_period_, float Kp_, float Ki_, float Kd_, float n_) {
    if (Kp_ >= 0.0) this->Kp = Kp_;
    if (Ki_ >= 0.0) this->Ki = Ki_;
    if (Kd_ >= 0.0) this->Kd = Kd_;
    if (n_  >= 0.0) this->n  = n_;

    if (sample_period_ >= 0.0f) this->sample_period = sample_period_;
}


void DiscretePID::reset() {
    prev_output1 = 0.0;
    prev_output2 = 0.0;
    prev_error1  = 0.0;
    prev_error2  = 0.0;
}


float DiscretePID::step(float error) {
    float a0 = 1 + n * sample_period;
    float a1 = -(2 + n * sample_period);
    float a2 = 1;

    float b0 = Kp * (1 + n * sample_period) + Ki * sample_period * (1 + n * sample_period) + Kd * n;

    float b1 = -(Kp * (2 + n * sample_period) + Ki * sample_period + 2 * Kd * n);

    float b2 = Kp + Kd * n;

    float output = -(a1 / a0) * prev_output1 - (a2 / a0) * prev_output2 + (b0 / a0) * error + (b1 / a0) * prev_error1 + (b2 / a0) * prev_error2;

    prev_output2 = prev_output1;
    prev_output1 = output;
    prev_error2 = prev_error1;
    prev_error1 = error;

    return output;
}


float DiscretePID::operator()(float error) {
    return step(error);
}
