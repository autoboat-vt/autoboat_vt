#pragma once


class DiscretePID {
public:

    DiscretePID() {
        *this = DiscretePID(1.0, 0.0, 0.0, 0.0, 1.0);
    }


    DiscretePID(float sample_period_, float Kp_, float Ki_, float Kd_, float n_) {
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


    void set_gains(float Kp = -1.0, float Ki = -1.0, float Kd = -1.0, float n = -1.0, float sample_period = -1.0) {
        if (Kp >= 0.0) this->Kp = Kp;
        if (Ki >= 0.0) this->Ki = Ki;
        if (Kd >= 0.0) this->Kd = Kd;
        if (n  >= 0.0) this->n  = n;
        if (sample_period >= 0.0) this->sample_period = sample_period;
    }

    
    void reset() {
        prev_output1 = 0.0;
        prev_output2 = 0.0;
        prev_error1  = 0.0;
        prev_error2  = 0.0;
    }


    float step(float error) {
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


    inline float operator()(float error) { return step(error); }



private:
    float sample_period;
    float Kp, Ki, Kd, n;


    float prev_output1;
    float prev_output2;


    float prev_error1;
    float prev_error2;
};