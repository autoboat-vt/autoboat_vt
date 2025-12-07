#pragma once


class DiscretePID {
public:
    DiscretePID(double sample_period_, double Kp_, double Ki_, double Kd_, double n_) {
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


    void set_gains(double Kp = -1.0, double Ki = -1.0, double Kd = -1.0, double n = -1.0, double sample_period = -1.0) {
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

    double step(double error) {
        double a0 = 1 + n * sample_period;
        double a1 = -(2 + n * sample_period);
        double a2 = 1;

        double b0 = Kp * (1 + n * sample_period) + Ki * sample_period * (1 + n * sample_period) + Kd * n;

        double b1 = -(Kp * (2 + n * sample_period) + Ki * sample_period + 2 * Kd * n);

        double b2 = Kp + Kd * n;

        double output = -(a1 / a0) * prev_output1 - (a2 / a0) * prev_output2 + (b0 / a0) * error + (b1 / a0) * prev_error1 + (b2 / a0) * prev_error2;

        prev_output2 = prev_output1;
        prev_output1 = output;
        prev_error2 = prev_error1;
        prev_error1 = error;

        return output;
    }


    inline double operator()(double error) { return step(error); }



private:
    double sample_period;
    double Kp, Ki, Kd, n;


    double prev_output1;
    double prev_output2;


    double prev_error1;
    double prev_error2;
};