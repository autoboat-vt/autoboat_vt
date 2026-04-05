#pragma once


/**
 * @class DiscretePID
 * @brief Modified discrete PID controller using a low pass filter for the derivative term.
 * 
 * Reference: https://www.scilab.org/discrete-time-pid-controller-implementation
 */
class DiscretePID {
public:

    /**
     * @brief Construct a new Discrete PID object (default).
     */
    DiscretePID();

    /**
     * @brief Construct a new Discrete PID object with gains.
     * @param sample_period_ Sample period for the discrete PID controller.
     * @param Kp_ Proportional gain.
     * @param Ki_ Integral gain.
     * @param Kd_ Derivative gain.
     * @param n_ Cutoff frequency for the low pass filter on the derivative term.
     */
    DiscretePID(float sample_period_, float Kp_, float Ki_, float Kd_, float n_);

    /**
     * @brief Sets gains to the specified values.
     * 
     * If an argument is negative, the existing value is retained.
     */
    void set_gains(float sample_period_ = -1.0, float Kp_ = -1.0, float Ki_ = -1.0, float Kd_ = -1.0, float n_ = -1.0);

    /**
     * @brief Resets the PID controller to its initial state.
     */
    void reset();

    /**
     * @brief Steps the PID controller with the given error and returns the output.
     * @param error The current error value.
     * @return float The output of the PID controller.
     */
    float step(float error);

    /**
     * @brief Operator overload to call the step method.
     * @param error The current error value.
     * @return float The output of the PID controller.
     */
    float operator()(float error);


private:
    float sample_period;
    float Kp, Ki, Kd, n;

    float prev_output1;
    float prev_output2;

    float prev_error1;
    float prev_error2;
};