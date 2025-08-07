class Discrete_PID:
    """
    Main resource: https://www.scilab.org/discrete-time-pid-controller-implementation.
    
    Modified discrete PID controller using a low pass filter only for the derivative term.
    The equations in this file were derived from the z-transform of a PID controller with a low pass filter on the derivative term.
    
    n is used instead of just calling it the low_pass_filter_cutoff_frequency so the equations are more legible but that is all that term is.
    """
    def __init__(self, sample_period, Kp=1, Ki=0, Kd=0, n=0):
        """
        n represents the cutoff frequency for the low pass filter applied to the derivative term of the pid controller
        
        All of the other parameters should be fairly standard for a PID controller
        """
        
        self.sample_period = sample_period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.n = n
        
        self.prev_output1 = 0.
        self.prev_output2 = 0.
        
        self.prev_error1 = 0.
        self.prev_error2 = 0.
    
    
    def set_gains(self, Kp=None, Ki=None, Kd=None, n=None, sample_period=None):
        """
        Sets gains to specified value. If input is None then they are set as the already existing value.
        So if Kp is None for instance then its value just doesn't change.
        
        n represents the cutoff frequency for the low pass filter applied to the derivative term of the pid controller
        """
        
        self.Kp = Kp or self.Kp
        self.Ki = Ki or self.Ki
        self.Kd = Kd or self.Kd
        self.n =  n  or self.n
        self.sample_period = sample_period or self.sample_period
        
        
    def reset(self):
        self.__init__(self.sample_period, self.Kp, self.Ki, self.Kd, self.n, self.sample_period)
        
        
    def step(self, error):
        a0 = 1 + self.n * self.sample_period
        a1 = - (2 + self.n * self.sample_period)
        a2 = 1
        
        b0 = self.Kp * (1 + self.n * self.sample_period) + self.Ki * self.sample_period * (1 + self.n * self.sample_period) + self.Kd * self.n
        b1 = - (self.Kp * (2 + self.n * self.sample_period) + self.Ki * self.sample_period + 2 * self.Kd * self.n)
        b2 = self.Kp + self.Kd * self.n
        
        output = - (a1/a0) * self.prev_output1 - (a2/a0) * self.prev_output2 + (b0/a0) * error + (b1/a0) * self.prev_error1 + (b2/a0) * self.prev_error2

        self.prev_output2 = self.prev_output1
        self.prev_output1 = output
        self.prev_error2 = self.prev_error1
        self.prev_error1 = error

        return output
    
    
    def __call__(self, error):
        return self.step(error)
    