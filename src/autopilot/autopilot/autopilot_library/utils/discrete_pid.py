class DiscretePID:
    """
    Modified discrete PID controller using a low pass filter only for the derivative term.

    Reference
    ---------
    https://www.scilab.org/discrete-time-pid-controller-implementation.

    Note
    ----
    ``low_pass_filter_cutoff_frequency`` is represented by ``n`` to improve readability in the equations.
    """

    def __init__(self, sample_period: float, Kp: float, Ki: float, Kd: float, n: float) -> None:
        """
        Parameters
        ----------
        sample_period
            Sample period for the discrete PID controller.
        Kp
            Proportional gain.
        Ki
            Integral gain.
        Kd
            Derivative gain.
        n
            Cutoff frequency for the low pass filter on the derivative term.
        """

        self.sample_period = sample_period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.n = n

        self.prev_output1: float = 0.0
        self.prev_output2: float = 0.0

        self.prev_error1: float = 0.0
        self.prev_error2: float = 0.0

    def __call__(self, error: float) -> float:
        """
        Calls the ``step`` method with the given error and returns the output.

        Parameters
        ----------
        error
            The current error value to be processed by the PID controller.

        Returns
        -------
        float
            The output of the PID controller after processing the input error.
        """

        return self.step(error)

    def set_gains(
        self,
        Kp: float | None = None,
        Ki: float | None = None,
        Kd: float | None = None,
        n: float | None = None,
        sample_period: float | None = None,
    ) -> None:
        """
        Sets gains to the specified value. If an argument is ``None`` then they are set as the already existing value.

        Examples
        --------
        Set ``Kp`` to 2.0 and ``Ki`` to 0.5, leaving ``Kd``, ``n``, and
        ``sample_period`` unchanged::

            >>> pid_controller.set_gains(Kp=2.0, Ki=0.5)

        Leave all gains unchanged::

            >>> pid_controller.set_gains()

        Parameters
        ----------
        Kp
            Proportional gain.
        Ki
            Integral gain.
        Kd
            Derivative gain.
        n
            Cutoff frequency for the low pass filter on the derivative term.
        sample_period
            Sample period for the discrete PID controller.
        """

        self.Kp = Kp or self.Kp
        self.Ki = Ki or self.Ki
        self.Kd = Kd or self.Kd
        self.n =  n  or self.n
        self.sample_period = sample_period or self.sample_period



    def reset(self) -> None:
        """Resets the PID controller to its initial state."""

        self.__init__(self.sample_period, self.Kp, self.Ki, self.Kd, self.n, self.sample_period)



    def step(self, error: float) -> float:
        """
        Steps the PID controller with the given error and returns the output.

        Parameters
        ----------
        error
            The current error value to be processed by the PID controller.

        Returns
        -------
        float
            The output of the PID controller after processing the input error.
        """

        a0 = 1 + self.n * self.sample_period
        a1 = -(2 + self.n * self.sample_period)
        a2 = 1

        b0 = self.Kp * (1 + self.n * self.sample_period) + self.Ki * self.sample_period * (1 + self.n * self.sample_period) + self.Kd * self.n
        b1 = -(self.Kp * (2 + self.n * self.sample_period) + self.Ki * self.sample_period + 2 * self.Kd * self.n)
        b2 = self.Kp + self.Kd * self.n

        output = -(a1 / a0) * self.prev_output1 - (a2 / a0) * self.prev_output2 + (b0 / a0) * error + (b1 / a0) * self.prev_error1 + (b2 / a0) * self.prev_error2
        

        self.prev_output2 = self.prev_output1
        self.prev_output1 = output
        self.prev_error2 = self.prev_error1
        self.prev_error1 = error

        return output
