# used to specify what is available to import from this file
__all__ = ["DiscretePID"]

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

    def __init__(self, sample_period: float, k_p: float, k_i: float, k_d: float, n: float) -> None:
        """
        Parameters
        ----------
        sample_period
            Sample period for the discrete PID controller.
        k_p
            Proportional gain.
        k_i
            Integral gain.
        k_d
            Derivative gain.
        n
            Cutoff frequency for the low pass filter on the derivative term.
        """

        self.sample_period = sample_period
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.n = n

        self.prev_output1: float = 0.0
        self.prev_output2: float = 0.0

        self.prev_error1: float = 0.0
        self.prev_error2: float = 0.0


    def reset(self) -> None:
        """Resets the PID controller to its initial state."""

        self.__init__(self.sample_period, self.k_p, self.k_i, self.k_d, self.n, self.sample_period)


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
        k_p: float | None = None,
        k_i: float | None = None,
        k_d: float | None = None,
        n: float | None = None,
        sample_period: float | None = None,
    ) -> None:
        """
        Sets gains to the specified value.
        If an argument is ``None`` then they are set as the already existing value.

        Examples
        --------
        Set ``k_p`` to 2.0 and ``k_i`` to 0.5, leaving ``k_d``, ``n``, and
        ``sample_period`` unchanged::

            >>> pid_controller.set_gains(k_p=2.0, k_i=0.5)

        Leave all gains unchanged::

            >>> pid_controller.set_gains()

        Parameters
        ----------
        k_p
            Proportional gain.
        k_i
            Integral gain.
        k_d
            Derivative gain.
        n
            Cutoff frequency for the low pass filter on the derivative term.
        sample_period
            Sample period for the discrete PID controller.
        """

        self.k_p = k_p or self.k_p
        self.k_i = k_i or self.k_i
        self.k_d = k_d or self.k_d
        self.n = n or self.n
        self.sample_period = sample_period or self.sample_period


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

        b0 = (
            self.k_p * (1 + self.n * self.sample_period)
            + self.k_i * self.sample_period * (1 + self.n * self.sample_period)
            + self.k_d * self.n
        )
        b1 = -(self.k_p * (2 + self.n * self.sample_period) + self.k_i * self.sample_period + 2 * self.k_d * self.n)
        b2 = self.k_p + self.k_d * self.n

        output = (
            -(a1 / a0) * self.prev_output1
            - (a2 / a0) * self.prev_output2
            + (b0 / a0) * error
            + (b1 / a0) * self.prev_error1
            + (b2 / a0) * self.prev_error2
        )

        self.prev_output2 = self.prev_output1
        self.prev_output1 = output
        self.prev_error2 = self.prev_error1
        self.prev_error1 = error

        return output
