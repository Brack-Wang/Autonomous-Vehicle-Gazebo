#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.set_point = set_point

        self.sample_time = sample_time                       
        self.out_limits = out_limits
        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback):                         
        currentTime = time.time()
        currentErr= self.set_point - feedback
        changeTime = currentTime - self.last_time
        changeErr = currentErr - self.last_err
        self.last_time = currentTime
        self.last_err = currentErr
        err = changeErr / changeTime
        self.p_term = currentErr
        self.i_term += currentErr * changeTime
        self.d_term = err
        out=  self.Kp * self.p_term + self.Ki * self.i_term + self.Kd * self.d_term
        print(out)
        limitation = self.out_limits
        if out>np.max(limitation):
            out=np.max(limitation)
        if out<np.min(limitation):
            out=np.min(limitation)
        self.output = out

    def __call__(self, feeback_val):
        return self.update(feeback_val)
