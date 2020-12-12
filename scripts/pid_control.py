#!/usr/bin/python

import numpy as np

class controller(object):
    def __init__(self,kpv,kpw,lim_v,lim_w):
        self.kpv=kpv
        self.kpw=kpw
        self.lim_v=lim_v
        self.lim_w=lim_w
    
    def linear_control(self,error):
        c_signal=self.kpv*error
        c_signal=min(c_signal,self.lim_v)
        c_signal=max(c_signal,-self.lim_v)
        return c_signal

    def angular_control(self,error):
        c_signal=self.kpw*error
        c_signal=min(c_signal,self.lim_w)
        c_signal=max(c_signal,-self.lim_w)
        return c_signal