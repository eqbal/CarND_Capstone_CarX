
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement ** Mani Adds ***
        self.vehicle_mass    = args[0]
        self.fuel_capacity   = args[1]
        self.brake_deadband  = args[2]
        self.decel_limit     = args[3]
        self.accel_limit     = args[4]
        self.wheel_radius    = args[5]
        self.wheel_base      = args[6]
        self.steer_ratio     = args[7]
        self.max_lat_accel   = args[8]
        self.max_steer_angle = args[9]
    

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs *** Mani Adds ***
        throttle    = args[0]
        steer       = args[1]
        brake       = 0.0 // Currently set to zero..
        
        # Return throttle, brake, steer
        return throttle, brake, steer

