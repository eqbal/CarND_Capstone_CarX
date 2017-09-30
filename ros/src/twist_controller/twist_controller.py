
import time
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH	    = 0.44704
MAX_SPEED   = 40.0


class Controller(object):
    def __init__(self, *args, **kwargs):
       
	self.throttle_pid = PID(kwargs['throttle_kp'], kwargs['throttle_ki'], kwargs['throttle_kd'])	      
	self.yaw_control = YawController(kwargs['wheel_base'], 
					 kwargs['steer_ratio'],
                                         kwargs['min_speed'], 
					 kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'])

        self.last_time = None


    def control(self, target_v, target_omega, current_v, dbw_enabled):

        # target_v and target_omega are desired linear and angular velocities

        if self.last_time is None or not dbw_enabled:
            self.last_time = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.last_time

        error = min(target_v.x, MAX_SPEED * ONE_MPH) - current_v.x

        throttle = self.throttle_pid.step(error, dt)
        throttle = max(0.0, min(1.0, throttle))

        if error < 0:
            brake = -10.0 * error   # Proportional braking
            brake = max(brake, 1.0)
            throttle = 0.0
        else:
            brake = 0.0

	steer = self.yaw_control.get_steering(target_v.x, target_omega.z, current_v.x)

        self.last_time = time.time()
	
	return throttle, brake, steer


