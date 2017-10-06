
import time
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH	    = 0.44704
MAX_SPEED   = 40.0
min_speed   = 1.0 * ONE_MPH

throttle_Kp = 0.3
throttle_Ki = 0.003
throttle_Kd = 4.0

steering_Kp = 0.3
steering_Ki = 0.003
steering_Kd = 4.0


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, wheel_radius, accel_limit, decel_limit, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
     
        self.w_radius           = wheel_radius
        self.d_limit            = decel_limit
        self.a_limit            = accel_limit
        self.fuel_capacity      = fuel_capacity
        self.vehicle_mass       = vehicle_mass
        self.max_steer_angle    = max_steer_angle
        
        self.throttle_pid       = PID(throttle_Kp, throttle_Ki, throttle_Kd, -abs(self.d_limit), abs(self.a_limit))
        self.steer_pid          = PID(steering_Kp, steering_Ki, steering_Kd, -(self.max_steer_angle), self.max_steer_angle)
        self.yaw_control        = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.last_time          = None

    def control(self, target_v, target_omega, current_v, current_omega, dbw_enabled):

        # Calculate the desired throttle based on target_v, target_omega and current_v
        # target_v and target_omega are desired linear and angular velocities

        if self.last_time is None or not dbw_enabled:
            self.last_time = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.last_time

        t_error = min(target_v.x, MAX_SPEED * ONE_MPH) - current_v.x
        s_error = target_omega.z - current_omega.z

        throttle = self.throttle_pid.step(t_error, dt)

        # Max throttle is 1.0
        throttle = max(0.0, min(1.0, throttle))

        # Brake or decelerate only if the target velocity is lower than the current velocity
        # Brake value is in N/m and is calculated using car mass, acceleration and wheel radius
        # longitudinal force = mass of car * acceleration (or deceleration)
        # Torque = longitudinal force * wheel radius, which is supplied as brake value
        # Add passenger weight as needed for accurate measurements
        
        v_mass  = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

        if t_error < 0: # Needs to decelerate
            deceleration        = (target_v.x - current_v.x) / dt
            longitudinal_force  = v_mass * deceleration
            brake               = longitudinal_force * self.w_radius
            if brake < self.d_limit:
                brake = - 5   # Limited to decelartion limits
            throttle = 0.0
        else:
            brake = 0.0

        # Steering control is using Yaw Control and look for bounds..
        
        steer = self.yaw_control.get_steering(target_v.x, target_omega.z, current_v.x) - self.steer_pid(s_error, dt)
        steer = max(-abs(self.max_steer_angle), min(abs(self.max_steer_angle), steer))

        self.last_time = time.time()

        return throttle, brake, steer


