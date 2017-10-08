import time
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH     = 0.44704
MAX_SPEED   = 40.0


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, decel_limit, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, Kp, Ki, Kd):

        min_speed           = 1.0 * ONE_MPH
        self.throttle_pid   = PID(Kp, Ki, Kd)
        self.yaw_control    = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.v_mass         = vehicle_mass
        self.w_radius       = wheel_radius
        self.d_limit        = decel_limit

        self.last_time      = None


    def control(self, target_v, target_omega, current_v, dbw_enabled):

        # Calculate the desired throttle based on target_v, target_omega and current_v
        # target_v and target_omega are desired linear and angular velocities

        if self.last_time is None or not dbw_enabled:
            self.last_time = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.last_time

        # Assumed maximum speed is in Kmph

        error = min(target_v.x, MAX_SPEED * 0.277778) - current_v.x

        throttle = self.throttle_pid.step(error, dt)

        # Max throttle is 1.0
        throttle = max(0.0, min(1.0, throttle))

        # Brake or decelerate only if the target velocity is lower than the current velocity
        # Brake value is in N/m and is calculated using car mass, acceleration and wheel radius
        # longitudinal force = mass of car * acceleration (or deceleration)
        # Torque = longitudinal force * wheel radius, which is supplied as brake value
        #
        # Further refinements can be done by adding the mass of fuel and the passengers to the mass
        # of the car in real world scenario
        #
        if error < 0: # Needs to decelerate

            deceleration        = abs(error) / dt

            if abs(deceleration) > abs(self.d_limit)*500:
                deceleration = self.d_limit*500  # Limited to decelartion limits
            longitudinal_force  = self.v_mass * deceleration
            brake               = longitudinal_force * self.w_radius
            throttle        = 0.0
        else:
            brake       = 0.0

        # Steering control is using Yaw Control..
        steer = self.yaw_control.get_steering(target_v.x, target_omega.z, current_v.x)

        self.last_time = time.time()

        return throttle, brake, steer
