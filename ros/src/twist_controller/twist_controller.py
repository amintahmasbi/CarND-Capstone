from yaw_controller import YawController
import numpy as np
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity,
            brake_deadband, decel_limit, accel_limit,
            wheel_radius, wheel_base, steer_ratio,
            max_lat_accel, max_steer_angle,
            min_speed, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.min_speed = min_speed

        self.previous_linear_velocity = None
        self.speed_controller = PID(2., 0., 0.) # values provided by DataSpeedInc for MKZ
        self.accel_controller = PID(0.4, 0.1, 0., mn=0., mx=self.accel_limit)  # values provided by DataSpeedInc for MKZ
        self.accel_lowpassfilter = LowPassFilter(0.5, 0.2)  # values provided by DataSpeedInc for MKZ

        self.steering_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pass

    def control(self, target_linear_velocity, target_angular_velocity,
            current_linear_velocity, current_angular_velocity, control_rate,
            dbw_is_enabled, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs

        if self.previous_linear_velocity is None:
            self.previous_linear_velocity = current_linear_velocity
        else:
            raw_accel = control_rate * (current_linear_velocity.x - self.previous_linear_velocity.x) # TODO: parameterize: 50 comes from the dbw_node Rate value
            self.accel_lowpassfilter.filt(raw_accel)

        velocity_error = target_linear_velocity.x - current_linear_velocity.x
        accel = self.speed_controller.step(velocity_error, 1./ control_rate)

        steer = 0.
        throttle =0.
        brake = 0.

        if accel >= 0.:
            throttle = self.accel_controller.step(accel-self.accel_lowpassfilter.get(), 1./ control_rate)
        else: # TODO: Check if accel requested for braking is smaller than brake_deadband 
            accel = max(self.decel_limit, accel)
            brake = -accel * self.vehicle_mass * self.wheel_radius #TODO: Add passanger weights and consider GAS weight

        steer = self.steering_controller.get_steering(target_linear_velocity.x,target_angular_velocity.z,current_linear_velocity.x)
        steer = max(-self.max_steer_angle, min(self.max_steer_angle, steer))

        if not dbw_is_enabled:
            self.accel_controller.reset()
            self.speed_controller.reset()

        self.previous_linear_velocity = current_linear_velocity
        # Return throttle, brake, steer
        return throttle, brake, steer
