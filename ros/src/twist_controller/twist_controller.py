from yaw_controller import YawController
import numpy as np

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

        self.steering_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pass

    def control(self, target_linear_velocity, target_angular_velocity,
            current_linear_velocity, dbw_is_enabled, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        tlv = np.abs(np.array([target_linear_velocity.x, target_linear_velocity.y, target_linear_velocity.z]))
        tav = np.abs(np.array([target_angular_velocity.x, target_angular_velocity.y, target_angular_velocity.z]))
        clv = np.abs(np.array([current_linear_velocity.x, current_linear_velocity.y, current_linear_velocity.z]))

        steer = self.steering_controller.get_steering(tlv[0],tav[0],clv[0])

        throttle = 1.
        brake = 0.
        # Return throttle, brake, steer
        return throttle, brake, steer
