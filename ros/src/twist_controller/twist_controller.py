from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)
        
        # Parameters
        kp = 1.0# 0.8
        ki = 0.1# 0.005
        kd = 0.1# 0.3
        mn = 0.
        mx = 0.2
        
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02  # sample time
        
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Calculate Throttle Value
        current_vel = self.vel_lpf.filt(current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        # Steering Control
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
       
        # Throttle Control
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        # If target linear velocity = 0, then go very slow
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400
            
        # If throttle is really small and velocity error < 0 (i.e. we're going faster than we want to be)
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)               # a negative number
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle, brake, steering