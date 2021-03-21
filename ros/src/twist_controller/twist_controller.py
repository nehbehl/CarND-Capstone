"""
TEMPORARY CODE: 

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
"""

import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        # Initialize yaw controller object (this will give steering angle)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # Initialize a PID controller object for throttle
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.  # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        # Initialize low pass fitler object to cutoff high frequency values of velocity 
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
           
        
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer
        
        """
        Importance of dbw_enabled:
        We can turn the drive by wire on and off in the car.
        So as we're sitting, waiting for a traffic light,I might turn the dbw off
        Reason: We don't want to accumulate error for integral term
        So when it is not enabled, we'll reset the controller and return 0,0,0
        """
         
        if not dbw_enabled: 
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        # Call filt method in our self.vel_lpf object of low pass filter
        current_vel = self.vel_lpf.filt(current_vel)
        
        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target velocity: {0}".format(linear_vel))
        # rospy.logwarn("Target angular velocity: {0}\n".format(angular_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))
        
        # Call get_steering method from yaw controller object
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        # rospy.logwarn("Steering : {0}".format(steering))
       
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        # Find the sample time for each step of our PID controller
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        # For throttle, we're stepping through our throttle controller 
        # (in ref. to PID controller object)
        throttle = self.throttle_controller.step(vel_error, sample_time)
        
        brake = 0

        # First check: 
        # If our linear velocity (our target) is 0 and 
        # we're going very slow, our current velocity is less than 0.1
        # We should try to stop: Set throttle=0, apply lots of brake        
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700 #N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2
        
        # Or else, throttle is also really small (PID is letting up on the throttle)
        # and the velocity error is less than 0 (negative in this case): we're going faster
        # decel would be a -ve number (but brake should be positive)      
        
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius #Torque N*m
        
        
        # rospy.logwarn("Throttle : {0}".format(throttle))
        # rospy.logwarn("Brake : {0}".format(brake))
        # return 1., 0., 0.
        return throttle, brake, steering
            
