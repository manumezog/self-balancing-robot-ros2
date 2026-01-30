"""
Self-Balancing Robot PID Controller

This module implements a PID controller for a two-wheeled self-balancing robot.
It subscribes to tilt angle measurements and publishes motor velocity commands
to keep the robot upright and stable.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIDController(Node):
    """
    PID Controller node for the self-balancing robot.
    
    This controller uses proportional, integral, and derivative feedback to maintain
    the robot's balance by adjusting wheel motor speeds based on tilt angle measurements.
    It includes safety features like kill switches and anti-windup mechanisms.
    """
    def __init__(self):
        """
        Initialize the PID controller node.
        
        Sets up ROS 2 subscribers and publishers, configures PID parameters,
        and initializes state variables for the control loop.
        """
        super().__init__('pid_controller')
        
        # --- ROS 2 Communication Setup ---
        # Subscribe to tilt angle sensor data (in radians) and publish motor commands
        self.subscription = self.create_subscription(Float32, '/robot_tilt', self.tilt_callback, 10)
        self.publisher = self.create_publisher(Float32, '/cmd_vel_wheels', 10)

        # --- PID Tuning Parameters ---
        # These values have been tuned for stable balance with reasonable responsiveness
        self.kp = 180.0   # Proportional gain: Controls responsiveness to current tilt error
        self.ki = 20.0    # Integral gain: Helps correct steady-state offset (set low to prevent oscillations)
        self.kd = 75.0    # Derivative gain: Dampens oscillations and prevents overshoot
        
        # --- State Variables for PID Calculations ---
        self.last_error = 0.0       # Previous error, used to calculate derivative term
        self.integral = 0.0         # Accumulated error, used for integral term
        self.target_angle = 0.06    # Target tilt angle in radians (~3.4 degrees forward)
                                    # Slight forward tilt enables steady-state forward movement
        
        # --- Soft Start Mechanism ---
        # Prevents derivative kick on first run by initializing last_error properly
        self.first_run = True 

        self.get_logger().info("--- PID CONTROLLER ONLINE: LOGS RESTORED ---")

    def tilt_callback(self, msg):
        """
        Process tilt angle measurements and compute motor control commands.
        
        This callback is invoked whenever a new tilt angle is received. It implements
        the PID control law with safety features (kill switch) and anti-windup logic
        to stabilize the robot's balance.
        
        Args:
            msg (Float32): Tilt angle in radians. Positive values indicate forward tilt.
        """
        current_tilt = msg.data
        tilt_deg = current_tilt * 57.2958  # Convert radians to degrees for safety checks
        
        # --- SAFETY: Kill Switch ---
        # If the robot tilts beyond 45 degrees, it's falling over.
        # Immediately stop all motors and exit (no recovery possible at this angle).
        if abs(tilt_deg) > 45.0:
            # Reset integral state to avoid jerky recovery
            self.integral = 0.0
            self.last_error = 0.0
            
            # Publish zero velocity command (stop motors)
            stop_msg = Float32()
            stop_msg.data = 0.0
            self.publisher.publish(stop_msg)
            return  # Exit early - don't attempt recovery
        # ==============================

        # --- PID Control Calculation ---
        # Calculate error: how far we are from the target angle
        error = self.target_angle - current_tilt
        
        # Soft Start: Prevent derivative kick on initialization
        # On first run, set last_error equal to current error (derivative = 0)
        if self.first_run:
            self.last_error = error
            self.first_run = False

        # --- Anti-Windup: Integral Term Management ---
        # The integral term accumulates error over time, but can cause problems
        # if the robot is far from vertical (in a falling state).
        # Only integrate small errors to avoid "wind-up" during large disturbances.
        if abs(error) < 0.2:  # Only accumulate if within ~11 degrees of vertical
            self.integral += error
        else:
            # If error is large, reset integral memory (don't remember the past)
            # This prevents the controller from overshooting when recovering
            self.integral = 0.0
            
        # Clamp integral to prevent excessive accumulation
        # Limits the integral term's maximum contribution to the output
        self.integral = max(min(self.integral, 200.0), -200.0)
        # ============================================

        # --- Calculate Derivative Term ---
        # Rate of change of error - used to anticipate and dampen oscillations
        derivative = error - self.last_error
        
        # --- PID Output Calculation ---
        # Combine proportional, integral, and derivative terms
        pid_sum = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # Negate the output: positive error (forward tilt) requires negative motor command (backward)
        output = -1.0 * pid_sum 
        
        # --- Motor Saturation Limit ---
        # Clamp output to safe motor velocity range [-200, 200]
        output = max(min(output, 200.0), -200.0)

        # --- Publish Motor Command ---
        # Send the computed velocity to the motor controller
        cmd_msg = Float32()
        cmd_msg.data = float(output)
        self.publisher.publish(cmd_msg)
        
        # Store current error for next iteration's derivative calculation
        self.last_error = error

def main(args=None):
    """
    Entry point for the PID controller node.
    
    Initializes the ROS 2 system, creates and spins the PID controller node,
    and handles graceful shutdown.
    
    Args:
        args: Optional command-line arguments to pass to ROS 2.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create and run the controller node
    node = PIDController()
    rclpy.spin(node)  # Keep the node running and processing callbacks
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the controller node when this script is executed directly
    main()