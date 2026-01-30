import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Communication
        self.subscription = self.create_subscription(Float32, '/robot_tilt', self.tilt_callback, 10)
        self.publisher = self.create_publisher(Float32, '/cmd_vel_wheels', 10)

        # PID Tuning (Robust Values)
        self.kp = 180.0   # Strength to lift the weight
        self.ki = 20.0     # Integral OFF
        self.kd = 75.0    # Heavy braking to stop overshoot
        
        self.last_error = 0.0
        self.integral = 0.0
        self.target_angle = 0.06  # Slight forward tilt for steady state movement 
        
        # Soft Start Flag
        self.first_run = True 

        self.get_logger().info("--- PID CONTROLLER ONLINE: LOGS RESTORED ---")

    def tilt_callback(self, msg):
        current_tilt = msg.data
        tilt_deg = current_tilt * 57.2958 # Convert for safety checks
        
        # --- 1. KILL SWITCH (Safety First) ---
        # If we are falling over (> 45 degrees), stop everything.
        if abs(tilt_deg) > 45.0:
            self.integral = 0.0      # Reset memory
            self.last_error = 0.0
            
            # Send Stop Command
            stop_msg = Float32()
            stop_msg.data = 0.0
            self.publisher.publish(stop_msg)
            return # Exit function immediately
        # -------------------------------------

        error = self.target_angle - current_tilt
        
        # Soft Start Check
        if self.first_run:
            self.last_error = error
            self.first_run = False

        # --- 2. INTEGRAL ANTI-WINDUP (The Fix) ---
        # Only build "memory" if the error is small (near vertical).
        # This prevents the robot from obsessing over big falls.
        if abs(error) < 0.2:  # Only integrate if within ~11 degrees
            self.integral += error
        else:
            self.integral = 0.0 # Forget the past if we are fighting for survival
            
        # Optional: Hard Clamp on Integral size
        self.integral = max(min(self.integral, 200.0), -200.0)
        # -----------------------------------------

        derivative = error - self.last_error
        
        # PID Calc
        pid_sum = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = -1.0 * pid_sum 
        
        # Motor Limit
        output = max(min(output, 200.0), -200.0)

        # Publish
        cmd_msg = Float32()
        cmd_msg.data = float(output)
        self.publisher.publish(cmd_msg)
        
        self.last_error = error

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()