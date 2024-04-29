import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import Adafruit_PCA9685


pwm = Adafruit_PCA9685.PCA9685()
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print(f"{pulse_length}us per period")
    pulse_length //= 4096     # 12 bits of resolution
    print(f"{pulse_length}us per bit")
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
# Function to move servo
def move_servo(channel, angle):
    servo_min = 150  # Min pulse length out of 4096
    servo_max = 600  # Max pulse length out of 4096
    
    # Convert angle to pulse
    pulse = servo_min + (servo_max - servo_min) * angle / 180.0
    pwm.set_pwm(channel, 0, int(pulse))



class MotorAngleSubscriber(Node):
    def __init__(self):
        super().__init__('motor_angle_subscriber')
        self.subscription = self.create_subscription(Float64MultiArray, 'motorangles', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.pca = Adafruit_PCA9685.PCA9685()
        

    def listener_callback(self, msg):
        self.get_logger().info('Received motor angles: "%s"' % msg.data)
        # Example: set motor 0 to the first angle value
        FR = [msg.data[0],msg.data[1],msg.data[2]]
        FL= [msg.data[3],msg.data[4],msg.data[5]]
        BR= [msg.data[6],msg.data[7],msg.data[8]]
        BL= [msg.data[9],msg.data[10],msg.data[11]]
        
        move_servo(0,FR[0])
        move_servo(1,FR[1])
        move_servo(2,FR[2])
        
        move_servo(4,FL[0])
        move_servo(5,FL[1])
        move_servo(6,FL[2])
        
        move_servo(8,BR[0])
        move_servo(9,BR[1])
        move_servo(10,BR[2])
        
        move_servo(12,BL[0])
        move_servo(13,BL[1])
        move_servo(14,BL[2])
        
        
        
        
        

def main(args=None):
    rclpy.init(args=args)
    motor_angle_subscriber = MotorAngleSubscriber()
    rclpy.spin(motor_angle_subscriber)
    motor_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()