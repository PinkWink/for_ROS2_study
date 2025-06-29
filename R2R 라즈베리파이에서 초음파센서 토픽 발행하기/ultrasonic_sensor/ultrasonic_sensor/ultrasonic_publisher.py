#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class Ultrasonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, False)
        time.sleep(0.5)

    def get_dist_mm(self):
        """
        거리를 cm 단위로 측정하고 mm 단위로 변환하여 반환합니다.
        """
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001) # 10us
        GPIO.output(self.trig_pin, False)

        pulse_start = time.time()
        timeout = pulse_start + 0.1 # 타임아웃 설정 (100ms)

        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return -1.0 

        pulse_end = time.time()
        timeout = pulse_end + 0.1 

        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return -1.0 

        pulse_duration = pulse_end - pulse_start
        # 속도(343m/s) * 시간 / 2 * 1000 (mm 변환)
        distance_mm = pulse_duration * 171500 
        
        return distance_mm

    def cleanup(self): 
        GPIO.cleanup([self.trig_pin, self.echo_pin])

class UltrasonicPublisher(Node):

    def __init__(self):
        super().__init__('ultrasonic_publisher_node')

        self.declare_parameter('trig_pin', 23, 
            descriptor=rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Ultrasonic sensor TRIG pin number.'))
        self.declare_parameter('echo_pin', 24, 
            descriptor=rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Ultrasonic sensor ECHO pin number.'))
        self.declare_parameter('ultra_sonic_sample_rate', 0.1, 
            descriptor=rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Publishing rate in seconds.'))
        self.declare_parameter('min_range_mm', 100.0, 
            descriptor=rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Minimum measurement range in mm.'))
        self.declare_parameter('max_range_mm', 800.0, 
            descriptor=rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Maximum measurement range in mm.'))

        trig_pin = self.get_parameter('trig_pin').get_parameter_value().integer_value
        echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        sample_rate = self.get_parameter('ultra_sonic_sample_rate').get_parameter_value().double_value
        self.min_range_mm = self.get_parameter('min_range_mm').get_parameter_value().double_value
        self.max_range_mm = self.get_parameter('max_range_mm').get_parameter_value().double_value

        self.get_logger().info(f"--- Ultrasonic Sensor Node Parameters ---")
        self.get_logger().info(f"TRIG Pin: {trig_pin}, ECHO Pin: {echo_pin}")
        self.get_logger().info(f"Sample Rate: {sample_rate}s ({1.0/sample_rate:.1f} Hz)")
        self.get_logger().info(f"Measurement Range: {self.min_range_mm:.1f}mm ~ {self.max_range_mm:.1f}mm")
        self.get_logger().info(f"-----------------------------------------")

        try:
            self.sensor = Ultrasonic(trig_pin, echo_pin)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            self.get_logger().error("Are you running this on a Raspberry Pi with root/gpio permissions?")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(Float32, 'ultra_sonic', 10)
        self.timer = self.create_timer(sample_rate, self.timer_callback)

    def timer_callback(self):
        distance = self.sensor.get_dist_mm()
        
        if self.min_range_mm <= distance <= self.max_range_mm:
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)
        elif distance == -1.0:
            self.get_logger().warn('Measurement timed out.')
        else:
            self.get_logger().warn(f'Measured distance {distance:.2f}mm is out of range ({self.min_range_mm:.1f} ~ {self.max_range_mm:.1f}mm).')

    def on_shutdown(self):
        """ 노드 종료 시 GPIO 정리 """
        self.get_logger().info('Shutting down, cleaning up GPIO.')
        self.sensor.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()