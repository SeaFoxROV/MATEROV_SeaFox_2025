import sys 

import rclpy

from rclpy.node import Node 

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

from scipy.optimize import curve_fit

from os import path

PATH = path.dirname(__file__)

class newton_to_pwm(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        """Initialize this node"""
        super().__init__("thrust")
        
        self.pwm_fit_params = newton_to_pwm.generate_pwm_fit_params()

        self.subscription = self.create_subscription(Float32MultiArray, "motor_values", self.pwm_callback,10)
        
        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)

    @staticmethod
    def newtons_to_pwm(x: float, a: float, b: float, c: float, d: float, e: float, f: float) -> float:
        """
        Converts desired newtons into its corresponding PWM value

        Args:
            x: The force in newtons desired
            a-f: Arbitrary parameters to map newtons to pwm, see __generate_curve_fit_params()

        Returns:
            PWM value corresponding to the desired thrust
        """
        return (a * x**5) + (b * x**4) + (c * x**3) + (d * x**2) + (e * x) + f

    @staticmethod
    def generate_pwm_fit_params():
        x = []
        y = []

        with open(PATH + "data/newtons_to_pwm.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(newton_to_pwm.newtons_to_pwm, x, y)
        return optimal_params

    def pwm_callback(self, twist_msg):
        pwm_values = Int16MultiArray()
        pwm_values.data = [0] * 8
        motor_values = self.generate_motor_values(twist_msg)
        for index, newton in enumerate(motor_values):
            pwm_values.data[index] = int(newton_to_pwm.newtons_to_pwm(
                newton,
                self.pwm_fit_params[0],
                self.pwm_fit_params[1],
                self.pwm_fit_params[2],
                self.pwm_fit_params[3],
                self.pwm_fit_params[4],
                self.pwm_fit_params[5]))
            pwm_values.data[index] = 1900 if pwm_values.data[index] > 1900 else 1100 if pwm_values.data[index] < 1100 else pwm_values.data[index]
            if newton == 0: pwm_values.data[index] = 1500
        self.pwm_pub.publish(pwm_values)
    
    def __del__(self):
        pwm_values = Int16MultiArray()
        pwm_values.data = [1500] * 8
        self.pwm_pub.publish(pwm_values)

def main(args=None):
    rclpy.init(args=args)
    node = newton_to_pwm()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)