


MAX_FWD_THRUST = 36.3826715 * 2 # N
MAX_REV_THRUST = -28.6354180 * 2 # N

TOTAL_CURRENT_LIMIT = 70 # A
ESC_CURRENT_LIMIT = 40 # A

import sys

import rclpy

from rclpy.node import Node 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from scipy.optimize import curve_fit
import numpy as np

from os import path

PATH = path.dirname(__file__)

class twist_to_newtons(Node):
    """
    Class that implements the kinematics.
    """

    def __init__(self):
        super().__init__("twist_to_newtons")

        self.motor_positions = [ # [X, Y, Z] Posiciones de los motores respecto al centro del ROV
            [ 0.200,  0.130,  0.004], # Motor 0
            [ 0.200, -0.130,  0.047], # Motor 1
            [-0.200,  0.130,  0.047], # Motor 2
            [-0.200, -0.130,  0.047], # Motor 3
            [ 0.198,  0.156, -0.038], # Motor 4
            [ 0.198, -0.156, -0.038], # Motor 5
            [-0.198,  0.156, -0.038], # Motor 6
            [-0.198, -0.156, -0.038]  # Motor 7
        ]
        
        self.motor_thrusts = [ # [X, Y, Z] Fuerzas descompuestas respecto a la orientacion del ROV
            [    0.0,     0.0, -1.0],   # Motor 0
            [    0.0,     0.0,  1.0],   # Motor 1 
            [    0.0,     0.0,  1.0],   # Motor 2
            [    0.0,     0.0, -1.0],   # Motor 3
            [-0.7071,  0.7071,  0.0],   # Motor 4
            [-0.7071, -0.7071,  0.0],   # Motor 5
            [ 0.7071,  0.7071,  0.0],   # Motor 6
            [ 0.7071, -0.7071,  0.0]    # Motor 7
        ]

        self.center_of_mass = [0.0,0.0,0.0] #[X,Y,Z] Posicion del centro de masa respecto al centro del ROV 

        self.motor_config = self.generate_motor_config(self.center_of_mass)

        self.inverse_config = np.linalg.pinv(self.motor_config, rcond=1e-15, hermitian=False)

        
        self.thrust_pub = self.create_publisher(Float32MultiArray, "motor_values", 10)
        self.subscription = self.create_subscription(Twist, "desired_twist", self.thrust_callback, 10)

        self.thrust_fit_params = twist_to_newtons.generate_thrust_fit_param()

    def generate_motor_config(self, center_of_mass_offset):
        """
        Generate the motor configuration matrix based on motor positions and thrust. Allows for
        a shifting center of mass, so the motor configuration can be regenerated dynamically to
        account for center of mass shifts when lifting objects.

        Returns:
            Motor configuration matrix based on motor orientation, position, and location of center of mass
        """
        shifted_positons = [(np.subtract(motor, center_of_mass_offset).tolist())
                            for motor in self.motor_positions]
        torques = np.cross(shifted_positons, self.motor_thrusts)

        return [
            [thrust[0] for thrust in self.motor_thrusts], # Fx (N)
            [thrust[1] for thrust in self.motor_thrusts], # Fy (N)
            [thrust[2] for thrust in self.motor_thrusts], # Fz (N)
            [torque[0] for torque in torques],            # Rx (N*m)
            [torque[1] for torque in torques],            # Ry (N*m)
            [torque[2] for torque in torques]             # Rz (N*m)
        ]
    
    @staticmethod
    def __thrust_to_current(x: float, a: float, b: float, c: float, d: float, e: float, f: float, g: float) -> float:
        """
        Estimates current draw based on given thrust

        Args:
            x: Thrust being produced in newtons.
            a-f: Arbitrary parameters to map thrust to current, see generate_thrust_fit_params()

        Returns:
            Current (estimated) to be drawn in amps.
        """
        return (a * x**6) + (b * x**5) + (c * x**4) + (d * x**3) + (e * x**2) + (f * x) + (g)

    @staticmethod
    def generate_thrust_fit_param() -> list:
        """
        Generates Optimal Parameters for __thrust_to_current() to have a best fit

        Returns:
            List of optimal parameters
        """
        x = list()
        y = list()

        with open(PATH + "/data/thrust_to_current.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(twist_to_newtons.__thrust_to_current, x, y)
        return optimal_params

    def get_polynomial_coef(self, mv: list, limit: float) -> list:
        """
        Generates a list of the coefficients for a polynomial, the input of which is the
        motor scaling factor and the roots of the function are the maximum scaling factor.

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:
            A list of the coefficients of a 5th degree polynomial function, where the input of said
            function is the scaling factor and the output is the current (A) draw
        """
        return [self.thrust_fit_params[0] * sum([thrust**6 for thrust in mv]),
                self.thrust_fit_params[1] * sum([thrust**5 for thrust in mv]),
                self.thrust_fit_params[2] * sum([thrust**4 for thrust in mv]),
                self.thrust_fit_params[3] * sum([thrust**3 for thrust in mv]),
                self.thrust_fit_params[4] * sum([thrust**2 for thrust in mv]),
                self.thrust_fit_params[5] * sum(mv),
                self.thrust_fit_params[6] * len(mv) - limit]
    
    def get_current_scalar_value(self, mv: list, limit: float) -> float:
        """
        Given a motor value list and a current limit, return the best scaling factor

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist
            limit: The current limit we would like to stay under in amperes (TOTAL_CURRENT_LIMIT or ESC_CURRENT_LIMIT)

        Returns:
            A valid scaling factor
        """
        # Get coefficients for function given the motor values given and the current (Amp) limits
        coef_list = self.get_polynomial_coef(mv, limit)
        # Find roots
        potential_scaling_factors = np.roots(coef_list).tolist()
        # Ignore nonreal and negative scaling factors
        real_positive = [scalar.real for scalar in potential_scaling_factors if scalar.imag == 0 and scalar.real >= 0]
        # Return valid roots
        return min(real_positive)
    
    def get_minimum_current_scalar(self, mv: list) -> float:
        """
        Returns a scalar which shows the maximum amount of thrust the robot can produce for the
        given direction without exceeding total current (A) limits, or the current (A) limit of
        either ESC

        Args:
            mv: The motor values in newtons that when produced will result in our desired twist

        Returns:
            The largest scalar we can scale those motor values by without exceeding the total current
            (A) limit and the current limit of each ESC
        """
        # All motors
        total_scalar = self.get_current_scalar_value(mv, TOTAL_CURRENT_LIMIT)
        # First four motors / motors on esc 1
        esc1_scalar = self.get_current_scalar_value(mv[:4], ESC_CURRENT_LIMIT)
        # Second four motors / motors on esc 2
        esc2_scalar = self.get_current_scalar_value(mv[4:], ESC_CURRENT_LIMIT)

        return min(total_scalar, esc1_scalar, esc2_scalar)


    def get_thrust_limit_scalar(self, motor_values: list) -> float:
        """
        Generate scaling factor based on thrust limitations

        Args:
            motor_values: The motor values in newtons that when produced will result in our desired twist

        Returns:
            Largest scalar the motor values can be scaled by without exceeding thrust limits
        """
        # Scalar is infinite if 0, since there is no limit to how large it can be scaled
        return min([(MAX_FWD_THRUST / thrust) if thrust > 0
                    else ((MAX_REV_THRUST / thrust) if thrust < 0
                        else float("inf"))
                    for thrust in motor_values])
    
    def generate_motor_values(self, twist_msg):
        """Called every time the twist publishes a message."""

        # Convert the X,Y,Z,R,P,Y to thrust settings for each motor. 
        motor_values = []

        # Convert Twist to single vector for multiplication
        twist_array = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]

        if twist_array == [0, 0, 0, 0, 0, 0]:
            return [0.0 for motor in range(8)] # No thrust needed

        # Multiply twist with inverse of motor config to get motor effort values
        motor_values = np.matmul(self.inverse_config, twist_array).tolist()

        thrust_scalar = self.get_thrust_limit_scalar(motor_values)
        current_scalar = self.get_minimum_current_scalar(motor_values)
        # Scalar will be the smaller of the two, largest value in twist array
        # will be percentage of that maximum
        scalar = min(thrust_scalar, current_scalar) * max([abs(val) for val in twist_array])

        # scale and return motor values
        return [thrust * scalar for thrust in motor_values]
    

    def thrust_callback(self, twist_msg):
        thrust_msg = Float32MultiArray()
        thrust_msg.data = self.generate_motor_values(twist_msg)
        self.thrust_pub.publish(thrust_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = twist_to_newtons()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()    


if __name__ == "__main__":
    main(sys.argv)