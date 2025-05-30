import sys 

import rclpy

from rclpy.node import Node 

from std_msgs.msg import Float32MultiArray,Int16MultiArray

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
            
        self.create_subscription(Float32MultiArray, '/joystick_data', self.joystick_callback, 100)

        self.pwm_pub = self.create_publisher(Int16MultiArray, "pwm_values", 10)

        self.joystick_data = None

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

        with open(PATH + "/data/newtons_to_pwm.tsv", "r") as file:
            for data_point in file:
                data = data_point.split("\t")
                x.append(data[0])
                y.append(data[1])

        optimal_params, param_covariance = curve_fit(newton_to_pwm.newtons_to_pwm, x, y)
        return optimal_params
    
    def joystick_callback(self, msg):
        """Callback para guardar los datos del joystick"""
        self.joystick_data = msg.data
    
    def pwm_callback(self, motor_values):
        # Creamos un nuevo mensaje para publicar PWM
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [1500] * 6

        # Iteramos sobre los valores recibidos en motor_values.data
        for index, newton in enumerate(motor_values.data):
            pwm = int(newton_to_pwm.newtons_to_pwm(
                newton,
                self.pwm_fit_params[0],
                self.pwm_fit_params[1],
                self.pwm_fit_params[2],
                self.pwm_fit_params[3],
                self.pwm_fit_params[4],
                self.pwm_fit_params[5]
            ))
            # Limitar el rango de pwm
            up = 1750
            down = 1250
            
            pwm = up if pwm > up else down if pwm < down else pwm
        
            # Si el valor en newton es 0, lo asignamos a 1500
            if newton == 0:
                pwm = 1500
            pwm_msg.data[index] = pwm
        # if pwm_msg.data[3] > 1700 and pwm_msg.data[2] < 1200:

        if pwm_msg.data[2]>1600:
            pwm_msg.data[2] = 1850
        if pwm_msg.data[2]<1400:
            pwm_msg.data[2] = 1150
        if pwm_msg.data[3]>1600:
            pwm_msg.data[3] = 1850
        if pwm_msg.data[3]<1400:
            pwm_msg.data[3] = 1150

        if self.joystick_data is not None:
            if bool(self.joystick_data[8]):#derecha
                pwm_msg.data[0] = 1250 
                pwm_msg.data[1] = 1750 
                pwm_msg.data[4] = 1750 
                pwm_msg.data[5] = 1250 
            if bool(self.joystick_data[7]):#izquierda
                pwm_msg.data[0] = 1750 
                pwm_msg.data[1] = 1250 
                pwm_msg.data[4] = 1250 
                pwm_msg.data[5] = 1750 

        pwm_msg.data[3] += 15
        self.pwm_pub.publish(pwm_msg)

    
    def __del__(self):
        pwm_values = Int16MultiArray()
        pwm_values.data = [1500] * 6
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