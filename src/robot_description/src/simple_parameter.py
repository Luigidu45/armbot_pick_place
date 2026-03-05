import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParameterResult

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "Antonio")
        
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result == SetParameterResult()

        for param in params:
            if param.name == "simple_int_param"
