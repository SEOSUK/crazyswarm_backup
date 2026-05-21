import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from crazyflie_py import Crazyswarm


class SuInterface(Node):
    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        super().__init__('su_interface')

        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.param_client = self.create_client(SetParameters, '/crazyflie_server/set_parameters')
        self.get_logger().info('su_interface node ready.')

    def keyboard_callback(self, msg):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'o':
            self.cf.arm(True)
            self.get_logger().info('ARM command sent.')
        elif input_char == 'p':
            self.cf.arm(False)
            self.get_logger().info('DISARM command sent.')
        elif input_char == 'm':
            self.trigger_zero_bias()

    def trigger_zero_bias(self):
        if not self.param_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning('crazyflie_server set_parameters service not ready')
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='cf2.params.su_wrench.zeroBias',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=1,
                ),
            ),
        ]
        future = self.param_client.call_async(req)
        future.add_done_callback(self._zero_bias_done)
        self.get_logger().info('ZERO BIAS command sent.')

    def _zero_bias_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'ZERO BIAS request failed: {exc}')
            return

        if not response.results:
            self.get_logger().warning('ZERO BIAS request returned no results')
            return

        result = response.results[0]
        if result.successful:
            self.get_logger().info('ZERO BIAS result: cf2.params.su_wrench.zeroBias=ok')
        else:
            reason = f' ({result.reason})' if result.reason else ''
            self.get_logger().warning(
                'ZERO BIAS result: cf2.params.su_wrench.zeroBias=fail' + reason
            )

    def shutdown(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)



def main(args=None):
    node = SuInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
