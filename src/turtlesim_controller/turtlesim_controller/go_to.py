import rclpy
import argparse
from turtlesim_interfaces.srv import Trigger


def main(args=None):
    parser = argparse.ArgumentParser(description='Move turtle to goal.')
    parser.add_argument('-x',
                        action='store',
                        dest='x',
                        type=float,
                        default=0.0,
                        required=True,
                        help='Coordinate on the x-axis')
    parser.add_argument('-y',
                        action='store',
                        dest='y',
                        type=float,
                        default=0.0,
                        required=True,
                        help='Coordinate on the y-axis')
    parser.add_argument('-t',
                        '--tolerance',
                        action='store',
                        dest='tolerance',
                        type=float,
                        default=0.1,
                        required=False,
                        help='Acceptable tolerance')
    # getting the arguments
    args = parser.parse_args()
    # init ROS2 client communications
    rclpy.init(args=None)
    # creates a node
    node = rclpy.create_node('turtlesim_client')
    # make node client of 'move_to'
    client = node.create_client(Trigger, 'move_to')
    # request parameters
    request = Trigger.Request()
    request.x = args.x
    request.y = args.y
    request.delta = args.tolerance
    # waites for a response
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # gets result when receives response
    result = future.result()
    node.get_logger().info('Response: %s %d' % (result.message, result.success))
    # destroys node
    node.destroy_node()
    # close all ROS2 communications
    rclpy.shutdown()


if __name__ == "__main__":
    main()
