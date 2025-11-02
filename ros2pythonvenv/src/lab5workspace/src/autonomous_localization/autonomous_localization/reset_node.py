
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

# A very simple example of a node which objective is to call a specific service.
# In this particular case, the service is the reinitialize_global_localization service provided by AMCL.
# This is indeed a service of type Empty and provided by the amcl node (ROS2, NAV2 TurtleBot3 stack).
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('reinitialize_global_localization_client')              # Name of the node
        self.cli = self.create_client(Empty, 'reinitialize_global_localization') # Create the client for the specific service
        while not self.cli.wait_for_service(timeout_sec=1.0):                    # Wait until the service is available
            self.get_logger().info('service not available, waiting...')
        self.req = Empty.Request()                                               # Initialize the request message as an empty one (since the service is of type Empty)

    def send_request(self): self.future = self.cli.call_async(self.req)          # Actually call the service

def main():
    rclpy.init()                                                                 # Initialize the ROS2 Python client library
    minimal_client = MinimalClientAsync()                                        # Create an instance of the client node defined above
    minimal_client.send_request()                                                # Call the service
    while rclpy.ok():                                                            # Keep the node alive until we get the response! 
                                                                                 # in particular, rclpy.ok() returns False only when the node is killed (e.g. with Ctrl-C or with rclpy.shutdown())
        rclpy.spin_once(minimal_client)                                          # Spin once the node (without any timeout, the default one is None which is considered as infinite)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()                        # Notice: we actually don't expect any response, since the service is of type Empty
                                                                                 # Indeed, here we just check if the call was successful or not
            except Exception as e:
                minimal_client.get_logger().info('Call failed %r' % (e,))
            else:
                minimal_client.get_logger().info('Call succeeded!')
            break
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()