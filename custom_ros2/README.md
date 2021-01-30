# custom_ros2

```python
from custom_ros2 import Node
import rclpy

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

```