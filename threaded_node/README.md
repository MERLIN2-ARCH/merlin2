# threaded_node

```python
from threaded_node.node import Node
import rclpy

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

```