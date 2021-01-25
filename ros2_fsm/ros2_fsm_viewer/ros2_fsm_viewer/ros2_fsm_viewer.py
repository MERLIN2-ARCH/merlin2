
from PyQt5.QtGui import (
    QFontMetrics,
    QFont,
    QImage
)

from QGraphViz.Engines import Dot
from QGraphViz.DotParser import (
    Graph,
    GraphType
)
from QGraphViz.QGraphViz import (
    QGraphViz,
    QGraphVizManipulationMode
)
from PyQt5.QtWidgets import (
    QFileDialog,
    QDialog,
    QApplication,
    QWidget,
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QComboBox,
    QPushButton,
    QInputDialog,
    QLineEdit,
    QLabel
)
import sys
import time
from custom_ros2 import Node
import rclpy
from threading import Thread
from ros2_fsm_interfaces.msg import Status


class Ros2FsmViewer(Node):

    def __init__(self):

        super().__init__("ros2_fsm_viewer")

        self.__started = False

        thread = Thread(target=self.start_app)
        thread.start()

        self.create_subscription(Status,
                                 "fsm_viewer",
                                 self.fsm_viewer_cb,
                                 10)

    def start_app(self):

        app = QApplication(sys.argv)
        show_subgraphs = True
        self.qgv = QGraphViz(
            show_subgraphs=show_subgraphs,
            auto_freeze=True,

            hilight_Nodes=True,
            hilight_Edges=True
        )
        self.qgv.setStyleSheet("background-color:white;")
        # Create A new Graph using Dot layout engine
        self.qgv.new(Dot(Graph("Main_Graph"), show_subgraphs=show_subgraphs,
                         font=QFont("Arial", 12), margins=[20, 20]))
        self.qgv.build()

        w = QMainWindow()
        w.setWindowTitle('ROS2 FSM Viewer')

        wi = QWidget()
        wi.setLayout(QVBoxLayout())
        w.setCentralWidget(wi)
        wi.layout().addWidget(self.qgv)

        hpanel = QHBoxLayout()
        wi.layout().addLayout(hpanel)

        self.__started = w.showNormal() is None

    def fsm_viewer_cb(self, msg):

        while not self.__started:
            time.sleep(0.05)

        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)

    node = Ros2FsmViewer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
