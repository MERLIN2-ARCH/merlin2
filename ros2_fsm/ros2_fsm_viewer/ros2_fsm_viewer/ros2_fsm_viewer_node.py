
from typing import List
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from ros2_fsm_interfaces.msg import (
    Status,
    StateInfo,
    State,
    Transition
)
from flask import Flask
import json


class Ros2FsmViewer(Node):

    def __init__(self):

        super().__init__("ros2_fsm_viewer")

        self.__started = False
        self.__fsm_dict = {}

        thread = Thread(target=self.start_subscriber)
        thread.start()

        self.start_server()

    def start_server(self):
        app = Flask("ros2_fsm_viewer")

        @app.route("/get_fsms", methods=["GET"])
        def get_fsms():
            return json.dumps(self.__fsm_dict)

        @app.route("/get_fsm/<fsm_name>",  methods=["GET"])
        def get_fsm(fsm_name):

            fsm_name = fsm_name.upper()

            if fsm_name in self.__fsm_dict:
                return json.dumps(self.__fsm_dict[fsm_name])

            return json.dumps({})

        self.__started = True
        app.run(host="localhost", port=5000)

    def start_subscriber(self):
        self.create_subscription(Status,
                                 "fsm_viewer",
                                 self.fsm_viewer_cb,
                                 10)

        rclpy.spin(self)

    def transition_msg_to_dict(self, tansitions: List[Transition]):
        transition_dict = {}

        for transition in tansitions:
            transition_dict[transition.outcome] = transition.state

        return transition_dict

    def state_info_msg_to_dict(self, msg: StateInfo):
        state_info_dict = {
            "state_name": msg.state_name,
            "transitions": self.transition_msg_to_dict(msg.transitions),
            "outcomes": msg.outcomes,
            "is_fsm": False
        }
        return state_info_dict

    def state_msg_to_dict(self, msg: State):
        if msg.is_fsm:
            fsm_dict = self.state_info_msg_to_dict(msg.state)
            fsm_dict["is_fsm"] = True
            fsm_dict["states"] = []
            fsm_dict["current_state"] = msg.current_state

            for state in msg.states:
                fsm_dict["states"].append(self.state_info_msg_to_dict(state))

            return fsm_dict

        else:
            return self.state_info_msg_to_dict(msg.state)

    def msg_to_dict(self, msg: Status):
        msg_dict = {
            "fsm_name": msg.fsm_name,
            "current_state": msg.current_state,
            "fsm_structure": {
                "final_outcomes": msg.fsm_structure.final_outcomes,
                "states": [

                ]
            }
        }

        for state in msg.fsm_structure.states:
            msg_dict["fsm_structure"]["states"].append(
                self.state_msg_to_dict(state))

        return msg_dict

    def fsm_viewer_cb(self, msg: Status):

        while not self.__started:
            time.sleep(0.05)

        # self.get_logger().info(str(msg))
        self.__fsm_dict[msg.fsm_name.upper()] = self.msg_to_dict(msg)
        # self.get_logger().info(str(self.__fsm_dict[msg.fsm_name]))


def main(args=None):
    rclpy.init(args=args)

    Ros2FsmViewer()

    rclpy.shutdown()


if __name__ == '__main__':
    main()