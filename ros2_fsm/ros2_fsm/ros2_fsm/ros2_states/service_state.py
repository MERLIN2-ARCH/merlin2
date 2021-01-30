

from ros2_fsm.basic_fsm import State
from .basic_outcomes import BasicOutomes


class ServiceState(State):

    def __init__(self, node, srv_type, srv_name, create_request_handler, response_handler=None):

        self.__service_client = node.create_client(srv_type, srv_name)

        self.__create_request_handler = create_request_handler
        self.__response_handler = response_handler

        super().__init__([BasicOutomes.SUCC, BasicOutomes.ABOR])

    def _create_request(self, blackboard):
        return self.__create_request_handler(blackboard)

    def execute(self, blackboard):

        request = self._create_request(blackboard)
        self.__service_client.wait_for_service()

        try:
            response = self.__service_client.call(request)

            if self.__response_handler:
                self.__response_handler(blackboard, response)

            return BasicOutomes.SUCC
        except:
            return BasicOutomes.ABOR
