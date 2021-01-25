
import time
from threading import Thread, Lock
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient as ActionClient2
from rclpy.action import CancelResponse


class ActionClient(ActionClient2):

    def __init__(self, node, action_type, action_name):
        self._status = GoalStatus.STATUS_UNKNOWN
        self.__status_lock = Lock()
        self.__goal_handle = None
        self.__goal_thread = None
        self.__result = None
        self.__working = False
        super().__init__(node, action_type, action_name)

    def get_status(self):
        with self.__status_lock:
            return self._status

    def _set_status(self, status):
        with self.__status_lock:
            self._status = status

    def is_aborted(self):
        return self.get_status() == GoalStatus.STATUS_ABORTED

    def is_succeeded(self):
        return self.get_status() == GoalStatus.STATUS_SUCCEEDED

    def is_canceled(self):
        return self.get_status() == GoalStatus.STATUS_CANCELED

    def is_working(self):
        return self.__working

    def wait_for_result(self):
        self.__goal_thread.join()

    def get_result(self):
        return self.__result

    def __send_goal(self, goal):

        self.__result = None

        send_goal_future = self.send_goal_async(goal)

        # wait for acceptance
        while not send_goal_future.done() and not self.is_canceled():
            time.sleep(1)

        # check acceptance
        self.__goal_handle = send_goal_future.result()
        if not self.__goal_handle.accepted:

            # change status
            if self.is_canceled():
                return
            self._set_status(GoalStatus.STATUS_ABORTED)
            self.__working = False
            return

        # change status
        if self.is_canceled():
            return
        self._set_status(GoalStatus.STATUS_ACCEPTED)

        # get result
        get_result_future = self.__goal_handle.get_result_async()

        # change status
        if self.is_canceled():
            return
        self._set_status(GoalStatus.STATUS_EXECUTING)

        # wait for result
        while not get_result_future.done() and not self.is_canceled():
            time.sleep(1)

        # change status
        if self.is_canceled():
            return
        self._set_status(get_result_future.result().status)

        self.__result = get_result_future.result().result

        self.__working = False

    def send_goal(self, goal):
        self.__working = True
        self.__goal_thread = Thread(target=self.__send_goal, args=(goal,))
        self._set_status(GoalStatus.STATUS_UNKNOWN)
        self.__goal_thread.start()

    def __cancel_goal(self):

        old_status = self.get_status()

        cancel_goal_future = self._cancel_goal_async(self.__goal_handle)
        self._set_status(GoalStatus.STATUS_CANCELING)
        while not cancel_goal_future.done():
            time.sleep(1)

        result = cancel_goal_future.result()
        if not result != CancelResponse.ACCEPT:
            self._set_status(old_status)
            return False

        self._set_status(GoalStatus.STATUS_CANCELED)

        self.__working = False

        return True

    def cancel_goal(self):
        if self.__goal_handle:
            return self.__cancel_goal()

        else:
            return False
