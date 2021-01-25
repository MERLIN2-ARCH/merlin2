

from typing import List
from .state import State


class CbState(State):

    def __init__(self, outcomes: List[str], cb):

        super().__init__(outcomes)
        self._cb = cb

    def execute(self, shared_data):
        return self._cb(shared_data)
