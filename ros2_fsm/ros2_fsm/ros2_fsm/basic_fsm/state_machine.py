

from typing import Dict, List
from .state import State
from .shared_data import SharedData


class StateMachine(State):
    def __init__(self, outcomes: List[str]):

        super().__init__(outcomes)

        self._states = {}
        self._start_state = None
        self._current_state = None
        self._shared_data = None

    def add_state(self,
                  name: str,
                  state: State,
                  transitions: Dict[str, str] = None):

        if not transitions:
            transitions = {}

        name = name.upper()

        self._states[name] = {
            "state": state,
            "transitions": transitions
        }

        if not self._start_state:
            self._start_state = name

    def set_start(self, name: str):
        self._start_state = name.upper()

    def cancel_state(self):
        super().cancel_state()
        if self._current_state:
            self._states[self._current_state]["state"].cancel_state()

    def execute(self, shared_data: SharedData = None):

        if not shared_data:
            self._shared_data = SharedData()
        else:
            self._shared_data = shared_data

        self._current_state = self._start_state

        state = self._states[self._start_state]

        while True:
            outcome = state["state"](self._shared_data)

            if outcome in state["transitions"]:
                self._current_state = state["transitions"][outcome]
                state = self._states[self._current_state]

            elif outcome in self._outcomes:
                self._current_state = None
                self._shared_data = None

                return outcome

            else:
                raise Exception("Outcome (" + outcome + ") without transition")

    def get_states(self) -> Dict[str, str]:
        return self._states

    def get_current_state(self) -> str:
        if self._current_state:
            return self._current_state

        return ""

    def __str__(self):
        return str(self._states)
