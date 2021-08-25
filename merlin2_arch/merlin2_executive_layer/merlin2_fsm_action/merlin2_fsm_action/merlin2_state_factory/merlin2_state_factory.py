
""" MERLIN2 State Factory """

from yasmin import State
from .merlin2_basic_states import Merlin2BasicStates
from .states import (
    Merlin2NavigationState,
    Merlin2TtsState,
    Merlin2SttState
)


class Merlin2StateFactory:
    """ MERLIN2 State Factory Class """

    def __init__(self):
        self.merlin2_basic_states = Merlin2BasicStates
        self.__enum_to_state = {
            self.merlin2_basic_states.NAVIGATION: Merlin2NavigationState,
            self.merlin2_basic_states.TTS: Merlin2TtsState,
            self.merlin2_basic_states.STT: Merlin2SttState
        }

    def create_state(self, state: int, **kwargs) -> State:
        """ create a pddl dao factory of a given family

        Args:
            state (int): number of the state

        Returns:
            State: state object
        """

        args_dict = {}

        state_object = self.__enum_to_state[state]
        init_args = list(state_object.__init__.__code__.co_varnames)

        for key, value in kwargs.items():
            if key in init_args:
                args_dict[key] = value

        return state_object(**args_dict)
