
""" Pddl Action Dto """

from typing import List
from pddl_dao.pddl_dto.pddl_dto import PddlDto
from pddl_dao.pddl_dto.pddl_condition_efect_dto import PddlConditionEffectDto
from pddl_dao.pddl_dto.pddl_object_dto import PddlObjectDto


class PddlActionDto(PddlDto):
    """ Pddl Action Dto Class """

    def __init__(self, action_name: str,
                 parameters_list: List[PddlObjectDto] = None,
                 conditions_list: List[PddlConditionEffectDto] = None,
                 effects_list: List[PddlConditionEffectDto] = None,
                 durative: str = True,
                 duration: int = 10):

        self.set_action_name(action_name)
        self.set_parameters_list(parameters_list)
        self.set_durative(durative)
        self.set_duration(duration)
        self.set_conditions_list(conditions_list)
        self.set_effects_list(effects_list)

        PddlDto.__init__(self)

    def get_action_name(self) -> str:
        """ pdd action name getter

        Returns:
            str: pddl action name
        """

        return self._action_name

    def set_action_name(self, action_name: str):
        """ pddl action name setter

        Args:
            action_name (str): pddl action name
        """

        self._action_name = action_name

    def get_durative(self) -> bool:
        """ durative getter

        Returns:
            bool: is this a durative action
        """

        return self._durative

    def set_durative(self, durative: bool):
        """ durative setter

        Args:
            durative (bool): is this a durative action
        """

        self._durative = durative

    def get_duration(self) -> int:
        """ duration getter

        Returns:
            int: action duration
        """

        return self._duration

    def set_duration(self, duration: int):
        """ duration setter

        Args:
            duration (int): action duration
        """

        self._duration = duration

    def get_parameters_list(self) -> List[PddlConditionEffectDto]:
        """ parameters list getter

        Returns:
            List[PddlConditionEffectDto]: list of action parameters
        """

        return self._parameters_list

    def set_parameters_list(self, parameters_list: List[PddlConditionEffectDto]):
        """ parameters list setter

        Args:
            parameters_list (List[PddlConditionEffectDto]): list of action parameters
        """

        if parameters_list:
            self._parameters_list = parameters_list
        else:
            self._parameters_list = []

    def get_conditions_list(self) -> List[PddlConditionEffectDto]:
        """ conditions list getter

        Returns:
            List[PddlConditionEffectDto]: list of action conditions
        """

        return self._conditions_list

    def set_conditions_list(self, conditions_list: List[PddlConditionEffectDto]):
        """ conditions list setter

        Args:
            conditions_list (List[PddlConditionEffectDto]): list of action conditions
        """

        if conditions_list:
            self._conditions_list = conditions_list
        else:
            self._conditions_list = []

    def get_effects_list(self) -> List[PddlConditionEffectDto]:
        """ effects list getter

        Returns:
            List[PddlConditionEffectDto]: list of action effects
        """
        return self._effects_list

    def set_effects_list(self, effects_list: List[PddlConditionEffectDto]):
        """ effects list setter

        Args:
            effects_list (List[PddlConditionEffectDto]): list of action effects
        """

        if effects_list:
            self._effects_list = effects_list
        else:
            self._effects_list = []

    def __str__(self):
        string = "(:"

        # durative
        if self._durative:
            string += "durative-"
        string += "action " + self._action_name

        # parameters
        string += "\n\t:parameters ("
        for parameter in self._parameters_list:
            string += " ?" + parameter.get_object_name() + " - " + \
                parameter.get_pddl_type().get_type_name()
        string += ")"

        # duration
        if self._durative:
            string += "\n\t:duration (= ?duration " + str(self._duration) + ")"

        # conditions
        if self._durative:
            string += "\n\t:condition (and"
        else:
            string += "\n\t:precondition (and"
        for condi in self._conditions_list:
            string += "\n\t\t" + str(condi)
        string += "\n\t)"

        # effects
        string += "\n\t:effect (and"
        for effect in self._effects_list:
            string += "\n\t\t" + str(effect)
        string += "\n\t)"

        string += "\n)"

        return string
