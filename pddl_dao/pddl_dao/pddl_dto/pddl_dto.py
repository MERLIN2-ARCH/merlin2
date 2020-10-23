
""" Pddl Dto Abstract Class """

from abc import ABC, abstractmethod


class PddlDto(ABC):
    """ Pddl Dto Abstract Class """

    @abstractmethod
    def __str__(self):
        return "PddlDto abstract class"

    @abstractmethod
    def __eq__(self, other) -> bool:
        return False
