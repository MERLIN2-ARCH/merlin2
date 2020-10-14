
""" Pddl Dto Abstract Class """

from abc import ABC, abstractmethod


class PddlDto(ABC):
    """ Pddl Dto Abstract Class """

    @abstractmethod
    def __str__(self):
        return "PddlDto abstract class"
