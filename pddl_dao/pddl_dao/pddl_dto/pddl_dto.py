from abc import ABC, abstractmethod


class PDDL_DTO(ABC):

    @abstractmethod
    def __str__(self):
        return "PDDL_DTO abstract class"
