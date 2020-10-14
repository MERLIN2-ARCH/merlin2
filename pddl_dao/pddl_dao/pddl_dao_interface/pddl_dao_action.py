
from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_action import PddlDtoAction


class PDDL_DAO_Action(ABC):

    @abstractmethod
    def get(self, action_name: str) -> PddlDtoAction:
        pass

    @abstractmethod
    def get_all(self) -> List[PddlDtoAction]:
        pass

    @abstractmethod
    def _save(self, pddl_dto_action: PddlDtoAction) -> bool:
        pass

    @abstractmethod
    def _update(self, pddl_dto_action: PddlDtoAction) -> bool:
        pass

    @abstractmethod
    def save(self, pddl_dto_action: PddlDtoAction) -> bool:
        pass

    @abstractmethod
    def delete(self, pddl_dto_action: PddlDtoAction) -> bool:
        pass

    @abstractmethod
    def delete_all(self) -> bool:
        pass
