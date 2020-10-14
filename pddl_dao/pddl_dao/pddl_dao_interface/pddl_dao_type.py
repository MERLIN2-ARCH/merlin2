
from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_type import PddlDtoType


class PDDL_DAO_Type(ABC):

    @abstractmethod
    def get(self, type_name: str) -> PddlDtoType:
        pass

    @abstractmethod
    def get_all(self) -> List[PddlDtoType]:
        pass

    @abstractmethod
    def _save(self, pddl_dto_type: PddlDtoType) -> bool:
        pass

    @abstractmethod
    def _update(self, pddl_dto_type: PddlDtoType) -> bool:
        pass

    @abstractmethod
    def save(self, pddl_dto_type: PddlDtoType) -> bool:
        pass

    @abstractmethod
    def delete(self, pddl_dto_type: PddlDtoType) -> bool:
        pass

    @abstractmethod
    def delete_all(self) -> bool:
        pass
