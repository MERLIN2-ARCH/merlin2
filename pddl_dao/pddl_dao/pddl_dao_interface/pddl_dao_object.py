
from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_object import PddlDtoObject


class PDDL_DAO_Object:

    @abstractmethod
    def get(self, object_name: str) -> PddlDtoObject:
        pass

    @abstractmethod
    def get_all(self) -> List[PddlDtoObject]:
        pass

    @abstractmethod
    def _save(self, pddl_dto_object: PddlDtoObject) -> bool:
        pass

    @abstractmethod
    def _update(self, pddl_dto_object: PddlDtoObject) -> bool:
        pass

    @abstractmethod
    def save(self, pddl_dto_object: PddlDtoObject) -> bool:
        pass

    @abstractmethod
    def delete(self, pddl_dto_object: PddlDtoObject) -> bool:
        pass

    @abstractmethod
    def delete_all(self) -> bool:
        pass
