
from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_predicate import PddlDtoPredicate


class PDDL_DAO_Predicate(ABC):

    @abstractmethod
    def get(self, predicate_name: str) -> PddlDtoPredicate:
        pass

    @abstractmethod
    def get_all(self) -> List[PddlDtoPredicate]:
        pass

    @abstractmethod
    def _save(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        pass

    @abstractmethod
    def _update(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        pass

    @abstractmethod
    def save(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        pass

    @abstractmethod
    def delete(self, pddl_dto_predicate: PddlDtoPredicate) -> bool:
        pass

    @abstractmethod
    def delete_all(self) -> bool:
        pass
