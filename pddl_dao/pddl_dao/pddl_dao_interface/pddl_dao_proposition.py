
from abc import ABC, abstractmethod
from typing import List
from pddl_dao.pddl_dto.pddl_dto_proposition import PDDL_DTO_Proposition


class PDDL_DAO_Proposition(ABC):

    @abstractmethod
    def get_by_predicate(self, predicate_name: str) -> List[PDDL_DTO_Proposition]:
        pass

    @abstractmethod
    def get_goals(self) -> List[PDDL_DTO_Proposition]:
        pass

    @abstractmethod
    def get_all(self) -> List[PDDL_DTO_Proposition]:
        pass

    @abstractmethod
    def _save(self, pddl_dto_proposition: PDDL_DTO_Proposition) -> bool:
        pass

    @abstractmethod
    def _update(self, pddl_dto_proposition: PDDL_DTO_Proposition) -> bool:
        pass

    @abstractmethod
    def save(self, pddl_dto_proposition: PDDL_DTO_Proposition) -> bool:
        pass

    @abstractmethod
    def delete(self, pddl_dto_proposition: PDDL_DTO_Proposition) -> bool:
        pass

    @abstractmethod
    def delete_all(self) -> bool:
        pass
