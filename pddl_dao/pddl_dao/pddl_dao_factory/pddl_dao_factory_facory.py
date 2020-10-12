from enum import Enum, auto
from pddl_dao.pddl_dao_factory.pddl_dao_factories.mongoengine_pddl_dao_factory import Mongoengine_PDDL_DAO_Factory
from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PDDL_DAO_Factory


class PDDL_DAO_Families(Enum):
    MONGOENGINE = auto()


class PDDL_DAO_FactoryFactory:

    def __init__(self):
        self.pddl_dao_families = PDDL_DAO_Families
        self.__pddl_dao_type_families = {
            self.pddl_dao_families.MONGOENGINE: Mongoengine_PDDL_DAO_Factory
        }

    def create_pddl_dao_factory(self, family: int) -> PDDL_DAO_Factory:
        return self.__pddl_dao_type_families[family]()
