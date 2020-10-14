
""" Pddl Dao Factory of Factories """

from enum import Enum, auto
from pddl_dao.pddl_dao_factory.pddl_dao_factories.mongoengine_pddl_dao_factory import (
    MongoenginePddlDaoFactory
)
from pddl_dao.pddl_dao_factory.pddl_dao_factories.pddl_dao_factory import PddlDaoFactory


class PddlDaoFamilies(Enum):
    """ Enum Class of Pddl Dao Families """

    MONGOENGINE = auto()


class PddlDaoFactoryFactory:
    """ Pddl Dao Factory of Factories Class """

    def __init__(self):
        self.pddl_dao_families = PddlDaoFamilies
        self.__pddl_dao_type_families = {
            self.pddl_dao_families.MONGOENGINE: MongoenginePddlDaoFactory
        }

    def create_pddl_dao_factory(self, family: int) -> PddlDaoFactory:
        """ create a pddl dao factory of a given family

        Args:
            family (int): number of the pddl dao family to create

        Returns:
            PddlDaoFactory: pddl dao factory
        """

        return self.__pddl_dao_type_families[family]()
