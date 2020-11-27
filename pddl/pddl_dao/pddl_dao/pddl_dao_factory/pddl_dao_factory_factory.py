
""" Pddl Dao Factory of Factories """

from pddl_dao.pddl_dao_factory.pddl_dao_families import PddlDaoFamilies

from pddl_dao.pddl_dao_factory.pddl_dao_factories import (
    PddlDaoFactory,
    Merlin2PddlDaoFactory,
    MongoenginePddlDaoFactory
)


class PddlDaoFactoryFactory:
    """ Pddl Dao Factory of Factories Class """

    def __init__(self):
        self.pddl_dao_families = PddlDaoFamilies
        self.__families_to_factory = {
            self.pddl_dao_families.MONGOENGINE: MongoenginePddlDaoFactory,
            self.pddl_dao_families.MERLIN2: Merlin2PddlDaoFactory
        }

    def create_pddl_dao_factory(self, family: int, **kwargs) -> PddlDaoFactory:
        """ create a pddl dao factory of a given family

        Args:
            family (int): number of the pddl dao family to create

        Returns:
            PddlDaoFactory: pddl dao factory
        """

        args_dict = {}

        pddl_dao_factory = self.__families_to_factory[family]
        init_args = list(pddl_dao_factory.__init__.__code__.co_varnames)

        for key, value in kwargs.items():
            if key in init_args:
                args_dict[key] = value

        return pddl_dao_factory(**args_dict)
