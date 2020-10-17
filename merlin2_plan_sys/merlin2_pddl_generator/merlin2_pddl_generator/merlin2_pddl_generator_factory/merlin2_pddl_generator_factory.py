
""" Merlin2 Pddl Generator Factory """

from typing import Type
from merlin2_pddl_generator.merlin2_pddl_generators.generic_merlin2_pddl_generator import (
    GenericMerlin2PddlGenerator
)
from merlin2_pddl_generator.merlin2_pddl_generators.mongoengine_merlin2_pddl_generator import (
    MongoengineMerlin2PddlGenerator
)
from pddl_dao.pddl_dao_factory.pddl_dao_families import PddlDaoFamilies


class Merlin2PddlGeneratorFactory:
    """ Merlin2 Pddl Generator Factory """

    def __init__(self):
        self.pddl_dao_families = PddlDaoFamilies
        self.__pddl_dao_type_families = {
            self.pddl_dao_families.MONGOENGINE: MongoengineMerlin2PddlGenerator
        }

    def create_pddl_generator(self, family: int, **kwargs) -> Type[GenericMerlin2PddlGenerator]:
        """ return the class of the pddl generator of a given family

        Args:
            family (int): number of the pddl dao family to create

        Returns:
            Type[GenericMerlin2PddlGenerator]: class of a   pddl generator
        """

        args_dict = {}

        generator = self.__pddl_dao_type_families[family]
        init_args = list(generator.__init__.__code__.co_varnames)

        for key, value in kwargs.items():
            if key in init_args:
                args_dict[key] = value

        return generator(**args_dict)
