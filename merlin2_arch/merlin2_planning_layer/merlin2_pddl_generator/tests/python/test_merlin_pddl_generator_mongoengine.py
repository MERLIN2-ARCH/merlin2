# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from kant_dao.dao_factory import (
    DaoFactoryMethod,
    DaoFamilies
)

from tests_merlin2_pddl_generator_basic.test_merlin_pddl_generator import TestMerlin2PddlProblemGenerator


class TestMerlin2PddlProblemGeneratorMongoengine(TestMerlin2PddlProblemGenerator):

    def setUp(self):
        dao_factory_method = DaoFactoryMethod()
        self.dao_factory = dao_factory_method.create_dao_factory(
            DaoFamilies.MONGO, uri="mongodb://localhost:27017/merlin2_tests")
        super().setUp()


del (TestMerlin2PddlProblemGenerator)
