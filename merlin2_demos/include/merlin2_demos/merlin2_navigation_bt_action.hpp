// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef MERLIN2_NAVIGATION_BT_ACTION_HPP
#define MERLIN2_NAVIGATION_BT_ACTION_HPP

#include <memory>
#include <string>

#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "kant_dto/pddl_predicate_dto.hpp"
#include "kant_dto/pddl_type_dto.hpp"
#include "merlin2_bt_action/merlin2_bt_action.hpp"

namespace merlin2 {
namespace action {

class Merlin2NavigationBtAction : public merlin2::action::Merlin2BtAction {

public:
  Merlin2NavigationBtAction();

  void cancel_action() override;
  std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
  create_parameters() override;
  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
  create_conditions() override;
  std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
  create_effects() override;

private:
  std::shared_ptr<kant::dto::PddlTypeDto> wp_type;
  std::shared_ptr<kant::dto::PddlPredicateDto> robot_at;

  std::shared_ptr<kant::dto::PddlObjectDto> org;
  std::shared_ptr<kant::dto::PddlObjectDto> dst;
};

} // namespace action
} // namespace merlin2

#endif