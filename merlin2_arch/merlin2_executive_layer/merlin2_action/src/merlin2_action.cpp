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

#include "merlin2_action/merlin2_action.hpp"

using namespace merlin2::action;
using std::placeholders::_1;

Merlin2Action::Merlin2Action(std::string a_name, bool durative)
    : simple_node::Node(a_name, "merlin2"), kant::dto::PddlActionDto(a_name) {

  this->set_durative(durative);

  kant::dao::ParameterLoader parameter_loader(this);
  this->dao_factory = parameter_loader.get_dao_factory();
  this->pddl_action_dao = this->dao_factory->create_pddl_action_dao();
}

Merlin2Action::~Merlin2Action() {
  delete this->dao_factory;
  delete this->pddl_action_dao;
}

void Merlin2Action::destroy_action() {
  auto dto_ptr = std::make_shared<kant::dto::PddlActionDto>(
      ((kant::dto::PddlActionDto *)this)->get_name());

  dto_ptr->set_duration(((kant::dto::PddlActionDto *)this)->get_duration());
  dto_ptr->set_durative(((kant::dto::PddlActionDto *)this)->get_durative());
  dto_ptr->set_parameters(((kant::dto::PddlActionDto *)this)->get_parameters());
  dto_ptr->set_conditions(((kant::dto::PddlActionDto *)this)->get_conditions());
  dto_ptr->set_effects(((kant::dto::PddlActionDto *)this)->get_effects());
  this->pddl_action_dao->delete_one(dto_ptr);
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
Merlin2Action::create_parameters() {
  return {};
}

std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
Merlin2Action::create_conditions() {
  return {};
}

std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
Merlin2Action::create_efects() {
  return {};
}

void Merlin2Action::start_action() {
  auto pddl_parameter_dto_list = this->create_parameters();
  auto pddl_effect_dto_list = this->create_efects();
  auto pddl_condition_dto_list = this->create_conditions();

  ((kant::dto::PddlActionDto *)this)->set_parameters(pddl_parameter_dto_list);
  this->set_effects(pddl_effect_dto_list);
  this->set_conditions(pddl_condition_dto_list);

  if (!this->save_action()) {
    RCLCPP_ERROR(this->get_logger(), "Wrong PDDL action: %s",
                 +((kant::dto::PddlActionDto *)this)->get_name().c_str());
    throw std::string("Wrong PDDL action: " +
                      ((kant::dto::PddlActionDto *)this)->get_name());
  }

  this->action_server =
      this->create_action_server<merlin2_msgs::action::DispatchAction>(
          ((kant::dto::PddlActionDto *)this)->get_name(),
          std::bind(&Merlin2Action::execute_server, this, _1),
          std::bind(&Merlin2Action::cancel_callback, this));
}

bool Merlin2Action::save_action() {

  auto dto_ptr = std::make_shared<kant::dto::PddlActionDto>(
      ((kant::dto::PddlActionDto *)this)->get_name());

  dto_ptr->set_duration(((kant::dto::PddlActionDto *)this)->get_duration());
  dto_ptr->set_durative(((kant::dto::PddlActionDto *)this)->get_durative());
  dto_ptr->set_parameters(((kant::dto::PddlActionDto *)this)->get_parameters());
  dto_ptr->set_conditions(((kant::dto::PddlActionDto *)this)->get_conditions());
  dto_ptr->set_effects(((kant::dto::PddlActionDto *)this)->get_effects());

  return this->pddl_action_dao->save(dto_ptr);
}

void Merlin2Action::cancel_callback() { this->cancel_action(); }

void Merlin2Action::execute_server(
    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<merlin2_msgs::action::DispatchAction>>
        goal_handle) {

  auto result =
      std::make_shared<merlin2_msgs::action::DispatchAction::Result>();
  bool succeed = this->run_action(goal_handle->get_goal()->action);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);

  } else {
    if (succeed) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
}
