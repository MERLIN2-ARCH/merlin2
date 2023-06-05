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

#ifndef MERLIN2_ACTION_HPP
#define MERLIN2_ACTION_HPP

#include <memory>
#include <string>
#include <vector>

#include "kant_dao/dao_factory/dao_factories/dao_factory.hpp"
#include "kant_dao/parameter_loader.hpp"
#include "kant_dto/pddl_action_dto.hpp"
#include "kant_dto/pddl_condition_effect_dto.hpp"
#include "kant_dto/pddl_object_dto.hpp"
#include "merlin2_msgs/action/dispatch_action.hpp"
#include "merlin2_msgs/msg/plan_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_node/actions/action_server.hpp"
#include "simple_node/node.hpp"

namespace merlin2 {
namespace action {

class Merlin2Action : public simple_node::Node,
                      public kant::dto::PddlActionDto {

public:
  Merlin2Action(std::string a_name, bool durative = true);
  ~Merlin2Action();

  void start_action();
  void destroy_action();

  virtual bool run_action(merlin2_msgs::msg::PlanAction goal) = 0;
  virtual void cancel_action() = 0;
  virtual std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
  create_parameters();
  virtual std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
  create_conditions();
  virtual std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
  create_efects();

  kant::dao::dao_factory::dao_factories::DaoFactory *dao_factory;
  kant::dao::dao_interface::PddlActionDao *pddl_action_dao;

private:
  bool save_action();
  void cancel_callback();
  void execute_server(
      std::shared_ptr<
          rclcpp_action::ServerGoalHandle<merlin2_msgs::action::DispatchAction>>
          goal_handle);

  std::shared_ptr<
      simple_node::actions::ActionServer<merlin2_msgs::action::DispatchAction>>
      action_server;
};

} // namespace action
} // namespace merlin2

#endif