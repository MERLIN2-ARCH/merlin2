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

#include <exception>
#include <signal.h>

#include "merlin2_demo/merlin2_navigation_bt_action.hpp"

class InterruptException : public std::exception {
public:
  InterruptException(int s) : S(s) {}
  int S;
};
void sig_to_exception(int s) { throw InterruptException(s); }

using namespace merlin2::action;

Merlin2NavigationBtAction::Merlin2NavigationBtAction()
    : Merlin2BtAction("navigation") {

  this->wp_type =
      std::make_shared<kant::dto::PddlTypeDto>(kant::dto::PddlTypeDto("wp"));
  this->robot_at = std::make_shared<kant::dto::PddlPredicateDto>(
      kant::dto::PddlPredicateDto("robot_at", {this->wp_type}));

  this->org = std::make_shared<kant::dto::PddlObjectDto>(
      kant::dto::PddlObjectDto(this->wp_type, "o"));
  this->dst = std::make_shared<kant::dto::PddlObjectDto>(
      kant::dto::PddlObjectDto(this->wp_type, "d"));

  this->start_action();
}

void Merlin2NavigationBtAction::cancel_action() {

  this->cancel_tree();

  auto anywhere =
      std::make_shared<kant::dto::PddlObjectDto>(this->wp_type, "anywhere");
  auto prop = std::make_shared<kant::dto::PddlPropositionDto>(
      kant::dto::PddlPropositionDto(this->robot_at, {anywhere}));

  auto pddl_proposition_dao = this->dao_factory->create_pddl_proposition_dao();
  pddl_proposition_dao->save(prop);
}

std::vector<std::shared_ptr<kant::dto::PddlObjectDto>>
Merlin2NavigationBtAction::create_parameters() {
  return {this->org, this->dst};
}

std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
Merlin2NavigationBtAction::create_conditions() {

  auto condition_1 = std::make_shared<kant::dto::PddlConditionEffectDto>(
      kant::dto::PddlConditionEffectDto(this->robot_at, {this->org},
                                        kant::dto::AT_START));

  return {condition_1};
}

std::vector<std::shared_ptr<kant::dto::PddlConditionEffectDto>>
Merlin2NavigationBtAction::create_efects() {
  auto effect_1 = std::make_shared<kant::dto::PddlConditionEffectDto>(
      kant::dto::PddlConditionEffectDto(this->robot_at, {this->dst},
                                        kant::dto::AT_END));

  auto effect_2 = std::make_shared<kant::dto::PddlConditionEffectDto>(
      kant::dto::PddlConditionEffectDto(this->robot_at, {this->org}, true,
                                        kant::dto::AT_START));

  return {effect_1, effect_2};
}

int main(int argc, char *argv[]) {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_to_exception;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  rclcpp::init(argc, argv);
  std::shared_ptr<Merlin2NavigationBtAction> node;

  try {
    node = std::make_shared<Merlin2NavigationBtAction>();
    node->join_spin();
  } catch (InterruptException &e) {
    node->destroy_action();
  }

  rclcpp::shutdown();
  return 0;
}