
#include "merlin2_demo/merlin2_navigation_bt_action.hpp"

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

  this->tree->haltTree();

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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Merlin2NavigationBtAction>();
  node->join_spin();
  rclcpp::shutdown();
  return 0;
}