
#include "kant_dao/parameter_loader.hpp"

#include "merlin2_action/merlin2_action.hpp"

using namespace merlin2;
using std::placeholders::_1;

Merlin2Action::Merlin2Action(std::string a_name, bool durative)
    : simple_node::Node(a_name), kant::dto::PddlActionDto(a_name) {

  auto parameter_loader = kant::dao::ParameterLoader(this);
  this->dao_factory = parameter_loader.get_dao_factory();
  this->pddl_action_dao = this->dao_factory->create_pddl_action_dao();

  // creating and saving the action
  this->set_durative(durative);
  auto pddl_parameter_dto_list = this->create_parameters();
  auto pddl_effect_dto_list = this->create_efects();
  auto pddl_condition_dto_list = this->create_conditions();

  ((kant::dto::PddlActionDto *)this)->set_parameters(pddl_parameter_dto_list);
  this->set_effects(pddl_effect_dto_list);
  this->set_conditions(pddl_condition_dto_list);

  if (!this->save_action()) {
    throw std::string("Wrong PDDL action: " + a_name);
  }

  this->action_server = this->create_action_server<
      merlin2_arch_interfaces::action::DispatchAction>(
      a_name, std::bind(&Merlin2Action::execute_server, this, _1),
      std::bind(&Merlin2Action::cancel_callback, this));
}

bool Merlin2Action::save_action() {
  std::shared_ptr<kant::dto::PddlActionDto> dto_ptr(
      (kant::dto::PddlActionDto *)this);
  return this->pddl_action_dao->save(dto_ptr);
}

void Merlin2Action::cancel_callback() { this->cancel_action(); }

void Merlin2Action::execute_server(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        merlin2_arch_interfaces::action::DispatchAction>>
        goal_handle) {

  auto result = std::make_shared<
      merlin2_arch_interfaces::action::DispatchAction::Result>();
  bool succeed = this->run_action(goal_handle->get_goal()->action);

  if (this->action_server->is_canceled()) {
    this->action_server->wait_for_canceling();
    goal_handle->canceled(result);
  }

  else {
    if (succeed) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
}