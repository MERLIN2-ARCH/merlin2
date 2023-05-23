
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
#include "merlin2_arch_interfaces/action/dispatch_action.hpp"
#include "merlin2_arch_interfaces/msg/plan_action.hpp"
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

  virtual bool run_action(merlin2_arch_interfaces::msg::PlanAction goal) = 0;
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
  void execute_server(std::shared_ptr<rclcpp_action::ServerGoalHandle<
                          merlin2_arch_interfaces::action::DispatchAction>>
                          goal_handle);

  std::shared_ptr<simple_node::actions::ActionServer<
      merlin2_arch_interfaces::action::DispatchAction>>
      action_server;
};

} // namespace action
} // namespace merlin2

#endif