

#ifndef MERLIN2_BT_ACTION_HPP
#define MERLIN2_BT_ACTION_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "merlin2_action/merlin2_action.hpp"

namespace merlin2 {
namespace action {

class Merlin2BtAction : public Merlin2Action {

public:
  Merlin2BtAction(std::string a_name);

  bool run_action(merlin2_arch_interfaces::msg::PlanAction goal) override;
  virtual void cancel_action() override;

protected:
  BT::BehaviorTreeFactory bt_factory;
  std::unique_ptr<BT::PublisherZMQ> groot_monitor;
  std::shared_ptr<BT::Tree> tree;
  BT::Blackboard::Ptr blackboard;
};

} // namespace action
} // namespace merlin2

#endif