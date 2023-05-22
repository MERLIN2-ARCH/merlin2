
#ifndef BT_NODES__WAYPOINT_NAVIGATION_HPP
#define BT_NODES__WAYPOINT_NAVIGATION_HPP

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "merlin2_arch_interfaces/action/dispatch_action.hpp"
#include "simple_node/actions/action_client.hpp"
#include "simple_node/node.hpp"
#include "waypoint_navigation_interfaces/action/navigate_to_wp.hpp"

namespace merlin2 {
namespace bt_nodes {

class WaypointNavigation : public BT::ActionNodeBase {
public:
  WaypointNavigation(const std::string &name,
                     const BT::NodeConfiguration &config)
      : BT::ActionNodeBase(name, config) {

    simple_node::Node::SharedPtr node;
    config.blackboard->get("node", node);

    this->action_client =
        ((simple_node::Node *)node.get())
            ->create_action_client<
                waypoint_navigation_interfaces::action::NavigateToWp>(
                "/waypoint_navigation/navigate_to_wp");
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<merlin2_arch_interfaces::msg::PlanAction>(
        "merlin2_action_goal")};
  }

  void halt() override { this->action_client->cancel_goal(); }

  BT::NodeStatus tick() override {

    if (!this->action_client->is_working()) {

      if (this->action_client->is_terminated() && !this->destination.empty()) {

        this->destination = "";

        if (this->action_client->is_canceled() &&
            this->action_client->is_aborted()) {
          return BT::NodeStatus::FAILURE;
        } else if (this->action_client->is_succeeded()) {
          return BT::NodeStatus::SUCCESS;
        }

      } else if (this->destination.empty()) {

        merlin2_arch_interfaces::msg::PlanAction plan_action_goal;
        this->config().blackboard->get("merlin2_action_goal", plan_action_goal);
        this->destination = plan_action_goal.objects[1];

        auto goal =
            waypoint_navigation_interfaces::action::NavigateToWp::Goal();
        goal.wp_id = this->destination;
        this->action_client->send_goal(goal);

        return BT::NodeStatus::RUNNING;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

private:
  std::shared_ptr<simple_node::actions::ActionClient<
      waypoint_navigation_interfaces::action::NavigateToWp>>
      action_client;

  std::string destination;
};

} // namespace bt_nodes
} // namespace merlin2

#endif // BT_NODES__WAYPOINT_NAVIGATION_HPP