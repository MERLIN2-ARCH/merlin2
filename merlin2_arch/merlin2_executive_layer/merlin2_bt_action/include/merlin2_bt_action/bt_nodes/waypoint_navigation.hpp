
#ifndef BT_NODES__WAYPOINT_NAVIGATION_HPP
#define BT_NODES__WAYPOINT_NAVIGATION_HPP

#include <memory>
#include <string>

#include "example_interfaces/action/fibonacci.hpp"
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

    std::shared_ptr<simple_node::Node> node;
    config.blackboard->template get<std::shared_ptr<simple_node::Node>>("node",
                                                                        node);

    config.blackboard->template get<std::string>("destination",
                                                 this->destination);

    this->action_client = node->create_action_client<
        waypoint_navigation_interfaces::action::NavigateToWp>(
        "/waypoint_navigation/navigate_to_wp");
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("destination")};
  }

  void halt() override { this->action_client->cancel_goal(); }

  BT::NodeStatus tick() override {

    if (!this->action_client->is_working()) {

      if (this->action_client->is_terminated()) {
        if (this->action_client->is_canceled() &&
            this->action_client->is_aborted()) {
          return BT::NodeStatus::FAILURE;
        } else if (this->action_client->is_succeeded()) {
          return BT::NodeStatus::SUCCESS;
        }

      } else {
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