
#include "merlin2_bt_action/bt_nodes/waypoint_navigation.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<merlin2::bt_nodes::WaypointNavigation>(
      "waypoint_navigation");
}