
#include "behaviortree_cpp_v3/bt_factory.h"

#include "merlin2_bt_action/bt_nodes/waypoint_navigation.hpp"

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<merlin2::bt_nodes::WaypointNavigation>(
      "waypoint_navigation");
}