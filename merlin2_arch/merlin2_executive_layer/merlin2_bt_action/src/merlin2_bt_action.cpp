// Copyright (C) 2023 Miguel Ángel González Santamarta

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

#include <vector>

#include "merlin2_bt_action/merlin2_bt_action.hpp"

using namespace merlin2::action;
using std::placeholders::_1;

Merlin2BtAction::Merlin2BtAction(std::string a_name) : Merlin2Action(a_name) {

  // tree parms
  std::string bt_file_path =
      this->declare_parameter("bt_file_path", "tree.xml");
  std::vector<std::string> plugins =
      this->declare_parameter("plugins", std::vector<std::string>());
  this->tick_rate = this->declare_parameter("tick_rate", 1);

  // groot params
  bool enable_groot_monitoring =
      this->declare_parameter("enable_groot_monitoring", true);
  int max_msg_per_second = this->declare_parameter("max_msg_per_second", 25);
  int publisher_port = this->declare_parameter("publisher_port", 1666);
  int server_port = this->declare_parameter("server_port", 1667);

  // load plugins
  RCLCPP_INFO(this->get_logger(), "Loading plugins");
  for (const auto &p : plugins) {
    this->bt_factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  // load tree
  this->blackboard = BT::Blackboard::create();
  std::shared_ptr<simple_node::Node> shared_this((simple_node::Node *)this);
  this->blackboard->set("node", shared_from_this());

  RCLCPP_INFO(this->get_logger(), "Loading tree from file: %s",
              bt_file_path.c_str());
  this->tree = std::make_shared<BT::Tree>(
      this->bt_factory.createTreeFromFile(bt_file_path, this->blackboard));

  // groot
  if (enable_groot_monitoring) {
    this->groot_monitor = std::make_unique<BT::PublisherZMQ>(
        *this->tree, max_msg_per_second, publisher_port, server_port);
  }
}

bool Merlin2BtAction::run_action(merlin2_msgs::msg::PlanAction goal) {

  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  this->blackboard->set("merlin2_action_goal", goal);
  rclcpp::Rate loop_rate(this->tick_rate);

  while (result == BT::NodeStatus::RUNNING) {
    try {
      result = this->tree->rootNode()->executeTick();
    } catch (const BT::LogicError &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (const BT::RuntimeError &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    }
    loop_rate.sleep();
  }

  return result == BT::NodeStatus::SUCCESS;
}

void Merlin2BtAction::cancel_action() { this->cancel_tree(); }

void Merlin2BtAction::cancel_tree() { this->tree->haltTree(); }