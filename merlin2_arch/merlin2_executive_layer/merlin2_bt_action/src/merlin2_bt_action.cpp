
#include <vector>

#include "merlin2_bt_action/merlin2_bt_action.hpp"

using namespace merlin2;
using std::placeholders::_1;

Merlin2BtAction::Merlin2BtAction(std::string a_name) : Merlin2Action(a_name) {

  // tree parms
  std::string bt_file_path =
      this->declare_parameter("bt_file_path", "tree.xml");
  std::vector<std::string> plugins =
      this->declare_parameter("plugins", std::vector<std::string>());

  // groot params
  int max_msg_per_second = this->declare_parameter("max_msg_per_second", 25);
  int publisher_port = this->declare_parameter("publisher_port", 1666);
  int server_port = this->declare_parameter("server_port", 1667);

  // load tree
  this->blackboard = BT::Blackboard::create();
  this->blackboard->set("node", shared_from_this());

  RCLCPP_INFO(this->get_logger(), "Loading tree from file");
  this->tree = std::make_shared<BT::Tree>(
      this->bt_factory.createTreeFromFile(bt_file_path, this->blackboard));

  RCLCPP_INFO(this->get_logger(), "Loading plugins");
  for (const auto &p : plugins) {
    this->bt_factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  // groot
  this->groot_monitor = std::make_unique<BT::PublisherZMQ>(
      *this->tree, max_msg_per_second, publisher_port, server_port);
}

bool Merlin2BtAction::run_action(
    merlin2_arch_interfaces::msg::PlanAction goal) {

  bool finished = false;
  BT::NodeStatus result;
  this->blackboard->set("merlin2_action_goal", goal);

  while (!finished) {
    try {
      result = this->tree->rootNode()->executeTick();
    } catch (const BT::LogicError &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (const BT::RuntimeError &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (const std::exception &e) {
    }

    switch (result) {
    case BT::NodeStatus::IDLE:
    case BT::NodeStatus::SUCCESS:
    case BT::NodeStatus::FAILURE:
      finished = true;
      break;
    case BT::NodeStatus::RUNNING:
      finished = true;
      break;
    }
  }

  return result == BT::NodeStatus::SUCCESS;
}

void Merlin2BtAction::cancel_action() { this->tree->haltTree(); }