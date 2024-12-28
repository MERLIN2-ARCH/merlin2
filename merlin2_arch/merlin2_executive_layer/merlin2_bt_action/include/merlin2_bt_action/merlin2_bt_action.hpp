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

  bool run_action(merlin2_msgs::msg::PlanAction goal) override;
  virtual void cancel_action() override;
  void cancel_tree();

protected:
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory bt_factory;
  std::unique_ptr<BT::PublisherZMQ> groot_monitor;

private:
  std::shared_ptr<BT::Tree> tree;
  int tick_rate;
};

} // namespace action
} // namespace merlin2

#endif