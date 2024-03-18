// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <vector>

#include "plansys2_pddl_parser/Utils.h"

#include "plansys2_executor/ActionExecutor.hpp"

namespace plansys2
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ActionExecutor::ActionExecutor(
  const std::string & action,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(node), state_(IDLE), completion_(0.0)
{
  action_hub_pub_ = node_->create_publisher<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());
  action_hub_sub_ = node_->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    std::bind(&ActionExecutor::action_hub_callback, this, _1));

  RCLCPP_INFO(node->get_logger(), "Early timeout: %f", node->get_parameter("early_timeout").as_double());
  RCLCPP_INFO(node->get_logger(), "Late timeout: %f", node->get_parameter("late_timeout").as_double());
  state_time_ = node_->now();

  action_ = action;
  action_name_ = get_name(action);
  action_params_ = get_params(action);
  start_execution_ = node_->now();
  state_time_ = start_execution_;

  early_timeout_ = node->get_parameter("early_timeout").as_double();
  late_timeout_ = node->get_parameter("late_timeout").as_double();
  confidence_quantile_ = node->get_parameter("confidence_quantile").as_double();

}

void
ActionExecutor::action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  last_msg = *msg;

  switch (msg->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
    case plansys2_msgs::msg::ActionExecution::REJECT:
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      // These cases have no meaning requester
      break;
    case plansys2_msgs::msg::ActionExecution::WAIT:
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
      if (msg->arguments == action_params_ && msg->action == action_name_) {
        involved_auction_nodes_[msg->node_id] = *msg;
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
      if (state_ != RUNNING || msg->arguments != action_params_ || msg->action != action_name_ ||
        msg->node_id != current_performer_id_)
      {
        return;
      }
      feedback_ = msg->status;
      completion_ = msg->completion;
      state_time_ = node_->now();

      break;
    case plansys2_msgs::msg::ActionExecution::FINISH:
      if (msg->arguments == action_params_ &&
        msg->action == action_name_ && msg->node_id == current_performer_id_)
      {
        if (msg->success) {
          state_ = SUCCESS;
        } else {
          state_ = FAILURE;
        }

        feedback_ = msg->status;
        completion_ = msg->completion;

        state_time_ = node_->now();

        action_hub_pub_->on_deactivate();
        action_hub_pub_ = nullptr;
        action_hub_sub_ = nullptr;
        involved_auction_nodes_ = {};
      }
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Msg %d type not recognized in %s executor requester",
        msg->type, action_.c_str());
      break;
  }
}

void
ActionExecutor::confirm_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CONFIRM;
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;
  RCLCPP_INFO(node_->get_logger(), "Confirming %s to %s", action_.c_str(), node_id.c_str());
  action_hub_pub_->publish(msg);
}

void
ActionExecutor::reject_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REJECT;
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;
  RCLCPP_INFO(node_->get_logger(), "Rejecting %s to %s", action_.c_str(), node_id.c_str());
  action_hub_pub_->publish(msg);
}

void
ActionExecutor::request_for_performers()
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REQUEST;
  msg.node_id = node_->get_name();
  msg.action = action_name_;
  msg.arguments = action_params_;
  RCLCPP_INFO(node_->get_logger(), "Requesting %s", action_.c_str());
  action_hub_pub_->publish(msg);
}

BT::NodeStatus
ActionExecutor::get_status()
{
  switch (state_) {
    case IDLE:
      return BT::NodeStatus::IDLE;
      break;
    case DEALING:
    case RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
      break;
    case FAILURE:
      return BT::NodeStatus::FAILURE;
      break;
    default:
      return BT::NodeStatus::IDLE;
      break;
  }
}

bool
ActionExecutor::is_finished()
{
  return state_ == SUCCESS || state_ == FAILURE;
}

BT::NodeStatus
ActionExecutor::tick(const rclcpp::Time & now)
{
  switch (state_) {
    case IDLE:
      state_ = DEALING;
      state_time_ = node_->now();

      action_hub_pub_->on_activate();

      completion_ = 0.0;
      feedback_ = "";

      request_for_performers();
      waiting_timer_ = node_->create_wall_timer(
        1s, std::bind(&ActionExecutor::wait_timeout, this));
      break;
    case DEALING:
      {
        auto time_since_dealing = (node_->now() - state_time_).seconds();
        RCLCPP_INFO(node_->get_logger(), "Time since dealing: %f", time_since_dealing);

        if(time_since_dealing > late_timeout_){
          RCLCPP_INFO(node_->get_logger(), "Time since dealing greater than late timeout");
          if(at_least_one_node_responded_with_finite_cost()){
            RCLCPP_INFO(node_->get_logger(), "Managing auction closure due to late timeout.");  
            manage_auction_closure();
          }
          else{
            RCLCPP_ERROR(
                         node_->get_logger(),
                         "Aborting %s. Timeout after requesting for %f seconds", action_.c_str(),
                         late_timeout_);
            state_ = FAILURE;
          }
        }
        else if(time_since_dealing > early_timeout_ && all_nodes_already_responded() && at_least_one_node_has_finite_cost()){
          RCLCPP_INFO(node_->get_logger(), "Managing auction closure due to early timeout.");
          manage_auction_closure();
        }
        else{
          RCLCPP_INFO(node_->get_logger(), "Waiting for performers to respond.");
          //waiting agents performers responses
        }
      }
      break;
    case RUNNING:
      break;
    case SUCCESS:
    case FAILURE:
    case CANCELLED:
      break;
    default:
      break;
  }

  return get_status();
}

void
ActionExecutor::cancel()
{
  state_ = CANCELLED;
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CANCEL;
  msg.node_id = current_performer_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

std::string
ActionExecutor::get_name(const std::string & action_expr)
{
  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

std::vector<std::string>
ActionExecutor::get_params(const std::string & action_expr)
{
  std::vector<std::string> ret;

  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  working_action_expr = working_action_expr.substr(delim + 1);

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = working_action_expr.find(" ", start);
    auto param = working_action_expr.substr(
      start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  
  return ret;
}

void
ActionExecutor::wait_timeout()
{
  // RCLCPP_WARN(node_->get_logger(), "No action performer for %s. retrying", action_.c_str());
  // request_for_performers();
}

// std::string
// ActionExecutor::solve_auction()
// {
//   std::map<std::string, plansys2_msgs::msg::ActionExecution> response_action_executions;
//   std::copy_if(involved_auction_nodes_.begin(), involved_auction_nodes_.end(), std::inserter(response_action_executions, response_action_executions.end()),
//         [](const auto& involved_node) {
//             return involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE;
//         });
//   // plansys2_msgs::msg::ActionExecution cheapes_action_execution;
//   auto cheapes_action_response = std::min_element(response_action_executions.begin(), response_action_executions.end(),
//     [](const auto& lhs, const auto& rhs) {
//         return lhs.second.action_cost.nominal_cost < rhs.second.action_cost.nominal_cost;
//     });

//   return (*cheapes_action_response).first;
// }

// plansys2::msg::ActionExecution
// ActionExecutor::solve_auction()
// {
//   std::map<std::string, plansys2_msgs::msg::ActionExecution> response_action_executions;
//   std::copy_if(involved_auction_nodes_.begin(), involved_auction_nodes_.end(), std::inserter(response_action_executions, response_action_executions.end()),
//         [](const auto& involved_node) {
//             return involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE;
//         });
//   // plansys2_msgs::msg::ActionExecution cheapes_action_execution;
//   auto cheapes_action_response = std::min_element(response_action_executions.begin(), response_action_executions.end(),
//     [](const auto& lhs, const auto& rhs) {
//         return lhs.second.action_cost.nominal_cost < rhs.second.action_cost.nominal_cost;
//     });

//   return (*cheapes_action_response).second;
// }

plansys2_msgs::msg::ActionExecution::SharedPtr
ActionExecutor::solve_auction()
{
  RCLCPP_INFO( node_->get_logger(), "Solving auction for %s", action_.c_str());
  std::map<std::string, plansys2_msgs::msg::ActionExecution> response_action_executions;
  std::copy_if(involved_auction_nodes_.begin(), involved_auction_nodes_.end(), std::inserter(response_action_executions, response_action_executions.end()),
        [](const auto& involved_node) {
            return involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE;
        });
  // plansys2_msgs::msg::ActionExecution cheapes_action_execution;
  auto cheapest_action_execution = std::min_element(response_action_executions.begin(), response_action_executions.end(),
    [this](const auto& lhs, const auto& rhs) {
      RCLCPP_INFO(node_->get_logger(), "Comparing %s with %s", lhs.first.c_str(), rhs.first.c_str());
      RCLCPP_INFO(node_->get_logger(), "Confidence %f", confidence_quantile_);
      RCLCPP_INFO(node_->get_logger(), "Std 1 %f", lhs.second.action_cost.std_dev_cost);
      RCLCPP_INFO(node_->get_logger(), "Std 2 %f", rhs.second.action_cost.std_dev_cost);  
      
        return lhs.second.action_cost.nominal_cost + confidence_quantile_ * lhs.second.action_cost.std_dev_cost < rhs.second.action_cost.nominal_cost + confidence_quantile_ * rhs.second.action_cost.std_dev_cost;
    });

  return std::make_shared<plansys2_msgs::msg::ActionExecution>((*cheapest_action_execution).second);
}


bool 
ActionExecutor::all_nodes_already_responded()
{
  return std::all_of(involved_auction_nodes_.begin(), involved_auction_nodes_.end(),
    [](const auto& involved_node) {
        return (involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE);
    });
}

bool 
ActionExecutor::at_least_one_node_already_responded()
{
  return std::any_of(involved_auction_nodes_.begin(), involved_auction_nodes_.end(),
    [](const auto& involved_node) {
        return (involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE);
    });
}

bool 
ActionExecutor::at_least_one_node_responded_with_finite_cost()
{
  // for(const auto& involved_node : involved_auction_nodes_){
  //   RCLCPP_INFO(node_->get_logger(), "Node %s has cost %f", involved_node.first.c_str(), involved_node.second.action_cost.nominal_cost);
  //   RCLCPP_INFO(node_->get_logger(), "Node %s has type %d", involved_node.first.c_str(), involved_node.second.type);
  // }
  return std::any_of(involved_auction_nodes_.begin(), involved_auction_nodes_.end(),
    [](const auto& involved_node) {
        return (involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE && involved_node.second.action_cost.nominal_cost != std::numeric_limits<double>::infinity());
    });
}

bool
ActionExecutor::at_least_one_node_has_finite_cost()
{
  return std::any_of(involved_auction_nodes_.begin(), involved_auction_nodes_.end(),
    [](const auto& involved_node) {
        return (involved_node.second.type == plansys2_msgs::msg::ActionExecution::RESPONSE && involved_node.second.action_cost.nominal_cost != std::numeric_limits<double>::infinity());
    });

}

void ActionExecutor::reject_others_performers(const std::string & performer_node_id)
{
  std::for_each(involved_auction_nodes_.begin(), involved_auction_nodes_.end(),
  [performer_node_id,this](const auto& involved_node) {
      if (involved_node.second.node_id != performer_node_id) {
        this->reject_performer(involved_node.second.node_id);
      }
  });
}

void ActionExecutor::manage_auction_closure()
{
  plansys2_msgs::msg::ActionExecution::SharedPtr performer = solve_auction(); 
  confirm_performer(performer->node_id);
  reject_others_performers(performer->node_id);
  current_performer_id_ = performer->node_id;
  state_ = RUNNING;
  waiting_timer_ = nullptr;
  start_execution_ = node_->now();
  state_time_ = node_->now();
}

}  // namespace plansys2
