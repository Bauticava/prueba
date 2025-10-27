// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/PlanPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "xarm_msgs/srv/plan_pose.hpp"


#ifndef XARM_MSGS__SRV__DETAIL__PLAN_POSE__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__PLAN_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/plan_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanPose_Request_target
{
public:
  Init_PlanPose_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xarm_msgs::srv::PlanPose_Request target(::xarm_msgs::srv::PlanPose_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::PlanPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::PlanPose_Request>()
{
  return xarm_msgs::srv::builder::Init_PlanPose_Request_target();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanPose_Response_success
{
public:
  Init_PlanPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xarm_msgs::srv::PlanPose_Response success(::xarm_msgs::srv::PlanPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::PlanPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::PlanPose_Response>()
{
  return xarm_msgs::srv::builder::Init_PlanPose_Response_success();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_PlanPose_Event_response
{
public:
  explicit Init_PlanPose_Event_response(::xarm_msgs::srv::PlanPose_Event & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::PlanPose_Event response(::xarm_msgs::srv::PlanPose_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::PlanPose_Event msg_;
};

class Init_PlanPose_Event_request
{
public:
  explicit Init_PlanPose_Event_request(::xarm_msgs::srv::PlanPose_Event & msg)
  : msg_(msg)
  {}
  Init_PlanPose_Event_response request(::xarm_msgs::srv::PlanPose_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlanPose_Event_response(msg_);
  }

private:
  ::xarm_msgs::srv::PlanPose_Event msg_;
};

class Init_PlanPose_Event_info
{
public:
  Init_PlanPose_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanPose_Event_request info(::xarm_msgs::srv::PlanPose_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlanPose_Event_request(msg_);
  }

private:
  ::xarm_msgs::srv::PlanPose_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::PlanPose_Event>()
{
  return xarm_msgs::srv::builder::Init_PlanPose_Event_info();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__PLAN_POSE__BUILDER_HPP_
