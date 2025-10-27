// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/Call.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "xarm_msgs/srv/call.hpp"


#ifndef XARM_MSGS__SRV__DETAIL__CALL__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__CALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/call__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xarm_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::Call_Request>()
{
  return ::xarm_msgs::srv::Call_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_Call_Response_message
{
public:
  explicit Init_Call_Response_message(::xarm_msgs::srv::Call_Response & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::Call_Response message(::xarm_msgs::srv::Call_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::Call_Response msg_;
};

class Init_Call_Response_ret
{
public:
  Init_Call_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Call_Response_message ret(::xarm_msgs::srv::Call_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_Call_Response_message(msg_);
  }

private:
  ::xarm_msgs::srv::Call_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::Call_Response>()
{
  return xarm_msgs::srv::builder::Init_Call_Response_ret();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_Call_Event_response
{
public:
  explicit Init_Call_Event_response(::xarm_msgs::srv::Call_Event & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::Call_Event response(::xarm_msgs::srv::Call_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::Call_Event msg_;
};

class Init_Call_Event_request
{
public:
  explicit Init_Call_Event_request(::xarm_msgs::srv::Call_Event & msg)
  : msg_(msg)
  {}
  Init_Call_Event_response request(::xarm_msgs::srv::Call_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Call_Event_response(msg_);
  }

private:
  ::xarm_msgs::srv::Call_Event msg_;
};

class Init_Call_Event_info
{
public:
  Init_Call_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Call_Event_request info(::xarm_msgs::srv::Call_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Call_Event_request(msg_);
  }

private:
  ::xarm_msgs::srv::Call_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::Call_Event>()
{
  return xarm_msgs::srv::builder::Init_Call_Event_info();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__CALL__BUILDER_HPP_
