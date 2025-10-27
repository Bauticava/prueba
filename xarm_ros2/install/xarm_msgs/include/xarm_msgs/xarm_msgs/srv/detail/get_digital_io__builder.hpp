// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/GetDigitalIO.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "xarm_msgs/srv/get_digital_io.hpp"


#ifndef XARM_MSGS__SRV__DETAIL__GET_DIGITAL_IO__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__GET_DIGITAL_IO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/get_digital_io__struct.hpp"
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
auto build<::xarm_msgs::srv::GetDigitalIO_Request>()
{
  return ::xarm_msgs::srv::GetDigitalIO_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_GetDigitalIO_Response_digitals
{
public:
  explicit Init_GetDigitalIO_Response_digitals(::xarm_msgs::srv::GetDigitalIO_Response & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::GetDigitalIO_Response digitals(::xarm_msgs::srv::GetDigitalIO_Response::_digitals_type arg)
  {
    msg_.digitals = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Response msg_;
};

class Init_GetDigitalIO_Response_message
{
public:
  explicit Init_GetDigitalIO_Response_message(::xarm_msgs::srv::GetDigitalIO_Response & msg)
  : msg_(msg)
  {}
  Init_GetDigitalIO_Response_digitals message(::xarm_msgs::srv::GetDigitalIO_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetDigitalIO_Response_digitals(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Response msg_;
};

class Init_GetDigitalIO_Response_ret
{
public:
  Init_GetDigitalIO_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDigitalIO_Response_message ret(::xarm_msgs::srv::GetDigitalIO_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_GetDigitalIO_Response_message(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::GetDigitalIO_Response>()
{
  return xarm_msgs::srv::builder::Init_GetDigitalIO_Response_ret();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_GetDigitalIO_Event_response
{
public:
  explicit Init_GetDigitalIO_Event_response(::xarm_msgs::srv::GetDigitalIO_Event & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::GetDigitalIO_Event response(::xarm_msgs::srv::GetDigitalIO_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Event msg_;
};

class Init_GetDigitalIO_Event_request
{
public:
  explicit Init_GetDigitalIO_Event_request(::xarm_msgs::srv::GetDigitalIO_Event & msg)
  : msg_(msg)
  {}
  Init_GetDigitalIO_Event_response request(::xarm_msgs::srv::GetDigitalIO_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetDigitalIO_Event_response(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Event msg_;
};

class Init_GetDigitalIO_Event_info
{
public:
  Init_GetDigitalIO_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetDigitalIO_Event_request info(::xarm_msgs::srv::GetDigitalIO_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetDigitalIO_Event_request(msg_);
  }

private:
  ::xarm_msgs::srv::GetDigitalIO_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::GetDigitalIO_Event>()
{
  return xarm_msgs::srv::builder::Init_GetDigitalIO_Event_info();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__GET_DIGITAL_IO__BUILDER_HPP_
