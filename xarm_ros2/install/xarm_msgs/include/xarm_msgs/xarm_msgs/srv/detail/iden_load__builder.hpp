// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/IdenLoad.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "xarm_msgs/srv/iden_load.hpp"


#ifndef XARM_MSGS__SRV__DETAIL__IDEN_LOAD__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__IDEN_LOAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/iden_load__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_IdenLoad_Request_estimated_mass
{
public:
  Init_IdenLoad_Request_estimated_mass()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xarm_msgs::srv::IdenLoad_Request estimated_mass(::xarm_msgs::srv::IdenLoad_Request::_estimated_mass_type arg)
  {
    msg_.estimated_mass = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::IdenLoad_Request>()
{
  return xarm_msgs::srv::builder::Init_IdenLoad_Request_estimated_mass();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_IdenLoad_Response_datas
{
public:
  explicit Init_IdenLoad_Response_datas(::xarm_msgs::srv::IdenLoad_Response & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::IdenLoad_Response datas(::xarm_msgs::srv::IdenLoad_Response::_datas_type arg)
  {
    msg_.datas = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Response msg_;
};

class Init_IdenLoad_Response_message
{
public:
  explicit Init_IdenLoad_Response_message(::xarm_msgs::srv::IdenLoad_Response & msg)
  : msg_(msg)
  {}
  Init_IdenLoad_Response_datas message(::xarm_msgs::srv::IdenLoad_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_IdenLoad_Response_datas(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Response msg_;
};

class Init_IdenLoad_Response_ret
{
public:
  Init_IdenLoad_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IdenLoad_Response_message ret(::xarm_msgs::srv::IdenLoad_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_IdenLoad_Response_message(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::IdenLoad_Response>()
{
  return xarm_msgs::srv::builder::Init_IdenLoad_Response_ret();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_IdenLoad_Event_response
{
public:
  explicit Init_IdenLoad_Event_response(::xarm_msgs::srv::IdenLoad_Event & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::IdenLoad_Event response(::xarm_msgs::srv::IdenLoad_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Event msg_;
};

class Init_IdenLoad_Event_request
{
public:
  explicit Init_IdenLoad_Event_request(::xarm_msgs::srv::IdenLoad_Event & msg)
  : msg_(msg)
  {}
  Init_IdenLoad_Event_response request(::xarm_msgs::srv::IdenLoad_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_IdenLoad_Event_response(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Event msg_;
};

class Init_IdenLoad_Event_info
{
public:
  Init_IdenLoad_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IdenLoad_Event_request info(::xarm_msgs::srv::IdenLoad_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_IdenLoad_Event_request(msg_);
  }

private:
  ::xarm_msgs::srv::IdenLoad_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::IdenLoad_Event>()
{
  return xarm_msgs::srv::builder::Init_IdenLoad_Event_info();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__IDEN_LOAD__BUILDER_HPP_
