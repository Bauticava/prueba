// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from xarm_msgs:srv/FtCaliLoad.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "xarm_msgs/srv/ft_cali_load.hpp"


#ifndef XARM_MSGS__SRV__DETAIL__FT_CALI_LOAD__STRUCT_HPP_
#define XARM_MSGS__SRV__DETAIL__FT_CALI_LOAD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Request __attribute__((deprecated))
#else
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Request __declspec(deprecated)
#endif

namespace xarm_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FtCaliLoad_Request_
{
  using Type = FtCaliLoad_Request_<ContainerAllocator>;

  explicit FtCaliLoad_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->association_setting_tcp_load = false;
      this->m = 0.325f;
      this->x = -17.0f;
      this->y = 9.0f;
      this->z = 11.8f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->association_setting_tcp_load = false;
      this->m = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  explicit FtCaliLoad_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->association_setting_tcp_load = false;
      this->m = 0.325f;
      this->x = -17.0f;
      this->y = 9.0f;
      this->z = 11.8f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->association_setting_tcp_load = false;
      this->m = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  // field types and members
  using _datas_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _datas_type datas;
  using _association_setting_tcp_load_type =
    bool;
  _association_setting_tcp_load_type association_setting_tcp_load;
  using _m_type =
    float;
  _m_type m;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;

  // setters for named parameter idiom
  Type & set__datas(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->datas = _arg;
    return *this;
  }
  Type & set__association_setting_tcp_load(
    const bool & _arg)
  {
    this->association_setting_tcp_load = _arg;
    return *this;
  }
  Type & set__m(
    const float & _arg)
  {
    this->m = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Request
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Request
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FtCaliLoad_Request_ & other) const
  {
    if (this->datas != other.datas) {
      return false;
    }
    if (this->association_setting_tcp_load != other.association_setting_tcp_load) {
      return false;
    }
    if (this->m != other.m) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const FtCaliLoad_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FtCaliLoad_Request_

// alias to use template instance with default allocator
using FtCaliLoad_Request =
  xarm_msgs::srv::FtCaliLoad_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xarm_msgs


#ifndef _WIN32
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Response __attribute__((deprecated))
#else
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Response __declspec(deprecated)
#endif

namespace xarm_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FtCaliLoad_Response_
{
  using Type = FtCaliLoad_Response_<ContainerAllocator>;

  explicit FtCaliLoad_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ret = 0;
      this->message = "";
    }
  }

  explicit FtCaliLoad_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ret = 0;
      this->message = "";
    }
  }

  // field types and members
  using _ret_type =
    int16_t;
  _ret_type ret;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__ret(
    const int16_t & _arg)
  {
    this->ret = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Response
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Response
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FtCaliLoad_Response_ & other) const
  {
    if (this->ret != other.ret) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const FtCaliLoad_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FtCaliLoad_Response_

// alias to use template instance with default allocator
using FtCaliLoad_Response =
  xarm_msgs::srv::FtCaliLoad_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xarm_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Event __attribute__((deprecated))
#else
# define DEPRECATED__xarm_msgs__srv__FtCaliLoad_Event __declspec(deprecated)
#endif

namespace xarm_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FtCaliLoad_Event_
{
  using Type = FtCaliLoad_Event_<ContainerAllocator>;

  explicit FtCaliLoad_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit FtCaliLoad_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xarm_msgs::srv::FtCaliLoad_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<xarm_msgs::srv::FtCaliLoad_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Event
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__xarm_msgs__srv__FtCaliLoad_Event
    std::shared_ptr<xarm_msgs::srv::FtCaliLoad_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FtCaliLoad_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const FtCaliLoad_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FtCaliLoad_Event_

// alias to use template instance with default allocator
using FtCaliLoad_Event =
  xarm_msgs::srv::FtCaliLoad_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace xarm_msgs

namespace xarm_msgs
{

namespace srv
{

struct FtCaliLoad
{
  using Request = xarm_msgs::srv::FtCaliLoad_Request;
  using Response = xarm_msgs::srv::FtCaliLoad_Response;
  using Event = xarm_msgs::srv::FtCaliLoad_Event;
};

}  // namespace srv

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__FT_CALI_LOAD__STRUCT_HPP_
