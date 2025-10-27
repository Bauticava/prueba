// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from xarm_msgs:msg/MoveVelocity.idl
// generated code does not contain a copyright notice
#ifndef XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/msg/detail/move_velocity__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
bool cdr_serialize_xarm_msgs__msg__MoveVelocity(
  const xarm_msgs__msg__MoveVelocity * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
bool cdr_deserialize_xarm_msgs__msg__MoveVelocity(
  eprosima::fastcdr::Cdr &,
  xarm_msgs__msg__MoveVelocity * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__msg__MoveVelocity(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__msg__MoveVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
bool cdr_serialize_key_xarm_msgs__msg__MoveVelocity(
  const xarm_msgs__msg__MoveVelocity * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_key_xarm_msgs__msg__MoveVelocity(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_key_xarm_msgs__msg__MoveVelocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, msg, MoveVelocity)();

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
