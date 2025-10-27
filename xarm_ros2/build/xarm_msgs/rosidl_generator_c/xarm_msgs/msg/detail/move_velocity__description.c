// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:msg/MoveVelocity.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/msg/detail/move_velocity__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__msg__MoveVelocity__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcb, 0x58, 0x6a, 0x62, 0xa8, 0x20, 0x62, 0xa5,
      0x7c, 0x0a, 0x37, 0xf4, 0xd9, 0x4b, 0xd8, 0x5a,
      0x15, 0x83, 0xf6, 0xad, 0xab, 0x5d, 0x9d, 0x46,
      0xce, 0x1d, 0x8d, 0x0f, 0x86, 0x02, 0xb2, 0xaa,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char xarm_msgs__msg__MoveVelocity__TYPE_NAME[] = "xarm_msgs/msg/MoveVelocity";

// Define type names, field names, and default values
static char xarm_msgs__msg__MoveVelocity__FIELD_NAME__speeds[] = "speeds";
static char xarm_msgs__msg__MoveVelocity__FIELD_NAME__is_sync[] = "is_sync";
static char xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__is_sync[] = "True";
static char xarm_msgs__msg__MoveVelocity__FIELD_NAME__is_tool_coord[] = "is_tool_coord";
static char xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__is_tool_coord[] = "False";
static char xarm_msgs__msg__MoveVelocity__FIELD_NAME__duration[] = "duration";
static char xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__duration[] = "-1.0";

static rosidl_runtime_c__type_description__Field xarm_msgs__msg__MoveVelocity__FIELDS[] = {
  {
    {xarm_msgs__msg__MoveVelocity__FIELD_NAME__speeds, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__MoveVelocity__FIELD_NAME__is_sync, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__is_sync, 4, 4},
  },
  {
    {xarm_msgs__msg__MoveVelocity__FIELD_NAME__is_tool_coord, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__is_tool_coord, 5, 5},
  },
  {
    {xarm_msgs__msg__MoveVelocity__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__msg__MoveVelocity__DEFAULT_VALUE__duration, 4, 4},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__msg__MoveVelocity__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__msg__MoveVelocity__TYPE_NAME, 26, 26},
      {xarm_msgs__msg__MoveVelocity__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following topic\n"
  "#   - vc_set_joint_velocity\n"
  "#   - vc_set_cartesian_velocity\n"
  "\n"
  "# vc_set_joint_velocity/vc_set_cartesian_velocity\n"
  "float32[] speeds\n"
  "\n"
  "# vc_set_joint_velocity\n"
  "bool is_sync        true\n"
  "\n"
  "# vc_set_cartesian_velocity\n"
  "bool is_tool_coord  false\n"
  "\n"
  "# the maximum duration of the spedd, over this time will automatically set the speed to 0\n"
  "#   duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained\n"
  "#   duration == 0: always effective, will not stop automativally\n"
  "#   duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0\n"
  "# avaiable for firmware_version >= 1.8.0\n"
  "float32 duration -1.0";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__msg__MoveVelocity__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__msg__MoveVelocity__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 702, 702},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__msg__MoveVelocity__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__msg__MoveVelocity__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
