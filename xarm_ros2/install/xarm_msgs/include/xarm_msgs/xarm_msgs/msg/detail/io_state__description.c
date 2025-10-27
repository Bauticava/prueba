// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/msg/detail/io_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__msg__IOState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2d, 0x64, 0xbc, 0x73, 0x93, 0xf6, 0xe9, 0xea,
      0x4e, 0x9d, 0x70, 0xbd, 0x5e, 0x73, 0x81, 0x9f,
      0xde, 0xb6, 0xec, 0x0b, 0x09, 0xa5, 0x55, 0xf6,
      0x3d, 0xe3, 0x60, 0x9a, 0xda, 0xf5, 0xaa, 0xc9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char xarm_msgs__msg__IOState__TYPE_NAME[] = "xarm_msgs/msg/IOState";

// Define type names, field names, and default values
static char xarm_msgs__msg__IOState__FIELD_NAME__digital_1[] = "digital_1";
static char xarm_msgs__msg__IOState__FIELD_NAME__digital_2[] = "digital_2";
static char xarm_msgs__msg__IOState__FIELD_NAME__analog_1[] = "analog_1";
static char xarm_msgs__msg__IOState__FIELD_NAME__analog_2[] = "analog_2";

static rosidl_runtime_c__type_description__Field xarm_msgs__msg__IOState__FIELDS[] = {
  {
    {xarm_msgs__msg__IOState__FIELD_NAME__digital_1, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__IOState__FIELD_NAME__digital_2, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__IOState__FIELD_NAME__analog_1, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__IOState__FIELD_NAME__analog_2, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__msg__IOState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__msg__IOState__TYPE_NAME, 21, 21},
      {xarm_msgs__msg__IOState__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# for indicating 2 digital and 2 analog Input port state\n"
  "\n"
  "int32 digital_1\n"
  "\n"
  "int32 digital_2\n"
  "\n"
  "float32 analog_1\n"
  "\n"
  "float32 analog_2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__msg__IOState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__msg__IOState__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 127, 127},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__msg__IOState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__msg__IOState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
