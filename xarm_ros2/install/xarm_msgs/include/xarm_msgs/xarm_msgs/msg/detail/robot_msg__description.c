// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:msg/RobotMsg.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/msg/detail/robot_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__msg__RobotMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfd, 0xb1, 0x7c, 0x24, 0x94, 0x1f, 0xbb, 0xe2,
      0x77, 0x3b, 0x89, 0x96, 0x04, 0xde, 0x97, 0x87,
      0x80, 0xd6, 0xc9, 0xf7, 0x2b, 0xda, 0x69, 0x5f,
      0x87, 0xbb, 0x1d, 0xce, 0x26, 0xb9, 0xcd, 0x49,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char xarm_msgs__msg__RobotMsg__TYPE_NAME[] = "xarm_msgs/msg/RobotMsg";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__header[] = "header";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__state[] = "state";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__mode[] = "mode";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__cmdnum[] = "cmdnum";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__mt_brake[] = "mt_brake";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__mt_able[] = "mt_able";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__err[] = "err";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__warn[] = "warn";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__angle[] = "angle";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__pose[] = "pose";
static char xarm_msgs__msg__RobotMsg__FIELD_NAME__offset[] = "offset";

static rosidl_runtime_c__type_description__Field xarm_msgs__msg__RobotMsg__FIELDS[] = {
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__cmdnum, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__mt_brake, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__mt_able, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__err, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__warn, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__RobotMsg__FIELD_NAME__offset, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__msg__RobotMsg__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__msg__RobotMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__msg__RobotMsg__TYPE_NAME, 22, 22},
      {xarm_msgs__msg__RobotMsg__FIELDS, 11, 11},
    },
    {xarm_msgs__msg__RobotMsg__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "# feedback information of the controlled robot\n"
  "\n"
  "# state of robot:\n"
  "#\\t1: RUNNING, executing motion command.\n"
  "#\\t2: SLEEPING, not in execution, but ready to move.\n"
  "#\\t3: PAUSED, paused in the middle of unfinished motion.\n"
  "#\\t4: STOPPED, not ready for any motion commands.\n"
  "#\\t5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.\n"
  "int16 state\n"
  "\n"
  "# mode of robot:\n"
  "#\\t0 for POSITION mode.(position control by xarm controller box, execute api standard commands)\n"
  "#\\t1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)\n"
  "#\\t2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)\n"
  "int16 mode\n"
  "\n"
  "# cmdnum: number of commands waiting in the buffer.\n"
  "int16 cmdnum\n"
  "\n"
  "# mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled\n"
  "int16 mt_brake\n"
  "\n"
  "# mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled\n"
  "int16 mt_able\n"
  "\n"
  "# error code (if non-zero)\n"
  "int16 err\n"
  "\n"
  "# warning code (if non-zero)\n"
  "int16 warn\n"
  "\n"
  "# current joint angles expressed in radian.\n"
  "float32[] angle\n"
  "\n"
  "# current TCP Cartesian position expressed in mm(position), radian(orientation)  \n"
  "float32[6] pose\n"
  "\n"
  "# TCP offset from center of flange, with respect to tool frame.\n"
  "float32[6] offset";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__msg__RobotMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__msg__RobotMsg__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1343, 1343},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__msg__RobotMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__msg__RobotMsg__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
