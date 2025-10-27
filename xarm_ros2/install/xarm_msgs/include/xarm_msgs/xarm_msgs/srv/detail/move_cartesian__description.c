// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/MoveCartesian.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/move_cartesian__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCartesian__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x99, 0xc7, 0x2a, 0x73, 0xd7, 0x7b, 0x6e,
      0x25, 0x61, 0xa8, 0x59, 0xbf, 0xb5, 0xcd, 0x15,
      0x91, 0x8d, 0xec, 0xed, 0x61, 0x96, 0x36, 0xcf,
      0xcd, 0xc4, 0x49, 0x4d, 0x84, 0x61, 0xcf, 0x05,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCartesian_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x52, 0x27, 0x57, 0x99, 0x41, 0x58, 0xb8,
      0x1e, 0x60, 0x54, 0x66, 0xab, 0x98, 0x86, 0x6a,
      0x39, 0x07, 0x5f, 0x32, 0xfd, 0x87, 0x77, 0x08,
      0x15, 0xe1, 0xef, 0xdc, 0x1d, 0x7a, 0xd5, 0xdb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCartesian_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4f, 0x73, 0x9a, 0x66, 0x31, 0x93, 0x8c, 0x60,
      0x76, 0xf6, 0x79, 0xe6, 0xd9, 0x58, 0x4f, 0x75,
      0x66, 0xf9, 0x37, 0xec, 0x95, 0xde, 0xfa, 0x75,
      0xd5, 0xad, 0x72, 0x05, 0xdd, 0xf7, 0x7e, 0x78,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCartesian_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0xa5, 0x64, 0x29, 0x89, 0xfa, 0x81, 0xc7,
      0x5a, 0x09, 0x50, 0xb9, 0x5e, 0xe0, 0x7a, 0xc7,
      0x02, 0xc9, 0xe9, 0x20, 0x02, 0x49, 0x03, 0xa8,
      0x07, 0x1e, 0x86, 0xbd, 0x4e, 0x16, 0x2b, 0xe5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char xarm_msgs__srv__MoveCartesian__TYPE_NAME[] = "xarm_msgs/srv/MoveCartesian";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__MoveCartesian_Event__TYPE_NAME[] = "xarm_msgs/srv/MoveCartesian_Event";
static char xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME[] = "xarm_msgs/srv/MoveCartesian_Request";
static char xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME[] = "xarm_msgs/srv/MoveCartesian_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCartesian__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__MoveCartesian__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__MoveCartesian__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCartesian__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCartesian__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCartesian_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveCartesian__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCartesian__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCartesian__TYPE_NAME, 27, 27},
      {xarm_msgs__srv__MoveCartesian__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveCartesian__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveCartesian_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveCartesian_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__MoveCartesian_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__pose[] = "pose";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__speed[] = "speed";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__speed[] = "0.0";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__acc[] = "acc";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__acc[] = "0.0";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__mvtime[] = "mvtime";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__mvtime[] = "0.0";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__wait[] = "wait";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__wait[] = "False";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__timeout[] = "timeout";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__timeout[] = "-1.0";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__radius[] = "radius";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__radius[] = "-1.0";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__is_tool_coord[] = "is_tool_coord";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__is_tool_coord[] = "False";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__relative[] = "relative";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__relative[] = "False";
static char xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__motion_type[] = "motion_type";
static char xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__motion_type[] = "0";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCartesian_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__speed, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__acc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__acc, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__mvtime, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__mvtime, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__wait, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__wait, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__timeout, 4, 4},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__radius, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__radius, 4, 4},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__is_tool_coord, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__is_tool_coord, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__relative, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__relative, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__FIELD_NAME__motion_type, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCartesian_Request__DEFAULT_VALUE__motion_type, 1, 1},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCartesian_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
      {xarm_msgs__srv__MoveCartesian_Request__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCartesian_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__MoveCartesian_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCartesian_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCartesian_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCartesian_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
      {xarm_msgs__srv__MoveCartesian_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCartesian_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveCartesian_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCartesian_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCartesian_Event__TYPE_NAME, 33, 33},
      {xarm_msgs__srv__MoveCartesian_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveCartesian_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveCartesian_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveCartesian_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - set_position\n"
  "#   - set_tool_position\n"
  "#   - set_position_aa\n"
  "#   - set_servo_cartesian\n"
  "#   - set_servo_cartesian_aa\n"
  "\n"
  "# set_position/set_tool_position/set_position_aa/set_servo_cartesian/set_servo_cartesian_aa\n"
  "float32[] pose\n"
  "float32 speed       0\n"
  "float32 acc         0\n"
  "float32 mvtime      0\n"
  "\n"
  "# set_position/set_position_aa/set_tool_position\n"
  "bool wait           false\n"
  "float32 timeout     -1\n"
  "\n"
  "# set_position/set_tool_position/set_position_aa\n"
  "float32 radius      -1\n"
  "\n"
  "# set_position_aa/set_servo_cartesian/set_servo_cartesian_aa\n"
  "bool is_tool_coord  false\n"
  "\n"
  "# set_position_aa/set_servo_cartesian_aa\n"
  "bool relative       false\n"
  "\n"
  "# set_position/set_tool_position/set_position_aa\n"
  "#   motion_type == 0: default, linear planning\n"
  "#   motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible\n"
  "#   motion_type == 2: direct transfer to IK using joint planning\n"
  "#   Note: \n"
  "#       1. only available if firmware_version >= 1.11.100\n"
  "#       2. when motion_type is 1 or 2, linear motion cannot be guaranteed\n"
  "#       3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage\n"
  "#           speed = speed / max_tcp_speed * max_joint_speed\n"
  "#           acc = acc / max_tcp_acc * max_joint_acc\n"
  "#       4. if there is no suitable IK, a C40 error will be triggered\n"
  "uint8 motion_type 0\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCartesian__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCartesian__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 1511, 1511},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCartesian_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCartesian_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCartesian_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCartesian_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCartesian_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCartesian_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCartesian__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCartesian__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveCartesian_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveCartesian_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__MoveCartesian_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCartesian_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCartesian_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCartesian_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCartesian_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCartesian_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCartesian_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveCartesian_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveCartesian_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
