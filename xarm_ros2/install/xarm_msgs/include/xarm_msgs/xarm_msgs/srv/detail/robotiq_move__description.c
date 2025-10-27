// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/RobotiqMove.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/robotiq_move__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqMove__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa2, 0x5f, 0xef, 0xbf, 0x86, 0xf3, 0x7f, 0x13,
      0xa9, 0x52, 0xf2, 0xa2, 0xcb, 0x36, 0x4c, 0xb3,
      0x8a, 0xce, 0x9b, 0x9a, 0x39, 0xea, 0x23, 0x92,
      0x32, 0x3b, 0x5f, 0x35, 0x97, 0x93, 0x7c, 0x37,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqMove_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x7b, 0x19, 0x0b, 0x5c, 0x53, 0xd9, 0x31,
      0x26, 0x6c, 0xdc, 0x1e, 0x9d, 0xc1, 0x1a, 0x18,
      0x46, 0xba, 0x3f, 0xfc, 0x03, 0xbe, 0x3b, 0x94,
      0x96, 0x80, 0x0f, 0x49, 0xab, 0x59, 0x4e, 0xc4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqMove_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8f, 0x36, 0x07, 0x26, 0x2f, 0x0d, 0xf6, 0x94,
      0x24, 0x68, 0xea, 0xea, 0xf8, 0x2b, 0x4b, 0x83,
      0x9b, 0x28, 0x5d, 0x7f, 0x12, 0xc0, 0x6c, 0x7a,
      0xe0, 0x1b, 0xf8, 0xf6, 0xe2, 0x5c, 0x00, 0x5b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqMove_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0xa3, 0xb9, 0x98, 0x1b, 0x3d, 0x84, 0x63,
      0xe7, 0x7d, 0xa3, 0xfa, 0x57, 0x6a, 0x51, 0xd3,
      0x1e, 0x89, 0x3c, 0xc1, 0xa2, 0x8c, 0xeb, 0x66,
      0x14, 0xf0, 0xc6, 0x4a, 0xc9, 0xf6, 0xb1, 0x6b,
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

static char xarm_msgs__srv__RobotiqMove__TYPE_NAME[] = "xarm_msgs/srv/RobotiqMove";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__RobotiqMove_Event__TYPE_NAME[] = "xarm_msgs/srv/RobotiqMove_Event";
static char xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME[] = "xarm_msgs/srv/RobotiqMove_Request";
static char xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME[] = "xarm_msgs/srv/RobotiqMove_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqMove__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__RobotiqMove__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__RobotiqMove__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqMove__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqMove__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqMove_Event__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__RobotiqMove__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Event__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqMove__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqMove__TYPE_NAME, 25, 25},
      {xarm_msgs__srv__RobotiqMove__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__RobotiqMove__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__RobotiqMove_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__RobotiqMove_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__RobotiqMove_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__pos[] = "pos";
static char xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__speed[] = "speed";
static char xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__speed[] = "255";
static char xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__force[] = "force";
static char xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__force[] = "255";
static char xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__wait[] = "wait";
static char xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__wait[] = "True";
static char xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__timeout[] = "timeout";
static char xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__timeout[] = "5.0";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqMove_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__pos, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__speed, 3, 3},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__force, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__force, 3, 3},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__wait, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__wait, 4, 4},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__RobotiqMove_Request__DEFAULT_VALUE__timeout, 3, 3},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqMove_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
      {xarm_msgs__srv__RobotiqMove_Request__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__message[] = "message";
static char xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__ret_data[] = "ret_data";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqMove_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Response__FIELD_NAME__ret_data, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqMove_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
      {xarm_msgs__srv__RobotiqMove_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqMove_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__RobotiqMove_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqMove_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqMove_Event__TYPE_NAME, 31, 31},
      {xarm_msgs__srv__RobotiqMove_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__RobotiqMove_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__RobotiqMove_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__RobotiqMove_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - robotiq_set_position\n"
  "#   - robotiq_open\n"
  "#   - robotiq_close\n"
  "\n"
  "# robotiq_set_position\n"
  "uint8 pos\n"
  "\n"
  "# robotiq_set_position/robotiq_open/robotiq_close\n"
  "uint8 speed     0xFF\n"
  "uint8 force     0xFF\n"
  "bool wait       true\n"
  "float32 timeout 5\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message\n"
  "\n"
  "uint8[] ret_data";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqMove__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqMove__TYPE_NAME, 25, 25},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 334, 334},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqMove_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqMove_Request__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqMove_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqMove_Response__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqMove_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqMove_Event__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqMove__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqMove__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__RobotiqMove_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__RobotiqMove_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__RobotiqMove_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqMove_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqMove_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqMove_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqMove_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqMove_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqMove_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__RobotiqMove_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__RobotiqMove_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
