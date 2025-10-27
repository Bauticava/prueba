// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/MoveCircle.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/move_circle__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCircle__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd6, 0xd6, 0xe4, 0x2c, 0x93, 0xf8, 0x24, 0xe1,
      0x4d, 0xb2, 0xe6, 0xf6, 0xf3, 0xe2, 0x28, 0x73,
      0xb8, 0xd7, 0x46, 0xc9, 0xd8, 0x71, 0x6b, 0xce,
      0x66, 0xc7, 0xf1, 0xaa, 0x55, 0x21, 0x2a, 0xda,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCircle_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2b, 0x08, 0xd9, 0xc8, 0xd0, 0xa1, 0x51, 0x3a,
      0xf3, 0x9b, 0x68, 0xac, 0xd6, 0x24, 0xea, 0x66,
      0xb4, 0x0d, 0x69, 0x11, 0x39, 0xe2, 0x07, 0x2e,
      0xb6, 0x31, 0x42, 0xdb, 0x08, 0x77, 0x44, 0x00,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCircle_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xae, 0xdc, 0xcf, 0xd3, 0x15, 0xb5, 0xa8, 0xe2,
      0x8c, 0xa7, 0x04, 0x30, 0x95, 0x95, 0x73, 0x2f,
      0xfa, 0x46, 0xee, 0x0c, 0x2a, 0x17, 0xec, 0x57,
      0x77, 0x7c, 0x8b, 0x51, 0x8c, 0xa7, 0xc3, 0x64,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveCircle_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x32, 0x70, 0xe2, 0x72, 0x05, 0xf8, 0x74,
      0x30, 0x7d, 0x00, 0x0d, 0x8c, 0xee, 0xc6, 0x3c,
      0xf5, 0xe4, 0x0d, 0xa3, 0x68, 0xc1, 0x37, 0x41,
      0x98, 0x8b, 0x43, 0x36, 0x02, 0x4f, 0x8c, 0xab,
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

static char xarm_msgs__srv__MoveCircle__TYPE_NAME[] = "xarm_msgs/srv/MoveCircle";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__MoveCircle_Event__TYPE_NAME[] = "xarm_msgs/srv/MoveCircle_Event";
static char xarm_msgs__srv__MoveCircle_Request__TYPE_NAME[] = "xarm_msgs/srv/MoveCircle_Request";
static char xarm_msgs__srv__MoveCircle_Response__TYPE_NAME[] = "xarm_msgs/srv/MoveCircle_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCircle__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__MoveCircle__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__MoveCircle__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCircle__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCircle__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveCircle_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveCircle__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCircle__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCircle__TYPE_NAME, 24, 24},
      {xarm_msgs__srv__MoveCircle__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveCircle__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveCircle_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveCircle_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__MoveCircle_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__pose1[] = "pose1";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__pose2[] = "pose2";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__percent[] = "percent";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__speed[] = "speed";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__speed[] = "0.0";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__acc[] = "acc";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__acc[] = "0.0";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__mvtime[] = "mvtime";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__mvtime[] = "0.0";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__wait[] = "wait";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__wait[] = "False";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__timeout[] = "timeout";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__timeout[] = "-1.0";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__is_tool_coord[] = "is_tool_coord";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__is_tool_coord[] = "False";
static char xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__is_axis_angle[] = "is_axis_angle";
static char xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__is_axis_angle[] = "False";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCircle_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__pose1, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__pose2, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__percent, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__speed, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__acc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__acc, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__mvtime, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__mvtime, 3, 3},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__wait, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__wait, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__timeout, 4, 4},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__is_tool_coord, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__is_tool_coord, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__FIELD_NAME__is_axis_angle, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveCircle_Request__DEFAULT_VALUE__is_axis_angle, 5, 5},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCircle_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
      {xarm_msgs__srv__MoveCircle_Request__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCircle_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__MoveCircle_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCircle_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCircle_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Response__FIELD_NAME__message, 7, 7},
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
xarm_msgs__srv__MoveCircle_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
      {xarm_msgs__srv__MoveCircle_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveCircle_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveCircle_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveCircle_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveCircle_Event__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__MoveCircle_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveCircle_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveCircle_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveCircle_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - move_circle\n"
  "\n"
  "float32[] pose1\n"
  "float32[] pose2\n"
  "\n"
  "float32 percent\n"
  "float32 speed       0\n"
  "float32 acc         0\n"
  "float32 mvtime      0\n"
  "bool wait           false\n"
  "float32 timeout     -1\n"
  "bool is_tool_coord  false\n"
  "bool is_axis_angle  false\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCircle__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCircle__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 319, 319},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCircle_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCircle_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCircle_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCircle_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveCircle_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveCircle_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCircle__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCircle__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveCircle_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveCircle_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__MoveCircle_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCircle_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCircle_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCircle_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCircle_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveCircle_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveCircle_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveCircle_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveCircle_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
