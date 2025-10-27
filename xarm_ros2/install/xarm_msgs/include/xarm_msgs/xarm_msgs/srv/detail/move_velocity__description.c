// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/MoveVelocity.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/move_velocity__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveVelocity__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0xce, 0x29, 0xaa, 0x93, 0x36, 0x5d, 0x67,
      0xad, 0xd3, 0xe5, 0x42, 0xc1, 0xac, 0x7c, 0x2e,
      0x99, 0x67, 0xb9, 0x8d, 0x32, 0x42, 0xbe, 0xd1,
      0x4c, 0x43, 0x08, 0x61, 0x51, 0x9e, 0xc4, 0x99,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveVelocity_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xbd, 0x7b, 0xd0, 0x7a, 0x19, 0x9b, 0xd7,
      0x38, 0x95, 0xd2, 0xa9, 0x1e, 0x2f, 0x39, 0x08,
      0x61, 0xb8, 0x95, 0x92, 0x07, 0x5c, 0x3f, 0xda,
      0xc3, 0xb8, 0x48, 0x30, 0x9f, 0x17, 0xb0, 0x00,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveVelocity_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4a, 0xea, 0x53, 0x2e, 0x21, 0xec, 0xb0, 0xad,
      0x7e, 0xc7, 0xa1, 0xd3, 0xa1, 0x37, 0x85, 0xa6,
      0x18, 0xd0, 0x4c, 0xa2, 0x91, 0x00, 0x2b, 0x7a,
      0x81, 0x0c, 0x7c, 0xb4, 0x1a, 0xdd, 0xc6, 0x9c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__MoveVelocity_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2b, 0x77, 0x1d, 0x29, 0x65, 0xed, 0x9b, 0x1f,
      0xed, 0x84, 0x19, 0x0e, 0xdb, 0x12, 0xac, 0xb5,
      0x3f, 0x1b, 0x54, 0x9c, 0x92, 0xad, 0x90, 0xb3,
      0x73, 0xd3, 0xa4, 0x52, 0x00, 0x97, 0xb8, 0x5d,
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

static char xarm_msgs__srv__MoveVelocity__TYPE_NAME[] = "xarm_msgs/srv/MoveVelocity";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__MoveVelocity_Event__TYPE_NAME[] = "xarm_msgs/srv/MoveVelocity_Event";
static char xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME[] = "xarm_msgs/srv/MoveVelocity_Request";
static char xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME[] = "xarm_msgs/srv/MoveVelocity_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__MoveVelocity__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__MoveVelocity__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__MoveVelocity__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveVelocity__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveVelocity__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__MoveVelocity_Event__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveVelocity__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Event__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveVelocity__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveVelocity__TYPE_NAME, 26, 26},
      {xarm_msgs__srv__MoveVelocity__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveVelocity__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveVelocity_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveVelocity_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__MoveVelocity_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__speeds[] = "speeds";
static char xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__is_sync[] = "is_sync";
static char xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__is_sync[] = "True";
static char xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__is_tool_coord[] = "is_tool_coord";
static char xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__is_tool_coord[] = "False";
static char xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__duration[] = "duration";
static char xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__duration[] = "-1.0";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveVelocity_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__speeds, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__is_sync, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__is_sync, 4, 4},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__is_tool_coord, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__is_tool_coord, 5, 5},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Request__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__MoveVelocity_Request__DEFAULT_VALUE__duration, 4, 4},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveVelocity_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
      {xarm_msgs__srv__MoveVelocity_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveVelocity_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__MoveVelocity_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveVelocity_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveVelocity_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Response__FIELD_NAME__message, 7, 7},
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
xarm_msgs__srv__MoveVelocity_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
      {xarm_msgs__srv__MoveVelocity_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__MoveVelocity_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__MoveVelocity_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__MoveVelocity_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__MoveVelocity_Event__TYPE_NAME, 32, 32},
      {xarm_msgs__srv__MoveVelocity_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__MoveVelocity_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__MoveVelocity_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__MoveVelocity_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
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
  "float32 duration -1.0\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveVelocity__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveVelocity__TYPE_NAME, 26, 26},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 736, 736},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveVelocity_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveVelocity_Request__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveVelocity_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveVelocity_Response__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__MoveVelocity_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__MoveVelocity_Event__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveVelocity__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveVelocity__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveVelocity_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveVelocity_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__MoveVelocity_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveVelocity_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveVelocity_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveVelocity_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveVelocity_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__MoveVelocity_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__MoveVelocity_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__MoveVelocity_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__MoveVelocity_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
