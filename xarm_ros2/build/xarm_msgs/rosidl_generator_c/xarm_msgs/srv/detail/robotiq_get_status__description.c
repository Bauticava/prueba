// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/RobotiqGetStatus.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/robotiq_get_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqGetStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x4d, 0x4a, 0x88, 0x92, 0xfb, 0xc3, 0xd6,
      0xa2, 0xdc, 0x6e, 0xd9, 0x7c, 0xd6, 0x5f, 0x0f,
      0x39, 0x2c, 0x85, 0x0a, 0xe5, 0x11, 0xe6, 0x36,
      0xa7, 0x87, 0xd5, 0x31, 0xee, 0x27, 0xf0, 0x8d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqGetStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x57, 0x98, 0xbb, 0xe8, 0xb3, 0x7a, 0x77, 0xa4,
      0x6c, 0x46, 0x3c, 0xa5, 0xd0, 0x2d, 0x68, 0x71,
      0xe7, 0x4d, 0xbd, 0xab, 0xf0, 0x38, 0x75, 0x6d,
      0xfd, 0x82, 0x54, 0xe0, 0xac, 0x64, 0x52, 0x7b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqGetStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0x4b, 0x5c, 0xd9, 0xb9, 0xc3, 0x31, 0x98,
      0x4f, 0xbf, 0x7b, 0x03, 0x5c, 0xc6, 0xa2, 0xd2,
      0xc3, 0x6f, 0xa4, 0x7c, 0xae, 0x52, 0xba, 0xed,
      0xf1, 0xfe, 0x63, 0x10, 0x1a, 0x4e, 0x5b, 0xd6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__RobotiqGetStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x05, 0xbe, 0xbf, 0x31, 0x71, 0x06, 0x58, 0x0d,
      0xb8, 0x96, 0xb1, 0xec, 0x53, 0xd6, 0xf7, 0xd6,
      0xdc, 0xf5, 0x27, 0x7d, 0x3e, 0xbf, 0xf4, 0x6a,
      0xea, 0x94, 0x33, 0x8c, 0x9e, 0x4d, 0x30, 0x02,
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

static char xarm_msgs__srv__RobotiqGetStatus__TYPE_NAME[] = "xarm_msgs/srv/RobotiqGetStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__RobotiqGetStatus_Event__TYPE_NAME[] = "xarm_msgs/srv/RobotiqGetStatus_Event";
static char xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME[] = "xarm_msgs/srv/RobotiqGetStatus_Request";
static char xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME[] = "xarm_msgs/srv/RobotiqGetStatus_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqGetStatus__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__RobotiqGetStatus_Event__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__RobotiqGetStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Event__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqGetStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqGetStatus__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__RobotiqGetStatus__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__RobotiqGetStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__RobotiqGetStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__RobotiqGetStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__RobotiqGetStatus_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqGetStatus_Request__FIELD_NAME__number_of_registers[] = "number_of_registers";
static char xarm_msgs__srv__RobotiqGetStatus_Request__DEFAULT_VALUE__number_of_registers[] = "3";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqGetStatus_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqGetStatus_Request__FIELD_NAME__number_of_registers, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__RobotiqGetStatus_Request__DEFAULT_VALUE__number_of_registers, 1, 1},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqGetStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
      {xarm_msgs__srv__RobotiqGetStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__message[] = "message";
static char xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__ret_data[] = "ret_data";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqGetStatus_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Response__FIELD_NAME__ret_data, 8, 8},
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
xarm_msgs__srv__RobotiqGetStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
      {xarm_msgs__srv__RobotiqGetStatus_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__RobotiqGetStatus_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__RobotiqGetStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__RobotiqGetStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__RobotiqGetStatus_Event__TYPE_NAME, 36, 36},
      {xarm_msgs__srv__RobotiqGetStatus_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__RobotiqGetStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__RobotiqGetStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__RobotiqGetStatus_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - robotiq_get_status\n"
  "\n"
  "uint8 number_of_registers 3\n"
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
xarm_msgs__srv__RobotiqGetStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqGetStatus__TYPE_NAME, 30, 30},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 156, 156},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqGetStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqGetStatus_Request__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqGetStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqGetStatus_Response__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__RobotiqGetStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__RobotiqGetStatus_Event__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqGetStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqGetStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__RobotiqGetStatus_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__RobotiqGetStatus_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__RobotiqGetStatus_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqGetStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqGetStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqGetStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqGetStatus_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__RobotiqGetStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__RobotiqGetStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__RobotiqGetStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__RobotiqGetStatus_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
