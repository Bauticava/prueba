// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/SetModbusTimeout.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/set_modbus_timeout__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__SetModbusTimeout__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xef, 0xda, 0x78, 0x0c, 0xe9, 0xcb, 0xe5, 0x7f,
      0x8c, 0x13, 0x49, 0x11, 0x18, 0x9b, 0xa2, 0xf5,
      0x10, 0x37, 0xd4, 0xe2, 0x8d, 0xd3, 0x90, 0x66,
      0xc9, 0x7e, 0xa5, 0x3b, 0x41, 0xe6, 0x5e, 0xee,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__SetModbusTimeout_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0x85, 0x01, 0x6c, 0x0e, 0x8a, 0xdf, 0x44,
      0xa7, 0x03, 0x0c, 0x3e, 0x77, 0xc8, 0x0f, 0x01,
      0x69, 0xa7, 0xa9, 0x97, 0x76, 0xdc, 0x10, 0x3f,
      0x5f, 0xc3, 0x3b, 0x79, 0xa6, 0xb3, 0xd8, 0x17,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__SetModbusTimeout_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbc, 0xba, 0x29, 0xf8, 0xfa, 0xf9, 0x1d, 0xf8,
      0x27, 0x62, 0x8f, 0x01, 0xda, 0xa7, 0x52, 0xcb,
      0x64, 0x20, 0xd9, 0xd3, 0x72, 0x71, 0xfc, 0x74,
      0x80, 0xb8, 0xe6, 0x6f, 0x0f, 0x92, 0xd1, 0xba,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__SetModbusTimeout_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1d, 0x1a, 0xfe, 0x2a, 0x68, 0x2d, 0x6f, 0x4c,
      0xcb, 0x67, 0x63, 0x56, 0xec, 0xc2, 0x5e, 0x81,
      0x8c, 0xf0, 0xcf, 0xa8, 0xb2, 0x79, 0x47, 0xe8,
      0x88, 0x9e, 0xee, 0x30, 0x89, 0x1c, 0x4e, 0xea,
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

static char xarm_msgs__srv__SetModbusTimeout__TYPE_NAME[] = "xarm_msgs/srv/SetModbusTimeout";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__SetModbusTimeout_Event__TYPE_NAME[] = "xarm_msgs/srv/SetModbusTimeout_Event";
static char xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME[] = "xarm_msgs/srv/SetModbusTimeout_Request";
static char xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME[] = "xarm_msgs/srv/SetModbusTimeout_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__SetModbusTimeout__FIELDS[] = {
  {
    {xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__SetModbusTimeout_Event__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__SetModbusTimeout__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Event__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__SetModbusTimeout__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__SetModbusTimeout__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__SetModbusTimeout__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__SetModbusTimeout__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__SetModbusTimeout_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__SetModbusTimeout_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__SetModbusTimeout_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__SetModbusTimeout_Request__FIELD_NAME__timeout[] = "timeout";
static char xarm_msgs__srv__SetModbusTimeout_Request__FIELD_NAME__is_transparent_transmission[] = "is_transparent_transmission";
static char xarm_msgs__srv__SetModbusTimeout_Request__DEFAULT_VALUE__is_transparent_transmission[] = "False";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__SetModbusTimeout_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__SetModbusTimeout_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Request__FIELD_NAME__is_transparent_transmission, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__SetModbusTimeout_Request__DEFAULT_VALUE__is_transparent_transmission, 5, 5},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__SetModbusTimeout_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
      {xarm_msgs__srv__SetModbusTimeout_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__SetModbusTimeout_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__SetModbusTimeout_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__SetModbusTimeout_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__SetModbusTimeout_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Response__FIELD_NAME__message, 7, 7},
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
xarm_msgs__srv__SetModbusTimeout_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
      {xarm_msgs__srv__SetModbusTimeout_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__SetModbusTimeout_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__SetModbusTimeout_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__SetModbusTimeout_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__SetModbusTimeout_Event__TYPE_NAME, 36, 36},
      {xarm_msgs__srv__SetModbusTimeout_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__SetModbusTimeout_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__SetModbusTimeout_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__SetModbusTimeout_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - set_tgpio_modbus_timeout\n"
  "\n"
  "# set the timeout parameter in modbus communication, in milliseconds\n"
  "int32 timeout\n"
  "\n"
  "# whether the set timeout is the timeout of transparent transmission\n"
  "bool is_transparent_transmission false\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__SetModbusTimeout__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__SetModbusTimeout__TYPE_NAME, 30, 30},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 308, 308},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__SetModbusTimeout_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__SetModbusTimeout_Request__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__SetModbusTimeout_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__SetModbusTimeout_Response__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__SetModbusTimeout_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__SetModbusTimeout_Event__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__SetModbusTimeout__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__SetModbusTimeout__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__SetModbusTimeout_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__SetModbusTimeout_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__SetModbusTimeout_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__SetModbusTimeout_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__SetModbusTimeout_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__SetModbusTimeout_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__SetModbusTimeout_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__SetModbusTimeout_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__SetModbusTimeout_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__SetModbusTimeout_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__SetModbusTimeout_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
