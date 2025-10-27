// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/BioGripperEnable.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/bio_gripper_enable__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__BioGripperEnable__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0xd8, 0x5c, 0xc0, 0x57, 0xdd, 0xe7, 0xe6,
      0x45, 0x71, 0x91, 0xa0, 0xcf, 0x17, 0x89, 0x06,
      0x76, 0x66, 0x1e, 0x6c, 0xfa, 0x87, 0xa1, 0x98,
      0x30, 0xd3, 0x25, 0xcb, 0x12, 0xf4, 0xe5, 0x16,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__BioGripperEnable_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0xcf, 0x8a, 0x8a, 0xda, 0x1d, 0x44, 0x45,
      0x56, 0x21, 0xe5, 0x6a, 0xea, 0x85, 0x49, 0x21,
      0x7c, 0xa7, 0xbb, 0x01, 0x5b, 0x50, 0x28, 0xf4,
      0x9e, 0xe1, 0xa6, 0x1e, 0xd4, 0x64, 0x90, 0x70,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__BioGripperEnable_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0x17, 0x99, 0xcc, 0xc0, 0xdd, 0x60, 0x2c,
      0xaa, 0x06, 0x95, 0xb3, 0x37, 0xcb, 0x68, 0x18,
      0x45, 0xe0, 0x98, 0x09, 0x9b, 0x57, 0x89, 0x7c,
      0x8a, 0x43, 0xc5, 0x2c, 0x99, 0x76, 0x03, 0x2a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__BioGripperEnable_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf5, 0x42, 0x04, 0xf5, 0xa4, 0xf5, 0x23, 0x9a,
      0xdd, 0x72, 0xfe, 0x49, 0x78, 0xc7, 0x33, 0x0a,
      0x63, 0x0b, 0x3b, 0x17, 0xd4, 0xe3, 0x57, 0xbb,
      0xf0, 0x5b, 0x97, 0xca, 0xd7, 0x83, 0xb3, 0x5a,
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

static char xarm_msgs__srv__BioGripperEnable__TYPE_NAME[] = "xarm_msgs/srv/BioGripperEnable";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__BioGripperEnable_Event__TYPE_NAME[] = "xarm_msgs/srv/BioGripperEnable_Event";
static char xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME[] = "xarm_msgs/srv/BioGripperEnable_Request";
static char xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME[] = "xarm_msgs/srv/BioGripperEnable_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__BioGripperEnable__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__BioGripperEnable__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__BioGripperEnable__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__BioGripperEnable__FIELDS[] = {
  {
    {xarm_msgs__srv__BioGripperEnable__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__BioGripperEnable_Event__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__BioGripperEnable__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Event__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__BioGripperEnable__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__BioGripperEnable__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__BioGripperEnable__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__BioGripperEnable__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__BioGripperEnable_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__BioGripperEnable_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__BioGripperEnable_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__enable[] = "enable";
static char xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__wait[] = "wait";
static char xarm_msgs__srv__BioGripperEnable_Request__DEFAULT_VALUE__wait[] = "False";
static char xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__timeout[] = "timeout";
static char xarm_msgs__srv__BioGripperEnable_Request__DEFAULT_VALUE__timeout[] = "3.0";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__BioGripperEnable_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__enable, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__wait, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__BioGripperEnable_Request__DEFAULT_VALUE__wait, 5, 5},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Request__FIELD_NAME__timeout, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__BioGripperEnable_Request__DEFAULT_VALUE__timeout, 3, 3},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__BioGripperEnable_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
      {xarm_msgs__srv__BioGripperEnable_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__BioGripperEnable_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__BioGripperEnable_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__BioGripperEnable_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__BioGripperEnable_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Response__FIELD_NAME__message, 7, 7},
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
xarm_msgs__srv__BioGripperEnable_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
      {xarm_msgs__srv__BioGripperEnable_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__BioGripperEnable_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__BioGripperEnable_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__BioGripperEnable_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__BioGripperEnable_Event__TYPE_NAME, 36, 36},
      {xarm_msgs__srv__BioGripperEnable_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__BioGripperEnable_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__BioGripperEnable_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__BioGripperEnable_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - set_bio_gripper_enable\n"
  "\n"
  "bool enable\n"
  "bool wait       false\n"
  "float32 timeout 3\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__BioGripperEnable__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__BioGripperEnable__TYPE_NAME, 30, 30},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 166, 166},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__BioGripperEnable_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__BioGripperEnable_Request__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__BioGripperEnable_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__BioGripperEnable_Response__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__BioGripperEnable_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__BioGripperEnable_Event__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__BioGripperEnable__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__BioGripperEnable__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__BioGripperEnable_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__BioGripperEnable_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__BioGripperEnable_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__BioGripperEnable_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__BioGripperEnable_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__BioGripperEnable_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__BioGripperEnable_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__BioGripperEnable_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__BioGripperEnable_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__BioGripperEnable_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__BioGripperEnable_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
