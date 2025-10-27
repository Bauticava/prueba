// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/FtCaliLoad.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/ft_cali_load__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtCaliLoad__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0xff, 0x15, 0x9c, 0xfe, 0xfc, 0x14, 0xc5,
      0xa7, 0x5e, 0xb4, 0x16, 0xfd, 0xd2, 0xae, 0x02,
      0xf5, 0xda, 0xbd, 0x8b, 0x6e, 0x9f, 0x48, 0xa2,
      0x20, 0xde, 0xa7, 0x4f, 0x2d, 0xd5, 0x9f, 0x3d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtCaliLoad_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6c, 0xc1, 0x6f, 0x5b, 0x9a, 0x63, 0xd1, 0xfd,
      0x9c, 0x43, 0x2f, 0x22, 0xb8, 0x34, 0x1a, 0x8b,
      0x1e, 0x18, 0x7d, 0x97, 0xad, 0x8c, 0xb7, 0x93,
      0xc7, 0x9e, 0xe7, 0xa8, 0x06, 0xbb, 0x25, 0xb8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtCaliLoad_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x47, 0xf4, 0x48, 0x2f, 0xe7, 0x4b, 0x1b, 0xed,
      0x19, 0x64, 0x93, 0x5b, 0x8b, 0x33, 0x3a, 0xa8,
      0xf2, 0x9b, 0xcf, 0x5c, 0xd4, 0x5b, 0xa9, 0x59,
      0xa5, 0x3e, 0xb8, 0xf3, 0x08, 0x74, 0x14, 0x90,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__FtCaliLoad_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x88, 0xf0, 0x67, 0xc4, 0xf9, 0x15, 0xa1, 0x45,
      0xa6, 0x31, 0x9a, 0x72, 0x2e, 0xa0, 0xef, 0xf3,
      0x7f, 0x7f, 0xc0, 0xec, 0x07, 0x4f, 0x69, 0x77,
      0x7c, 0x77, 0x99, 0x41, 0x5f, 0x6f, 0x47, 0x30,
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

static char xarm_msgs__srv__FtCaliLoad__TYPE_NAME[] = "xarm_msgs/srv/FtCaliLoad";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__FtCaliLoad_Event__TYPE_NAME[] = "xarm_msgs/srv/FtCaliLoad_Event";
static char xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME[] = "xarm_msgs/srv/FtCaliLoad_Request";
static char xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME[] = "xarm_msgs/srv/FtCaliLoad_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__FtCaliLoad__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__FtCaliLoad__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__FtCaliLoad__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtCaliLoad__FIELDS[] = {
  {
    {xarm_msgs__srv__FtCaliLoad__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__FtCaliLoad_Event__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__FtCaliLoad__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Event__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtCaliLoad__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtCaliLoad__TYPE_NAME, 24, 24},
      {xarm_msgs__srv__FtCaliLoad__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__FtCaliLoad__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__FtCaliLoad_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__FtCaliLoad_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__FtCaliLoad_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__datas[] = "datas";
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__association_setting_tcp_load[] = "association_setting_tcp_load";
static char xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__association_setting_tcp_load[] = "False";
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__m[] = "m";
static char xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__m[] = "0.325";
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__x[] = "x";
static char xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__x[] = "-17.0";
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__y[] = "y";
static char xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__y[] = "9.0";
static char xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__z[] = "z";
static char xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__z[] = "11.8";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtCaliLoad_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__datas, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__association_setting_tcp_load, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__association_setting_tcp_load, 5, 5},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__m, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__m, 5, 5},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__x, 5, 5},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__y, 3, 3},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__FtCaliLoad_Request__DEFAULT_VALUE__z, 4, 4},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtCaliLoad_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
      {xarm_msgs__srv__FtCaliLoad_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtCaliLoad_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__FtCaliLoad_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtCaliLoad_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__FtCaliLoad_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Response__FIELD_NAME__message, 7, 7},
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
xarm_msgs__srv__FtCaliLoad_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
      {xarm_msgs__srv__FtCaliLoad_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__FtCaliLoad_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__FtCaliLoad_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__FtCaliLoad_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__FtCaliLoad_Event__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__FtCaliLoad_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__FtCaliLoad_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__FtCaliLoad_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__FtCaliLoad_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - ft_sensor_cali_load\n"
  "\n"
  "# iden result([mass(kg)\\xef\\xbc\\x8cx_centroid(mm)\\xef\\xbc\\x8cy_centroid(mm)\\xef\\xbc\\x8cz_centroid(mm)\\xef\\xbc\\x8cFx_offset\\xef\\xbc\\x8cFy_offset\\xef\\xbc\\x8cFz_offset\\xef\\xbc\\x8cTx_offset\\xef\\xbc\\x8cTy_offset\\xef\\xbc\\x8cTz_ffset])\n"
  "float32[] datas\n"
  "\n"
  "# whether to convert the paramster to the corresponding tcp load and set, default is false\n"
  "bool association_setting_tcp_load   false\n"
  "float32 m   0.325\n"
  "float32 x   -17.0\n"
  "float32 y   9.0\n"
  "float32 z   11.8\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtCaliLoad__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtCaliLoad__TYPE_NAME, 24, 24},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 460, 460},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtCaliLoad_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtCaliLoad_Request__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtCaliLoad_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtCaliLoad_Response__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__FtCaliLoad_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__FtCaliLoad_Event__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtCaliLoad__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtCaliLoad__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__FtCaliLoad_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__FtCaliLoad_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__FtCaliLoad_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtCaliLoad_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtCaliLoad_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtCaliLoad_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtCaliLoad_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__FtCaliLoad_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__FtCaliLoad_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__FtCaliLoad_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__FtCaliLoad_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
