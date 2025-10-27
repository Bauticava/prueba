// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/IdenLoad.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/iden_load__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__IdenLoad__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf1, 0x13, 0x93, 0xbe, 0x8a, 0x8d, 0x20, 0x78,
      0x78, 0x64, 0xea, 0x54, 0x47, 0xcf, 0x74, 0xef,
      0xc0, 0x49, 0x4e, 0xed, 0xe8, 0x09, 0x4e, 0xd7,
      0x56, 0xe4, 0x0e, 0x72, 0x95, 0x4d, 0xd4, 0xb4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__IdenLoad_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x90, 0x39, 0xf4, 0x1a, 0x89, 0x0a, 0x88, 0x9e,
      0x8f, 0x78, 0x27, 0xe4, 0x7a, 0x7b, 0x13, 0x2a,
      0xc3, 0x17, 0x7a, 0x7c, 0x02, 0x62, 0x7c, 0x7e,
      0xaa, 0x68, 0x71, 0xb1, 0x02, 0xcc, 0x79, 0x5e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__IdenLoad_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0x35, 0xdf, 0xf6, 0xa7, 0x64, 0xfe, 0x9d,
      0xb1, 0x13, 0x5e, 0xa5, 0x6e, 0x94, 0xbe, 0x09,
      0xf6, 0x54, 0xc5, 0xe2, 0x55, 0xf1, 0xbb, 0xeb,
      0x42, 0x96, 0xd2, 0x81, 0xe6, 0xdf, 0x84, 0xb5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__IdenLoad_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0a, 0xf5, 0x3a, 0xb2, 0x59, 0x9f, 0xb0, 0xba,
      0xb4, 0x3d, 0xc5, 0x67, 0x2e, 0x0c, 0x36, 0xc0,
      0x5c, 0xef, 0xee, 0x56, 0xd3, 0x89, 0x6d, 0x7d,
      0x16, 0xbe, 0xd0, 0x1d, 0x30, 0x8a, 0x07, 0x3d,
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

static char xarm_msgs__srv__IdenLoad__TYPE_NAME[] = "xarm_msgs/srv/IdenLoad";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__IdenLoad_Event__TYPE_NAME[] = "xarm_msgs/srv/IdenLoad_Event";
static char xarm_msgs__srv__IdenLoad_Request__TYPE_NAME[] = "xarm_msgs/srv/IdenLoad_Request";
static char xarm_msgs__srv__IdenLoad_Response__TYPE_NAME[] = "xarm_msgs/srv/IdenLoad_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__IdenLoad__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__IdenLoad__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__IdenLoad__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__IdenLoad__FIELDS[] = {
  {
    {xarm_msgs__srv__IdenLoad__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__IdenLoad_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__IdenLoad__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__IdenLoad__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__IdenLoad__TYPE_NAME, 22, 22},
      {xarm_msgs__srv__IdenLoad__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__IdenLoad__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__IdenLoad_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__IdenLoad_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__IdenLoad_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__IdenLoad_Request__FIELD_NAME__estimated_mass[] = "estimated_mass";
static char xarm_msgs__srv__IdenLoad_Request__DEFAULT_VALUE__estimated_mass[] = "0.0";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__IdenLoad_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__IdenLoad_Request__FIELD_NAME__estimated_mass, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__IdenLoad_Request__DEFAULT_VALUE__estimated_mass, 3, 3},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__IdenLoad_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__IdenLoad_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__message[] = "message";
static char xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__datas[] = "datas";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__IdenLoad_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Response__FIELD_NAME__datas, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__IdenLoad_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
      {xarm_msgs__srv__IdenLoad_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__IdenLoad_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__IdenLoad_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__IdenLoad_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__IdenLoad_Event__TYPE_NAME, 28, 28},
      {xarm_msgs__srv__IdenLoad_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__IdenLoad_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__IdenLoad_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__IdenLoad_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - iden_tcp_load\n"
  "#   - ft_sensor_iden_load\n"
  "\n"
  "# estimated mass(kg), only required for Lite6 models via the `iden_tcp_load` service\n"
  "float32 estimated_mass 0.0\n"
  "\n"
  "---\n"
  "\n"
  "int16 ret\n"
  "string message\n"
  "\n"
  "# the result of identification\n"
  "#   iden_tcp_load: [mass(kg)\\xef\\xbc\\x8cx_centroid(mm)\\xef\\xbc\\x8cy_centroid(mm)\\xef\\xbc\\x8cz_centroid(mm)]\n"
  "#   ft_sensor_iden_load: [mass(kg)\\xef\\xbc\\x8cx_centroid(mm)\\xef\\xbc\\x8cy_centroid(mm)\\xef\\xbc\\x8cz_centroid(mm)\\xef\\xbc\\x8cFx_offset\\xef\\xbc\\x8cFy_offset\\xef\\xbc\\x8cFz_offset\\xef\\xbc\\x8cTx_offset\\xef\\xbc\\x8cTy_offset\\xef\\xbc\\x8cTz_ffset]\n"
  "float32[] datas";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__IdenLoad__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__IdenLoad__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 505, 505},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__IdenLoad_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__IdenLoad_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__IdenLoad_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__IdenLoad_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__IdenLoad_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__IdenLoad_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__IdenLoad__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__IdenLoad__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__IdenLoad_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__IdenLoad_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__IdenLoad_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__IdenLoad_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__IdenLoad_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__IdenLoad_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__IdenLoad_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__IdenLoad_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__IdenLoad_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__IdenLoad_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__IdenLoad_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
