// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:srv/GetSetModbusData.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/srv/detail/get_set_modbus_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__GetSetModbusData__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8a, 0xa1, 0x58, 0x01, 0xc8, 0x5c, 0xae, 0x36,
      0x54, 0xd3, 0x10, 0x3c, 0x83, 0xf1, 0x81, 0x83,
      0x81, 0xd8, 0x03, 0xa3, 0x2f, 0x83, 0x23, 0xde,
      0x2c, 0xf9, 0x2e, 0xf5, 0xd7, 0xfd, 0xf1, 0x8b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__GetSetModbusData_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa3, 0x9d, 0x93, 0xcd, 0x01, 0xa8, 0xeb, 0xc0,
      0x88, 0x7f, 0xed, 0x20, 0xbb, 0x7b, 0x71, 0x13,
      0xdc, 0xaf, 0x1d, 0xf9, 0xd8, 0x49, 0x7d, 0x47,
      0x6c, 0xd1, 0xbf, 0xb8, 0xc6, 0x61, 0xcc, 0xed,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__GetSetModbusData_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x99, 0x9a, 0x46, 0xb6, 0x82, 0x16, 0x98, 0x21,
      0x20, 0x4e, 0xbc, 0x58, 0x01, 0x0a, 0xdc, 0x3f,
      0x93, 0x57, 0x7a, 0x9e, 0x65, 0x0f, 0x14, 0x4b,
      0x99, 0xd8, 0xf3, 0xd8, 0xa1, 0x76, 0x0e, 0xc3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__srv__GetSetModbusData_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x90, 0xf3, 0x28, 0xf0, 0x26, 0x4c, 0x80, 0x2f,
      0x61, 0x36, 0x88, 0xbc, 0xc6, 0xba, 0x0d, 0x71,
      0x63, 0x53, 0xae, 0xcc, 0x3e, 0xb3, 0x6e, 0x66,
      0xcc, 0x70, 0xc6, 0x73, 0x3d, 0x6a, 0x2f, 0x57,
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

static char xarm_msgs__srv__GetSetModbusData__TYPE_NAME[] = "xarm_msgs/srv/GetSetModbusData";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char xarm_msgs__srv__GetSetModbusData_Event__TYPE_NAME[] = "xarm_msgs/srv/GetSetModbusData_Event";
static char xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME[] = "xarm_msgs/srv/GetSetModbusData_Request";
static char xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME[] = "xarm_msgs/srv/GetSetModbusData_Response";

// Define type names, field names, and default values
static char xarm_msgs__srv__GetSetModbusData__FIELD_NAME__request_message[] = "request_message";
static char xarm_msgs__srv__GetSetModbusData__FIELD_NAME__response_message[] = "response_message";
static char xarm_msgs__srv__GetSetModbusData__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__GetSetModbusData__FIELDS[] = {
  {
    {xarm_msgs__srv__GetSetModbusData__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {xarm_msgs__srv__GetSetModbusData_Event__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__GetSetModbusData__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Event__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__GetSetModbusData__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__GetSetModbusData__TYPE_NAME, 30, 30},
      {xarm_msgs__srv__GetSetModbusData__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__GetSetModbusData__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__GetSetModbusData_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__GetSetModbusData_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = xarm_msgs__srv__GetSetModbusData_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__modbus_data[] = "modbus_data";
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__modbus_length[] = "modbus_length";
static char xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__modbus_length[] = "0";
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__ret_length[] = "ret_length";
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__host_id[] = "host_id";
static char xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__host_id[] = "9";
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__is_transparent_transmission[] = "is_transparent_transmission";
static char xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__is_transparent_transmission[] = "False";
static char xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__use_503_port[] = "use_503_port";
static char xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__use_503_port[] = "False";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__GetSetModbusData_Request__FIELDS[] = {
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__modbus_data, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__modbus_length, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__modbus_length, 1, 1},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__ret_length, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__host_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__host_id, 1, 1},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__is_transparent_transmission, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__is_transparent_transmission, 5, 5},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__FIELD_NAME__use_503_port, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {xarm_msgs__srv__GetSetModbusData_Request__DEFAULT_VALUE__use_503_port, 5, 5},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__GetSetModbusData_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
      {xarm_msgs__srv__GetSetModbusData_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__ret[] = "ret";
static char xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__message[] = "message";
static char xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__ret_data[] = "ret_data";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__GetSetModbusData_Response__FIELDS[] = {
  {
    {xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__ret, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Response__FIELD_NAME__ret_data, 8, 8},
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
xarm_msgs__srv__GetSetModbusData_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
      {xarm_msgs__srv__GetSetModbusData_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__info[] = "info";
static char xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__request[] = "request";
static char xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field xarm_msgs__srv__GetSetModbusData_Event__FIELDS[] = {
  {
    {xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__srv__GetSetModbusData_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__srv__GetSetModbusData_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__srv__GetSetModbusData_Event__TYPE_NAME, 36, 36},
      {xarm_msgs__srv__GetSetModbusData_Event__FIELDS, 3, 3},
    },
    {xarm_msgs__srv__GetSetModbusData_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = xarm_msgs__srv__GetSetModbusData_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = xarm_msgs__srv__GetSetModbusData_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This format is suitable for the following services\n"
  "#   - getset_tgpio_modbus_data\n"
  "\n"
  "# modbus data to send\n"
  "uint8[] modbus_data\n"
  "\n"
  "# Specify the length of modbus data bytes to be sent, the default(less than or equal to 0) is the size of modbus_data\n"
  "int16 modbus_length 0\n"
  "\n"
  "# Specify the anticipated maximum respond data length in bytes\n"
  "int16 ret_length\n"
  "\n"
  "# host id, 9: END RS485, 10: Controller RS485\n"
  "uint8 host_id 9\n"
  "\n"
  "# whether to choose transparent transmission\n"
  "bool is_transparent_transmission false\n"
  "\n"
  "# whether to use port 503 for communication\n"
  "# if it is true, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission\n"
  "bool use_503_port false\n"
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
xarm_msgs__srv__GetSetModbusData__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__GetSetModbusData__TYPE_NAME, 30, 30},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 774, 774},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__GetSetModbusData_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__GetSetModbusData_Request__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__GetSetModbusData_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__GetSetModbusData_Response__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__srv__GetSetModbusData_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__srv__GetSetModbusData_Event__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__GetSetModbusData__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__GetSetModbusData__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__GetSetModbusData_Event__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__GetSetModbusData_Request__get_individual_type_description_source(NULL);
    sources[5] = *xarm_msgs__srv__GetSetModbusData_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__GetSetModbusData_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__GetSetModbusData_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__GetSetModbusData_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__GetSetModbusData_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__srv__GetSetModbusData_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__srv__GetSetModbusData_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *xarm_msgs__srv__GetSetModbusData_Request__get_individual_type_description_source(NULL);
    sources[4] = *xarm_msgs__srv__GetSetModbusData_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
