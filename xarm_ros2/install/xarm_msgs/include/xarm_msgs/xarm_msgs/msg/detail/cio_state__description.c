// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from xarm_msgs:msg/CIOState.idl
// generated code does not contain a copyright notice

#include "xarm_msgs/msg/detail/cio_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_xarm_msgs
const rosidl_type_hash_t *
xarm_msgs__msg__CIOState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x66, 0x8e, 0x60, 0xd1, 0xcc, 0x78, 0x7e, 0xab,
      0xd3, 0x57, 0x41, 0x14, 0x85, 0x63, 0xcf, 0x3e,
      0x17, 0x1c, 0x83, 0xf9, 0x37, 0x34, 0x6a, 0x36,
      0x2d, 0xe6, 0x5a, 0x5a, 0xfa, 0x5c, 0x0f, 0x5d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char xarm_msgs__msg__CIOState__TYPE_NAME[] = "xarm_msgs/msg/CIOState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char xarm_msgs__msg__CIOState__FIELD_NAME__header[] = "header";
static char xarm_msgs__msg__CIOState__FIELD_NAME__state[] = "state";
static char xarm_msgs__msg__CIOState__FIELD_NAME__code[] = "code";
static char xarm_msgs__msg__CIOState__FIELD_NAME__input_digitals[] = "input_digitals";
static char xarm_msgs__msg__CIOState__FIELD_NAME__output_digitals[] = "output_digitals";
static char xarm_msgs__msg__CIOState__FIELD_NAME__input_analogs[] = "input_analogs";
static char xarm_msgs__msg__CIOState__FIELD_NAME__output_analogs[] = "output_analogs";
static char xarm_msgs__msg__CIOState__FIELD_NAME__input_conf[] = "input_conf";
static char xarm_msgs__msg__CIOState__FIELD_NAME__output_conf[] = "output_conf";

static rosidl_runtime_c__type_description__Field xarm_msgs__msg__CIOState__FIELDS[] = {
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__input_digitals, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__output_digitals, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__input_analogs, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__output_analogs, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__input_conf, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {xarm_msgs__msg__CIOState__FIELD_NAME__output_conf, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription xarm_msgs__msg__CIOState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
xarm_msgs__msg__CIOState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {xarm_msgs__msg__CIOState__TYPE_NAME, 22, 22},
      {xarm_msgs__msg__CIOState__FIELDS, 9, 9},
    },
    {xarm_msgs__msg__CIOState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# contorller gpio module state\n"
  "int16 state\n"
  "\n"
  "# controller gpio module error code\n"
  "int16 code\n"
  "\n"
  "# input_digitals[0]: digital input functional gpio state\n"
  "# input_digitals[1]: digital input configuring gpio state\n"
  "#    CI0: (input_digitals[1] >> 0) & 0x0001\n"
  "#    CI1: (input_digitals[1] >> 1) & 0x0001\n"
  "#    CI7: (input_digitals[1] >> 7) & 0x0001\n"
  "#    DI0: (input_digitals[1] >> 8) & 0x0001\n"
  "#    DI1: (input_digitals[1] >> 9) & 0x0001\n"
  "#    DI7: (input_digitals[1] >> 15) & 0x0001\n"
  "uint16[2] input_digitals\n"
  "\n"
  "# output_digitals[0]: digital output functional gpio state\n"
  "# output_digitals[1]: digital output configuring gpio state\n"
  "#    CO0: (output_digitals[1] >> 0) & 0x0001\n"
  "#    CO1: (output_digitals[1] >> 1) & 0x0001\n"
  "#    CO7: (output_digitals[1] >> 7) & 0x0001\n"
  "#    DO0: (output_digitals[1] >> 8) & 0x0001\n"
  "#    DO1: (output_digitals[1] >> 9) & 0x0001\n"
  "#    DO7: (output_digitals[1] >> 15) & 0x0001\n"
  "uint16[2] output_digitals\n"
  "\n"
  "# input_analogs[0]: the value of AI0\n"
  "# input_analogs[1]: the value of AI1\n"
  "float32[2] input_analogs\n"
  "\n"
  "# output_analogs[0]: the value of AO0\n"
  "# output_analogs[1]: the value of AO1\n"
  "float32[2] output_analogs\n"
  "\n"
  "# digital input functional info\n"
  "uint8[16] input_conf\n"
  "\n"
  "# digital output functional info\n"
  "uint8[16] output_conf";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
xarm_msgs__msg__CIOState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {xarm_msgs__msg__CIOState__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1250, 1250},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
xarm_msgs__msg__CIOState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *xarm_msgs__msg__CIOState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
