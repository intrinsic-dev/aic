// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#include "aic_task_interfaces/msg/detail/task__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_aic_task_interfaces
const rosidl_type_hash_t *
aic_task_interfaces__msg__Task__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x31, 0x49, 0x2c, 0x3a, 0x2f, 0x03, 0x86, 0x93,
      0x56, 0xd1, 0x72, 0xe2, 0x9b, 0x97, 0x53, 0xd6,
      0x39, 0x67, 0x6b, 0x3c, 0x21, 0x74, 0x3a, 0xd2,
      0x60, 0x82, 0x9a, 0xdc, 0x5e, 0x72, 0x67, 0xa1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char aic_task_interfaces__msg__Task__TYPE_NAME[] = "aic_task_interfaces/msg/Task";

// Define type names, field names, and default values
static char aic_task_interfaces__msg__Task__FIELD_NAME__id[] = "id";
static char aic_task_interfaces__msg__Task__FIELD_NAME__cable_type[] = "cable_type";
static char aic_task_interfaces__msg__Task__FIELD_NAME__cable_name[] = "cable_name";
static char aic_task_interfaces__msg__Task__FIELD_NAME__plug_type[] = "plug_type";
static char aic_task_interfaces__msg__Task__FIELD_NAME__plug_name[] = "plug_name";
static char aic_task_interfaces__msg__Task__FIELD_NAME__port_type[] = "port_type";
static char aic_task_interfaces__msg__Task__FIELD_NAME__port_name[] = "port_name";
static char aic_task_interfaces__msg__Task__FIELD_NAME__target_module_name[] = "target_module_name";
static char aic_task_interfaces__msg__Task__FIELD_NAME__time_limit[] = "time_limit";

static rosidl_runtime_c__type_description__Field aic_task_interfaces__msg__Task__FIELDS[] = {
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__cable_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__cable_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__plug_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__plug_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__port_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__port_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__target_module_name, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aic_task_interfaces__msg__Task__FIELD_NAME__time_limit, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
aic_task_interfaces__msg__Task__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {aic_task_interfaces__msg__Task__TYPE_NAME, 28, 28},
      {aic_task_interfaces__msg__Task__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A unique ID for the cable insertion task.\n"
  "string id\n"
  "\n"
  "# The type of cable which should be inserted.\n"
  "string cable_type\n"
  "\n"
  "# The name of the cable which should be inserted, e.g., \"sfp_sc\".\n"
  "string cable_name\n"
  "\n"
  "# The type of plug at the end of the cable which should be inserted, e.g., \"sfp\".\n"
  "string plug_type\n"
  "\n"
  "# The name of plug at the end of the cable which should be inserted, e.g., \"sfp_module\".\n"
  "string plug_name\n"
  "\n"
  "# The type of port on the module side into which the plug should be inserted, e.g., \"sfp\".\n"
  "string port_type\n"
  "\n"
  "# The name of the port on the module side into which the plug should be inserted, e.g., \"sfp_port_0\".\n"
  "string port_name\n"
  "\n"
  "# The name of the component or module on which the port exists, e.g., \"nic_card_0\"\n"
  "string target_module_name\n"
  "\n"
  "# The number of seconds from the moment this request is received by when the task must complete.\n"
  "# todo(yadunund): should this be a system clock timestamp instead.\n"
  "uint64 time_limit";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
aic_task_interfaces__msg__Task__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {aic_task_interfaces__msg__Task__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 933, 933},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
aic_task_interfaces__msg__Task__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *aic_task_interfaces__msg__Task__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
