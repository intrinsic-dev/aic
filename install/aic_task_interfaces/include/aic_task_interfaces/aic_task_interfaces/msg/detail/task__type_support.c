// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aic_task_interfaces/msg/detail/task__rosidl_typesupport_introspection_c.h"
#include "aic_task_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aic_task_interfaces/msg/detail/task__functions.h"
#include "aic_task_interfaces/msg/detail/task__struct.h"


// Include directives for member types
// Member `id`
// Member `cable_type`
// Member `cable_name`
// Member `plug_type`
// Member `plug_name`
// Member `port_type`
// Member `port_name`
// Member `target_module_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aic_task_interfaces__msg__Task__init(message_memory);
}

void aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_fini_function(void * message_memory)
{
  aic_task_interfaces__msg__Task__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_member_array[9] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cable_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, cable_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cable_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, cable_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "plug_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, plug_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "plug_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, plug_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "port_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, port_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "port_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, port_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_module_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, target_module_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aic_task_interfaces__msg__Task, time_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_members = {
  "aic_task_interfaces__msg",  // message namespace
  "Task",  // message name
  9,  // number of fields
  sizeof(aic_task_interfaces__msg__Task),
  false,  // has_any_key_member_
  aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_member_array,  // message members
  aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_init_function,  // function to initialize message memory (memory has to be allocated)
  aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle = {
  0,
  &aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_members,
  get_message_typesupport_handle_function,
  &aic_task_interfaces__msg__Task__get_type_hash,
  &aic_task_interfaces__msg__Task__get_type_description,
  &aic_task_interfaces__msg__Task__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aic_task_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aic_task_interfaces, msg, Task)() {
  if (!aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle.typesupport_identifier) {
    aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aic_task_interfaces__msg__Task__rosidl_typesupport_introspection_c__Task_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
