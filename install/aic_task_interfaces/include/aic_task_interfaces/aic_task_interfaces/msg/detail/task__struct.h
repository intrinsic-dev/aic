// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/msg/task.h"


#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'id'
// Member 'cable_type'
// Member 'cable_name'
// Member 'plug_type'
// Member 'plug_name'
// Member 'port_type'
// Member 'port_name'
// Member 'target_module_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Task in the package aic_task_interfaces.
/**
  * A unique ID for the cable insertion task.
 */
typedef struct aic_task_interfaces__msg__Task
{
  rosidl_runtime_c__String id;
  /// The type of cable which should be inserted.
  rosidl_runtime_c__String cable_type;
  /// The name of the cable which should be inserted, e.g., "sfp_sc".
  rosidl_runtime_c__String cable_name;
  /// The type of plug at the end of the cable which should be inserted, e.g., "sfp".
  rosidl_runtime_c__String plug_type;
  /// The name of plug at the end of the cable which should be inserted, e.g., "sfp_module".
  rosidl_runtime_c__String plug_name;
  /// The type of port on the module side into which the plug should be inserted, e.g., "sfp".
  rosidl_runtime_c__String port_type;
  /// The name of the port on the module side into which the plug should be inserted, e.g., "sfp_port_0".
  rosidl_runtime_c__String port_name;
  /// The name of the component or module on which the port exists, e.g., "nic_card_0"
  rosidl_runtime_c__String target_module_name;
  /// The number of seconds from the moment this request is received by when the task must complete.
  /// todo(yadunund): should this be a system clock timestamp instead.
  uint64_t time_limit;
} aic_task_interfaces__msg__Task;

// Struct for a sequence of aic_task_interfaces__msg__Task.
typedef struct aic_task_interfaces__msg__Task__Sequence
{
  aic_task_interfaces__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_H_
