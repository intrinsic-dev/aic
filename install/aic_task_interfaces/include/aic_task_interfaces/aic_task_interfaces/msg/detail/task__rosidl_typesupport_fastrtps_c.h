// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice
#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "aic_task_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aic_task_interfaces/msg/detail/task__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
bool cdr_serialize_aic_task_interfaces__msg__Task(
  const aic_task_interfaces__msg__Task * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
bool cdr_deserialize_aic_task_interfaces__msg__Task(
  eprosima::fastcdr::Cdr &,
  aic_task_interfaces__msg__Task * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
size_t get_serialized_size_aic_task_interfaces__msg__Task(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
size_t max_serialized_size_aic_task_interfaces__msg__Task(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
bool cdr_serialize_key_aic_task_interfaces__msg__Task(
  const aic_task_interfaces__msg__Task * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
size_t get_serialized_size_key_aic_task_interfaces__msg__Task(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
size_t max_serialized_size_key_aic_task_interfaces__msg__Task(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aic_task_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aic_task_interfaces, msg, Task)();

#ifdef __cplusplus
}
#endif

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
