// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "aic_task_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "aic_task_interfaces/msg/detail/task__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace aic_task_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
cdr_serialize(
  const aic_task_interfaces::msg::Task & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  aic_task_interfaces::msg::Task & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
get_serialized_size(
  const aic_task_interfaces::msg::Task & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
max_serialized_size_Task(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
cdr_serialize_key(
  const aic_task_interfaces::msg::Task & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
get_serialized_size_key(
  const aic_task_interfaces::msg::Task & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
max_serialized_size_key_Task(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace aic_task_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_aic_task_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, aic_task_interfaces, msg, Task)();

#ifdef __cplusplus
}
#endif

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
