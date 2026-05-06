// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/msg/task.hpp"


#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__TRAITS_HPP_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aic_task_interfaces/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aic_task_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Task & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: cable_type
  {
    out << "cable_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cable_type, out);
    out << ", ";
  }

  // member: cable_name
  {
    out << "cable_name: ";
    rosidl_generator_traits::value_to_yaml(msg.cable_name, out);
    out << ", ";
  }

  // member: plug_type
  {
    out << "plug_type: ";
    rosidl_generator_traits::value_to_yaml(msg.plug_type, out);
    out << ", ";
  }

  // member: plug_name
  {
    out << "plug_name: ";
    rosidl_generator_traits::value_to_yaml(msg.plug_name, out);
    out << ", ";
  }

  // member: port_type
  {
    out << "port_type: ";
    rosidl_generator_traits::value_to_yaml(msg.port_type, out);
    out << ", ";
  }

  // member: port_name
  {
    out << "port_name: ";
    rosidl_generator_traits::value_to_yaml(msg.port_name, out);
    out << ", ";
  }

  // member: target_module_name
  {
    out << "target_module_name: ";
    rosidl_generator_traits::value_to_yaml(msg.target_module_name, out);
    out << ", ";
  }

  // member: time_limit
  {
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: cable_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cable_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cable_type, out);
    out << "\n";
  }

  // member: cable_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cable_name: ";
    rosidl_generator_traits::value_to_yaml(msg.cable_name, out);
    out << "\n";
  }

  // member: plug_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plug_type: ";
    rosidl_generator_traits::value_to_yaml(msg.plug_type, out);
    out << "\n";
  }

  // member: plug_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plug_name: ";
    rosidl_generator_traits::value_to_yaml(msg.plug_name, out);
    out << "\n";
  }

  // member: port_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port_type: ";
    rosidl_generator_traits::value_to_yaml(msg.port_type, out);
    out << "\n";
  }

  // member: port_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port_name: ";
    rosidl_generator_traits::value_to_yaml(msg.port_name, out);
    out << "\n";
  }

  // member: target_module_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_module_name: ";
    rosidl_generator_traits::value_to_yaml(msg.target_module_name, out);
    out << "\n";
  }

  // member: time_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Task & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aic_task_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aic_task_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aic_task_interfaces::msg::Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  aic_task_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aic_task_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const aic_task_interfaces::msg::Task & msg)
{
  return aic_task_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aic_task_interfaces::msg::Task>()
{
  return "aic_task_interfaces::msg::Task";
}

template<>
inline const char * name<aic_task_interfaces::msg::Task>()
{
  return "aic_task_interfaces/msg/Task";
}

template<>
struct has_fixed_size<aic_task_interfaces::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aic_task_interfaces::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aic_task_interfaces::msg::Task>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__TRAITS_HPP_
