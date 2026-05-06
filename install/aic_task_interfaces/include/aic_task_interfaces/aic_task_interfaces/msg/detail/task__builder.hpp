// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/msg/task.hpp"


#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aic_task_interfaces/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aic_task_interfaces
{

namespace msg
{

namespace builder
{

class Init_Task_time_limit
{
public:
  explicit Init_Task_time_limit(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::msg::Task time_limit(::aic_task_interfaces::msg::Task::_time_limit_type arg)
  {
    msg_.time_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_target_module_name
{
public:
  explicit Init_Task_target_module_name(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_time_limit target_module_name(::aic_task_interfaces::msg::Task::_target_module_name_type arg)
  {
    msg_.target_module_name = std::move(arg);
    return Init_Task_time_limit(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_port_name
{
public:
  explicit Init_Task_port_name(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_target_module_name port_name(::aic_task_interfaces::msg::Task::_port_name_type arg)
  {
    msg_.port_name = std::move(arg);
    return Init_Task_target_module_name(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_port_type
{
public:
  explicit Init_Task_port_type(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_port_name port_type(::aic_task_interfaces::msg::Task::_port_type_type arg)
  {
    msg_.port_type = std::move(arg);
    return Init_Task_port_name(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_plug_name
{
public:
  explicit Init_Task_plug_name(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_port_type plug_name(::aic_task_interfaces::msg::Task::_plug_name_type arg)
  {
    msg_.plug_name = std::move(arg);
    return Init_Task_port_type(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_plug_type
{
public:
  explicit Init_Task_plug_type(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_plug_name plug_type(::aic_task_interfaces::msg::Task::_plug_type_type arg)
  {
    msg_.plug_type = std::move(arg);
    return Init_Task_plug_name(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_cable_name
{
public:
  explicit Init_Task_cable_name(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_plug_type cable_name(::aic_task_interfaces::msg::Task::_cable_name_type arg)
  {
    msg_.cable_name = std::move(arg);
    return Init_Task_plug_type(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_cable_type
{
public:
  explicit Init_Task_cable_type(::aic_task_interfaces::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_cable_name cable_type(::aic_task_interfaces::msg::Task::_cable_type_type arg)
  {
    msg_.cable_type = std::move(arg);
    return Init_Task_cable_name(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

class Init_Task_id
{
public:
  Init_Task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_cable_type id(::aic_task_interfaces::msg::Task::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Task_cable_type(msg_);
  }

private:
  ::aic_task_interfaces::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::msg::Task>()
{
  return aic_task_interfaces::msg::builder::Init_Task_id();
}

}  // namespace aic_task_interfaces

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__BUILDER_HPP_
