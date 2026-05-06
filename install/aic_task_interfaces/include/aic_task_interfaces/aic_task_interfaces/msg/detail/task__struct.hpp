// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/msg/task.hpp"


#ifndef AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_
#define AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__aic_task_interfaces__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__aic_task_interfaces__msg__Task __declspec(deprecated)
#endif

namespace aic_task_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->cable_type = "";
      this->cable_name = "";
      this->plug_type = "";
      this->plug_name = "";
      this->port_type = "";
      this->port_name = "";
      this->target_module_name = "";
      this->time_limit = 0ull;
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id(_alloc),
    cable_type(_alloc),
    cable_name(_alloc),
    plug_type(_alloc),
    plug_name(_alloc),
    port_type(_alloc),
    port_name(_alloc),
    target_module_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
      this->cable_type = "";
      this->cable_name = "";
      this->plug_type = "";
      this->plug_name = "";
      this->port_type = "";
      this->port_name = "";
      this->target_module_name = "";
      this->time_limit = 0ull;
    }
  }

  // field types and members
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;
  using _cable_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cable_type_type cable_type;
  using _cable_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cable_name_type cable_name;
  using _plug_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _plug_type_type plug_type;
  using _plug_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _plug_name_type plug_name;
  using _port_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _port_type_type port_type;
  using _port_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _port_name_type port_name;
  using _target_module_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_module_name_type target_module_name;
  using _time_limit_type =
    uint64_t;
  _time_limit_type time_limit;

  // setters for named parameter idiom
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__cable_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cable_type = _arg;
    return *this;
  }
  Type & set__cable_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cable_name = _arg;
    return *this;
  }
  Type & set__plug_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->plug_type = _arg;
    return *this;
  }
  Type & set__plug_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->plug_name = _arg;
    return *this;
  }
  Type & set__port_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->port_type = _arg;
    return *this;
  }
  Type & set__port_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->port_name = _arg;
    return *this;
  }
  Type & set__target_module_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target_module_name = _arg;
    return *this;
  }
  Type & set__time_limit(
    const uint64_t & _arg)
  {
    this->time_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aic_task_interfaces::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const aic_task_interfaces::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aic_task_interfaces::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aic_task_interfaces::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aic_task_interfaces__msg__Task
    std::shared_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aic_task_interfaces__msg__Task
    std::shared_ptr<aic_task_interfaces::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->cable_type != other.cable_type) {
      return false;
    }
    if (this->cable_name != other.cable_name) {
      return false;
    }
    if (this->plug_type != other.plug_type) {
      return false;
    }
    if (this->plug_name != other.plug_name) {
      return false;
    }
    if (this->port_type != other.port_type) {
      return false;
    }
    if (this->port_name != other.port_name) {
      return false;
    }
    if (this->target_module_name != other.target_module_name) {
      return false;
    }
    if (this->time_limit != other.time_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  aic_task_interfaces::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace aic_task_interfaces

#endif  // AIC_TASK_INTERFACES__MSG__DETAIL__TASK__STRUCT_HPP_
