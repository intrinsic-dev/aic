// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aic_task_interfaces:msg/Task.idl
// generated code does not contain a copyright notice
#include "aic_task_interfaces/msg/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


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

bool
aic_task_interfaces__msg__Task__init(aic_task_interfaces__msg__Task * msg)
{
  if (!msg) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // cable_type
  if (!rosidl_runtime_c__String__init(&msg->cable_type)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // cable_name
  if (!rosidl_runtime_c__String__init(&msg->cable_name)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // plug_type
  if (!rosidl_runtime_c__String__init(&msg->plug_type)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // plug_name
  if (!rosidl_runtime_c__String__init(&msg->plug_name)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // port_type
  if (!rosidl_runtime_c__String__init(&msg->port_type)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // port_name
  if (!rosidl_runtime_c__String__init(&msg->port_name)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // target_module_name
  if (!rosidl_runtime_c__String__init(&msg->target_module_name)) {
    aic_task_interfaces__msg__Task__fini(msg);
    return false;
  }
  // time_limit
  return true;
}

void
aic_task_interfaces__msg__Task__fini(aic_task_interfaces__msg__Task * msg)
{
  if (!msg) {
    return;
  }
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // cable_type
  rosidl_runtime_c__String__fini(&msg->cable_type);
  // cable_name
  rosidl_runtime_c__String__fini(&msg->cable_name);
  // plug_type
  rosidl_runtime_c__String__fini(&msg->plug_type);
  // plug_name
  rosidl_runtime_c__String__fini(&msg->plug_name);
  // port_type
  rosidl_runtime_c__String__fini(&msg->port_type);
  // port_name
  rosidl_runtime_c__String__fini(&msg->port_name);
  // target_module_name
  rosidl_runtime_c__String__fini(&msg->target_module_name);
  // time_limit
}

bool
aic_task_interfaces__msg__Task__are_equal(const aic_task_interfaces__msg__Task * lhs, const aic_task_interfaces__msg__Task * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // cable_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cable_type), &(rhs->cable_type)))
  {
    return false;
  }
  // cable_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cable_name), &(rhs->cable_name)))
  {
    return false;
  }
  // plug_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->plug_type), &(rhs->plug_type)))
  {
    return false;
  }
  // plug_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->plug_name), &(rhs->plug_name)))
  {
    return false;
  }
  // port_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->port_type), &(rhs->port_type)))
  {
    return false;
  }
  // port_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->port_name), &(rhs->port_name)))
  {
    return false;
  }
  // target_module_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_module_name), &(rhs->target_module_name)))
  {
    return false;
  }
  // time_limit
  if (lhs->time_limit != rhs->time_limit) {
    return false;
  }
  return true;
}

bool
aic_task_interfaces__msg__Task__copy(
  const aic_task_interfaces__msg__Task * input,
  aic_task_interfaces__msg__Task * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // cable_type
  if (!rosidl_runtime_c__String__copy(
      &(input->cable_type), &(output->cable_type)))
  {
    return false;
  }
  // cable_name
  if (!rosidl_runtime_c__String__copy(
      &(input->cable_name), &(output->cable_name)))
  {
    return false;
  }
  // plug_type
  if (!rosidl_runtime_c__String__copy(
      &(input->plug_type), &(output->plug_type)))
  {
    return false;
  }
  // plug_name
  if (!rosidl_runtime_c__String__copy(
      &(input->plug_name), &(output->plug_name)))
  {
    return false;
  }
  // port_type
  if (!rosidl_runtime_c__String__copy(
      &(input->port_type), &(output->port_type)))
  {
    return false;
  }
  // port_name
  if (!rosidl_runtime_c__String__copy(
      &(input->port_name), &(output->port_name)))
  {
    return false;
  }
  // target_module_name
  if (!rosidl_runtime_c__String__copy(
      &(input->target_module_name), &(output->target_module_name)))
  {
    return false;
  }
  // time_limit
  output->time_limit = input->time_limit;
  return true;
}

aic_task_interfaces__msg__Task *
aic_task_interfaces__msg__Task__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aic_task_interfaces__msg__Task * msg = (aic_task_interfaces__msg__Task *)allocator.allocate(sizeof(aic_task_interfaces__msg__Task), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aic_task_interfaces__msg__Task));
  bool success = aic_task_interfaces__msg__Task__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aic_task_interfaces__msg__Task__destroy(aic_task_interfaces__msg__Task * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aic_task_interfaces__msg__Task__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aic_task_interfaces__msg__Task__Sequence__init(aic_task_interfaces__msg__Task__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aic_task_interfaces__msg__Task * data = NULL;

  if (size) {
    data = (aic_task_interfaces__msg__Task *)allocator.zero_allocate(size, sizeof(aic_task_interfaces__msg__Task), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aic_task_interfaces__msg__Task__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aic_task_interfaces__msg__Task__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
aic_task_interfaces__msg__Task__Sequence__fini(aic_task_interfaces__msg__Task__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      aic_task_interfaces__msg__Task__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

aic_task_interfaces__msg__Task__Sequence *
aic_task_interfaces__msg__Task__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aic_task_interfaces__msg__Task__Sequence * array = (aic_task_interfaces__msg__Task__Sequence *)allocator.allocate(sizeof(aic_task_interfaces__msg__Task__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aic_task_interfaces__msg__Task__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aic_task_interfaces__msg__Task__Sequence__destroy(aic_task_interfaces__msg__Task__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aic_task_interfaces__msg__Task__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aic_task_interfaces__msg__Task__Sequence__are_equal(const aic_task_interfaces__msg__Task__Sequence * lhs, const aic_task_interfaces__msg__Task__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aic_task_interfaces__msg__Task__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aic_task_interfaces__msg__Task__Sequence__copy(
  const aic_task_interfaces__msg__Task__Sequence * input,
  aic_task_interfaces__msg__Task__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aic_task_interfaces__msg__Task);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aic_task_interfaces__msg__Task * data =
      (aic_task_interfaces__msg__Task *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aic_task_interfaces__msg__Task__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aic_task_interfaces__msg__Task__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aic_task_interfaces__msg__Task__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
