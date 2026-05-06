// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aic_task_interfaces:action/InsertCable.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/action/insert_cable.h"


#ifndef AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__STRUCT_H_
#define AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task'
#include "aic_task_interfaces/msg/detail/task__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_Goal
{
  aic_task_interfaces__msg__Task task;
} aic_task_interfaces__action__InsertCable_Goal;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_Goal.
typedef struct aic_task_interfaces__action__InsertCable_Goal__Sequence
{
  aic_task_interfaces__action__InsertCable_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_Result
{
  /// Response
  /// True if the task succeeded.
  bool success;
  /// A message if the task succeeded or failed.
  rosidl_runtime_c__String message;
} aic_task_interfaces__action__InsertCable_Result;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_Result.
typedef struct aic_task_interfaces__action__InsertCable_Result__Sequence
{
  aic_task_interfaces__action__InsertCable_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_Feedback
{
  /// Feedback
  /// A description on how the task is progressing.
  rosidl_runtime_c__String message;
} aic_task_interfaces__action__InsertCable_Feedback;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_Feedback.
typedef struct aic_task_interfaces__action__InsertCable_Feedback__Sequence
{
  aic_task_interfaces__action__InsertCable_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "aic_task_interfaces/action/detail/insert_cable__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  aic_task_interfaces__action__InsertCable_Goal goal;
} aic_task_interfaces__action__InsertCable_SendGoal_Request;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_SendGoal_Request.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Request__Sequence
{
  aic_task_interfaces__action__InsertCable_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} aic_task_interfaces__action__InsertCable_SendGoal_Response;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_SendGoal_Response.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Response__Sequence
{
  aic_task_interfaces__action__InsertCable_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  aic_task_interfaces__action__InsertCable_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  aic_task_interfaces__action__InsertCable_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  aic_task_interfaces__action__InsertCable_SendGoal_Request__Sequence request;
  aic_task_interfaces__action__InsertCable_SendGoal_Response__Sequence response;
} aic_task_interfaces__action__InsertCable_SendGoal_Event;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_SendGoal_Event.
typedef struct aic_task_interfaces__action__InsertCable_SendGoal_Event__Sequence
{
  aic_task_interfaces__action__InsertCable_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} aic_task_interfaces__action__InsertCable_GetResult_Request;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_GetResult_Request.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Request__Sequence
{
  aic_task_interfaces__action__InsertCable_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "aic_task_interfaces/action/detail/insert_cable__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Response
{
  int8_t status;
  aic_task_interfaces__action__InsertCable_Result result;
} aic_task_interfaces__action__InsertCable_GetResult_Response;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_GetResult_Response.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Response__Sequence
{
  aic_task_interfaces__action__InsertCable_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  aic_task_interfaces__action__InsertCable_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  aic_task_interfaces__action__InsertCable_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  aic_task_interfaces__action__InsertCable_GetResult_Request__Sequence request;
  aic_task_interfaces__action__InsertCable_GetResult_Response__Sequence response;
} aic_task_interfaces__action__InsertCable_GetResult_Event;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_GetResult_Event.
typedef struct aic_task_interfaces__action__InsertCable_GetResult_Event__Sequence
{
  aic_task_interfaces__action__InsertCable_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "aic_task_interfaces/action/detail/insert_cable__struct.h"

/// Struct defined in action/InsertCable in the package aic_task_interfaces.
typedef struct aic_task_interfaces__action__InsertCable_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  aic_task_interfaces__action__InsertCable_Feedback feedback;
} aic_task_interfaces__action__InsertCable_FeedbackMessage;

// Struct for a sequence of aic_task_interfaces__action__InsertCable_FeedbackMessage.
typedef struct aic_task_interfaces__action__InsertCable_FeedbackMessage__Sequence
{
  aic_task_interfaces__action__InsertCable_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aic_task_interfaces__action__InsertCable_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__STRUCT_H_
