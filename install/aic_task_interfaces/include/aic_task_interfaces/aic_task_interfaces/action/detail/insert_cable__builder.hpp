// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aic_task_interfaces:action/InsertCable.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "aic_task_interfaces/action/insert_cable.hpp"


#ifndef AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__BUILDER_HPP_
#define AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aic_task_interfaces/action/detail/insert_cable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_Goal_task
{
public:
  Init_InsertCable_Goal_task()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aic_task_interfaces::action::InsertCable_Goal task(::aic_task_interfaces::action::InsertCable_Goal::_task_type arg)
  {
    msg_.task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_Goal>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_Goal_task();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_Result_message
{
public:
  explicit Init_InsertCable_Result_message(::aic_task_interfaces::action::InsertCable_Result & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_Result message(::aic_task_interfaces::action::InsertCable_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_Result msg_;
};

class Init_InsertCable_Result_success
{
public:
  Init_InsertCable_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_Result_message success(::aic_task_interfaces::action::InsertCable_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_InsertCable_Result_message(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_Result>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_Result_success();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_Feedback_message
{
public:
  Init_InsertCable_Feedback_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aic_task_interfaces::action::InsertCable_Feedback message(::aic_task_interfaces::action::InsertCable_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_Feedback>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_Feedback_message();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_SendGoal_Request_goal
{
public:
  explicit Init_InsertCable_SendGoal_Request_goal(::aic_task_interfaces::action::InsertCable_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_SendGoal_Request goal(::aic_task_interfaces::action::InsertCable_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Request msg_;
};

class Init_InsertCable_SendGoal_Request_goal_id
{
public:
  Init_InsertCable_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_SendGoal_Request_goal goal_id(::aic_task_interfaces::action::InsertCable_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_InsertCable_SendGoal_Request_goal(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_SendGoal_Request>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_SendGoal_Request_goal_id();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_SendGoal_Response_stamp
{
public:
  explicit Init_InsertCable_SendGoal_Response_stamp(::aic_task_interfaces::action::InsertCable_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_SendGoal_Response stamp(::aic_task_interfaces::action::InsertCable_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Response msg_;
};

class Init_InsertCable_SendGoal_Response_accepted
{
public:
  Init_InsertCable_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_SendGoal_Response_stamp accepted(::aic_task_interfaces::action::InsertCable_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_InsertCable_SendGoal_Response_stamp(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_SendGoal_Response>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_SendGoal_Response_accepted();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_SendGoal_Event_response
{
public:
  explicit Init_InsertCable_SendGoal_Event_response(::aic_task_interfaces::action::InsertCable_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_SendGoal_Event response(::aic_task_interfaces::action::InsertCable_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Event msg_;
};

class Init_InsertCable_SendGoal_Event_request
{
public:
  explicit Init_InsertCable_SendGoal_Event_request(::aic_task_interfaces::action::InsertCable_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_InsertCable_SendGoal_Event_response request(::aic_task_interfaces::action::InsertCable_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_InsertCable_SendGoal_Event_response(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Event msg_;
};

class Init_InsertCable_SendGoal_Event_info
{
public:
  Init_InsertCable_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_SendGoal_Event_request info(::aic_task_interfaces::action::InsertCable_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_InsertCable_SendGoal_Event_request(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_SendGoal_Event>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_SendGoal_Event_info();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_GetResult_Request_goal_id
{
public:
  Init_InsertCable_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aic_task_interfaces::action::InsertCable_GetResult_Request goal_id(::aic_task_interfaces::action::InsertCable_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_GetResult_Request>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_GetResult_Request_goal_id();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_GetResult_Response_result
{
public:
  explicit Init_InsertCable_GetResult_Response_result(::aic_task_interfaces::action::InsertCable_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_GetResult_Response result(::aic_task_interfaces::action::InsertCable_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Response msg_;
};

class Init_InsertCable_GetResult_Response_status
{
public:
  Init_InsertCable_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_GetResult_Response_result status(::aic_task_interfaces::action::InsertCable_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_InsertCable_GetResult_Response_result(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_GetResult_Response>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_GetResult_Response_status();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_GetResult_Event_response
{
public:
  explicit Init_InsertCable_GetResult_Event_response(::aic_task_interfaces::action::InsertCable_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_GetResult_Event response(::aic_task_interfaces::action::InsertCable_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Event msg_;
};

class Init_InsertCable_GetResult_Event_request
{
public:
  explicit Init_InsertCable_GetResult_Event_request(::aic_task_interfaces::action::InsertCable_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_InsertCable_GetResult_Event_response request(::aic_task_interfaces::action::InsertCable_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_InsertCable_GetResult_Event_response(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Event msg_;
};

class Init_InsertCable_GetResult_Event_info
{
public:
  Init_InsertCable_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_GetResult_Event_request info(::aic_task_interfaces::action::InsertCable_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_InsertCable_GetResult_Event_request(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_GetResult_Event>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_GetResult_Event_info();
}

}  // namespace aic_task_interfaces


namespace aic_task_interfaces
{

namespace action
{

namespace builder
{

class Init_InsertCable_FeedbackMessage_feedback
{
public:
  explicit Init_InsertCable_FeedbackMessage_feedback(::aic_task_interfaces::action::InsertCable_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::aic_task_interfaces::action::InsertCable_FeedbackMessage feedback(::aic_task_interfaces::action::InsertCable_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_FeedbackMessage msg_;
};

class Init_InsertCable_FeedbackMessage_goal_id
{
public:
  Init_InsertCable_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InsertCable_FeedbackMessage_feedback goal_id(::aic_task_interfaces::action::InsertCable_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_InsertCable_FeedbackMessage_feedback(msg_);
  }

private:
  ::aic_task_interfaces::action::InsertCable_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::aic_task_interfaces::action::InsertCable_FeedbackMessage>()
{
  return aic_task_interfaces::action::builder::Init_InsertCable_FeedbackMessage_goal_id();
}

}  // namespace aic_task_interfaces

#endif  // AIC_TASK_INTERFACES__ACTION__DETAIL__INSERT_CABLE__BUILDER_HPP_
