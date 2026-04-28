#ifndef INTRINSIC_ICON_ACTIONS_AGENT_BRIDGE_INFO_H_
#define INTRINSIC_ICON_ACTIONS_AGENT_BRIDGE_INFO_H_

#include "proto/agent_bridge.pb.h"

namespace intrinsic {
namespace icon {

// Contains information needed by clients to correctly describe the
// agent bridge action.
struct AgentBridgeInfo {
  // AgentBridge action type name and description
  static constexpr char kStreamingCommandName[] = "agent-bridge-motion-update";
  static constexpr char kActionTypeName[] = "intrinsic.agent_bridge";
  static constexpr char kActionDescription[] = "Starts an agent bridge action.";
  static constexpr char kSlotName[] = "arm";
  static constexpr char kSlotDescription[] =
      "The Action sends torque commands to this Part.";
  static constexpr char kForceTorqueSlotName[] = "ft_sensor";
  static constexpr char kForceTorqueSlotDescription[] =
      "If present, the Action reports force/torque readings from this part in "
      "its streaming output.";
  static constexpr char kSecondsSinceLastCommand[] =
      "intrinsic.agent_bridge.seconds_since_last_command";
  static constexpr char kStreamingOutputDescription[] =
      "Reports the current state of AgentBridge. Note that, if the robot does "
      "not have a wrist-mounted Force/Torque sensor, wrench_at_tip is reported "
      "as zero.";

  using StreamingCommand =
      ::intrinsic_proto::icon::actions::proto::MotionUpdate;
  using FixedParams =
      ::intrinsic_proto::icon::actions::proto::AgentBridgeCommand;
  using StreamingOutput =
      ::intrinsic_proto::icon::actions::proto::AgentBridgeStatus;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_AGENT_BRIDGE_INFO_H_