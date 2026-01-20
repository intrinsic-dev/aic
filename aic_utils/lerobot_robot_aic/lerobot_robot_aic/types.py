from typing import Any, TypedDict

MotionUpdateActionDict = TypedDict(
    "MotionUpdateActionDict",
    {
        "linear.x": float,
        "linear.y": float,
        "linear.z": float,
        "angular.x": float,
        "angular.y": float,
        "angular.z": float,
        "gripper_width_percent": float,
    },
)


def motion_update_action_features() -> dict[str, Any]:
    return {
        "linear.x": float,
        "linear.y": float,
        "linear.z": float,
        "angular.x": float,
        "angular.y": float,
        "angular.z": float,
        "gripper_width_percent": float,
    }
