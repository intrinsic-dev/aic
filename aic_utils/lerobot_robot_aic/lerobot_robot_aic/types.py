from typing import TypedDict

MotionUpdateActionDict = TypedDict(
    "MotionUpdateActionDict",
    {
        "linear.x": float,
        "linear.y": float,
        "linear.z": float,
        "angular.x": float,
        "angular.y": float,
        "angular.z": float,
        "gripper_target": float,
    },
)
