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

JointMotionUpdateActionDict = TypedDict(
    "JointMotionUpdateActionDict",
    {
        "shoulder_pan_joint": float,
        "shoulder_lift_joint": float,
        "elbow_joint": float,
        "wrist_1_joint": float,
        "wrist_2_joint": float,
        "wrist_3_joint": float,
        "gripper_target": float,
    },
)
