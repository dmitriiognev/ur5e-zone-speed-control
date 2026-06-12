"""Shared MediaPipe Pose landmark constants for the operator_detection pipeline."""

# MediaPipe Pose landmark names in model output order (index 0..32).
KEYPOINT_NAMES = [
    'NOSE',
    'LEFT_EYE_INNER', 'LEFT_EYE', 'LEFT_EYE_OUTER',
    'RIGHT_EYE_INNER', 'RIGHT_EYE', 'RIGHT_EYE_OUTER',
    'LEFT_EAR', 'RIGHT_EAR',
    'MOUTH_LEFT', 'MOUTH_RIGHT',
    'LEFT_SHOULDER', 'RIGHT_SHOULDER',
    'LEFT_ELBOW', 'RIGHT_ELBOW',
    'LEFT_WRIST', 'RIGHT_WRIST',
    'LEFT_PINKY', 'RIGHT_PINKY',
    'LEFT_INDEX', 'RIGHT_INDEX',
    'LEFT_THUMB', 'RIGHT_THUMB',
    'LEFT_HIP', 'RIGHT_HIP',
    'LEFT_KNEE', 'RIGHT_KNEE',
    'LEFT_ANKLE', 'RIGHT_ANKLE',
    'LEFT_HEEL', 'RIGHT_HEEL',
    'LEFT_FOOT_INDEX', 'RIGHT_FOOT_INDEX',
]


def is_invalid_joint(position) -> bool:
    """Return True if the joint carries the (0, 0, 0) 'missing / invalid' sentinel."""
    return abs(position.x) < 1e-6 and abs(position.y) < 1e-6 and abs(position.z) < 1e-6
