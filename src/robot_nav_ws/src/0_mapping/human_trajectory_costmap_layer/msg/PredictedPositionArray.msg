# PredictedPositionArray.msg
#
# This message aggregates multiple predicted positions with their
# associated uncertainty and confidence levels.  The header allows
# time‑stamping and frame identification.  Each element in the
# positions array contains a `PredictedPosition` describing the mean
# pose, confidence level and uncertainty of that prediction.

std_msgs/Header header
human_trajectory_costmap_layer/PredictedPosition[] positions