# PredictedPosition.msg
#
# This message represents a single predicted human position along with
# uncertainty information. Each prediction contains the predicted pose
# (position and orientation), the confidence level of the prediction,
# and the standard deviations along the x and y axes.  The confidence
# level corresponds to the probability mass of the confidence set as
# discussed in the paper on reliable probabilistic human trajectory
# prediction【567447048113654†L319-L331】.  Higher confidence levels imply
# larger uncertainty regions.

geometry_msgs/Pose pose
float32 confidence_level
float32 sigma_x
float32 sigma_y