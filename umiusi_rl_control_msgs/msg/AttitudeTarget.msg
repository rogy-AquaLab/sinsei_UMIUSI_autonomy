# Attitude(-velocity) setpoint for a low-level attitude-hold controller (e.g. rl_attitude_node).
# Modeled on mavros_msgs/AttitudeTarget: an ABSOLUTE attitude target + a feed-forward body velocity.
# This is the setpoint of the inner control loop, NOT a velocity/rate command (use
# geometry_msgs/Twist / cmd_vel for the pure-allocation / velocity path).

std_msgs/Header header

# type_mask: bit-OR of the IGNORE_* flags below. 0 = use both attitude and velocity.
uint8 type_mask
uint8 IGNORE_ATTITUDE = 1   # velocity only: do not hold an absolute attitude
uint8 IGNORE_VELOCITY = 2   # attitude only: zero the feed-forward velocity

geometry_msgs/Quaternion orientation   # target attitude (identity = upright / level)
geometry_msgs/Vector3 velocity         # feed-forward velocity in the target-body frame [m/s]
