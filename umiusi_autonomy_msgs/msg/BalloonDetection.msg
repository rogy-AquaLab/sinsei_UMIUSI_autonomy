# One detected balloon in a single onboard-camera frame, as produced by the learned detector
# (umiusi_perception.learned_detector.Detection). Bearings are vs. the optic axis, radians.
string  colour       # "red" | "yellow" | "blue"
int32   points       # scoring value of that colour (red +30 / yellow +10 / blue -10)
float32 azimuth      # bearing azimuth [rad]  (+ = toward image right / body +Z)
float32 elevation    # bearing elevation [rad] (+ = above the optic axis)
float32 range_m      # estimated distance to the balloon centre [m]
float32 confidence   # detector confidence 0..1
int32[4] bbox        # (u0, v0, u1, v1) pixel box, inclusive-exclusive
float32[2] centroid  # (u, v) box centroid in pixels
int32   area_px      # bbox area in pixels (apparent size)
