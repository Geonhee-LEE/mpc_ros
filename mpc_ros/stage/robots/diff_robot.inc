define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 180.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)


define mobile_robot position
(

  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
