int8                                    sensor_type             # Object sensor source, required
int8                                    id                      # Object ID, required
int8                                    label_type              # Classification, optional
############### 2D Rect
waytous_perception_msgs/Rect            rect                    # [x,y,w,h], required
float32                                 orientation             # Radian [-pi, pi], orientation of the object, optional
############### 3D Dimension
geometry_msgs/Pose                      pose                    # the pose of the center of the object, position is required, quaternion is optional
geometry_msgs/Vector3                   dimensions              # required
sensor_msgs/PointCloud2                 pointcloud              # the points in the object, optional
geometry_msgs/Polygon                   convex_hull             # the hull of the obejct, required
############### confidence use in common
float64                                 score                   # required
############### tracking info
int8                                    track_id                # required
geometry_msgs/Vector3                   velocity                # required
float64[]                               velocity_covariance     # required
float64                                 velocity_confidence     # required
int8                                    motion_state            # optional
geometry_msgs/Vector3[]                 trace                   # required
#---------------------sensor_type--------------------
int8 SENSOR_INVALID = 0
int8 SENSOR_LIDAR = 1
int8 SENSOR_CAMERA_LEFT = 2
int8 SENSOR_CAMERA_MID = 3
int8 SENSOR_CAMERA_RIGHT = 4
int8 SENSOR_RADAR = 5
int8 SENSOR_V2X=6
#---------------------label_type--------------------
int8 TYPE_UNKNOWN = 0
int8 TYPE_PEDESTRIAN = 1
int8 TYPE_CAR = 2
int8 TYPE_TRUCK = 3
int8 TYPE_DUMPER = 4
int8 TYPE_EXCAVATOR = 5
int8 TYPE_FLUSHER = 6
int8 TYPE_GRADER = 7
int8 TYPE_DOZER = 8
#---------------------motion--------------------
int8 MOTION_UNKNOWN = 0
int8 MOTION_MOVING = 1
int8 MOTION_STATIONARY = 2

