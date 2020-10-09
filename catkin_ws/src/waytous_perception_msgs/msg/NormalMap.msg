Header                  header
float32                 ogmwidth            #二维栅格地图宽
float32                 ogmheight           #二维栅格地图高
int16                   vehicle_x           #车辆在栅格图中的水平位置
int16                   vehicle_y           #车辆在栅格图中的竖直位置
float32                 rectangle_minx      #拟合矩形左边界
float32                 rectangle_maxx      #拟合矩形右边界
float32                 rectangle_miny      #拟合矩形下边界
float32                 rectangle_maxy      #拟合矩形上边界
float32                 slopeupdegree       #坡度值
float32                 slopedestination    #斜坡方向（0-360度）
bool                    vehiclepositionflag #斜坡类型（可用来判断是否为斜坡点，及正坡/负坡）

geometry_msgs/Vector3   normal              #normal
sensor_msgs/PointCloud2 pointcloud_with_normal

