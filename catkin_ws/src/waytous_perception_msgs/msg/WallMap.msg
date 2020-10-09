std_msgs/Header         header
int16                   ogmwidth        #二维栅格地图宽
int16                   ogmheight       #二维栅格地图高
int16                   vehicle_x       #车辆在栅格图中的水平位置
int16                   vehicle_y       #车辆在栅格图中的竖直位置
float32                 ogmresolution   #栅格分辨率
int16[]                 data            #栅格点（像素为1和2的栅格为挡墙）