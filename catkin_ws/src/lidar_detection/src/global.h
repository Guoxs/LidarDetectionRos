//
// Created by guoxs on 2020/10/9.
//

#ifndef SRC_GLOBAL_H
#define SRC_GLOBAL_H

//set point cloud range
Eigen::Vector4f minPoint(-10, -80, -5, 1);
Eigen::Vector4f maxPoint( 150, 150, 15, 1);
//whether recreate background file
bool recordBackground = false;
//background frame number
int backgroundNum = 50;

#endif //SRC_GLOBAL_H
