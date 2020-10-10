//
// Created by guoxs on 2020/10/9.
//

#ifndef SRC_GLOBAL_H
#define SRC_GLOBAL_H

//set point cloud range
Eigen::Vector4f minPoint(0, -55, -5, 1);
Eigen::Vector4f maxPoint( 100, 100, 15, 1);
//whether recreate background file
bool recordBackground = false;
//background frame number
int backgroundNum = 50;

#endif //SRC_GLOBAL_H
