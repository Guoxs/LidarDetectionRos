#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
    Eigen::Vector3f direction;
	float cube_length;
    float cube_width;
    float cube_height;

    Eigen::Vector3f vertex1;
    Eigen::Vector3f vertex2;
    Eigen::Vector3f vertex3;
    Eigen::Vector3f vertex4;
};
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};
#endif