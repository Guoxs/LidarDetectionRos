//
// Created by guoxs on 2020/9/24.
//
#include <pcl/common/common.h>
#include "../render/box.h"

const float EPSILON = 1e-6;

template<typename PointT>
class Minbox{
public:
    Minbox();
    
    ~Minbox();

    void ReconstructPolygon(typename pcl::PointCloud<PointT>::Ptr planeHull, BoxQ& box);

private:
    float ComputeAreaAlongOneEdge(typename pcl::PointCloud<PointT>::Ptr planeHull,
                                  size_t first_in_point, Eigen::Vector3f* center,
                                  float* lenth, float* width, Eigen::Vector3f* dir, BoxQ& box);
};

template<typename PointT>
Minbox<PointT>::Minbox() = default;

template<typename PointT>
Minbox<PointT>::~Minbox() = default;


template<typename PointT>
float Minbox<PointT>::ComputeAreaAlongOneEdge(
        typename pcl::PointCloud<PointT>::Ptr planeHull,
        size_t first_in_point, Eigen::Vector3f* center,
        float* lenth, float* width, Eigen::Vector3f* dir,
        BoxQ& box){
    std::vector<Eigen::Vector3f> ns;
    Eigen::Vector3f v(0.0, 0.0, 0.0);
    Eigen::Vector3f vn(0.0, 0.0, 0.0);
    Eigen::Vector3f n(0.0, 0.0, 0.0);
    float len = 0.0;
    float wid = 0.0;
    size_t index = (first_in_point + 1) % planeHull->points.size();
    for (size_t i = 0; i < planeHull->points.size(); ++i) {
        if (i != first_in_point && i != index) {
            // compute v
            Eigen::Vector3f o(0.0, 0.0, 0.0);
            Eigen::Vector3f a(0.0, 0.0, 0.0);
            Eigen::Vector3f b(0.0, 0.0, 0.0);
            o[0] = planeHull->points[i].x;
            o[1] = planeHull->points[i].y;
            o[2] = 0;
            b[0] = planeHull->points[first_in_point].x;
            b[1] = planeHull->points[first_in_point].y;
            b[2] = 0;
            a[0] = planeHull->points[index].x;
            a[1] = planeHull->points[index].y;
            a[2] = 0;
            float k =
                    ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
            k = k / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
            k = k * -1;
            // n is pedal of src
            n[0] = (b[0] - a[0]) * k + a[0];
            n[1] = (b[1] - a[1]) * k + a[1];
            n[2] = 0;
            // compute height from src to line
            Eigen::Vector3f edge1 = o - b;
            Eigen::Vector3f edge2 = a - b;
            // cross product
            float height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
            height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
            if (height > wid) {
                wid = height;
                v = o;
                vn = n;
            }
        } else {
            n[0] = planeHull->points[i].x;
            n[1] = planeHull->points[i].y;
            n[2] = 0;
        }
        ns.push_back(n);
    }
    size_t point_num1 = 0;
    size_t point_num2 = 0;
    for (size_t i = 0; i < ns.size() - 1; ++i) {
        Eigen::Vector3f p1 = ns[i];
        for (size_t j = i + 1; j < ns.size(); ++j) {
            Eigen::Vector3f p2 = ns[j];
            float dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                               (p1[1] - p2[1]) * (p1[1] - p2[1]));
            if (dist > len) {
                len = dist;
                point_num1 = i;
                point_num2 = j;
            }
        }
    }
    Eigen::Vector3f vp1 = v + ns[point_num1] - vn;
    Eigen::Vector3f vp2 = v + ns[point_num2] - vn;
    (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
    (*center)[2] = planeHull->points[0].z;
    box.vertex1 = vp1;
    box.vertex2 = vp2;
    box.vertex3 = ns[point_num1];
    box.vertex4 = ns[point_num2];
    if (len > wid) {
        *dir = ns[point_num2] - ns[point_num1];
    } else {
        *dir = vp1 - ns[point_num1];
    }
    *lenth = len > wid ? len : wid;
    *width = len > wid ? wid : len;
    return (*lenth) * (*width);
}


template<typename PointT>
void Minbox<PointT>::ReconstructPolygon(typename pcl::PointCloud<PointT>::Ptr planeHull, BoxQ& box)
{
    if (planeHull->points.size() <= 0) {
        return;
    }
    size_t max_point_index = 0;
    size_t min_point_index = 0;
    Eigen::Vector3f p;
    p[0] = planeHull->points[0].x;
    p[1] = planeHull->points[0].y;
    p[2] = planeHull->points[0].z;
    
    //ref_ct: origin, default = 0
    const Eigen::Vector3f ref_ct(0.0, 0.0, 0.0);
    Eigen::Vector3f max_point = p - ref_ct;
    Eigen::Vector3f min_point = p - ref_ct;
    for (size_t i = 1; i < planeHull->points.size(); ++i) {
        Eigen::Vector3f p;
        p[0] = planeHull->points[i].x;
        p[1] = planeHull->points[i].y;
        p[2] = planeHull->points[i].z;
        Eigen::Vector3f ray = p - ref_ct;
        // clock direction, find the right-most point
        if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON) {
            max_point = ray;                                           
            max_point_index = i;
        }
        // unclock direction, find the left-most point
        if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON) {
            min_point = ray;
            min_point_index = i;                                
        }
    }
    //以下代码为筛选有效边长,如果相邻的两个点在line的后面,则因为遮挡原因,视这条边为无效边
    //draw a line with left-most point and right-most point
    Eigen::Vector3f line = max_point - min_point;
    float total_len = 0;
    float max_dis = 0;
    bool has_out = false;
    for (size_t i = min_point_index, count = 0;
         count < planeHull->points.size();
         i = (i + 1) % planeHull->points.size(), ++count) {
        Eigen::Vector3f p_x;
        p_x[0] = planeHull->points[i].x;
        p_x[1] = planeHull->points[i].y;
        p_x[2] = planeHull->points[i].z;
        size_t j = (i + 1) % planeHull->points.size();
        if (j != min_point_index && j != max_point_index) {
            Eigen::Vector3f p;
            p[0] = planeHull->points[j].x;
            p[1] = planeHull->points[j].y;
            p[2] = planeHull->points[j].z;
            Eigen::Vector3f ray = p - min_point;
            //j点在line的靠近雷达一侧
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) { 
                float dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist - max_dis > EPSILON) {
                    max_dis = dist;
                }
            } else {
                // outline
                has_out = true;
            }
        } else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index)) {
            size_t k = (j + 1) % planeHull->points.size();
            Eigen::Vector3f p_k;
            p_k[0] = planeHull->points[k].x;
            p_k[1] = planeHull->points[k].y;
            p_k[2] = planeHull->points[k].z;
            Eigen::Vector3f p_j;
            p_j[0] = planeHull->points[j].x;
            p_j[1] = planeHull->points[j].y;
            p_j[2] = planeHull->points[j].z;
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
            } else {
                // outline
                has_out = true;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3f p;
            p[0] = planeHull->points[j].x;
            p[1] = planeHull->points[j].y;
            p[2] = planeHull->points[j].z;
            Eigen::Vector3f ray = p_x - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
                float dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist > max_dis) {
                    max_dis = dist;
                }
            } else {
                // outline
                has_out = true;
            }
        }
    }
    //截止这里,有效边筛选结束
    size_t count = 0;
    float min_area = std::numeric_limits<float>::max();
    for (size_t i = min_point_index; count < planeHull->points.size();
         i = (i + 1) % planeHull->points.size(), ++count) {
        Eigen::Vector3f p_x;
        p_x[0] = planeHull->points[i].x;
        p_x[1] = planeHull->points[i].y;
        p_x[2] = planeHull->points[i].z;
        size_t j = (i + 1) % planeHull->points.size();
        Eigen::Vector3f p_j;
        p_j[0] = planeHull->points[j].x;
        p_j[1] = planeHull->points[j].y;
        p_j[2] = planeHull->points[j].z;
        float dist = sqrt((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) +
                           (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
        if (dist < max_dis && (dist / total_len) < 0.5) {
            continue;
        }
        if (j != min_point_index && j != max_point_index) {
            Eigen::Vector3f p;
            p[0] = planeHull->points[j].x;
            p[1] = planeHull->points[j].y;
            p[2] = planeHull->points[j].z;
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3f center;
                float length = 0;
                float width = 0;
                Eigen::Vector3f dir;
                float area =
                        ComputeAreaAlongOneEdge(planeHull, i, &center, &length, &width, &dir, box);
                if (area < min_area) {
                    box.bboxTransform = center;
                    box.cube_length = length;
                    box.cube_width = width;
                    box.direction = dir;
                    min_area = area;
                }
            } else {
                // outline
            }
        } else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index)) {
            if (!has_out) {
                continue;
            }
            Eigen::Vector3f center;
            float length = 0;
            float width = 0;
            Eigen::Vector3f dir;
            float area =
                    ComputeAreaAlongOneEdge(planeHull, i, &center, &length, &width, &dir, box);
            if (area < min_area) {
                box.bboxTransform = center;
                box.cube_length = length;
                box.cube_width = width;
                box.direction = dir;
                min_area = area;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3f p;
            p[0] = planeHull->points[i].x;
            p[1] = planeHull->points[i].y;
            p[2] = planeHull->points[i].z;
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3f center;
                float length = 0.0;
                float width = 0.0;
                Eigen::Vector3f dir;
                float area =
                        ComputeAreaAlongOneEdge(planeHull, i, &center, &length, &width, &dir, box);
                if (area < min_area) {
                    box.bboxTransform = center;
                    box.cube_length = length;
                    box.cube_width = width;
                    box.direction = dir;
                    min_area = area;
                }
            } else {
                // outline
            }
        }
    }
    box.direction.normalize();
}