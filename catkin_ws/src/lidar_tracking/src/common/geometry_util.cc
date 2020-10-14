
#include "geometry_util.h"


void ConvertPolygon2Cloud(const geometry_msgs::Polygon& polygon, PointCloudType& cloud)
{
  cloud.points.clear();
  for (size_t i = 0; i < polygon.points.size(); i++)
  {
    PointType point;
    point.x=polygon.points[i].x;
    point.y=polygon.points[i].y;
    point.z=polygon.points[i].z;
    cloud.points.push_back(point); 
  }
  
}

void ConvertCloud2Polygon(const PointCloudType& cloud,geometry_msgs::Polygon& polygon)
{
  polygon.points.clear();
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x=cloud.points[i].x;
    point.y=cloud.points[i].y;
    point.z=cloud.points[i].z;
    polygon.points.push_back(point); 
    
  }
  
}

template <typename T>
T Clamp(const T value, T bound1, T bound2) 
{
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}



void TransformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         const std::vector<int>& indices,
                         pcl::PointCloud<pcl::PointXYZI>* trans_cloud) 
{
  // cout<<"indices size is  "<<indices.size()<<endl;
  // cout<<"plan hull size is  "<<cloud->points.size()<<endl;
    if (trans_cloud->points.size() != indices.size()) 
    {
        trans_cloud->points.resize(indices.size());
    }
    for (size_t i = 0; i < indices.size(); ++i) 
    {
        const pcl::PointXYZI& p = cloud->at(indices[i]);
        trans_cloud->points.at(i).x = p.x;
        trans_cloud->points.at(i).y = p.y;
        trans_cloud->points.at(i).z = p.z;
      
    }
}


void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
                         geometry_msgs::PolygonStamped* cloud_in_out) 
{
  
  for (std::size_t i = 0; i<cloud_in_out->polygon.points.size();++i) 
  {
    
    Eigen::Vector4f v;
    v<<cloud_in_out->polygon.points[i].x, cloud_in_out->polygon.points[i].y,
    cloud_in_out->polygon.points[i].z, 1;
    v = trans_mat * v;
    cloud_in_out->polygon.points[i].x = v.x();
    cloud_in_out->polygon.points[i].y = v.y();
    cloud_in_out->polygon.points[i].z = v.z();
  }
}

void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_out) 
{
  for (std::size_t i = 0; i < cloud_in_out->size(); ++i) 
  {
    pcl::PointXYZI& p = cloud_in_out->at(i);
    Eigen::Vector4f v(p.x, p.y, p.z, 1);
    v = trans_mat * v;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
  }
}

void TransformPointCloud(const Eigen::Matrix4f& trans_mat,
                         pcl::PointCloud<pcl::PointXYZI>& cloud_in_out) 
{
  for (std::size_t i = 0; i < cloud_in_out.size(); ++i) 
  {
    pcl::PointXYZI& p = cloud_in_out.at(i);
    Eigen::Vector4f v(p.x, p.y, p.z, 1);
    v = trans_mat * v;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
  }
}

/*
 * Vector & Matrix related methods
 */
void TransAffineToMatrix4(const Eigen::Vector3f& translation,
                          const Eigen::Vector4f& rotation,
                          Eigen::Matrix4f* trans_matrix) 
{
  const double t_x = translation(0);
  const double t_y = translation(1);
  const double t_z = translation(2);

  const double qx = rotation(0);
  const double qy = rotation(1);
  const double qz = rotation(2);
  const double qw = rotation(3);

  (*trans_matrix) << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
      2 * (qx * qz + qy * qw), t_x, 2 * (qx * qy + qz * qw),
      1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), t_y,
      2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
      1 - 2 * (qx * qx + qy * qy), t_z, 0, 0, 0, 1;
}

void ComputeMostConsistentBboxDirection(const Eigen::Vector3f& previous_dir,
                                        Eigen::Vector3f* current_dir) {
  float dot_val_00 =
      previous_dir(0) * (*current_dir)(0) + previous_dir(1) * (*current_dir)(1);
  float dot_val_01 =
      previous_dir(0) * (*current_dir)(1) - previous_dir(1) * (*current_dir)(0);
  if (fabs(dot_val_00) >= fabs(dot_val_01)) {
    if (dot_val_00 < 0) {
      (*current_dir) = -(*current_dir);
    }
  } else {
    if (dot_val_01 < 0) {
      (*current_dir) =
          Eigen::Vector3f((*current_dir)(1), -(*current_dir)(0), 0);
    } else {
      (*current_dir) =
          Eigen::Vector3f(-(*current_dir)(1), (*current_dir)(0), 0);
    }
  }
}

double VectorCosTheta2dXy(const Eigen::Vector3f& v1,
                          const Eigen::Vector3f& v2) {
  double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
  double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
  double cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  return cos_theta;
}

double VectorTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
  double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
  double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
  double cos_theta =
      (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);

  cos_theta = Clamp(cos_theta, 1.0, -1.0);

  double theta = acos(cos_theta);
  if (sin_theta < 0) {
    theta = -theta;
  }
  return theta;
}


