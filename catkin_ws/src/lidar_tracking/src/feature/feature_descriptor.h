/*2D bbox corner velocity measurment
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-15 16:57:22
 */


#ifndef FEATURE_DESCRIPTOR_H_
#define FEATURE_DESCRIPTOR_H_

//#include <algorithm>
//#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class FeatureDescriptor {
 public:
  // @brief intialize feature descriptor
  // @params[IN] cloud: given cloud for feature extraction
  // @return nothing
  explicit FeatureDescriptor(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    cloud_ = cloud;
  }
  ~FeatureDescriptor() {}

  // @brief compute histogram feature of given cloud
  // @params[IN] bin_size: bin size of histogram
  // @params[OUT] feature: histogram feature of given cloud
  // @return nothing
  void ComputeHistogram(const int bin_size, std::vector<float>* feature);

 private:
  void GetMinMaxCenter();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  pcl::PointXYZI min_pt_;
  pcl::PointXYZI max_pt_;
  pcl::PointXYZI center_pt_;
};  // class FeatureDescriptor


#endif  // FEATURE_DESCRIPTOR_H_
