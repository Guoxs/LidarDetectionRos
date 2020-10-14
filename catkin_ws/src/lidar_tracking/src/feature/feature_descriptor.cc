/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 14:57:52
 */

#include <float.h>
#include "feature_descriptor.h"


void FeatureDescriptor::ComputeHistogram(const int bin_size,
                                         std::vector<float>* feature) {
  GetMinMaxCenter();

  int xstep = bin_size;
  int ystep = bin_size;
  int zstep = bin_size;
  int stat_len = xstep + ystep + zstep;
  std::vector<int> stat_feat(stat_len, 0);
  float xsize = (max_pt_.x - min_pt_.x) / xstep + 0.000001;
  float ysize = (max_pt_.y - min_pt_.y) / ystep + 0.000001;
  float zsize = (max_pt_.z - min_pt_.z) / zstep + 0.000001;

  int pt_num = cloud_->points.size();
  for (int i = 0; i < pt_num; ++i) {
    pcl::PointXYZI& pt = cloud_->points[i];
    stat_feat[floor((pt.x - min_pt_.x) / xsize)]++;
    stat_feat[xstep + floor((pt.y - min_pt_.y) / ysize)]++;
    stat_feat[xstep + ystep + floor((pt.z - min_pt_.z) / zsize)]++;
  }
  // update feature
  (*feature).resize(stat_len);
  for (size_t i = 0; i < stat_feat.size(); ++i) {
    (*feature)[i] =
        static_cast<float>(stat_feat[i]) / static_cast<float>(pt_num);
  }
}

void FeatureDescriptor::GetMinMaxCenter() {
  float xsum = 0.0;
  float ysum = 0.0;
  float zsum = 0.0;
  min_pt_.x = min_pt_.y = min_pt_.z = FLT_MAX;
  max_pt_.x = max_pt_.y = max_pt_.z = -FLT_MAX;
  // min max pt
  int pt_num = cloud_->points.size();
  for (int i = 0; i < pt_num; ++i) {
    pcl::PointXYZI& pt = cloud_->points[i];
    xsum += pt.x;
    ysum += pt.y;
    zsum += pt.z;
    min_pt_.x = std::min(min_pt_.x, pt.x);
    max_pt_.x = std::max(max_pt_.x, pt.x);
    min_pt_.y = std::min(min_pt_.y, pt.y);
    max_pt_.y = std::max(max_pt_.y, pt.y);
    min_pt_.z = std::min(min_pt_.z, pt.z);
    max_pt_.z = std::max(max_pt_.z, pt.z);
  }
  // center position
  center_pt_.x = xsum / pt_num;
  center_pt_.y = ysum / pt_num;
  center_pt_.z = zsum / pt_num;
}
