#pragma once

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h>

constexpr const float kOctreeResolution = 0.1f;
constexpr const float kPitchResolution = 1.0 * (M_PI / 180.0f);
constexpr const float kYawResolution = 1.0 * (M_PI / 180.0f);

class RayCaster {
 public:
  RayCaster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
      : cloud_(cloud), octree_(kOctreeResolution) {
    octree_.setInputCloud(cloud_);
    octree_.addPointsFromInputCloud();
  }

  int RayCast(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) {
    std::vector<int> k_indices;
    // std::cerr << origin.transpose() << ',' << direction.transpose() <<
    // std::endl;
    const int n =
        octree_.getIntersectedVoxelIndices(origin, direction, k_indices, 1);
    if (n > 0) {
      return k_indices[0];
    }
    // indicate failure
    return -1;
  }
  void GeneratePointCloudFromPose(const Eigen::Isometry3f& pose,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr* cloud) {
    const Eigen::Vector3f& origin = pose.translation().cast<float>();
    const Eigen::Matrix3f& rotation = pose.linear().cast<float>();

    // std::cerr << cloud_->getMatrixXfMap().rowwise().minCoeff() << std::endl;
    // std::cerr << cloud_->getMatrixXfMap().rowwise().maxCoeff() << std::endl;

    std::vector<pcl::PointXYZ> points;
    // octree_.getOccupiedVoxelCenters(

    (*cloud)->clear();
    for (float yaw = -M_PI; yaw <= M_PI; yaw += kYawResolution) {
      for (float pitch = -M_PI / 2; pitch <= M_PI / 2;
           pitch += kPitchResolution) {
        // const Eigen::Quaternionf rot = Eigen::AngleAxisf(pitch,
        // Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw,
        // Eigen::Vector3f::UnitZ());
        const Eigen::Quaternionf rot =
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
        const Eigen::Vector3f& direction =
            rotation * (rot * Eigen::Vector3f::UnitX());
        // std::cerr << direction.transpose() << std::endl;
        const int voxel_index = RayCast(origin, direction);
        if (voxel_index < 0) {
          continue;
        }
        // points.emplace_back(cloud_->points[voxel_index]);
        (*cloud)->push_back(cloud_->points[voxel_index]);
      }
    }
  }

 private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
};
