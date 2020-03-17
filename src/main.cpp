#include <fmt/format.h>
#include <fmt/printf.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h>
#include <fstream>
#include <iostream>

#include "raycaster/raycaster.hpp"
#include "raycaster/twist.hpp"

/**
 * Apply constant velocity, as specified in the body frame.
 */
struct FrameManager {
  Eigen::Isometry3f initial_pose_;
  Twist<float> velocity_;

  Eigen::Isometry3f GetFrameAtStamp(const float stamp) {
    // std::cout << velocity_.Rotation().transpose() << "|" <<
    // velocity_.Translation().transpose() << std::endl;
    return initial_pose_ * Exp(velocity_ * stamp);
  }
};

template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> GetRigidTransform(
    const Eigen::Quaternion<Scalar>& rotation,
    const Eigen::Matrix<Scalar, 3, 1>& position) {
  return Eigen::Translation3f{position} * rotation;
}

void TestExpLogStuff() {
  const auto& T0 =
      GetRigidTransform(Eigen::Quaternionf::UnitRandom(),
                        Eigen::Vector3f{Eigen::Vector3f::Random()});
  const auto& v = GetRigidTransform(Eigen::Quaternionf::UnitRandom(),
                                    Eigen::Vector3f{Eigen::Vector3f::Random()});
  auto T = T0;
  for (int i = 0; i < 100; ++i) {
    // T = T * v;
    T = v * T;
  }
  std::cout << T.matrix() << std::endl;
  // auto T1 = T0*Exp(Log(v)*100);
  auto T1 = Exp(Log(v) * 100) * T0;
  std::cout << T1.matrix() << std::endl;
}

int main(const int argc, const char* argv[]) {
  TestExpLogStuff();
  std::string ply_file = "/home/jamiecho/Downloads/car-body-ply/Car body.ply";
  std::cout << argc << std::endl;
  if (argc >= 2) {
    std::cout << argv[1] << std::endl;
    ply_file = argv[1];
  }
  // Load source mesh file.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile(ply_file, *cloud);
  // Scale from mm -> m
  cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()) *= 0.001f;
  // cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()) *= 20.0;

  std::cout << cloud->getMatrixXfMap().rowwise().maxCoeff() << std::endl;
  std::cout << cloud->getMatrixXfMap().rowwise().minCoeff() << std::endl;

  RayCaster ray_caster{cloud};

  // Self
  FrameManager ego_vehicle{
      GetRigidTransform(
          Eigen::Quaternionf{Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())},
          Eigen::Vector3f{-30.0, 3.0, 2.0}),
      Log(Eigen::Translation3f{10.0f, 0.0, 0.0} *
          Eigen::AngleAxisf(0.4f, Eigen::Vector3f::UnitZ()))};

  // Obstacle
  FrameManager obs_vehicle{
      GetRigidTransform(
          Eigen::Quaternionf{Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitZ())},
          Eigen::Vector3f{0.0, 0.0, 2.0}),
      Log(Eigen::Translation3f{4.0f, 0.0, 0.0} *
          Eigen::AngleAxisf(-0.5f, Eigen::Vector3f::UnitZ()))};

  pcl::PointCloud<pcl::PointXYZ>::Ptr view(new pcl::PointCloud<pcl::PointXYZ>);
  float stamp = 0.0;
  for (int i = 0; i < 1024; ++i) {
    fmt::print("\r{}/{}", i, 1024);
    std::flush(std::cout);

    // Manage poses.
    const Eigen::Isometry3f& ego_rax_pose =
        ego_vehicle.GetFrameAtStamp(stamp);  // world_from_ego
    const Eigen::Isometry3f& obs_rax_pose =
        obs_vehicle.GetFrameAtStamp(stamp);  // world_from_obs

    // Map from rear axle to centroid.
    const Eigen::Isometry3f& ego_pose =
        ego_rax_pose * Eigen::Translation3f{2.0f, 0.0, 0.0};
    const Eigen::Isometry3f& obs_pose =
        obs_rax_pose * Eigen::Translation3f{2.0f, 0.0, 0.0};

    const Eigen::Isometry3f& obs_from_ego = obs_pose.inverse() * ego_pose;

    // Generated cloud is returned in `obs` frame.
    ray_caster.GeneratePointCloudFromPose(obs_from_ego, &view);

    // Convert cloud to world frame.
    pcl::transformPointCloud(*view, *view, obs_pose);

    pcl::io::savePLYFile(fmt::format("/tmp/view_{:04d}.ply", i), *view);
    std::ofstream(fmt::format("/tmp/pose_{:04d}.txt", i))
        << ego_pose.matrix() << std::endl;

    stamp += 0.01;
  }

  return 0;
}
