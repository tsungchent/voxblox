#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

inline TsdfMap::Config getTsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& nh) {
  TsdfMap::Config tsdf_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  nh->set_parameter(rclcpp::Parameter("tsdf_voxel_size", voxel_size));
  nh->set_parameter(rclcpp::Parameter("tsdf_voxels_per_side", voxels_per_side));
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(rclcpp::get_logger(""), "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(const rclcpp::Node::SharedPtr& nh) {
  ICP::Config icp_config;

  nh->set_parameter(rclcpp::Parameter("icp_min_match_ratio", icp_config.min_match_ratio));
  nh->set_parameter(rclcpp::Parameter("icp_subsample_keep_ratio", icp_config.subsample_keep_ratio));
  nh->set_parameter(rclcpp::Parameter("icp_mini_batch_size", icp_config.mini_batch_size));
  nh->set_parameter(rclcpp::Parameter("icp_refine_roll_pitch", icp_config.refine_roll_pitch));
  nh->set_parameter(rclcpp::Parameter("icp_inital_translation_weighting",
                   icp_config.inital_translation_weighting));
  nh->set_parameter(rclcpp::Parameter("icp_inital_rotation_weighting",
                   icp_config.inital_rotation_weighting));

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& nh) {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh);
  integrator_config.default_truncation_distance =
      tsdf_config.tsdf_voxel_size * 4;

  double truncation_distance = integrator_config.default_truncation_distance;
  double max_weight = integrator_config.max_weight;
  nh->set_parameter(rclcpp::Parameter("voxel_carving_enabled",
                   integrator_config.voxel_carving_enabled));
  nh->set_parameter(rclcpp::Parameter("truncation_distance", truncation_distance));
  nh->set_parameter(rclcpp::Parameter("max_ray_length_m", integrator_config.max_ray_length_m));
  nh->set_parameter(rclcpp::Parameter("min_ray_length_m", integrator_config.min_ray_length_m));
  nh->set_parameter(rclcpp::Parameter("max_weight", max_weight));
  nh->set_parameter(rclcpp::Parameter("use_const_weight", integrator_config.use_const_weight));
  nh->set_parameter(rclcpp::Parameter("use_weight_dropoff", integrator_config.use_weight_dropoff));
  nh->set_parameter(rclcpp::Parameter("allow_clear", integrator_config.allow_clear));
  nh->set_parameter(rclcpp::Parameter("start_voxel_subsampling_factor",
                   integrator_config.start_voxel_subsampling_factor));
  nh->set_parameter(rclcpp::Parameter("max_consecutive_ray_collisions",
                   integrator_config.max_consecutive_ray_collisions));
  nh->set_parameter(rclcpp::Parameter("clear_checks_every_n_frames",
                   integrator_config.clear_checks_every_n_frames));
  nh->set_parameter(rclcpp::Parameter("max_integration_time_s",
                   integrator_config.max_integration_time_s));
  nh->set_parameter(rclcpp::Parameter("anti_grazing", integrator_config.enable_anti_grazing));
  nh->set_parameter(rclcpp::Parameter("use_sparsity_compensation_factor",
                   integrator_config.use_sparsity_compensation_factor));
  nh->set_parameter(rclcpp::Parameter("sparsity_compensation_factor",
                   integrator_config.sparsity_compensation_factor));
  nh->set_parameter(rclcpp::Parameter("integration_order_mode",
                   integrator_config.integration_order_mode));

  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& nh) {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& nh) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(nh);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  nh->set_parameter(rclcpp::Parameter("esdf_euclidean_distance",
                         esdf_integrator_config.full_euclidean_distance));
  nh->set_parameter(rclcpp::Parameter("esdf_max_distance_m", esdf_integrator_config.max_distance_m));
  nh->set_parameter(rclcpp::Parameter("esdf_min_distance_m", esdf_integrator_config.min_distance_m));
  nh->set_parameter(rclcpp::Parameter("esdf_default_distance_m",
                   esdf_integrator_config.default_distance_m));
  nh->set_parameter(rclcpp::Parameter("esdf_min_diff_m", esdf_integrator_config.min_diff_m));
  nh->set_parameter(rclcpp::Parameter("clear_sphere_radius",
                   esdf_integrator_config.clear_sphere_radius));
  nh->set_parameter(rclcpp::Parameter("occupied_sphere_radius",
                   esdf_integrator_config.occupied_sphere_radius));
  nh->set_parameter(rclcpp::Parameter("esdf_add_occupied_crust",
                   esdf_integrator_config.add_occupied_crust));
  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& nh) {
  MeshIntegratorConfig mesh_integrator_config;

  nh->set_parameter(
      rclcpp::Parameter("mesh_min_weight", mesh_integrator_config.min_weight));
  nh->set_parameter(
      rclcpp::Parameter("mesh_use_color", mesh_integrator_config.use_color));

  return mesh_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
