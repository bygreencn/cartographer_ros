/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/assets_writer.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping_2d/proto/range_data_inserter_options.pb.h"
#include "cartographer_ros/map_writer.h"
#include "cartographer_ros/occupancy_grid.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/io/pcd_io.h>

#include <fstream>

namespace cartographer_ros {

// Writes an occupancy grid.
void Write2DAssets(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    const std::string& stem) {
  ::nav_msgs::OccupancyGrid occupancy_grid;
  BuildOccupancyGrid2D(trajectory_nodes, map_frame, submaps_options,
                       &occupancy_grid);
  WriteOccupancyGridToPgmAndYaml(occupancy_grid, stem);
  LOG(INFO) << ("trajectory_nodes_number: "+std::to_string(trajectory_nodes.size()));
  std::ofstream in;
  in.open("/home/yfb/trajectory2d.txt",std::ios::trunc); 
  pcl::PointCloud<pcl::PointXYZ> cloud_test;
  for(auto node_test:trajectory_nodes)
  {
      std::vector<Eigen::Vector3f>  returns =  node_test.constant_data->range_data_2d.returns;
      for(auto point:returns)
      {
        //Eigen::Quaternion<float> q_float = static_cast<Eigen::Quaternion<float>> (node_test.pose.rotation());
        //Eigen::Matrix<float, 3, 1> t_float = static_cast<Eigen::Matrix<float, 3, 1>> (node_test.pose.translation());
        //Eigen::Vector3f point_world = q_float.toRotationMatrix()*point+t_float;
        ::cartographer::transform::Rigid3d pose_d = node_test.pose;
        ::cartographer::transform::Rigid3f  pose_f = pose_d.cast<float>();
        //::cartographer::transform::Rigid3d pose_tracking = node_test.constant_data->tracking_to_pose;
        
        //std::cout<<"x:"<<pose_tracking.translation()[0]<<" y:"<<pose_tracking.translation()[1]<<" z:"<<pose_tracking.translation()[2]<<std::endl;
        //std::cout<<"x:"<<pose_f.translation()[0]<<" y:"<<pose_f.translation()[1]<<" z:"<<pose_f.translation()[2]<<std::endl;
        Eigen::Vector3f point_world = pose_f.rotation().toRotationMatrix()*point+pose_f.translation();
        pcl::PointXYZ point_temp_2d;
        point_temp_2d.x = point_world[0];
        point_temp_2d.y = point_world[1];
        point_temp_2d.z = point_world[2];
        cloud_test.push_back(point_temp_2d);
    }
  }

  pcl::io::savePCDFileBinary("/home/yfb/slam2d.pcd",cloud_test);
  for(const auto& node:trajectory_nodes)
  {
    //LOG(INFO)<<node.pose.DebugString();
    auto q = node.pose.rotation();
    std::string stem = std::to_string(node.time().time_since_epoch().count())+" "
    +std::to_string(node.pose.translation().x())+" "+std::to_string(node.pose.translation().y())+" "+std::to_string(node.pose.translation().z())+
    +" "+std::to_string(q.w())+" "+std::to_string(q.x())+" "+std::to_string(q.y())+" "+std::to_string(q.z())+"\n";
    in<<stem;
  }
  in.close();

}

// Writes X-ray images and PLY files from the 'trajectory_nodes'. The filenames
// will all start with 'stem'.
void Write3DAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
                       trajectory_nodes,
                   const double voxel_size, const std::string& stem) {
  namespace carto = ::cartographer;
  const auto file_writer_factory = [](const string& filename) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(filename);
  };

  carto::io::NullPointsProcessor null_points_processor;
  carto::io::XRayPointsProcessor xy_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitY())),
      {}, stem + "_xray_xy", file_writer_factory, &null_points_processor);
  carto::io::XRayPointsProcessor yz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())),
      {}, stem + "_xray_yz", file_writer_factory, &xy_xray_points_processor);
  carto::io::XRayPointsProcessor xz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitZ())),
      {}, stem + "_xray_xz", file_writer_factory, &yz_xray_points_processor);
  carto::io::PlyWritingPointsProcessor ply_writing_points_processor(
      file_writer_factory(stem + ".ply"), &xz_xray_points_processor);

  for (const auto& node : trajectory_nodes) {
    const carto::sensor::RangeData range_data =
        carto::sensor::TransformRangeData(
            carto::sensor::Decompress(node.constant_data->range_data_3d),
            node.pose.cast<float>());

    auto points_batch = carto::common::make_unique<carto::io::PointsBatch>();
    points_batch->origin = range_data.origin;
    points_batch->points = range_data.returns;
    ply_writing_points_processor.Process(std::move(points_batch));
  }
  ply_writing_points_processor.Flush();
}

}  // namespace cartographer_ros
