/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_COMMON_POINT_TYPES_H__
#define PCL_COMMON_POINT_TYPES_H__

#include <pcl/point_types.h>

namespace pcl
{
	/** \brief 点云坐标和时间*/
	struct PointXYZT;
	/** \brief 点云坐标、强度和时间*/
	struct PointXYZIT;
	/** \brief 点云坐标、强度、时间和法向量*/
	struct PointXYZITNormal;
	/** \brief LAS format 0*/
	struct Point0LAS;
	/** \brief LAS format 1*/
	struct Point1LAS;
	/** \brief LAS format 2*/
	struct Point2LAS;
	/** \brief LAS format 3*/
	struct Point3LAS;
	/** \brief 点的坐标、时间和姿态角*/
	struct PointXYZTRPH;
	/** \brief 点的坐标、时间、纬度、精度和姿态角*/
	struct PointXYZTBLRPH;
	/** \brief 点的坐标、时间、纬度、精度、姿态角和质量*/
	struct PointXYZTBLRPHQ;
	/** \brief 点云特征值*/
	struct EigenValue;
	/** \brief 点云特征值特征*/
	struct EigenValueFeature;
	/** \brief 点云高级信息*/
	struct PointCloudAdvancedInfo;
}

#include "cartographer_ros/point_types.hpp"


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZT,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(double, t, t)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZT, pcl::_PointXYZT)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZIT,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(double, t, t)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIT, pcl::_PointXYZIT)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZITNormal,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(double, t, t)
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	(float, curvature, curvature)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZITNormal, pcl::_PointXYZITNormal)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_Point0LAS,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(uint16_t, return_num, return_num)
	(uint16_t, num_of_returns, num_of_returns)
	(uint16_t, scan_direction, scan_direction)
	(uint16_t, line_num, line_num)
	(float, intensity, intensity)
	(uint32_t, label, label)
	(int8_t, scan_angle_rank, scan_angle_rank)
	(uint8_t, user_data, user_data)
	(uint16_t, p_source_ID, p_source_ID)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Point0LAS, pcl::_Point0LAS)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_Point1LAS,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(uint16_t, return_num, return_num)
	(uint16_t, num_of_returns, num_of_returns)
	(uint16_t, scan_direction, scan_direction)
	(uint16_t, line_num, line_num)
	(float, intensity, intensity)
	(uint32_t, label, label)
	(int8_t, scan_angle_rank, scan_angle_rank)
	(uint8_t, user_data, user_data)
	(uint16_t, p_source_ID, p_source_ID)
	(double, t, t)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Point1LAS, pcl::_Point1LAS)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_Point2LAS,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(uint16_t, return_num, return_num)
	(uint16_t, num_of_returns, num_of_returns)
	(uint16_t, scan_direction, scan_direction)
	(uint16_t, line_num, line_num)
	(float, intensity, intensity)
	(uint32_t, label, label)
	(int8_t, scan_angle_rank, scan_angle_rank)
	(uint8_t, user_data, user_data)
	(uint16_t, p_source_ID, p_source_ID)
	(float, rgb, rgb)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Point2LAS, pcl::_Point2LAS)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_Point3LAS,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(uint16_t, return_num, return_num)
	(uint16_t, num_of_returns, num_of_returns)
	(uint16_t, scan_direction, scan_direction)
	(uint16_t, line_num, line_num)
	(float, intensity, intensity)
	(uint32_t, label, label)
	(int8_t, scan_angle_rank, scan_angle_rank)
	(uint8_t, user_data, user_data)
	(uint16_t, p_source_ID, p_source_ID)
	(float, rgb, rgb)
	(double, t, t)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::Point3LAS, pcl::_Point3LAS)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZTBLRPH,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, t, t)
	(double, latitude, latitude)
	(double, longitude, longitude)
	(double, roll, roll)
	(double, pitch, pitch)
	(double, heading, heading)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZTRPH,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(double, t, t)
	(double, roll, roll)
	(double, pitch, pitch)
	(double, heading, heading)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZTBLRPHQ,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(double, t, t)
	(double, latitude, latitude)
	(double, longitude, longitude)
	(double, roll, roll)
	(double, pitch, pitch)
	(double, heading, heading)
	(int, Q, Q)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_EigenValue,
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	(float, curvature, curvature)
	(float, principal_x, principal_x)
	(float, principal_y, principal_y)
	(float, principal_z, principal_z)
	(float, eigenvalue_1, eigenvalue_1)
	(float, eigenvalue_2, eigenvalue_2)
	(float, eigenvalue_3, eigenvalue_3)
	)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::EigenValue, pcl::_EigenValue)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::EigenValueFeature,
	(float, linearity, linearity)
	(float, planarity, planarity)
	(float, scattering, scattering)
	(float, omnivariance, omnivariance)
	(float, eigenentropy, eigenentropy)
	(float, anisotropy, anisotropy)
	(float, esum, esum)
	(float, curvature, curvature)
	)
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointCloudAdvancedInfo,
	(double, min_x, min_x)
	(double, min_y, min_y)
	(double, min_z, min_z)
	(double, min_t, min_t)
	(double, max_x, max_x)
	(double, max_y, max_y)
	(double, max_z, max_z)
	(double, max_t, max_t)
	)

#endif // PCL_COMMON_POINT_TYPES_H__