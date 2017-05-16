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
#include "cartographer_ros/point_types.h"

namespace pcl
{
	std::ostream& operator << (std::ostream& os, const PointXYZT& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.t << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointXYZIT& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << "," << p.t << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointXYZITNormal& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << "," << p.t << ","
			<< p.normal_x << "," << p.normal_y << "," << p.normal_z << "," << p.curvature << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const Point0LAS& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.return_num << "," << p.return_num << "," << p.num_of_returns << "," << p.scan_direction <<
			"," << p.intensity << "," << p.label << " - " << p.scan_angle_rank << "," << p.user_data << "," << p.p_source_ID << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const Point1LAS& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.return_num << "," << p.num_of_returns << "," << p.scan_direction << "," << p.line_num <<
			"," << p.intensity << "," << p.label << " - " << p.scan_angle_rank << "," << p.user_data << "," << p.p_source_ID  << "," << p.t << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const Point2LAS& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.return_num << "," << p.num_of_returns << "," << p.scan_direction << "," << p.line_num <<
			"," << p.intensity << "," << p.label << " - " << p.scan_angle_rank << "," << p.user_data << "," << p.p_source_ID << "," << static_cast<int>(p.r) << "," << static_cast<int>(p.g) << ","
			<< static_cast<int>(p.b) << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const Point3LAS& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.return_num << "," << p.num_of_returns << "," << p.scan_direction << "," << p.line_num <<
			"," << p.intensity << "," << p.label << " - " << p.scan_angle_rank << "," << p.user_data << "," << p.p_source_ID << "," << static_cast<int>(p.r) << "," << static_cast<int>(p.g) << ","
			<< static_cast<int>(p.b) << "," << p.t << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointXYZTRPH& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << "," << p.t << "," <<  "," << p.roll << "," << p.pitch << "," << p.heading << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointXYZTBLRPH& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << "," << p.t << "," << p.latitude << "," << p.longitude << "," << p.roll << "," << p.pitch << "," << p.heading << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointXYZTBLRPHQ& p)
	{
		os << "(" << p.x << "," << p.y << "," << p.z << "," << p.t << "," << p.latitude << "," << p.longitude << "," 
			<< p.roll << "," << p.pitch << "," << p.heading << "," << p.Q << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const EigenValue& p)
	{
		os << "(" << p.normal_x << "," << p.normal_y << "," << p.normal_z << " - " << p.curvature << "-" << 
			p.principal_x << "," << p.principal_y << "-" << p.eigenvalue_1 << "," << p.eigenvalue_2 << "," << p.eigenvalue_3 << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const EigenValueFeature& p)
	{
		os << "(" << p.linearity << "," << p.planarity << "," << p.scattering << "," << p.omnivariance << "," << 
			p.eigenentropy << "," << p.anisotropy << "," << p.esum << "," << p.curvature << ")";
		return (os);
	}
	std::ostream& operator << (std::ostream& os, const PointCloudAdvancedInfo& p)
	{
		os << "(" << p.min_x << "," << p.min_y << "," << p.min_z << "," << p.min_t << "," <<
			p.max_x << "," << p.max_y << "," << p.max_z << "," << p.max_t << ")";
		return (os);
	}
}