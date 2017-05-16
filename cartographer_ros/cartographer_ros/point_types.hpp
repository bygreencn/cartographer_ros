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

#ifndef PCL_COMMON_POINT_TYPES_HPP__
#define PCL_COMMON_POINT_TYPES_HPP__

#include <pcl/impl/point_types.hpp>

#define PCL_POINT_TYPES_NEW              \
	(pcl::PointXYZT)                     \
	(pcl::PointXYZIT)                    \
	(pcl::PointXYZITNormal)              \
	(pcl::Point0LAS)                     \
	(pcl::Point1LAS)                     \
	(pcl::Point2LAS)                     \
	(pcl::Point3LAS)                     \
	(pcl::PointXYZTRPH)                  \
	(pcl::PointXYZTBLRPH)                \
	(pcl::PointXYZTBLRPHQ)               \
	(pcl::EigenValue)                    \
	(pcl::EigenValueFeature)      

#define PCL_POINT_TYPES_MIN              \
	(pcl::PointXYZ)                      \
	(pcl::PointXYZI)                     \
	(pcl::PointXYZT)                     \
	(pcl::PointXYZIT)                    \
	(pcl::Normal)                        \
	(pcl::PointNormal)                   \
	(pcl::PointXYZINormal)               \
	(pcl::PointXYZITNormal)              \
	(pcl::PointXYZTRPH)                  \
	(pcl::PointXYZTBLRPH)                \
	(pcl::PointXYZTBLRPHQ)

#define PCL_RGB_POINT_TYPES_NEW          \
	(pcl::Point2LAS)                     \
	(pcl::Point3LAS)                                          

#define PCL_XYZ_POINT_TYPES_NEW     \
	(pcl::PointXYZT)                \
	(pcl::PointXYZIT)               \
	(pcl::PointXYZITNormal)         \
	(pcl::Point0LAS)                \
	(pcl::Point1LAS)                \
	(pcl::Point2LAS)                \
	(pcl::Point3LAS)                \
	(pcl::PointXYZTRPH)             \
	(pcl::PointXYZTBLRPH)           \
	(pcl::PointXYZTBLRPHQ)

#define PCL_XYZ_POINT_TYPES_MIN     \
	(pcl::PointXYZ)                 \
	(pcl::PointXYZI)                \
	(pcl::PointXYZT)                \
	(pcl::PointXYZIT)               \
	(pcl::PointNormal)              \
	(pcl::PointXYZINormal)          \
	(pcl::PointXYZITNormal)                              

#define PCL_XYZL_POINT_TYPES_NEW    \
	(pcl::Point0LAS)                \
	(pcl::Point1LAS)                \
	(pcl::Point2LAS)                \
	(pcl::Point3LAS)                                

#define PCL_NORMAL_POINT_TYPES_NEW    \
	(pcl::PointXYZITNormal)           \
	(pcl::EigenValue)

#define PCL_NORMAL_POINT_TYPES_MIN    \
	(pcl::Normal)                     \
	(pcl::PointNormal)                \
	(pcl::PointXYZINormal)            \
	(pcl::PointXYZITNormal)           

#define PCL_FEATURE_POINT_TYPES_NEW   \
	(pcl::EigenValueFeature)

#define PCL_XYZT_POINT_TYPES       \
	(pcl::PointXYZT)               \
	(pcl::PointXYZIT)              \
	(pcl::PointXYZITNormal)        \
	(pcl::PointXYZTRPH)            \
	(pcl::PointXYZTBLRPH)          \
	(pcl::PointXYZTBLRPHQ)  

#define PCL_XYZT_POINT_TYPES_MIN       \
	(pcl::PointXYZT)                   \
	(pcl::PointXYZIT)                  \
	(pcl::Point1LAS)                   \
	(pcl::Point3LAS)                                     

#define PCL_TRAJECTORY_POINT_TYPES \
	(pcl::PointXYZT)               \
	(pcl::PointXYZTRPH)            \
	(pcl::PointXYZTBLRPH)          \
	(pcl::PointXYZTBLRPHQ)         

namespace pcl
{
#define PCL_ADD_UNION_PRINCIPAL4D \
	union EIGEN_ALIGN16 { \
	float data_p[4]; \
	float principal[3]; \
	struct { \
	float principal_x; \
	float principal_y; \
	float principal_z; \
	}; \
  };
#define PCL_ADD_EIGEN_MAPS_PRINCIPAL4D \
	inline pcl::Vector3fMap getPrincicalVector3fMap () { return (pcl::Vector3fMap (data_p)); } \
	inline pcl::Vector3fMapConst getPrincicalVector3fMap () const { return (pcl::Vector3fMapConst (data_p)); } \
	inline pcl::Vector4fMap getPrincicalVector4fMap () { return (pcl::Vector4fMap (data_p)); } \
	inline pcl::Vector4fMapConst getPrincicalVector4fMap () const { return (pcl::Vector4fMapConst (data_p)); }

#define PCL_ADD_PRINCIPAL4D \
	PCL_ADD_UNION_PRINCIPAL4D \
	PCL_ADD_EIGEN_MAPS_PRINCIPAL4D
#define PCL_ADD_UNION_EIGENVALUE4D \
	union EIGEN_ALIGN16 { \
	float data_e[4]; \
	float eigenvalue[3]; \
	struct { \
	float eigenvalue_1; \
	float eigenvalue_2; \
	float eigenvalue_3; \
	}; \
  };
#define PCL_ADD_EIGEN_MAPS_EIGENVALUE4D \
	inline pcl::Vector3fMap getEigenvalueVector3fMap () { return (pcl::Vector3fMap (data_e)); } \
	inline pcl::Vector3fMapConst getEigenvalueVector3fMap () const { return (pcl::Vector3fMapConst (data_e)); } \
	inline pcl::Vector4fMap getEigenvalueVector4fMap () { return (pcl::Vector4fMap (data_e)); } \
	inline pcl::Vector4fMapConst getEigenvalueVector4fMap () const { return (pcl::Vector4fMapConst (data_e)); }

#define PCL_ADD_EIGENVALUE4D \
	PCL_ADD_UNION_EIGENVALUE4D \
	PCL_ADD_EIGEN_MAPS_EIGENVALUE4D

	/** \brief 点云坐标和时间*/
	struct EIGEN_ALIGN16 _PointXYZT
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				double t;
			};
			float data_c[4];
		};
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZT& p);
	/** \brief 点云坐标和时间*/
	struct PointXYZT : public _PointXYZT
	{
		inline PointXYZT (const _PointXYZT &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			t = p.t;
		}

		inline PointXYZT ()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			t = 0.0f;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZT& p);
	};
	/** \brief 点云坐标、强度和时间*/
	struct EIGEN_ALIGN16 _PointXYZIT
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				float intensity;
				double t;
			};
			float data_c[4];
		};
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZIT& p);
	/** \brief 点云坐标、强度和时间*/
	struct PointXYZIT : public _PointXYZIT
	{
		inline PointXYZIT (const _PointXYZIT &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			t = p.t;
		}

		inline PointXYZIT ()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			t = 0.0f;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZIT& p);
	};
	/** \brief 点云坐标、强度、时间和法向量*/
	struct EIGEN_ALIGN16 _PointXYZITNormal
	{
		PCL_ADD_POINT4D;
		PCL_ADD_NORMAL4D;
		union
		{
			struct
			{
				float curvature;
				float intensity;
				double t;
			};
			float data_c[4];
		};
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZITNormal& p);
	/** \brief 点云坐标、强度、时间和法向量*/
	struct PointXYZITNormal : public _PointXYZITNormal
	{
		inline PointXYZITNormal(const _PointXYZITNormal &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			t = p.t;
			normal_x = p.normal_x;
			normal_y = p.normal_y;
			normal_z = p.normal_z;
			curvature = p.curvature;
		}

		inline PointXYZITNormal()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			t = 0.0f;
			normal_x = 0;
			normal_y = 0;
			normal_z = 0;
			curvature = 0;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZITNormal& p);
	};

	/** \brief LAS format 0*/
	struct EIGEN_ALIGN16 _Point0LAS
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				uint16_t return_num;//回波号
				uint16_t num_of_returns;
				uint16_t scan_direction;
				uint16_t line_num;//条带号
				float intensity;
				uint32_t label;
			};
			float data_c[4];
		};
		int8_t scan_angle_rank;
		uint8_t user_data;
		uint16_t p_source_ID;//point_source_ID
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Point0LAS& p);
	/** \brief LAS format 0*/
	struct Point0LAS : public _Point0LAS
	{
		inline Point0LAS(const _Point0LAS &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			label = p.label;
			return_num = p.return_num;
			num_of_returns = p.num_of_returns;
			scan_direction = p.scan_direction;
			line_num = p.line_num;
			scan_angle_rank = p.scan_angle_rank;
			user_data = p.user_data;
			p_source_ID = p.p_source_ID;
		}

		inline Point0LAS()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			label = 0;
			return_num = 0;
			num_of_returns = 1;
			scan_direction = 1;
			line_num = 0;
			scan_angle_rank = 0;
			user_data = 0;
			p_source_ID = 0;
		}
		friend std::ostream& operator << (std::ostream& os, const Point0LAS& p);
	};
	/** \brief LAS format 1*/
	struct EIGEN_ALIGN16 _Point1LAS
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				uint16_t return_num;//回波号
				uint16_t num_of_returns;
				uint16_t scan_direction;
				uint16_t line_num;//条带号
				float intensity;
				uint32_t label;
			};
			float data_c[4];
		};
		int8_t scan_angle_rank;
		uint8_t user_data;
		uint16_t p_source_ID;//point_source_ID
		double t;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Point1LAS& p);
	/** \brief LAS format 1*/
	struct Point1LAS : public _Point1LAS
	{
		inline Point1LAS(const _Point1LAS &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			label = p.label;
			return_num = p.return_num;
			num_of_returns = p.num_of_returns;
			scan_direction = p.scan_direction;
			line_num = p.line_num;
			t = p.t;
			scan_angle_rank = p.scan_angle_rank;
			user_data = p.user_data;
			p_source_ID = p.p_source_ID;
		}

		inline Point1LAS()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			label = 0;
			return_num = 0;
			num_of_returns = 1;
			scan_direction = 1;
			line_num = 0;
			t = 0.0f;
			scan_angle_rank = 0;
			user_data = 0;
			p_source_ID = 0;
		}
		friend std::ostream& operator << (std::ostream& os, const Point1LAS& p);
	};
	/** \brief LAS format 2*/
	struct EIGEN_ALIGN16 _Point2LAS
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				uint16_t return_num;//回波号
				uint16_t num_of_returns;
				uint16_t scan_direction;
				uint16_t line_num;//条带号
				float intensity;
				uint32_t label;
			};
			float data_c[4];
		};
		int8_t scan_angle_rank;
		uint8_t user_data;
		uint16_t p_source_ID;//point_source_ID
		PCL_ADD_RGB;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Point2LAS& p);
	/** \brief LAS format 2*/
	struct Point2LAS : public _Point2LAS
	{
		inline Point2LAS(const _Point2LAS &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			label = p.label;
			return_num = p.return_num;
			num_of_returns = p.num_of_returns;
			scan_direction = p.scan_direction;
			line_num = p.line_num;
			rgb = p.rgb;
			scan_angle_rank = p.scan_angle_rank;
			user_data = p.user_data;
			p_source_ID = p.p_source_ID;
		}

		inline Point2LAS()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			label = 0;
			return_num = 0;
			num_of_returns = 1;
			scan_direction = 1;
			line_num = 0;
			r = g = b = a = 0;
			scan_angle_rank = 0;
			user_data = 0;
			p_source_ID = 0;
		}
		friend std::ostream& operator << (std::ostream& os, const Point2LAS& p);
	};
	/** \brief LAS format 3*/
	struct EIGEN_ALIGN16 _Point3LAS
	{
		PCL_ADD_POINT4D;
		union
		{
			struct
			{
				uint16_t return_num;//回波号
				uint16_t num_of_returns;
				uint16_t scan_direction;
				uint16_t line_num;//条带号
				float intensity;
				uint32_t label;
			};
			float data_c[4];
		};
		int8_t scan_angle_rank;
		uint8_t user_data;
		uint16_t p_source_ID;//point_source_ID
		PCL_ADD_RGB;
		double t;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Point3LAS& p);
	/** \brief LAS format 3*/
	struct Point3LAS : public _Point3LAS
	{
		inline Point3LAS(const _Point3LAS &p)
		{
			x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
			intensity = p.intensity;
			label = p.label;
			return_num = p.return_num;
			num_of_returns = p.num_of_returns;
			scan_direction = p.scan_direction;
			line_num = p.line_num;
			rgb = p.rgb;
			t = p.t;
			scan_angle_rank = p.scan_angle_rank;
			user_data = p.user_data;
			p_source_ID = p.p_source_ID;
		}

		inline Point3LAS()
		{
			x = y = z = 0.0f;
			data[3] = 1.0f;
			intensity = 0.0f;
			label = 0;
			return_num = 0;
			num_of_returns = 1;
			scan_direction = 1;
			line_num = 0;
			r = g = b = a = 0;
			t = 0;
			scan_angle_rank = 0;
			user_data = 0;
			p_source_ID = 0;
		}
		friend std::ostream& operator << (std::ostream& os, const Point3LAS& p);
	};

	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZTRPH& p);
	/** \brief 点的坐标、时间和姿态角*/
	struct EIGEN_ALIGN16 PointXYZTRPH
	{
		PCL_ADD_POINT4D;
		double t;//时间
		double roll;
		double pitch;
		double heading;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			inline PointXYZTRPH (const PointXYZTRPH &p)
		{
			x = p.x;y = p.y;z = p.z;t = p.t;
			roll = p.roll;pitch = p.pitch;heading = p.heading;
		}
		inline PointXYZTRPH ()
		{
			x = 0.0;y = 0.0;z = 0.0;t = 0.0;
			roll = 0.0;pitch = 0.0;heading = 0.0;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZTRPH& p);
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZTBLRPH& p);
	/** \brief 点的坐标、时间、纬度、精度和姿态角*/
	struct EIGEN_ALIGN16 PointXYZTBLRPH
	{
		PCL_ADD_POINT4D;
		double t;//时间
		double latitude;
		double longitude;
		double roll;
		double pitch;
		double heading;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		inline PointXYZTBLRPH (const PointXYZTBLRPH &p)
		{
			x = p.x;y = p.y;z = p.z;t = p.t;
			latitude = p.latitude;longitude = p.longitude;
			roll = p.roll;pitch = p.pitch;heading = p.heading;
		}
		inline PointXYZTBLRPH ()
		{
			x = 0.0;y = 0.0;z = 0.0;t = 0.0;
			latitude = 0.0;longitude = 0.0;
			roll = 0.0;pitch = 0.0;heading = 0.0;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZTBLRPH& p);
	};
	/** \brief 点的坐标、时间、纬度、精度、姿态角和质量*/
	struct EIGEN_ALIGN16 PointXYZTBLRPHQ
	{
		PCL_ADD_POINT4D;
		double t;//时间
		double latitude;
		double longitude;
		double roll;
		double pitch;
		double heading;
		int Q;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			inline PointXYZTBLRPHQ (const PointXYZTBLRPHQ &p)
		{
			x = p.x;y = p.y;z = p.z;t = p.t;
			latitude = p.latitude;longitude = p.longitude;
			roll = p.roll;pitch = p.pitch;heading = p.heading;
			Q = p.Q;
		}
		inline PointXYZTBLRPHQ ()
		{
			x = 0.0;y = 0.0;z = 0.0;t = 0.0;
			latitude = 0.0;longitude = 0.0;
			roll = 0.0;pitch = 0.0;heading = 0.0;
			Q = 1;
		}
		friend std::ostream& operator << (std::ostream& os, const PointXYZTBLRPHQ& p);
	};

	/** \brief 点云特征值*/
	struct _EigenValue
	{
		PCL_ADD_NORMAL4D;
		union
		{
			struct
			{
				float curvature;
			};
			float data_c[4];
		};
		PCL_ADD_PRINCIPAL4D;
		PCL_ADD_EIGENVALUE4D;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const EigenValue& p);
	/** \brief 点云特征值*/
	struct EigenValue: public _EigenValue
	{
		inline EigenValue (const _EigenValue &p)
		{
			normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
			curvature = p.curvature;
			principal_x = p.principal_x; principal_y = p.principal_y; principal_z = p.principal_z; data_p[3] = 0.0f;
			eigenvalue_1 = p.eigenvalue_1; eigenvalue_2 = p.eigenvalue_2; eigenvalue_3 = p.eigenvalue_3; data_e[3] = 0.0f;
		}

		inline EigenValue ()
		{
			normal_x = 0.0; normal_y = 0.0; normal_z = 0.0; data_n[3] = 0.0f;
			curvature = 0.0;
			principal_x = 0.0; principal_y = 0.0; principal_z = 0.0; data_p[3] = 0.0f;
			eigenvalue_1 = 0.0; eigenvalue_2 = 0.0; eigenvalue_3 = 0.0; data_e[3] = 0.0f;
		}
		friend std::ostream& operator << (std::ostream& os, const EigenValue& p);
	};
	PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointCloudAdvancedInfo& p);
	/** \brief 点云特征值特征, 来源于Weinmann M, Jutzi B, Hinz S, et al. Semantic point cloud interpretation based on optimal neighborhoods, relevant features and efficient classifiers[J].
	  * \ ISPRS Journal of Photogrammetry and Remote Sensing, 2015,105:286-304.
	  */
	struct EigenValueFeature
	{
		union
		{
			struct
			{
				float linearity;//线性度
				float planarity;//平面度
				float scattering;//散乱度

				float omnivariance;//全方差
				float eigenentropy;//信息熵
				float anisotropy;//各向异性
				float esum;//和
				float curvature;//曲率
			};
			float data_f[8];
		};

		inline EigenValueFeature(const EigenValueFeature &p)
		{
			linearity = p.linearity; planarity = p.planarity; scattering = p.scattering; omnivariance = p.omnivariance;
			eigenentropy = p.eigenentropy; anisotropy = p.anisotropy; esum = p.esum; curvature = p.curvature;
		}
		inline EigenValueFeature()
		{
			linearity = 0.0; planarity = 0.0; scattering = 0.0; omnivariance = 0.0;
			eigenentropy = 0.0; anisotropy = 0.0; esum = 0.0; curvature = 0.0;
		}
		friend std::ostream& operator << (std::ostream& os, const EigenValueFeature& p);
	};
	/** \brief 点云高级信息*/
	struct PointCloudAdvancedInfo
	{
		union
		{
			struct
			{
				double min_x;
				double min_y;
				double min_z;
				double min_t;
				double max_x;
				double max_y;
				double max_z;
				double max_t;
			};
			double data_info[8];
		};

		inline PointCloudAdvancedInfo(const PointCloudAdvancedInfo &p)
		{
			min_x = p.min_x; min_y = p.min_y; min_z = p.min_z; min_t = p.min_t;
			max_x = p.max_x; max_y = p.max_y; max_z = p.max_z; max_t = p.max_t;
		}
		inline PointCloudAdvancedInfo()
		{
			min_x = 0.0; min_y = 0.0; min_z = 0.0; min_t = 0.0;
			max_x = 0.0; max_y = 0.0; max_z = 0.0; max_t = 0.0;
		}
		friend std::ostream& operator << (std::ostream& os, const PointCloudAdvancedInfo& p);
	};
}

#endif // PCL_COMMON_POINT_TYPES_HPP__