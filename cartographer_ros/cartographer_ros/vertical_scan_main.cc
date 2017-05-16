#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <fstream>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/bag_reader.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_msgs/TFMessage.h"
#include "urdf/model.h"

#include "cartographer_ros/point_types.h"

DEFINE_string(bag_filename,"","bag的路径");
DEFINE_string(traj_file,"","优化后的轨迹，包含平移和yaw");
DEFINE_string(traj_file_h,"","实时输出的轨迹，包含roll，pitch");
DEFINE_int32(min_time,150000,"最小时间，10^-7秒");
//DEFINE_strinf(ouput_path,"","");
//DEFINE_string(urdf_file,"","");

struct Pose3f
{
  Eigen::Vector3f tran;
  Eigen::Quaternionf quat;
  int64_t time;  
};

void ReadTraj(std::string file, std::vector<Pose3f> &pose_list)
{
  std::fstream in;
  in.open(file,std::ios::in);
  char line[1024]={0};
  while(in.getline(line, sizeof(line)))
  {
    std::stringstream word(line);
    Pose3f pose3f;
    word>>pose3f.time;
    
    word>>pose3f.tran[0];
    word>>pose3f.tran[1];
    word>>pose3f.tran[2];
    
    word>>pose3f.quat.w();
    word>>pose3f.quat.x();
    word>>pose3f.quat.y();
    word>>pose3f.quat.z();
    pose_list.emplace_back(pose3f);
  }
  in.clear();
  in.close();
}

Pose3f Interpolation(int64_t time,  std::vector<Pose3f> &pose_list)
{
  Pose3f pose_temp;
  pose_temp.time=0;
  int i=-1;
  auto GetVaule=[](int64_t x[], float y[], int64_t time)
  {
    float z;
    z=(y[0]*(float(time-x[1])) - y[1]*(float(time-x[0])))/float((x[0]-x[1]));
    return z;
  };
   
  for(auto it:pose_list)
  {
    ++i;
    if(time<pose_list.begin()->time||time>(pose_list.end()-1)->time)
    {
      continue;
    }
    
    if(abs(time-it.time)<FLAGS_min_time)
    {
      pose_temp.time = time;
      pose_temp.tran=it.tran;
      pose_temp.quat=it.quat;
      break;      
    }
    else if(it.time-time>0&&pose_list[i-1].time-time<0&&it.time-time<500000)
    {
      //pose_temp.time = time;
      int64_t x[2];
      float y[2];
      x[0]= pose_list[i-1].time;
      x[1]= pose_list[i].time;
      
      y[0]=pose_list[i-1].tran[0];
      y[1]=pose_list[i].tran[0];
      pose_temp.tran[0]=GetVaule(x, y, time);
      
       y[0]=pose_list[i-1].tran[1];
      y[1]=pose_list[i].tran[1];
      pose_temp.tran[1]=GetVaule(x, y, time);
      
       y[0]=pose_list[i-1].tran[2];
      y[1]=pose_list[i].tran[2];
      pose_temp.tran[2]=GetVaule(x, y, time);
      
      y[0]=pose_list[i-1].quat.w();
      y[1]=pose_list[i].quat.w();
      pose_temp.quat.w()=GetVaule(x, y, time);
      
      y[0]=pose_list[i-1].quat.x();
      y[1]=pose_list[i].quat.x();
      pose_temp.quat.x()=GetVaule(x, y, time);
      
      y[0]=pose_list[i-1].quat.y();
      y[1]=pose_list[i].quat.y();
      pose_temp.quat.y()=GetVaule(x, y, time);
      
      y[0]=pose_list[i-1].quat.z();
      y[1]=pose_list[i].quat.z();
      pose_temp.quat.z()=GetVaule(x, y, time);

    }
  }
return pose_temp;
}

float Dis(pcl::PointXYZT temp_p,Pose3f pose)
{
  float dis = pow((temp_p.x-pose.tran[0]),2)+pow((temp_p.y-pose.tran[1]),2);
  dis=sqrt(dis);
  return dis;  
}

int main(int argc, char **argv)
{
 google::ParseCommandLineFlags(&argc, &argv, true);
  
  std::string bag_filename = FLAGS_bag_filename;
  std::string traj_file = FLAGS_traj_file;
  std::string traj_file_h = FLAGS_traj_file_h;
  std::string output_path=bag_filename;
 /* int flag =0;
  for(int i =0;i<output_path.size();++i)
  {
    if(output_path[i]=='/')
    {
      flag = i;
    }
  }
output_path=output_path.substr(0,flag+1);*/
  
  std::vector<Pose3f> pose_list;
  ReadTraj(traj_file, pose_list);
  
   std::vector<Pose3f> pose_list_h;
  ReadTraj(traj_file_h, pose_list_h);

  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);
  pcl::PointCloud<pcl::PointXYZT> cloud;
  
  Pose3f ori_tran;
  ori_tran.tran[0]=0.2;
  ori_tran.tran[1]=0.0;
  ori_tran.tran[2]=0.3;
  
  ori_tran.quat.w()=0.0;
  ori_tran.quat.x()=0.707107;
  ori_tran.quat.y()=-0.0;
  ori_tran.quat.z()=0.707107;
  
  for (const rosbag::MessageInstance& msg : view)
  {
    std::string topic = msg.getTopic();
    if(topic=="/vertical_laser_scan")
    {
      auto temp = msg.instantiate<sensor_msgs::LaserScan>();
      auto scan_line = ::cartographer::sensor::ToPointCloud(::cartographer_ros::ToCartographer(*temp));
       int64_t time = cartographer_ros::FromRos(temp->header.stamp).time_since_epoch().count();
       
       	auto pose_geted = Interpolation(time, pose_list);
	auto pose_h = Interpolation(time, pose_list_h);
	if(pose_h.time==0)
	{
	  continue;
	}

      for(auto it:scan_line)
      {
	auto coor = it;
	coor=ori_tran.quat.toRotationMatrix()*coor+ori_tran.tran;
	coor = pose_geted.quat.toRotationMatrix()*pose_h.quat.toRotationMatrix()*coor+pose_geted.tran;
	
	pcl::PointXYZT temp_p;
	temp_p.t=time;
	temp_p.x=coor[0];
	temp_p.y=coor[1];
	temp_p.z=coor[2];
	if(Dis(temp_p,pose_geted)>10||Dis(temp_p,pose_geted)<0.3)
	{
	  continue;
	}
	
	cloud.push_back(temp_p);	
      }  
    }
 }
  std::cout<<std::to_string(cloud.size())<<std::endl;
  std::cout<<"output_path: "<<output_path+"_ver.pcd"<<std::endl;
  
  pcl::io::savePCDFileBinary(output_path+"_ver.pcd",cloud);
  return 0;
}
