#ifndef PROJECT_FUSION_ASSEMBLER_H
#define PROJECT_FUSION_ASSEMBLER_H

#define __APP_NAME__ "fusion_assembler"

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

#include "autoware_msgs/DetectedObjectArray.h"

class ROSFusionAssemblerApp
{
  ros::NodeHandle node_handle_;

  ros::Publisher publisher_assembled_objects_;
  std::vector<ros::Subscriber> object_array_subscribers_;

  std::map<std::string, autoware_msgs::DetectedObjectArray::ConstPtr> object_array_map_;
  size_t num_total_arrays_;

  bool assembling_;

  void DetectedObjectArrayCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_object_array_msg);

  void InitializeROSIo(ros::NodeHandle &in_private_handle);

public:
  void Run();

  ROSFusionAssemblerApp();
};

#endif //PROJECT_FUSION_ASSEMBLER_H
