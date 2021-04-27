#include "range_vision_fusion/fusion_assembler.h"

void
ROSFusionAssemblerApp::DetectedObjectArrayCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_object_array)
{
  if (!assembling_) {
    object_array_map_[in_object_array->header.frame_id] = in_object_array;
 
    if (object_array_map_.size() >= num_total_arrays_) {
      assembling_ = true;
      autoware_msgs::DetectedObjectArray total_object_array;
      total_object_array.objects.clear();
  
      for (auto entry : object_array_map_)
      {
        total_object_array.objects.insert(total_object_array.objects.end(), entry.second->objects.begin(), entry.second->objects.end());
      } 
      object_array_map_.clear();
    
      publisher_assembled_objects_.publish(total_object_array);

      assembling_ = false;
    }
  } 
}

void
ROSFusionAssemblerApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  std::vector<std::string> object_array_srcs;
  in_private_handle.getParam("object_array_srcs", object_array_srcs);

  num_total_arrays_ = object_array_srcs.size();
  
  for (const std::string& object_array_src : object_array_srcs)
  {
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, object_array_src.c_str());
    object_array_subscribers_.push_back(in_private_handle.subscribe(object_array_src, 1, &ROSFusionAssemblerApp::DetectedObjectArrayCallback, this));
  }

  std::string assembled_topic_str = "/detection/fusion_tools/objects";
  publisher_assembled_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(assembled_topic_str, 1);
  ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, assembled_topic_str.c_str());
}

void
ROSFusionAssemblerApp::Run()
{
  ros::NodeHandle private_node_handle("~");
  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);

}

ROSFusionAssemblerApp::ROSFusionAssemblerApp()
{
  num_total_arrays_ = 0;
  assembling_ = true;
}
