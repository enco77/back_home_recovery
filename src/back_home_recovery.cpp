
#include <back_home_recovery/back_home_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/pointer_cast.hpp>
#include <vector>
//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(back_home_recovery::BackHomeRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace back_home_recovery {
BackHomeRecovery::BackHomeRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void BackHomeRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle nh;
    initialized_ = true;
    _pub = nh.advertise<std_msgs::String>("ui_operation", 10);
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void BackHomeRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the BackHomeRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  else
  {
    ROS_WARN("Running BackHomeRecovery Robot go back to home pose");
    msg.data = "stop";
    _pub.publish(msg);
    ros::spinOnce();
    local_costmap_->updateMap();
    global_costmap_->updateMap();
  }
}

};
