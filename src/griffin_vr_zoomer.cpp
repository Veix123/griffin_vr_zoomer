#include "griffin_vr_zoomer/griffin_vr_zoomer.h"

#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <iostream>


GriffinVRZoomer::GriffinVRZoomer()
{
  nh_ = ros::NodeHandle("~");

  // Load rviz_plugin_manager service clients
  load_plugin_client_ = nh_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
  unload_plugin_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
  get_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_"
                                                              "config");
  set_plugin_config_client_ =
      nh_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_"
                                                              "config");

  waitForRvizPluginManager();

  // subscribe to griffin events
  std::string topic = "/griffin_powermate/events";
  nh_.getParam("powermate_topic", topic);
  sub_ = nh_.subscribe(topic, 100, &GriffinVRZoomer::griffinCallback, this);
  ROS_INFO_STREAM("Listening on topic: '" << topic<<"'");
}

GriffinVRZoomer::~GriffinVRZoomer()
{
  sub_.shutdown();
}

void GriffinVRZoomer::waitForRvizPluginManager()
{
  // Wait until rviz_plugin_manager clients become active or throw an error on timeout
  while (!load_plugin_client_.exists() || !unload_plugin_client_.exists() ||
         !set_plugin_config_client_.exists() || !get_plugin_config_client_.exists())
  {
    ROS_INFO("Waiting for rviz_plugin_manager ...");
    ros::Duration(1).sleep();
    if(!ros::ok())
    {
      throw std::runtime_error("Got interrupt from ROS");
    }
  }
  ROS_INFO("All rviz_plugin_manager services connected.");
}

void GriffinVRZoomer::griffinCallback(const griffin_powermate::PowermateEvent& msg)
{
  ROS_INFO_STREAM("Event arrived:" << msg);

  rviz_plugin_manager::PluginSetConfig set_cfg_msg;
  try
  {
    set_plugin_config_client_.call(set_cfg_msg);
  }
  catch(ros::Exception e)
  {
    ROS_ERROR_STREAM("Could not set config: " << e.what());
  }
}

int main(int argc, char* argv[])
{
  ROS_INFO_STREAM("Zoomer main");

  // ROS init
  ros::init(argc, argv, "griffin_vr_zoomer");

  try
  {
    GriffinVRZoomer zoomer;
    ros::spin();
  }
  catch(std::exception& e)
  {
    std::cout << "Got exception from GriffinVRZoomer: " << e.what()<< std::endl;
  }
}
