#include "griffin_vr_zoomer/griffin_vr_zoomer.h"

#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <iostream>

GriffinVRZoomer::GriffinVRZoomer(std::string rviz_ns, std::string griffin_ns)
  : rviz_nh_(rviz_ns), griffin_nh_(griffin_ns), plugin_id_(-1)
{
  // subscribe to griffin events
  std::string topic = "events";
  griffin_nh_.getParam("griffin_topic", topic);
  sub_ = griffin_nh_.subscribe(topic, 5, &GriffinVRZoomer::griffinCallback, this);
  ROS_INFO_STREAM("Listening on topic: '" << topic<<"'");
  // Load rviz_plugin_manager service clients
  load_plugin_client_ = rviz_nh_.serviceClient<rviz_plugin_manager::PluginLoad>("rviz_plugin_load");
  unload_plugin_client_ =
      rviz_nh_.serviceClient<rviz_plugin_manager::PluginUnload>("rviz_plugin_unload");
  get_plugin_config_client_ =
      rviz_nh_.serviceClient<rviz_plugin_manager::PluginGetConfig>("rviz_plugin_get_"
                                                              "config");
  set_plugin_config_client_ =
      rviz_nh_.serviceClient<rviz_plugin_manager::PluginSetConfig>("rviz_plugin_set_"
                                                              "config");

  waitForRvizPluginManager();

  rviz_plugin_manager::PluginLoad load_msg;
  load_msg.request.plugin_class = "OSVR Plugin for RViz";
  load_msg.request.plugin_name = "OSVR Plugin";
  load_msg.request.plugin_config = "{Follow RViz camera: false, Fullscreen: true, Target frame: spacenav, Offset: {X: -0.6, Y: 0.0, Z: 0.3}, Screen name: HDMI-1, Publish tf: false, Use tracker: false}";
  if(load_plugin_client_.call(load_msg) && load_msg.response.code == 0)
  {
    plugin_id_ = load_msg.response.plugin_uid;
  }
  else
  {
      throw std::runtime_error("Unable to call rviz_plugin_manager");
  }


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

  //build the config str;
  double sensitivity_x = 0.01;
  double sensitivity_z = -0.005;
  long integral = msg.integral - 60;
  std::stringstream conf;
  conf << "Offset: {X: " << integral*sensitivity_x;
  conf << ", Y: 0, Z: " << integral*sensitivity_z;
  conf << "}";

  rviz_plugin_manager::PluginSetConfig set_cfg_msg;
  set_cfg_msg.request.config = conf.str();
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
  // ROS init
  ros::init(argc, argv, "griffin_vr_zoomer");

  // get rviz and griffin namespaces
  std::string rviz_ns = "/rviz";
  std::string griffin_ns = "/griffin_powermate";
  ros::NodeHandle nh("~");
  nh.getParam("rviz_ns", rviz_ns);
  nh.getParam("griffin_ns", griffin_ns);

  try
  {
    GriffinVRZoomer zoomer(rviz_ns, griffin_ns);
    ros::spin();
  }
  catch(std::exception& e)
  {
    std::cout << "Got exception from GriffinVRZoomer: " << e.what()<< std::endl;
  }
}
