#ifndef GRIFFIN_VR_ZOOMER_H
#define GRIFFIN_VR_ZOOMER_H

#include <ros/ros.h>
#include "griffin_powermate/PowermateEvent.h"

#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"

class GriffinVRZoomer
{
public:
  GriffinVRZoomer(std::string rviz_ns, std::string griffin_ns);
  ~GriffinVRZoomer();

private:

  void waitForRvizPluginManager();
  void griffinCallback(const griffin_powermate::PowermateEvent& msg);

// nodehandles in rviz and griffin powermate namespaces
  ros::NodeHandle rviz_nh_;
  ros::NodeHandle griffin_nh_;

  // griffin event subscriber
  ros::Subscriber sub_;

  // rviz_plugin_manager clients
  ros::ServiceClient load_plugin_client_;
  ros::ServiceClient unload_plugin_client_;
  ros::ServiceClient set_plugin_config_client_;
  ros::ServiceClient get_plugin_config_client_;
};
#endif
