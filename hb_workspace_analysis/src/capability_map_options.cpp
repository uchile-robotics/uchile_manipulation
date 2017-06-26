#include "hb_workspace_analysis/capability_map_options.h"


namespace hb_workspace_analysis
{

CapabilityMapOptions::CapabilityMapOptions()
{}

bool CapabilityMapOptions::load(const ros::NodeHandle &nh)
{
  // Joint group
  if (!nh.hasParam("group_name"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"group_name\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("group_name", group_name);
  // Resolution
  if (!nh.hasParam("resolution"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"resolution\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("resolution", resolution);
  // Search factor
  if (!nh.hasParam("search_factor"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"search_factor\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("search_factor", search_factor);
  // Database parameters
  // Server
  if (!nh.hasParam("database/server"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"database/server\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("database/server", db_server);
  // Port
  if (!nh.hasParam("database/port"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"database/port\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("database/port", db_port);
  // Database name
  if (!nh.hasParam("database/name"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"database/name\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("database/name", db_name);
  // Collection
  if (!nh.hasParam("database/collection"))
  {
    ROS_ERROR_STREAM_NAMED("capability_map_options_loader",
                           "Capability map configuration parameter \"database/collection\" missing from parameter server. Searching in namespace: "
                               << nh.getNamespace());
    return false;
  }
  nh.getParam("database/collection", db_collection);
}

std::ostream &operator<<(std::ostream &os, const hb_workspace_analysis::CapabilityMapOptions &opt)
{
  os << "Capablity map options:" << std::endl;
  os << "Group name: \"" << opt.group_name << "\"" << std::endl;
  os << "Database:" << std::endl;
  os << "\tServer: \"" << opt.db_server << "\"" << std::endl;
  os << "\tPort: " << opt.db_port << std::endl;
  os << "\tDatabase name: \"" << opt.db_name << "\"" << std::endl;
  os << "\tCollection: \"" << opt.db_collection << "\"" << std::endl;

  return os;
}
}
