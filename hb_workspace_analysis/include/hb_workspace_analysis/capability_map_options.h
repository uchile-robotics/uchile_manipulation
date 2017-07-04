#ifndef PROJECT_CAPABILITY_MAP_OPTIONS_H_H
#define PROJECT_CAPABILITY_MAP_OPTIONS_H_H

#include <ros/ros.h>

namespace hb_workspace_analysis
{
  class CapabilityMapOptions
  {
  public:
    // Group name (MoveIt! joint group)
    std::string group_name;
    // Resolution of capability map
    double resolution;
    // Search range factor
    double search_factor;
    // Database
    std::string db_server;
    int db_port;
    std::string db_name;
    std::string db_collection;

  public:
    CapabilityMapOptions();

    bool load(const ros::NodeHandle &nh);

    friend std::ostream &operator<<(std::ostream &os, const CapabilityMapOptions &opt);
  };
}

#endif
