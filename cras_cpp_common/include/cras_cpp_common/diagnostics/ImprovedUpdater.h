#pragma once

#include <diagnostic_updater/diagnostic_updater.h>

namespace diagnostic_updater
{

/** \brief Automatically set hardware ID to hostname of the machine the node runs on. */
class ImprovedUpdater : public Updater
{
  public: ImprovedUpdater(ros::NodeHandle h = ros::NodeHandle(), ros::NodeHandle ph = ros::NodeHandle("~"),
      std::string node_name = ros::this_node::getName());
};

}

