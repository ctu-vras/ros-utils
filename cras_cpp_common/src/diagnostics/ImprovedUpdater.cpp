#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>

#include <unistd.h>

namespace diagnostic_updater
{

ImprovedUpdater::ImprovedUpdater(ros::NodeHandle h, ros::NodeHandle ph, std::string node_name) : Updater(h, ph, node_name)
{
  char hostname[1024];
  hostname[1023] = '\0';
  gethostname(hostname, 1023);
  this->setHardwareID(std::string(hostname));
}

}
