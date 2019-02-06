#include "cras_cpp_common/string_utils.hpp"

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

namespace cras {

void warnLeadingSlash(const std::string& s)
{
  ROS_WARN_STREAM("Found initial slash in " << s);
}

void stripLeadingSlash(std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    s.erase(0, 1);
  }
}

std::string stripLeadingSlash(const std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    return s.substr(1);
  }

  return s;
}

std::string prependIfNonEmpty(const std::string &str, const std::string &prefix)
{
  return str.empty() ? str : prefix + str;
}

std::string appendIfNonEmpty(const std::string &str, const std::string &suffix)
{
  return str.empty() ? str : str + suffix;
}

};