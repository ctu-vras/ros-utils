/**
 * \file
 * \brief A nodelet manager that can share its TF buffer with cras::NodeletWithSharedTfBuffer nodelets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/init.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager_sharing_tf_buffer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manager");
  cras::NodeletManagerSharingTfBuffer manager(ros::NodeHandle("~"));
  manager.init();
  ros::spin();
	return 0;
}
