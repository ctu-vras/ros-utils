/**
 * \file
 * \brief Unit test for loader_ros.h and nodelet_manager.h.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

// HACK: we need to alter the private lowlever_class_loader_ of pluginlib::ClassLoader
#define private public
#include <pluginlib/class_loader.hpp>
#undef private

#include <string>
#include <thread>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>

#include <cras_cpp_common/nodelet_utils/loader_ros.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager.h>
#include <cras_cpp_common/nodelet_utils/nodelet_manager_sharing_tf_buffer.h>
#include <cras_cpp_common/nodelet_utils/nodelet_with_shared_tf_buffer.hpp>
#include <cras_cpp_common/test_utils/preloading_class_loader.hpp>

class TestNodelet;
TestNodelet* nodeletInstance {nullptr};

class TestNodelet : public cras::NodeletWithSharedTfBuffer<nodelet::Nodelet>
{
public:
  TestNodelet()
  {
    nodeletInstance = this;
  }

  ~TestNodelet() override
  {
    nodeletInstance = nullptr;
  }

  void onInit() override
  {
    this->initialized = true;
  }

  bool initialized {false};
};

class TestLoader : public nodelet::Loader
{
public:
  TestLoader() : Loader([this](const std::string& lookupName)
    {
      this->lastLoadedName = lookupName;
      return boost::make_shared<TestNodelet>();
    })
  {
  }

  std::string lastLoadedName {};
};

class TestClassLoader : public pluginlib::ClassLoader<::nodelet::Nodelet>
{
public:
  TestClassLoader() : pluginlib::ClassLoader<nodelet::Nodelet>("cras_cpp_common", "nodelet::Nodelet")
  {
    PreloadingClassLoader<TestNodelet, nodelet::Nodelet>::preload("test", *this);
  }

  bool isClassLoaded(const std::string& lookup_name) override
  {
    return true;
  }

  std::string getClassType(const std::string& lookup_name) override
  {
    this->lastLoadedName = lookup_name;
    return "TestNodelet";
  }

  std::string lastLoadedName {};
};

class TestLoaderROS : public cras::LoaderROS
{
public:
  TestLoaderROS(nodelet::Loader* parent, const ros::NodeHandle& nh) : LoaderROS(parent, nh)
  {
  }
  using cras::LoaderROS::serviceLoad;
  using cras::LoaderROS::serviceUnload;
  using cras::LoaderROS::serviceList;
};

class TestManager : public cras::NodeletManager
{
public:
  explicit TestManager(ros::NodeHandle nh) : cras::NodeletManager(nh)
  {
  }

  void init() override
  {
    NodeletManager::init();
    this->classLoader = std::make_unique<TestClassLoader>();
  }

  std::string getLastLoadedName() const
  {
    return dynamic_cast<TestClassLoader*>(this->classLoader.get())->lastLoadedName;
  }
};

class TestTfManager : public cras::NodeletManagerSharingTfBuffer
{
public:
  explicit TestTfManager(ros::NodeHandle nh) : cras::NodeletManagerSharingTfBuffer(nh)
  {
  }

  void init() override
  {
    cras::NodeletManagerSharingTfBuffer::init();
    this->classLoader = std::make_unique<TestClassLoader>();
  }

  std::string getLastLoadedName() const
  {
    return dynamic_cast<TestClassLoader*>(this->classLoader.get())->lastLoadedName;
  }

  using cras::NodeletManagerSharingTfBuffer::buffer;
  using cras::NodeletManagerSharingTfBuffer::listener;
};

TEST(LoaderROS, ServicesDirect)  // NOLINT
{
  TestLoader loader;
  TestLoaderROS loaderRos(&loader, ros::NodeHandle());

  nodelet::NodeletList::Request listRequest;
  nodelet::NodeletList::Response listResponse;
  EXPECT_TRUE(loaderRos.serviceList(listRequest, listResponse));
  EXPECT_EQ(0u, listResponse.nodelets.size());

  nodelet::NodeletLoad::Request loadRequest;
  nodelet::NodeletLoad::Response loadResponse;

  loadRequest.name = "test";
  loadRequest.type = "test/type";
  loader.lastLoadedName = "";
  EXPECT_EQ(nullptr, nodeletInstance);
  EXPECT_TRUE(loaderRos.serviceLoad(loadRequest, loadResponse));
  EXPECT_TRUE(loadResponse.success);
  EXPECT_EQ(loadRequest.type, loader.lastLoadedName);
  ASSERT_NE(nullptr, nodeletInstance);
  EXPECT_TRUE(nodeletInstance->initialized);

  EXPECT_TRUE(loaderRos.serviceList(listRequest, listResponse));
  EXPECT_EQ(1u, listResponse.nodelets.size());
  EXPECT_EQ("test", listResponse.nodelets[0]);

  nodelet::NodeletUnload::Request unloadRequest;
  nodelet::NodeletUnload::Response unloadResponse;

  unloadRequest.name = "test";
  EXPECT_NE(nullptr, nodeletInstance);
  EXPECT_TRUE(loaderRos.serviceUnload(unloadRequest, unloadResponse));
  EXPECT_TRUE(unloadResponse.success);
  EXPECT_EQ(nullptr, nodeletInstance);

  EXPECT_TRUE(loaderRos.serviceList(listRequest, listResponse));
  EXPECT_EQ(0u, listResponse.nodelets.size());
}

TEST(LoaderROS, ServicesROS)  // NOLINT
{
  ros::NodeHandle nh;
  TestLoader loader;
  TestLoaderROS loaderRos(&loader, nh);

  bool serviceFound {false};
  bool finished {false};

  auto spinUntilFinished = [&]()
  {
    for (size_t i = 0; i < 100 && !finished; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  };

  auto listService = std::make_unique<ros::ServiceClient>();
  nodelet::NodeletList::Request listRequest;
  nodelet::NodeletList::Response listResponse;

  std::thread t([&]()
  {
    auto service = nh.serviceClient<nodelet::NodeletList>("list");
    ASSERT_TRUE(service.waitForExistence());
    serviceFound = true;
    *listService = service;

    EXPECT_TRUE(service.call(listRequest, listResponse));
    EXPECT_EQ(0u, listResponse.nodelets.size());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  ASSERT_TRUE(serviceFound);
  EXPECT_TRUE(finished);

  ASSERT_EQ(nullptr, nodeletInstance);
  finished = false;
  t = std::thread([&]()
  {
    nodelet::NodeletLoad::Request loadRequest;
    nodelet::NodeletLoad::Response loadResponse;

    auto service = nh.serviceClient<nodelet::NodeletLoad>("load_nodelet");
    ASSERT_TRUE(service.waitForExistence());

    loadRequest.name = "test";
    loadRequest.type = "test/type";
    loader.lastLoadedName = "";
    EXPECT_TRUE(service.call(loadRequest, loadResponse));
    EXPECT_TRUE(loadResponse.success);
    EXPECT_EQ(loadRequest.type, loader.lastLoadedName);
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
  ASSERT_NE(nullptr, nodeletInstance);
  EXPECT_TRUE(nodeletInstance->initialized);

  finished = false;
  t = std::thread([&]()
  {
    EXPECT_TRUE(listService->call(listRequest, listResponse));
    EXPECT_EQ(1u, listResponse.nodelets.size());
    EXPECT_EQ("test", listResponse.nodelets[0]);
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);

  ASSERT_NE(nullptr, nodeletInstance);
  finished = false;
  t = std::thread([&]()
  {
    nodelet::NodeletUnload::Request unloadRequest;
    nodelet::NodeletUnload::Response unloadResponse;

    auto service = nh.serviceClient<nodelet::NodeletUnload>("unload_nodelet");
    ASSERT_TRUE(service.waitForExistence());

    unloadRequest.name = "test";
    EXPECT_TRUE(service.call(unloadRequest, unloadResponse));
    EXPECT_TRUE(unloadResponse.success);
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
  ASSERT_EQ(nullptr, nodeletInstance);

  finished = false;
  t = std::thread([&]()
  {
    EXPECT_TRUE(listService->call(listRequest, listResponse));
    EXPECT_EQ(0u, listResponse.nodelets.size());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
}

TEST(NodeletManager, Services)  // NOLINT
{
  ros::NodeHandle nh("~");
  TestManager manager(nh);
  manager.init();

  bool serviceFound {false};
  bool finished {false};

  auto spinUntilFinished = [&]()
  {
    for (size_t i = 0; i < 100 && !finished; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  };

  auto listService = std::make_unique<ros::ServiceClient>();
  nodelet::NodeletList::Request listRequest;
  nodelet::NodeletList::Response listResponse;

  std::thread t([&]()
  {
    auto service = nh.serviceClient<nodelet::NodeletList>("list");
    ASSERT_TRUE(service.waitForExistence());
    serviceFound = true;
    *listService = service;

    EXPECT_TRUE(service.call(listRequest, listResponse));
    EXPECT_EQ(0u, listResponse.nodelets.size());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  ASSERT_TRUE(serviceFound);
  EXPECT_TRUE(finished);

  EXPECT_EQ(nullptr, nodeletInstance);
  finished = false;
  t = std::thread([&]()
  {
    nodelet::NodeletLoad::Request loadRequest;
    nodelet::NodeletLoad::Response loadResponse;

    auto service = nh.serviceClient<nodelet::NodeletLoad>("load_nodelet");
    ASSERT_TRUE(service.waitForExistence());

    loadRequest.name = "test";
    loadRequest.type = "test/type";
    EXPECT_TRUE(service.call(loadRequest, loadResponse));
    EXPECT_TRUE(loadResponse.success);
    EXPECT_EQ(loadRequest.type, manager.getLastLoadedName());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
  ASSERT_NE(nullptr, nodeletInstance);
  EXPECT_TRUE(nodeletInstance->initialized);

  finished = false;
  t = std::thread([&]()
  {
    EXPECT_TRUE(listService->call(listRequest, listResponse));
    EXPECT_EQ(1u, listResponse.nodelets.size());
    EXPECT_EQ("test", listResponse.nodelets[0]);
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);

  EXPECT_NE(nullptr, nodeletInstance);
  finished = false;
  t = std::thread([&]()
  {
    nodelet::NodeletUnload::Request unloadRequest;
    nodelet::NodeletUnload::Response unloadResponse;

    auto service = nh.serviceClient<nodelet::NodeletUnload>("unload_nodelet");
    ASSERT_TRUE(service.waitForExistence());

    unloadRequest.name = "test";
    EXPECT_TRUE(service.call(unloadRequest, unloadResponse));
    EXPECT_TRUE(unloadResponse.success);
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
  EXPECT_EQ(nullptr, nodeletInstance);

  finished = false;
  t = std::thread([&]()
  {
    EXPECT_TRUE(listService->call(listRequest, listResponse));
    EXPECT_EQ(0u, listResponse.nodelets.size());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
}

TEST(NodeletManagerSharingTfBuffer, ShareBuffer)  // NOLINT
{
  ros::NodeHandle nh("~");
  TestTfManager manager(nh);
  manager.init();

  EXPECT_NE(nullptr, manager.buffer);
  EXPECT_NE(nullptr, manager.listener);

  const auto time = ros::Time::now();
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf.transform.translation.x = 1;
  tf.transform.rotation.w = 1;
  manager.buffer->setTransform(tf, "test");

  ASSERT_TRUE(manager.buffer->canTransform("test", "test2", time));
  const auto tf2 = manager.buffer->lookupTransform("test", "test2", time);
  EXPECT_EQ(tf, tf2);

  bool finished {false};

  auto spinUntilFinished = [&]()
  {
    for (size_t i = 0; i < 100 && !finished; ++i)
    {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  };

  EXPECT_EQ(nullptr, nodeletInstance);
  finished = false;
  std::thread t([&]()
  {
    nodelet::NodeletLoad::Request loadRequest;
    nodelet::NodeletLoad::Response loadResponse;

    auto service = nh.serviceClient<nodelet::NodeletLoad>("load_nodelet");
    ASSERT_TRUE(service.waitForExistence());

    loadRequest.name = "test";
    loadRequest.type = "test/type";
    EXPECT_TRUE(service.call(loadRequest, loadResponse));
    EXPECT_TRUE(loadResponse.success);
    EXPECT_EQ(loadRequest.type, manager.getLastLoadedName());
    finished = true;
  });
  t.detach();
  spinUntilFinished();
  EXPECT_TRUE(finished);
  ASSERT_NE(nullptr, nodeletInstance);
  EXPECT_TRUE(nodeletInstance->initialized);
  EXPECT_TRUE(nodeletInstance->usesSharedBuffer());

  auto& buffer = nodeletInstance->getBuffer();
  auto& rawBuffer = nodeletInstance->getBuffer().getRawBuffer();
  EXPECT_TRUE(buffer.canTransform("test", "test2", time, ros::Duration(0)));
  EXPECT_TRUE(buffer.canTransform("test", "test2", time));
  EXPECT_TRUE(rawBuffer.canTransform("test", "test2", time));
  const auto tf3 = buffer.lookupTransform("test", "test2", time, ros::Duration(0));
  const auto tf4 = buffer.lookupTransform("test", "test2", time);
  const auto tf5 = rawBuffer.lookupTransform("test", "test2", time);
  EXPECT_EQ(tf, tf3);
  EXPECT_EQ(tf, tf4);
  EXPECT_EQ(tf, tf5);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nodelet_manager");
  ros::start();
  return RUN_ALL_TESTS();
}
