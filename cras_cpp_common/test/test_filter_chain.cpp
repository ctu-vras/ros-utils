/**
 * \file
 * \brief Unit test for filter_chain.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

// HACK: to enable dynamically adding instances of filters
// TODO: find a non-hack way to test it (probably configure()/clear() calls)
#include <sstream>  // has to be there, otherwise we encounter build problems
#include <ros/common.h>
#define private protected
#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_chain.h>
#endif
#undef private

#include "gtest/gtest.h"

#include <string>

#include <cras_cpp_common/filter_utils/filter_chain.hpp>

#define TRACE SCOPED_TRACE("Failure happened here");

template <typename T>
class TestChain : public cras::FilterChain<T>
{
public:
  TestChain(const std::string& dataType,
            const typename cras::FilterChain<T>::FilterFinishedCallback& filterCallback = {},
            const typename cras::FilterChain<T>::FilterStartCallback& filterStartCallback = {}) :
    cras::FilterChain<T>(dataType, filterCallback, filterStartCallback)
  {
  }

  void addFilter(std::shared_ptr<filters::FilterBase<T>> filter)
  {
    this->reference_pointers_.push_back(filter);
    this->updateActiveFilters();
  }
};

template <typename T>
class TestFilter : public cras::FilterBase<T>
{
public:
  explicit TestFilter(T value) : value(std::move(value))
  {
    this->configure();
  }

  bool update(const T& data_in, T& data_out) override
  {
    data_out = value;
    return true;
  }

  const nodelet::Nodelet* getNodelet() const
  {
    return this->nodelet;
  }

protected:
  bool configure() override
  {
    XmlRpc::XmlRpcValue config;
    config["name"] = "test_filter";
    config["type"] = "cras_cpp_common/TestFilter";
    this->loadConfiguration(config);
    return true;
  }

  T value;
};

TEST(FilterChain, Basic)
{
  XmlRpc::XmlRpcValue config;
  config.setSize(1);
  config[0]["name"] = "mean";
  config[0]["type"] = "filters/MeanFilterDouble";
  config[0]["params"]["number_of_observations"] = 5;

  TestChain<double> chain("double");
  EXPECT_TRUE(chain.configure(config, "test"));

  double out = 1.0;

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(0.0, out);

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(5.0, out);

  EXPECT_TRUE(chain.update(20.0, out));
  EXPECT_EQ(10.0, out);

  auto f = std::make_shared<TestFilter<double>>(3.0);
  chain.addFilter(f);

  chain.disableFilter("test_filter");

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(10.0, out);

  chain.enableFilter("test_filter");

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(3.0, out);

  chain.setDisabledFilters({"test_filter", "mean"});

  EXPECT_TRUE(chain.update(7.0, out));
  EXPECT_EQ(7.0, out);

  chain.enableFilter("test_filter");

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(3.0, out);

  chain.enableFilter("mean");
  chain.disableFilter("test_filter");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(10.0, out);

  chain.enableFilter("test_filter");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(3.0, out);

  chain.enableFilter("nonexistent");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(3.0, out);

  chain.enableFilter("test_filter");
  chain.enableFilter("test_filter");
  chain.enableFilter("test_filter");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(3.0, out);

  chain.enableFilter("test_filter");
  chain.disableFilter("test_filter");
  chain.enableFilter("test_filter");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(3.0, out);

  chain.disableFilter("test_filter");
  chain.enableFilter("test_filter");
  chain.disableFilter("test_filter");

  EXPECT_TRUE(chain.update(50.0, out));
  EXPECT_EQ(10.0, out);

  chain.enableFilter("test_filter");
  chain.disableFilter("mean");
  chain.disableFilter("nonexistent2");

  EXPECT_TRUE(chain.update(50.0, out));
  EXPECT_EQ(3.0, out);

  EXPECT_TRUE(chain.clear());

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(10.0, out);

  chain.enableFilter("test_filter");
  chain.enableFilter("mean");

  // chain is cleared, so the filters should not be there
  EXPECT_TRUE(chain.update(5.0, out));
  EXPECT_EQ(5.0, out);

  EXPECT_TRUE(chain.configure(config, "test"));

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(0.0, out);

  EXPECT_TRUE(chain.update(10.0, out));
  EXPECT_EQ(5.0, out);

  EXPECT_TRUE(chain.update(20.0, out));
  EXPECT_EQ(10.0, out);

  chain.addFilter(f);

  EXPECT_TRUE(chain.update(50.0, out));
  EXPECT_EQ(3.0, out);

  // check that disabling a filter before clear has no effect to the filter after new configuration
  chain.disableFilter("mean");
  EXPECT_TRUE(chain.clear());
  chain.configure(config, "test");

  EXPECT_TRUE(chain.update(0.0, out));
  EXPECT_EQ(0.0, out);

  EXPECT_TRUE(chain.update(20.0, out));
  EXPECT_EQ(10.0, out);
}

TEST(FilterChain, Callback)
{
  XmlRpc::XmlRpcValue config;
  config.setSize(3);
  config[0]["name"] = "inc1";
  config[0]["type"] = "filters/IncrementFilterInt";
  config[1]["name"] = "inc2";
  config[1]["type"] = "filters/IncrementFilterInt";
  config[2]["name"] = "inc3";
  config[2]["type"] = "filters/IncrementFilterInt";

  size_t numCalled = 0;
  auto cb = [&numCalled](const int& data, const size_t filterNum, const std::string& name, const std::string& type,
    const bool success)
  {
    if (filterNum < 3)
    {
      EXPECT_EQ(filterNum + 1, data);
      EXPECT_EQ("inc" + std::to_string(filterNum + 1), name);
      EXPECT_EQ("filters/IncrementFilterInt", type);
      EXPECT_TRUE(success);
    }
    else
    {
      EXPECT_EQ(10, data);
      EXPECT_EQ("test_filter", name);
      EXPECT_EQ("cras_cpp_common/TestFilter", type);
      EXPECT_TRUE(success);
    }
    numCalled++;
  };

  size_t numStartedCalled = 0;
  auto cbStarted = [&numStartedCalled](const int& data, const size_t filterNum, const std::string& name,
    const std::string& type)
  {
    if (filterNum < 3)
    {
      EXPECT_EQ(filterNum, data);
      EXPECT_EQ("inc" + std::to_string(filterNum + 1), name);
      EXPECT_EQ("filters/IncrementFilterInt", type);
    }
    else
    {
      EXPECT_EQ(3, data);
      EXPECT_EQ("test_filter", name);
      EXPECT_EQ("cras_cpp_common/TestFilter", type);
    }
    numStartedCalled++;
  };

  TestChain<int> chain("int", cb, cbStarted);
  EXPECT_TRUE(chain.configure(config, "test"));

  auto f = std::make_shared<TestFilter<int>>(10);
  chain.addFilter(f);

  int out = 1;

  numCalled = 0;
  numStartedCalled = 0;
  {TRACE; EXPECT_TRUE(chain.update(0.0, out));}
  EXPECT_EQ(10, out);
  EXPECT_EQ(4, numCalled);
  EXPECT_EQ(4, numStartedCalled);

  chain.disableFilter("test_filter");

  numCalled = 0;
  numStartedCalled = 0;
  {TRACE; EXPECT_TRUE(chain.update(0.0, out));}
  EXPECT_EQ(3, out);
  EXPECT_EQ(3, numCalled);
  EXPECT_EQ(3, numStartedCalled);

  chain.enableFilter("test_filter");

  numCalled = 0;
  numStartedCalled = 0;
  {TRACE; EXPECT_TRUE(chain.update(0.0, out));}
  EXPECT_EQ(10, out);
  EXPECT_EQ(4, numCalled);
  EXPECT_EQ(4, numStartedCalled);

  chain.disableFilter("inc3");
  chain.disableFilter("test_filter");

  numCalled = 0;
  numStartedCalled = 0;
  {TRACE; EXPECT_TRUE(chain.update(0.0, out));}
  EXPECT_EQ(2, out);
  EXPECT_EQ(2, numCalled);
  EXPECT_EQ(2, numStartedCalled);

  size_t numCalled2 = 0;
  auto cb2 = [&numCalled2](const int& data, const size_t filterNum, const std::string& name, const std::string& type,
    const bool success)
  {
    EXPECT_EQ(1, data);
    EXPECT_EQ("inc2", name);
    EXPECT_EQ("filters/IncrementFilterInt", type);
    EXPECT_TRUE(success);
    numCalled2++;
  };

  size_t numStartedCalled2 = 0;
  auto cbStarted2 = [&numStartedCalled2](const int& data, const size_t filterNum, const std::string& name,
    const std::string& type)
  {
    EXPECT_EQ(0, data);
    EXPECT_EQ("inc2", name);
    EXPECT_EQ("filters/IncrementFilterInt", type);
    numStartedCalled2++;
  };

  chain.disableFilter("inc1");
  chain.setFilterStartCallback(cbStarted2);
  chain.setFilterFinishedCallback(cb2);

  numCalled = 0;
  numCalled2 = 0;
  numStartedCalled = 0;
  numStartedCalled2 = 0;
  {TRACE; EXPECT_TRUE(chain.update(0.0, out));}
  EXPECT_EQ(1, out);
  EXPECT_EQ(0, numCalled);
  EXPECT_EQ(1, numCalled2);
  EXPECT_EQ(0, numStartedCalled);
  EXPECT_EQ(1, numStartedCalled2);
}

class TestNodelet : public nodelet::Nodelet
{
  void onInit() override
  {
  }
};

TEST(FilterChain, SetNodelet)
{
  XmlRpc::XmlRpcValue config;
  config.setSize(1);
  config[0]["name"] = "inc";
  config[0]["type"] = "filters/IncrementFilterInt";

  TestChain<int> chain("int");
  EXPECT_TRUE(chain.configure(config, "test"));

  auto f = std::make_shared<TestFilter<int>>(10);
  chain.addFilter(f);

  EXPECT_EQ(nullptr, f->getNodelet());

  TestNodelet n;
  chain.setNodelet(&n);

  EXPECT_EQ(&n, f->getNodelet());

  chain.setNodelet(nullptr);

  EXPECT_EQ(nullptr, f->getNodelet());

  // test that nodelets are set for disabled filters, too
  chain.disableFilter("test_filter");

  chain.setNodelet(&n);

  EXPECT_EQ(&n, f->getNodelet());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
