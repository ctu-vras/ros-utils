/**
 * \file
 * \brief Unit test for filter_base.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>
#include <list>
#include <map>
#include <string>
#include <tuple>

#include <cras_cpp_common/filter_utils/filter_base.hpp>

#include "get_param_test.inc"

using namespace cras;

class FilterUtils : public testing::TestWithParam<std::tuple<bool>> {};

struct TestFilter : public cras::FilterBase<std::string>, public GetParamTest<TestFilter>
{
  bool isChainConfig;
  explicit TestFilter(bool isChainConfig) : isChainConfig(isChainConfig) {}
  bool update(const std::string &data_in, std::string &data_out) override
  {
    return false;
  }

  bool configure() override
  {
    return true;
  }

  template <typename T, typename U>
  void test(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->getParamVerbose((param), (def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->getParamVerbose((param), cras::optional(def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->getParam((param), (def)));
    EXPECT_EQ((expected), this->getParam((param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_s(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->getParamVerbose((param), (def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->getParamVerbose((param), cras::optional(def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->getParam((param), (def)));
    EXPECT_EQ((expected), this->getParam((param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_near(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->getParamVerbose((param), (def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->getParamVerbose((param), cras::optional(def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_GT(1e-6, distance((expected), this->getParam((param), (def))));
    EXPECT_GT(1e-6, distance((expected), this->getParam((param), cras::optional(def))));
  }

  template <typename T>
  void test_defaults(const std::string& param, const T& def)
  {
    this->test_s((param), (def), (def), true);
  }

  void testMisc()
  {
    // test that getParam result type is the same as the default value
    static_assert(std::is_same<bool, decltype(this->getParam("bool_True", false))>::value);
    static_assert(std::is_same<bool, decltype(this->getParam("bool_True", cras::optional(false)))>::value);

    // test that C-string methods return a std::string result
    static_assert(std::is_same<std::string, decltype(this->getParam("bool_True", "0"))>::value);
    static_assert(std::is_same<std::string, decltype(this->getParam("bool_True", cras::optional("")))>::value);

    // test non-verbose getParam
    EXPECT_EQ(true, this->getParam("bool_True", false));
    // test explicit conversion of GetParamResult to the contained type
    EXPECT_EQ(true, static_cast<bool>(this->getParamVerbose("bool_True", false)));
    // test implicit conversion of GetParamResult to the contained type
    { bool r = this->getParamVerbose("bool_True", false); EXPECT_EQ(true, r); }

    // test double NaN behavior
    { auto r = this->getParamVerbose("double_nan", 0.0);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    { auto r = this->getParamVerbose("double_nan", cras::optional(0.0));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->getParam("double_nan", 0.0)));
    EXPECT_TRUE(std::isnan(this->getParam("double_nan", cras::optional(0.0))));

    { auto r = this->getParamVerbose("nonexistent", std::numeric_limits<double>::quiet_NaN());
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = this->getParamVerbose("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->getParam("nonexistent", std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(this->getParam("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()))));
  };


  public: void testOptions()
  {
    GetParamOptions<bool, bool> opts;
    GetParamResult<bool> r(false, {});
    const auto& ns = this->param->getNamespace();

    Appender log;
    ros::console::register_appender(&log);

    // test reading an existing param with default config
    opts = {}; log.reset();
    r = this->getParamVerbose("bool_False", true, "", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing reports about using default value as info messages
    opts = {}; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing reports about using default value as warning messages
    opts = {}; opts.printDefaultAsWarn = true; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Warn, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test not printing log messages
    opts = {}; opts.printMessages = false; log.reset();
    r = this->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ("", log.str);
    EXPECT_EQ(ros::console::Level::Count, log.level);

    // required parameter is missing
    opts = {}; log.reset();
    EXPECT_THROW(this->getParamVerbose<bool>("nonexistent", std::nullopt, "", opts), GetParamException);
    try
    {
      this->getParamVerbose<bool>("nonexistent", std::nullopt, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_TRUE(e.info.requiredMissing);
      EXPECT_FALSE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent.", e.info.message);
      EXPECT_EQ(e.info.message, log.str);
      EXPECT_EQ(e.info.messageLevel, log.level);
    }

    // test throwing exception on convert failure
    opts = {}; opts.throwIfConvertFails = true; log.reset();
    EXPECT_THROW(this->getParamVerbose("str_a", false, "", opts), GetParamException);
    try
    {
      this->getParamVerbose("str_a", false, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_FALSE(e.info.requiredMissing);
      EXPECT_TRUE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                     "Expected type bool, got type string with value <value>a</value>.", e.info.message);
      EXPECT_EQ(e.info.message, log.str);
      EXPECT_EQ(e.info.messageLevel, log.level);
    }

    // test failed conversion without throwing exception
    opts = {}; opts.throwIfConvertFails = false; log.reset();
    r = this->getParamVerbose("str_a", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                   "Expected type bool, got type string with value <value>a</value>. "
                   "Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing custom units
    opts = {}; log.reset();
    r = this->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test custom toString and toResult methods
    opts = {}; log.reset();
    opts.paramToStr = [](const bool& v) -> std::string {return v ? "tRuE" : "fAlSe"; };
    opts.resultToStr = [](const bool& v) -> std::string {return v ? "TRUE" : "FALSE"; };
    opts.toResult = [](const bool& v) -> bool { if (v) throw std::runtime_error("fail"); return v; };
    // reading false succeeds
    r = this->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    // reading true throws exception
    r = this->getParamVerbose("bool_True", false, "bools", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot convert value 'tRuE' of parameter bool_True to requested type bool (error: fail). "
                   "Assigning default: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test reading a parameter whose conversion from XmlRpc type fails
    log.reset();
    auto r2 = this->getParamVerbose("int_max", static_cast<short>(1), "", {});  // NOLINT
    EXPECT_TRUE(r2.info.defaultUsed);
    EXPECT_FALSE(r2.info.requiredMissing);
    EXPECT_TRUE(r2.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r2.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter int_max found with correct XmlRpc type int and value "
                   "<value><i4>2147483647</i4></value>, but its conversion to type short int has failed "
                   "due to the following errors: [\"Value 2147483647 is out of bounds <-32768, 32767>.\"]. "
                   "Assigning default: 1.", r2.info.message);
    EXPECT_EQ(r2.info.message, log.str);
    EXPECT_EQ(r2.info.messageLevel, log.level);

    // test reading a parameter whose conversion from XmlRpc type fails using a custom toParam function
    opts = {}; log.reset();
    opts.toParam =
      [](const XmlRpc::XmlRpcValue& x, bool& v, bool skipNonConvertible = false,
        std::list<std::string>* errors = nullptr) -> bool
      {
        if (static_cast<bool>(x))
        {
          if (errors)
            errors->push_back("cannot convert");
          return false;
        }
        v = false;
        return true;
      };
    r = this->getParamVerbose("bool_True", false, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter bool_True found with correct XmlRpc type bool and value "
                   "<value><boolean>1</boolean></value>, but its conversion to type bool has failed "
                   "due to the following errors: [\"cannot convert\"]. "
                   "Assigning default: False.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
  }
};

std::shared_ptr<TestFilter> getFilter(bool paramsFromChain)
{
  static std::map<bool, std::shared_ptr<TestFilter>> cache;
  if (cache.find(paramsFromChain) != cache.end())
    return cache[paramsFromChain];

  static ros::NodeHandle nh;  // without the static modifier, rosconsole logging turns off and logging tests fail
  auto filter = std::make_shared<TestFilter>(paramsFromChain);
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<std::string>>(filter);
  if (paramsFromChain)
  {
    XmlRpc::XmlRpcValue value;
    nh.getParam("test_chain_config", value);
    filterBase->configure(value[0]);
  }
  else
  {
    // rosparam load cannot load empty dict parameter from the YAML file
    nh.setParam("test_dict_config/params/dict_empty", std::map<std::string, std::string>());
    filterBase->configure("test_dict_config", nh);
  }
  cache[paramsFromChain] = filter;

  return filter;
}

TEST_P(FilterUtils, XmlRpcValue)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testXmlRpcValue(std::get<0>(GetParam()));
}

TEST_P(FilterUtils, Bool)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testBool();
}

TEST_P(FilterUtils, Int)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testInt();
}

TEST_P(FilterUtils, Float)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testFloat();
}

TEST_P(FilterUtils, Double)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testDouble();
}

TEST_P(FilterUtils, Cstring)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testCstring();
}

TEST_P(FilterUtils, String)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testString();
}

TEST_P(FilterUtils, List)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testList();
}

TEST_P(FilterUtils, Dict)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testDict();
}

TEST_P(FilterUtils, Ros)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testRos();
}

TEST_P(FilterUtils, GeometryMsgs)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testGeometryMsgs();
}

TEST_P(FilterUtils, TF2)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testTF2();
}

TEST_P(FilterUtils, Eigen)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testEigen();
}

TEST_P(FilterUtils, Nested)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testNested();
}

TEST_P(FilterUtils, Misc)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testMisc();
}

TEST_P(FilterUtils, Options)
{
  auto filter = getFilter(std::get<0>(GetParam()));
  filter->testOptions();
}

INSTANTIATE_TEST_CASE_P(TestConfigs, FilterUtils, ::testing::Values(false, true));

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_filter_base");
  return RUN_ALL_TESTS();
}
