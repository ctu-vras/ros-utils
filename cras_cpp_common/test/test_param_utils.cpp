/**
 * \file
 * \brief Unit test for param_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <ros/ros.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>

#include "get_param_test.inc"

using namespace cras;

class TestLogHelper : public cras::LogHelper
{
protected:
  void printDebug(const std::string& text) const override
  {
    this->str = text;
    this->level = ros::console::Level::Debug;
  }
  void printInfo(const std::string& text) const override
  {
    this->str = text;
    this->level = ros::console::Level::Info;
  }
  void printWarn(const std::string& text) const override
  {
    this->str = text;
    this->level = ros::console::Level::Warn;
  }
  void printError(const std::string& text) const override
  {
    this->str = text;
    this->level = ros::console::Level::Error;
  }
  void printFatal(const std::string& text) const override
  {
    this->str = text;
    this->level = ros::console::Level::Fatal;
  }
public:
  void reset()
  {
    this->str = "";
    this->level = ros::console::Level::Count;
  }
  mutable std::string str {};
  mutable ros::console::Level level {ros::console::Level::Count};
};

struct ParamUtilsTest : public GetParamTest<ParamUtilsTest>
{
  std::shared_ptr<GetParamAdapter> params {nullptr};
  TestLogHelper logger {};
  
  template <typename T, typename U>
  void test(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = cras::getParamVerbose(*this->params, (param), (def), "",  {}, &this->logger);
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(*this->params, (param), cras::optional(def), "",  {}, &this->logger);
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), cras::getParam(*this->params, (param), (def), "",  {}, &this->logger));
    EXPECT_EQ((expected), cras::getParam(*this->params, (param), cras::optional(def), "",  {}, &this->logger));
  }
  
  template <typename T, typename U>
  void test_s(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = cras::getParamVerbose(*this->params, (param), (def), "", {}, &this->logger);
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(*this->params, (param), cras::optional(def), "",  {}, &this->logger);
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), cras::getParam(*this->params, (param), (def), "",  {}, &this->logger));
    EXPECT_EQ((expected), cras::getParam(*this->params, (param), cras::optional(def), "",  {}, &this->logger));
  }

  template <typename T, typename U>
  void test_near(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = cras::getParamVerbose(*this->params, (param), (def), "",  {}, &this->logger);
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(*this->params, (param), cras::optional(def), "",  {}, &this->logger);
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_GT(1e-6, distance((expected), cras::getParam(*this->params, (param), (def), "",  {}, &this->logger)));
    EXPECT_GT(1e-6, distance((expected),
      cras::getParam(*this->params, (param), cras::optional(def), "",  {}, &this->logger)));
  }

  template <typename T>
  void test_defaults(const std::string& param, const T& def)
  {
    this->test_s((param), (def), (def), true);
  }
  
  void testMisc()
  {
    // test that getParam result type is the same as the default value
    static_assert(std::is_same<
      bool,
      decltype(cras::getParam(*this->params, "bool_True", false, "", {}, &this->logger))
    >::value);
    static_assert(std::is_same<
      bool,
      decltype(cras::getParam(*this->params, "bool_True", cras::optional(false), "", {}, &this->logger))
    >::value);

    // test that C-string methods return a std::string result
    static_assert(std::is_same<
      std::string,
      decltype(cras::getParam(*this->params, "bool_True", "0", "", {}, &this->logger))
    >::value);
    static_assert(std::is_same<
      std::string,
      decltype(cras::getParam(*this->params, "bool_True", cras::optional(""), "", {}, &this->logger))
    >::value);

    // test non-verbose getParam
    EXPECT_EQ(true, cras::getParam(*this->params, "bool_True", false, "", {}, &this->logger));
    // test explicit conversion of GetParamResult to the contained type
    EXPECT_EQ(true, static_cast<bool>(cras::getParamVerbose(*this->params, "bool_True", false, "", {}, &this->logger)));
    // test implicit conversion of GetParamResult to the contained type
    { bool r = cras::getParamVerbose(*this->params, "bool_True", false, "", {}, &this->logger); EXPECT_EQ(true, r); }

    // test double NaN behavior
    { auto r = cras::getParamVerbose(*this->params, "double_nan", 0.0, "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(*this->params, "double_nan", cras::optional(0.0), "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "double_nan", 0.0, "", {}, &this->logger)));
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "double_nan", cras::optional(0.0), "", {}, &this->logger)));

    { auto r = cras::getParamVerbose(*this->params, "nonexistent", std::numeric_limits<double>::quiet_NaN(), "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(*this->params, "nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()), "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "nonexistent", std::numeric_limits<double>::quiet_NaN(), "", {}, &this->logger)));
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()), "", {}, &this->logger)));
  };

  public: void testOptions()
  {
    GetParamOptions<bool, bool> opts;
    GetParamResult<bool> r(false, {});
    const auto& ns = this->params->getNamespace();

    auto& log = this->logger;
    
    // test reading an existing param with default config
    opts = {}; log.reset();
    r = cras::getParamVerbose(*this->params, "bool_False", true, "", opts, &this->logger);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing reports about using default value as info messages
    opts = {}; log.reset();
    r = cras::getParamVerbose(*this->params, "nonexistent", true, "", opts, &this->logger);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test printing reports about using default value as warning messages
    opts = {}; opts.printDefaultAsWarn = true; log.reset();
    r = cras::getParamVerbose(*this->params, "nonexistent", true, "", opts, &this->logger);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Warn, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);

    // test not printing log messages
    opts = {}; opts.printMessages = false; log.reset();
    r = cras::getParamVerbose(*this->params, "nonexistent", true, "", opts, &this->logger);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ("", log.str);
    EXPECT_EQ(ros::console::Level::Count, log.level);

    // required parameter is missing
    opts = {}; log.reset();
    EXPECT_THROW(cras::getParamVerbose<bool>(*this->params, "nonexistent", std::nullopt, "", opts, &this->logger),
      GetParamException);
    try
    {
      cras::getParamVerbose<bool>(*this->params, "nonexistent", std::nullopt, "", opts, &this->logger);
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
    EXPECT_THROW(cras::getParamVerbose(*this->params, "str_a", false, "", opts, &this->logger), GetParamException);
    try
    {
      cras::getParamVerbose(*this->params, "str_a", false, "", opts, &this->logger);
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
    r = cras::getParamVerbose(*this->params, "str_a", true, "", opts, &this->logger);
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
    r = cras::getParamVerbose(*this->params, "bool_False", true, "bools", opts, &this->logger);
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
    r = cras::getParamVerbose(*this->params, "bool_False", true, "bools", opts, &this->logger);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, log.str);
    EXPECT_EQ(r.info.messageLevel, log.level);
    // reading true throws exception
    r = cras::getParamVerbose(*this->params, "bool_True", false, "bools", opts, &this->logger);
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
    auto r2 = cras::getParamVerbose(*this->params, "int_max", static_cast<short>(1), "", {}, &this->logger);
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
    r = cras::getParamVerbose(*this->params, "bool_True", false, "", opts, &this->logger);
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

class TestParamAdapter : public GetParamAdapter
{
public:
  explicit TestParamAdapter(const std::string& ns) : ns(ns) {}
  ~TestParamAdapter() override = default;

  bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const noexcept override
  {
    if (!this->hasParam(name))
      return false;
    value = this->params.at(name);
    return true;
  }

  bool hasParam(const std::string& name) const noexcept override
  {
    return this->params.find(name) != this->params.end();
  }
  
  std::string getNamespace() const noexcept override
  {
    return this->ns;
  }
  
  std::shared_ptr<GetParamAdapter> getNamespaced(const std::string& subNs) const noexcept(false) override
  {
    const auto newNs = this->getNamespace() + "/" + subNs;
    XmlRpc::XmlRpcValue v;
    if (!this->getParam(subNs, v))
      throw std::runtime_error("Parameter " + newNs + " not found");
    return std::make_shared<XmlRpcValueGetParamAdapter>(v, newNs);
  }

  std::unordered_map<std::string, XmlRpc::XmlRpcValue> params {};

private:
  std::string ns;
};

template <typename T>
void setArray(XmlRpc::XmlRpcValue& x, const std::vector<T>& vec)
{
  x.setSize(vec.size());
  for (size_t i = 0; i < vec.size(); ++i)
    x[i] = vec.at(i);
}

template <typename T>
void setMap(XmlRpc::XmlRpcValue& x, const std::unordered_map<std::string, T>& map)
{
  x.begin();  // morph into struct
  for (const auto& pair : map)
    x[pair.first] = pair.second;
}

std::shared_ptr<TestParamAdapter> getParams()
{
  static std::shared_ptr<TestParamAdapter> p;
  
  if (p)
    return p;

  p = std::make_shared<TestParamAdapter>("dict");
  
  p->params["bool_True"] = true;
  p->params["bool_False"] = false;
  p->params["bool_true"] = true;
  p->params["bool_false"] = false;
  p->params["int_0"] = 0;
  p->params["int_1"] = 1;
  p->params["int_2"] = 2;
  p->params["int_minus_1"] = -1;
  p->params["int_max"] = INT_MAX;
  p->params["int_min"] = INT_MIN;
  p->params["double_0"] = 0.0;
  p->params["double_1"] = 1.0;
  p->params["double_3_14"] = 3.14;
  p->params["double_minus_1"] = -1.0;
  p->params["double_minus_3_14"] = -3.14;
  p->params["double_lowest"] = -1.79769e+308;
  p->params["double_min"] = 2.22507e-308;
  p->params["double_max"] = 1.79769e+308;
  p->params["double_nan"] = std::numeric_limits<double>::quiet_NaN();
  p->params["double_inf"] = std::numeric_limits<double>::infinity();
  p->params["double_minus_inf"] = -std::numeric_limits<double>::infinity();
  p->params["str_empty"] = "";
  p->params["str_a"] = "a";
  p->params["str_asd"] = "asd";
  setArray<int>(p->params["list_empty"], {});
  setArray<bool>(p->params["list_bool"], { true, false, true }); 
  setArray<int>(p->params["list_int"], { 0, 1, -1 });
  setArray<int>(p->params["list_uint"], { 0, 1, 2 });
  setArray<double>(p->params["list_double"], { 0.0, 1.0, -1.0 });
  setArray<std::string>(p->params["list_str"], { "a", "b", "cde" });
  setMap<int>(p->params["dict_empty"], {});
  setMap<bool>(p->params["dict_bool"], {{"a", true}, {"b", false}, {"c", true}});
  setMap<int>(p->params["dict_int"], {{"a", 0}, {"b", 1}, {"c", -1}});
  setMap<int>(p->params["dict_uint"], {{"a", 0}, {"b", 1}, {"c", 2}});
  setMap<double>(p->params["dict_double"], {{"aaa", 0.0}, {"bbb", 1.0}, {"ccc", -1.0}});
  setMap<std::string>(p->params["dict_str"], {{"aaa", "a"}, {"bbb", "b"}, {"ccc", "c"}});
  p->params["dict_mixed"][""] = 0; p->params["dict_mixed"]["bbb"] = 1.0; p->params["dict_mixed"]["ccc"] = false;
  XmlRpc::XmlRpcValue recurs1; recurs1["b"] = 1; recurs1["c"] = 2;
  XmlRpc::XmlRpcValue recurs2; recurs2["e"] = 3; recurs2["f"] = 4; recurs2["g"] = 5;
  p->params["dict_str_recursive"]["a"] = recurs1; p->params["dict_str_recursive"]["d"] = recurs2;
  XmlRpc::XmlRpcValue crazy1; setArray<int>(crazy1["aa"], {0, 1});
  XmlRpc::XmlRpcValue crazy2; setArray<int>(crazy2["bb"], {-2, -3});
  p->params["dict_crazy"]["a"] = crazy1; p->params["dict_crazy"]["b"] = crazy2;
  setArray<double>(p->params["quat3"], {M_PI, 0.0, 0.0 });
  setArray<double>(p->params["quat4"], {1.0, 0.0, 0.0, 0.0 });
  setArray<double>(p->params["tf6"], {1.0, 2.0, 3.0, M_PI, 0.0, 0.0});
  setArray<double>(p->params["tf7"], {1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0});
  setArray<double>(p->params["tf16"],
    {1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 2.0, 0.0, 0.0, -1.0, 3.0, 0.0, 0.0, 0.0, 1.0});
  p->params["debug"]["pcl"]["inside"] = false;
  p->params["test"]["negative"] = -1;
  p->params["sensor"]["min_distance"] = 0.1;
  p->params["frames"]["fixed"] = "odom";
  p->params["transforms"]["buffer_length"] = 60.0;
  setArray<std::string>(p->params["ignored_links"]["bounding_sphere"], {"antenna", "base_link::big_collision_box"});
  setMap<double>(p->params["body_model"]["per_link"]["padding"], {{"laser::shadow", 0.015}, {"base_link", 0.05}});
  setMap<std::string>(p->params["body_model"]["per_link"]["all_wrong"], {{"laser", "0.015"}, {"base_link", "0.05"}});
  setMap<double>(p->params["body_model"]["per_link"]["scale"], {
    {"antenna::contains", 1.2},
    {"antenna::bounding_sphere", 1.2},
    {"antenna::bounding_box", 1.2},
    {"*::big_collision_box::contains", 2.0},
    {"*::big_collision_box::bounding_sphere", 2.0},
    {"*::big_collision_box::bounding_box", 2.0},
    {"*::big_collision_box::shadow", 3.0}
  });
  
  return p;
}

TEST(ParamUtils, XmlRpcValue)
{
  ParamUtilsTest t {.params = getParams()};
  t.testXmlRpcValue(true);
}

TEST(ParamUtils, Bool)
{
  ParamUtilsTest t {.params = getParams()};
  t.testBool();
}

TEST(ParamUtils, Int)
{
  ParamUtilsTest t {.params = getParams()};
  t.testInt();
}

TEST(ParamUtils, Float)
{
  ParamUtilsTest t {.params = getParams()};
  t.testFloat();
}

TEST(ParamUtils, Double)
{
  ParamUtilsTest t {.params = getParams()};
  t.testDouble();
}

TEST(ParamUtils, Cstring)
{
  ParamUtilsTest t {.params = getParams()};
  t.testCstring();
}

TEST(ParamUtils, String)
{
  ParamUtilsTest t {.params = getParams()};
  t.testString();
}

TEST(ParamUtils, List)
{
  ParamUtilsTest t {.params = getParams()};
  t.testList();
}

TEST(ParamUtils, Dict)
{
  ParamUtilsTest t {.params = getParams()};
  t.testDict();
}

TEST(ParamUtils, Ros)
{
  ParamUtilsTest t {.params = getParams()};
  ros::Time::init();
  t.testRos();
}

TEST(ParamUtils, GeometryMsgs)
{
  ParamUtilsTest t {.params = getParams()};
  t.testGeometryMsgs();
}

TEST(ParamUtils, TF2)
{
  ParamUtilsTest t {.params = getParams()};
  t.testTF2();
}

TEST(ParamUtils, Nested)
{
  ParamUtilsTest t {.params = getParams()};
  t.testNested();
}

TEST(ParamUtils, Misc)
{
  ParamUtilsTest t {.params = getParams()};
  t.testMisc();
}

TEST(ParamUtils, Options)
{
  ParamUtilsTest t {.params = getParams()};
  t.testOptions();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_param_utils");
  return RUN_ALL_TESTS();
}