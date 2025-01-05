/**
 * \file
 * \brief Unit test for param_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>

#include "get_param_test.inc"

using namespace cras;

class TestLogHelper : public cras::LogHelper
{
public:
  void initializeImpl() const override
  {
  }

  void initializeLogLocationImpl(
    ros::console::LogLocation* loc, const std::string& name, ros::console::Level level) const override
  {
    loc->level_ = level;
    loc->initialized_ = true;
    loc->logger_enabled_ = true;
  };

  void logString(void* logger, ros::console::Level level, const std::string& str, const char* file, uint32_t line,
    const char* function) const override
  {
    this->str = str;
    this->level = level;
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

struct ParamUtilsGetParamTest : public GetParamTest<ParamUtilsGetParamTest>
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

    { auto r = cras::getParamVerbose(
        *this->params, "nonexistent", std::numeric_limits<double>::quiet_NaN(), "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = cras::getParamVerbose(
        *this->params, "nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()), "", {}, &this->logger);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "nonexistent",
      std::numeric_limits<double>::quiet_NaN(), "", {}, &this->logger)));
    EXPECT_TRUE(std::isnan(cras::getParam(*this->params, "nonexistent",
      cras::optional(std::numeric_limits<double>::quiet_NaN()), "", {}, &this->logger)));
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
    auto r2 = cras::getParamVerbose(*this->params, "int_max", static_cast<short>(1), "", {}, &this->logger);  // NOLINT
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

struct TestParamHelper : public ParamHelper
{
  explicit TestParamHelper(const LogHelperPtr& _log) : ParamHelper(_log)
  {
  }

  std::shared_ptr<TestLogHelper> getLog() const
  {
    return std::dynamic_pointer_cast<TestLogHelper>(this->log);
  }
};

struct TestBoundParamHelper : public BoundParamHelper
{
  TestBoundParamHelper(const LogHelperPtr& _log, const GetParamAdapterPtr& _param) : BoundParamHelper(_log, _param)
  {
  }

  std::shared_ptr<TestLogHelper> getLog() const
  {
    return std::dynamic_pointer_cast<TestLogHelper>(this->log);
  }

  GetParamAdapterPtr getParams() const
  {
    return this->param;
  }
};

struct ParamHelperGetParamTest : public GetParamTest<ParamHelperGetParamTest>
{
  TestParamHelper* p {nullptr};
  GetParamAdapterPtr params {nullptr};

  template <typename T, typename U>
  void test(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose(*this->params, (param), (def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose(*this->params, (param), cras::optional(def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->p->getParam(*this->params, (param), (def)));
    EXPECT_EQ((expected), this->p->getParam(*this->params, (param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_s(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose(*this->params, (param), (def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose(*this->params, (param), cras::optional(def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->p->getParam(*this->params, (param), (def)));
    EXPECT_EQ((expected), this->p->getParam(*this->params, (param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_near(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose(*this->params, (param), (def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose(*this->params, (param), cras::optional(def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_GT(1e-6, distance((expected), this->p->getParam(*this->params, (param), (def))));
    EXPECT_GT(1e-6, distance((expected), this->p->getParam(*this->params, (param), cras::optional(def))));
  }

  template <typename T>
  void test_defaults(const std::string& param, const T& def)
  {
    this->test_s((param), (def), (def), true);
  }

  void testMisc()
  {
    // test that getParam result type is the same as the default value
    static_assert(std::is_same<bool, decltype(this->p->getParam(*this->params, "bool_True", false))>::value);
    static_assert(std::is_same<bool,
      decltype(this->p->getParam(*this->params, "bool_True", cras::optional(false)))>::value);

    // test that C-string methods return a std::string result
    static_assert(std::is_same<std::string, decltype(this->p->getParam(*this->params, "bool_True", "0"))>::value);
    static_assert(std::is_same<std::string,
      decltype(this->p->getParam(*this->params, "bool_True", cras::optional("")))>::value);

    // test non-verbose getParam
    EXPECT_EQ(true, this->p->getParam(*this->params, "bool_True", false));
    // test explicit conversion of GetParamResult to the contained type
    EXPECT_EQ(true, static_cast<bool>(this->p->getParamVerbose(*this->params, "bool_True", false)));
    // test implicit conversion of GetParamResult to the contained type
    { bool r = this->p->getParamVerbose(*this->params, "bool_True", false); EXPECT_EQ(true, r); }

    // test double NaN behavior
    { auto r = this->p->getParamVerbose(*this->params, "double_nan", 0.0);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose(*this->params, "double_nan", cras::optional(0.0));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->p->getParam(*this->params, "double_nan", 0.0)));
    EXPECT_TRUE(std::isnan(this->p->getParam(*this->params, "double_nan", cras::optional(0.0))));

    { auto r = this->p->getParamVerbose(*this->params, "nonexistent", std::numeric_limits<double>::quiet_NaN());
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose(*this->params, "nonexistent",
        cras::optional(std::numeric_limits<double>::quiet_NaN()));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->p->getParam(*this->params, "nonexistent", std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(this->p->getParam(*this->params, "nonexistent",
      cras::optional(std::numeric_limits<double>::quiet_NaN()))));
  };

  public: void testOptions()
  {
    GetParamOptions<bool, bool> opts;
    GetParamResult<bool> r(false, {});
    const auto& ns = this->params->getNamespace();

    // test reading an existing param with default config
    opts = {}; this->p->getLog().reset();
    r = this->p->getParamVerbose(*this->params, "bool_False", true, "", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing reports about using default value as info messages
    opts = {}; this->p->getLog()->reset();
    r = this->p->getParamVerbose(*this->params, "nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing reports about using default value as warning messages
    opts = {}; opts.printDefaultAsWarn = true; this->p->getLog()->reset();
    r = this->p->getParamVerbose(*this->params, "nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Warn, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test not printing log messages
    opts = {}; opts.printMessages = false; this->p->getLog()->reset();
    r = this->p->getParamVerbose(*this->params, "nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ("", this->p->getLog()->str);
    EXPECT_EQ(ros::console::Level::Count, this->p->getLog()->level);

    // required parameter is missing
    opts = {}; this->p->getLog()->reset();
    EXPECT_THROW(this->p->getParamVerbose<bool>(*this->params, "nonexistent", std::nullopt, "", opts),
      GetParamException);
    try
    {
      this->p->getParamVerbose<bool>(*this->params, "nonexistent", std::nullopt);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_TRUE(e.info.requiredMissing);
      EXPECT_FALSE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent.", e.info.message);
      EXPECT_EQ(e.info.message, this->p->getLog()->str);
      EXPECT_EQ(e.info.messageLevel, this->p->getLog()->level);
    }

    // test throwing exception on convert failure
    opts = {}; opts.throwIfConvertFails = true; this->p->getLog()->reset();
    EXPECT_THROW(this->p->getParamVerbose(*this->params, "str_a", false, "", opts), GetParamException);
    try
    {
      this->p->getParamVerbose(*this->params, "str_a", false, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_FALSE(e.info.requiredMissing);
      EXPECT_TRUE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                     "Expected type bool, got type string with value <value>a</value>.", e.info.message);
      EXPECT_EQ(e.info.message, this->p->getLog()->str);
      EXPECT_EQ(e.info.messageLevel, this->p->getLog()->level);
    }

    // test failed conversion without throwing exception
    opts = {}; opts.throwIfConvertFails = false; this->p->getLog()->reset();
    r = this->p->getParamVerbose(*this->params, "str_a", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                   "Expected type bool, got type string with value <value>a</value>. "
                   "Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing custom units
    opts = {}; this->p->getLog()->reset();
    r = this->p->getParamVerbose(*this->params, "bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test custom toString and toResult methods
    opts = {}; this->p->getLog()->reset();
    opts.paramToStr = [](const bool& v) -> std::string {return v ? "tRuE" : "fAlSe"; };
    opts.resultToStr = [](const bool& v) -> std::string {return v ? "TRUE" : "FALSE"; };
    opts.toResult = [](const bool& v) -> bool { if (v) throw std::runtime_error("fail"); return v; };
    // reading false succeeds
    r = this->p->getParamVerbose(*this->params, "bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);
    // reading true throws exception
    r = this->p->getParamVerbose(*this->params, "bool_True", false, "bools", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot convert value 'tRuE' of parameter bool_True to requested type bool (error: fail). "
                   "Assigning default: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test reading a parameter whose conversion from XmlRpc type fails
    this->p->getLog()->reset();
    auto r2 = this->p->getParamVerbose(*this->params, "int_max", static_cast<short>(1));  // NOLINT
    EXPECT_TRUE(r2.info.defaultUsed);
    EXPECT_FALSE(r2.info.requiredMissing);
    EXPECT_TRUE(r2.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r2.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter int_max found with correct XmlRpc type int and value "
                   "<value><i4>2147483647</i4></value>, but its conversion to type short int has failed "
                   "due to the following errors: [\"Value 2147483647 is out of bounds <-32768, 32767>.\"]. "
                   "Assigning default: 1.", r2.info.message);
    EXPECT_EQ(r2.info.message, this->p->getLog()->str);
    EXPECT_EQ(r2.info.messageLevel, this->p->getLog()->level);

    // test reading a parameter whose conversion from XmlRpc type fails using a custom toParam function
    opts = {}; this->p->getLog()->reset();
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
    r = this->p->getParamVerbose(*this->params, "bool_True", false, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter bool_True found with correct XmlRpc type bool and value "
                   "<value><boolean>1</boolean></value>, but its conversion to type bool has failed "
                   "due to the following errors: [\"cannot convert\"]. "
                   "Assigning default: False.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);
  }
};

struct BoundParamHelperGetParamTest : public GetParamTest<BoundParamHelperGetParamTest>
{
  TestBoundParamHelper* p {nullptr};

  template <typename T, typename U>
  void test(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose((param), (def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose((param), cras::optional(def));
      EXPECT_EQ((expected), r); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->p->getParam((param), (def)));
    EXPECT_EQ((expected), this->p->getParam((param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_s(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose((param), (def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose((param), cras::optional(def));
      EXPECT_EQ((expected), r.value); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_EQ((expected), this->p->getParam((param), (def)));
    EXPECT_EQ((expected), this->p->getParam((param), cras::optional(def)));
  }

  template <typename T, typename U>
  void test_near(const std::string& param, const T& def, const U& expected, const bool defUsed)
  {
    { auto r = this->p->getParamVerbose((param), (def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose((param), cras::optional(def));
      EXPECT_GT(1e-6, distance((expected), r.value)); EXPECT_EQ((defUsed), r.info.defaultUsed); }
    EXPECT_GT(1e-6, distance((expected), this->p->getParam((param), (def))));
    EXPECT_GT(1e-6, distance((expected), this->p->getParam((param), cras::optional(def))));
  }

  template <typename T>
  void test_defaults(const std::string& param, const T& def)
  {
    this->test_s((param), (def), (def), true);
  }

  void testMisc()  // NOLINT
  {
    // test that getParam result type is the same as the default value
    static_assert(std::is_same<bool, decltype(this->p->getParam("bool_True", false))>::value);
    static_assert(std::is_same<bool,
      decltype(this->p->getParam("bool_True", cras::optional(false)))>::value);

    // test that C-string methods return a std::string result
    static_assert(std::is_same<std::string, decltype(this->p->getParam("bool_True", "0"))>::value);
    static_assert(std::is_same<std::string,
      decltype(this->p->getParam("bool_True", cras::optional("")))>::value);

    // test non-verbose getParam
    EXPECT_EQ(true, this->p->getParam("bool_True", false));
    // test explicit conversion of GetParamResult to the contained type
    EXPECT_EQ(true, static_cast<bool>(this->p->getParamVerbose("bool_True", false)));
    // test implicit conversion of GetParamResult to the contained type
    { bool r = this->p->getParamVerbose("bool_True", false); EXPECT_EQ(true, r); }

    // test double NaN behavior
    { auto r = this->p->getParamVerbose("double_nan", 0.0);
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose("double_nan", cras::optional(0.0));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(false, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->p->getParam("double_nan", 0.0)));
    EXPECT_TRUE(std::isnan(this->p->getParam("double_nan", cras::optional(0.0))));

    { auto r = this->p->getParamVerbose("nonexistent", std::numeric_limits<double>::quiet_NaN());
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    { auto r = this->p->getParamVerbose("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()));
      EXPECT_TRUE(std::isnan(r)); EXPECT_EQ(true, r.info.defaultUsed); }
    EXPECT_TRUE(std::isnan(this->p->getParam("nonexistent", std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(this->p->getParam("nonexistent", cras::optional(std::numeric_limits<double>::quiet_NaN()))));
  };

  public: void testOptions()  // NOLINT
  {
    GetParamOptions<bool, bool> opts;
    GetParamResult<bool> r(false, {});
    const auto& ns = this->p->getParams()->getNamespace();

    // test reading an existing param with default config
    opts = {}; this->p->getLog().reset();
    r = this->p->getParamVerbose("bool_False", true, "", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing reports about using default value as info messages
    opts = {}; this->p->getLog()->reset();
    r = this->p->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing reports about using default value as warning messages
    opts = {}; opts.printDefaultAsWarn = true; this->p->getLog()->reset();
    r = this->p->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Warn, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test not printing log messages
    opts = {}; opts.printMessages = false; this->p->getLog()->reset();
    r = this->p->getParamVerbose("nonexistent", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent. Assigning default: True.", r.info.message);
    EXPECT_EQ("", this->p->getLog()->str);
    EXPECT_EQ(ros::console::Level::Count, this->p->getLog()->level);

    // required parameter is missing
    opts = {}; this->p->getLog()->reset();
    EXPECT_THROW(this->p->getParamVerbose<bool>("nonexistent", std::nullopt, "", opts),
      GetParamException);
    try
    {
      this->p->getParamVerbose<bool>("nonexistent", std::nullopt);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_TRUE(e.info.requiredMissing);
      EXPECT_FALSE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Cannot find value for parameter: nonexistent.", e.info.message);
      EXPECT_EQ(e.info.message, this->p->getLog()->str);
      EXPECT_EQ(e.info.messageLevel, this->p->getLog()->level);
    }

    // test throwing exception on convert failure
    opts = {}; opts.throwIfConvertFails = true; this->p->getLog()->reset();
    EXPECT_THROW(this->p->getParamVerbose("str_a", false, "", opts), GetParamException);
    try
    {
      this->p->getParamVerbose("str_a", false, "", opts);
    }
    catch (const GetParamException& e)
    {
      EXPECT_FALSE(e.info.defaultUsed);
      EXPECT_FALSE(e.info.requiredMissing);
      EXPECT_TRUE(e.info.convertFailed);
      EXPECT_EQ(ros::console::Level::Error, e.info.messageLevel);
      EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                     "Expected type bool, got type string with value <value>a</value>.", e.info.message);
      EXPECT_EQ(e.info.message, this->p->getLog()->str);
      EXPECT_EQ(e.info.messageLevel, this->p->getLog()->level);
    }

    // test failed conversion without throwing exception
    opts = {}; opts.throwIfConvertFails = false; this->p->getLog()->reset();
    r = this->p->getParamVerbose("str_a", true, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter str_a found, but it has wrong XmlRpc type. "
                   "Expected type bool, got type string with value <value>a</value>. "
                   "Assigning default: True.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test printing custom units
    opts = {}; this->p->getLog()->reset();
    r = this->p->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: False bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test custom toString and toResult methods
    opts = {}; this->p->getLog()->reset();
    opts.paramToStr = [](const bool& v) -> std::string {return v ? "tRuE" : "fAlSe"; };
    opts.resultToStr = [](const bool& v) -> std::string {return v ? "TRUE" : "FALSE"; };
    opts.toResult = [](const bool& v) -> bool { if (v) throw std::runtime_error("fail"); return v; };
    // reading false succeeds
    r = this->p->getParamVerbose("bool_False", true, "bools", opts);
    EXPECT_FALSE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_FALSE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Info, r.info.messageLevel);
    EXPECT_EQ(ns + ": Found parameter: bool_False, value: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);
    // reading true throws exception
    r = this->p->getParamVerbose("bool_True", false, "bools", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Cannot convert value 'tRuE' of parameter bool_True to requested type bool (error: fail). "
                   "Assigning default: FALSE bools.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);

    // test reading a parameter whose conversion from XmlRpc type fails
    this->p->getLog()->reset();
    auto r2 = this->p->getParamVerbose("int_max", static_cast<short>(1));  // NOLINT
    EXPECT_TRUE(r2.info.defaultUsed);
    EXPECT_FALSE(r2.info.requiredMissing);
    EXPECT_TRUE(r2.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r2.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter int_max found with correct XmlRpc type int and value "
                   "<value><i4>2147483647</i4></value>, but its conversion to type short int has failed "
                   "due to the following errors: [\"Value 2147483647 is out of bounds <-32768, 32767>.\"]. "
                   "Assigning default: 1.", r2.info.message);
    EXPECT_EQ(r2.info.message, this->p->getLog()->str);
    EXPECT_EQ(r2.info.messageLevel, this->p->getLog()->level);

    // test reading a parameter whose conversion from XmlRpc type fails using a custom toParam function
    opts = {}; this->p->getLog()->reset();
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
    r = this->p->getParamVerbose("bool_True", false, "", opts);
    EXPECT_TRUE(r.info.defaultUsed);
    EXPECT_FALSE(r.info.requiredMissing);
    EXPECT_TRUE(r.info.convertFailed);
    EXPECT_EQ(ros::console::Level::Error, r.info.messageLevel);
    EXPECT_EQ(ns + ": Parameter bool_True found with correct XmlRpc type bool and value "
                   "<value><boolean>1</boolean></value>, but its conversion to type bool has failed "
                   "due to the following errors: [\"cannot convert\"]. "
                   "Assigning default: False.", r.info.message);
    EXPECT_EQ(r.info.message, this->p->getLog()->str);
    EXPECT_EQ(r.info.messageLevel, this->p->getLog()->level);
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
  p->params["body_model"]["robot_description_param"] = "test_robot_description";
  setMap<double>(p->params["body_model"]["inflation"], {{"scale", 1.1}, {"padding", 0.01}});
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

TEST(GetParam, XmlRpcValue)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testXmlRpcValue(true);
}

TEST(GetParam, Bool)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testBool();
}

TEST(GetParam, Int)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testInt();
}

TEST(GetParam, Float)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testFloat();
}

TEST(GetParam, Double)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testDouble();
}

TEST(GetParam, Cstring)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testCstring();
}

TEST(GetParam, String)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testString();
}

TEST(GetParam, List)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testList();
}

TEST(GetParam, Dict)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testDict();
}

TEST(GetParam, Ros)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  ros::Time::init();
  t.testRos();
}

TEST(GetParam, GeometryMsgs)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testGeometryMsgs();
}

TEST(GetParam, TF2)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testTF2();
}

TEST(GetParam, Eigen)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testEigen();
}

TEST(GetParam, Nested)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testNested();
}

TEST(GetParam, Misc)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testMisc();
}

TEST(GetParam, Options)  // NOLINT
{
  ParamUtilsGetParamTest t; t.params = getParams();
  t.testOptions();
}

TEST(ParamHelper, Bool)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testBool();
}

TEST(ParamHelper, Int)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testInt();
}

TEST(ParamHelper, Float)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testFloat();
}

TEST(ParamHelper, Double)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testDouble();
}

TEST(ParamHelper, Cstring)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testCstring();
}

TEST(ParamHelper, String)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testString();
}

TEST(ParamHelper, List)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testList();
}

TEST(ParamHelper, Dict)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testDict();
}

TEST(ParamHelper, Ros)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  ros::Time::init();
  t.testRos();
}

TEST(ParamHelper, GeometryMsgs)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testGeometryMsgs();
}

TEST(ParamHelper, TF2)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testTF2();
}

TEST(ParamHelper, Eigen)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testEigen();
}

TEST(ParamHelper, Nested)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testNested();
}

TEST(ParamHelper, Misc)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testMisc();
}

TEST(ParamHelper, Options)  // NOLINT
{
  TestParamHelper p(std::make_shared<TestLogHelper>());
  EXPECT_NE(nullptr, p.getLogger());
  ParamHelperGetParamTest t; t.p = &p; t.params = getParams();
  t.testOptions();
}

TEST(BoundParamHelper, Bool)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testBool();
}

TEST(BoundParamHelper, Int)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testInt();
}

TEST(BoundParamHelper, Float)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testFloat();
}

TEST(BoundParamHelper, Double)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testDouble();
}

TEST(BoundParamHelper, Cstring)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testCstring();
}

TEST(BoundParamHelper, String)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testString();
}

TEST(BoundParamHelper, List)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testList();
}

TEST(BoundParamHelper, Dict)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testDict();
}

TEST(BoundParamHelper, Ros)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  ros::Time::init();
  t.testRos();
}

TEST(BoundParamHelper, GeometryMsgs)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testGeometryMsgs();
}

TEST(BoundParamHelper, TF2)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testTF2();
}

TEST(BoundParamHelper, Eigen)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testEigen();
}

TEST(BoundParamHelper, Nested)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testNested();
}

TEST(BoundParamHelper, Misc)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testMisc();
}

TEST(BoundParamHelper, Options)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_NE(nullptr, p.getLogger());
  BoundParamHelperGetParamTest t; t.p = &p;
  t.testOptions();
}

TEST(BoundParamHelper, HasParam)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_TRUE(p.hasParam("bool_True"));
  EXPECT_FALSE(p.hasParam("nonexistent"));
  EXPECT_TRUE(p.hasParam("body_model/inflation/scale", true));  // search nested
  EXPECT_FALSE(p.hasParam("body_model/inflation/scale", false));  // do not search nested
  EXPECT_TRUE(p.hasParam("test/negative", true));  // search nested
  EXPECT_FALSE(p.hasParam("test/negative", false));  // do not search nested
  EXPECT_FALSE(p.hasParam("non/existent", true));  // search nested
  EXPECT_FALSE(p.hasParam("non/existent", false));  // do not search nested
}

TEST(BoundParamHelper, ParamsInNamespace)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  auto p2 = p.paramsInNamespace("body_model");
  EXPECT_FALSE(p2->hasParam("bool_True"));
  EXPECT_FALSE(p2->hasParam("nonexistent"));
  EXPECT_FALSE(p2->hasParam("body_model/inflation/scale", true));  // search nested
  EXPECT_FALSE(p2->hasParam("body_model/inflation/scale", false));  // do not search nested
  EXPECT_FALSE(p2->hasParam("test/negative", true));  // search nested
  EXPECT_FALSE(p2->hasParam("test/negative", false));  // do not search nested
  EXPECT_FALSE(p2->hasParam("non/existent", true));  // search nested
  EXPECT_FALSE(p2->hasParam("non/existent", false));  // do not search nested

  EXPECT_TRUE(p2->hasParam("robot_description_param"));
  EXPECT_TRUE(p2->hasParam("inflation/scale", true));  // search nested
  EXPECT_FALSE(p2->hasParam("inflation/scale", false));  // do not search nested
  EXPECT_FALSE(p2->hasParam("negative", true));  // search nested
  EXPECT_FALSE(p2->hasParam("negative", false));  // do not search nested
  EXPECT_FALSE(p2->hasParam("existent", true));  // search nested
  EXPECT_FALSE(p2->hasParam("existent", false));  // do not search nested

  EXPECT_EQ("test_robot_description", p2->getParam("robot_description_param", "fail"));
  EXPECT_EQ(1.1, p2->getParam("inflation/scale", 1.0));
}

TEST(BoundParamHelper, GetNamespace)  // NOLINT
{
  TestBoundParamHelper p(std::make_shared<TestLogHelper>(), getParams());
  EXPECT_EQ("dict", p.getNamespace());
}

TEST(GetParamAdapters, NodeHandle)  // NOLINT
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  cras::NodeHandleGetParamAdapter aPub(nh);
  cras::NodeHandleGetParamAdapter aPrivate(pnh);

  EXPECT_TRUE(aPub.hasParam("test_chain_config"));
  EXPECT_TRUE(aPub.hasParam("test_dict_config"));
  EXPECT_FALSE(aPub.hasParam("nonexistent"));
  EXPECT_FALSE(aPub.hasParam("name"));

  EXPECT_EQ("/", aPub.getNamespace());
  EXPECT_EQ("/test_param_utils", aPrivate.getNamespace());

  XmlRpc::XmlRpcValue x;
  EXPECT_TRUE(aPub.getParam("test_chain_config", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeArray, x.getType());
  EXPECT_TRUE(aPub.getParam("test_dict_config", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, x.getType());
  // if lookup fails, x is not changed
  EXPECT_FALSE(aPub.getParam("nonexistent", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, x.getType());
  EXPECT_FALSE(aPub.getParam("1", x));  // no exception is thrown when an invalid name is passed

  // Getting / works from any namespace
  EXPECT_TRUE(aPub.getParam("/", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, x.getType()); auto xPub = x;
  EXPECT_TRUE(aPrivate.getParam("/", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, x.getType()); auto xPrivate = x;

  EXPECT_LE(2u, xPub.size());
  EXPECT_LE(2u, xPrivate.size());
  EXPECT_TRUE(xPub.hasMember("test_dict_config"));
  EXPECT_TRUE(xPrivate.hasMember("test_dict_config"));
  EXPECT_TRUE(xPub.hasMember("test_chain_config"));
  EXPECT_TRUE(xPrivate.hasMember("test_chain_config"));

  ros::NodeHandle dnh("test_dict_config");
  cras::NodeHandleGetParamAdapter aDict(dnh);
  auto aDict2 = aPub.getNamespaced("test_dict_config");

  EXPECT_TRUE(aDict.hasParam("name")); EXPECT_EQ(aDict.hasParam("name"), aDict2->hasParam("name"));
  EXPECT_TRUE(aDict.hasParam("type")); EXPECT_EQ(aDict.hasParam("type"), aDict2->hasParam("type"));
  EXPECT_TRUE(aDict.hasParam("params")); EXPECT_EQ(aDict.hasParam("params"), aDict2->hasParam("params"));
  EXPECT_FALSE(aDict.hasParam("non")); EXPECT_EQ(aDict.hasParam("non"), aDict2->hasParam("non"));

  EXPECT_EQ("/test_dict_config", aDict.getNamespace());
  EXPECT_EQ("/test_dict_config", aDict2->getNamespace());
}

TEST(GetParamAdapters, XmlRpcValue)  // NOLINT
{
  XmlRpc::XmlRpcValue params;
  getParams()->getParam("body_model", params);
  cras::XmlRpcValueGetParamAdapter a(params, "ns");

  EXPECT_TRUE(a.hasParam("inflation"));
  EXPECT_TRUE(a.hasParam("per_link"));
  EXPECT_FALSE(a.hasParam("nonexistent"));
  EXPECT_FALSE(a.hasParam("scale"));

  EXPECT_EQ("ns", a.getNamespace());

  XmlRpc::XmlRpcValue x;
  EXPECT_TRUE(a.getParam("inflation", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeStruct, x.getType());
  EXPECT_TRUE(a.getParam("robot_description_param", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeString, x.getType());
  // if lookup fails, x is not changed
  EXPECT_FALSE(a.getParam("nonexistent", x)); EXPECT_EQ(XmlRpc::XmlRpcValue::TypeString, x.getType());
  EXPECT_FALSE(a.getParam("1", x));  // no exception is thrown when an invalid name is passed

  auto a2 = a.getNamespaced("inflation");
  EXPECT_TRUE(a2->hasParam("scale")); a2->getParam("scale", x); EXPECT_EQ(1.1, static_cast<double>(x));
  EXPECT_TRUE(a2->hasParam("padding")); a2->getParam("padding", x); EXPECT_EQ(0.01, static_cast<double>(x));
  EXPECT_FALSE(a2->hasParam("non"));

  EXPECT_EQ("ns/inflation", a2->getNamespace());

  XmlRpc::XmlRpcValue badTypeParams = 1;
  EXPECT_THROW(cras::XmlRpcValueGetParamAdapter(badTypeParams, "ns"), std::runtime_error);
}

TEST(GetParamOptions, Constructor)  // NOLINT
{
  {  // test that the default parameters of the constructor work as intended
    GetParamOptions<double> options;
    EXPECT_TRUE(options.printMessages);
    EXPECT_FALSE(options.printDefaultAsWarn);
    EXPECT_FALSE(options.throwIfConvertFails);
    EXPECT_TRUE(options.allowNestedParams);
    EXPECT_TRUE(options.origNamespace.empty());
    EXPECT_TRUE(options.origParamName.empty());
    EXPECT_EQ(cras::to_string(0.0), options.paramToStr(0));
    EXPECT_EQ(cras::to_string(0.0), options.resultToStr(0));
    EXPECT_EQ(3.14, options.toResult(3.14));
    XmlRpc::XmlRpcValue x = 3.14;
    double d;
    EXPECT_TRUE(options.toParam(x, d, true, nullptr));
    EXPECT_EQ(3.14, d);
  }

#ifdef HAS_DESIGNATED_INITIALIZERS
  {  // test braced-initializer construction
    GetParamOptions<double> options{.throwIfConvertFails = true, .origParamName = "asd"};
    EXPECT_TRUE(options.printMessages);
    EXPECT_FALSE(options.printDefaultAsWarn);
    EXPECT_TRUE(options.throwIfConvertFails);
    EXPECT_TRUE(options.allowNestedParams);
    EXPECT_TRUE(options.origNamespace.empty());
    EXPECT_EQ("asd", options.origParamName);
    EXPECT_EQ(cras::to_string(0.0), options.paramToStr(0));
    EXPECT_EQ(cras::to_string(0.0), options.resultToStr(0));
    EXPECT_EQ(3.14, options.toResult(3.14));
    XmlRpc::XmlRpcValue x = 3.14;
    double d;
    EXPECT_TRUE(options.toParam(x, d, true, nullptr));
    EXPECT_EQ(3.14, d);
  }
#endif

  // the following just tests it is possible to compile the default functions
  {GetParamOptions<double> options; options.toResult({});}
  {GetParamOptions<bool> options; options.toResult({});}
  {GetParamOptions<XmlRpc::XmlRpcValue> options; options.toResult({});}
  {GetParamOptions<double, bool> options; options.toResult({});}
  {GetParamOptions<bool, double> options; options.toResult({});}
  {GetParamOptions<ros::Rate> options; options.toResult(10);}
  {GetParamOptions<ros::Rate, double> options; options.toResult(10);}
  {GetParamOptions<ros::Duration> options; options.toResult({});}
  {GetParamOptions<ros::Duration, int> options; options.toResult({});}
  {GetParamOptions<Eigen::Vector3d, int> options; }  // no way to test toResult default implementation, it is invalid
  {GetParamOptions<std::map<std::string, std::map<std::string, int>>> options; options.toResult({});}
  {GetParamOptions<std::map<std::string, std::map<std::string, XmlRpc::XmlRpcValue>>> options; options.toResult({});}
  {GetParamOptions<std::map<std::string, std::map<std::string, std::vector<std::map<std::string, bool>>>>> options;
    options.toResult({});}

  {
    const auto options = GetParamConvertingOptions<int, std::string>(
      [](const int i) {return cras::to_string(i);}, [](const std::string& s) {return cras::parseInt32(s);});
    EXPECT_EQ("1", options.resultToStr(1));
    EXPECT_EQ(1, options.toResult("1"));
    EXPECT_EQ("1", options.paramToStr("1"));
  }
}

template <typename T>
std::string testToStr(const T&)
{
  return "test";
}

template <typename T>
std::string testToStr2(const T&)
{
  return "test2";
}

TEST(GetParamOptions, Copy)  // NOLINT
{
  GetParamOptions<double> oDouble;
  oDouble.printMessages = false; oDouble.printDefaultAsWarn = true; oDouble.throwIfConvertFails = true;
  oDouble.allowNestedParams = false; oDouble.paramToStr = &testToStr<double>; oDouble.resultToStr = &testToStr<double>;
  oDouble.toResult = [](const double&) {return 1.0;};
  oDouble.toParam = [](const XmlRpc::XmlRpcValue&, double& v, bool = true, std::list<std::string>* = nullptr) -> bool
    {
      v = 2.0;
      return true;
    };

  // test copy-assignment from a different type
  GetParamOptions<bool> oBool;
  oBool = oDouble;
  EXPECT_EQ(oDouble.printMessages, oBool.printMessages);
  EXPECT_EQ(oDouble.printDefaultAsWarn, oBool.printDefaultAsWarn);
  EXPECT_EQ(oDouble.throwIfConvertFails, oBool.throwIfConvertFails);
  EXPECT_EQ(oDouble.allowNestedParams, oBool.allowNestedParams);
  // check that the functions were not somehow copied
  bool b {true};
  XmlRpc::XmlRpcValue x = false;
  EXPECT_EQ(false, oBool.toResult(false));
  EXPECT_TRUE(oBool.toParam(x, b, true, nullptr)); EXPECT_FALSE(b);
  EXPECT_EQ(cras::to_string(true), oBool.resultToStr(true));
  EXPECT_EQ(cras::to_string(true), oBool.paramToStr(true));

  // test copy-assignment to the same type
  GetParamOptions<double> oDouble2;
  oDouble2 = oDouble;
  EXPECT_EQ(oDouble.printMessages, oDouble2.printMessages);
  EXPECT_EQ(oDouble.printDefaultAsWarn, oDouble2.printDefaultAsWarn);
  EXPECT_EQ(oDouble.throwIfConvertFails, oDouble2.throwIfConvertFails);
  EXPECT_EQ(oDouble.allowNestedParams, oDouble2.allowNestedParams);
  // check that the functions were copied
  double d {0};
  XmlRpc::XmlRpcValue x2 = 3.0;
  EXPECT_EQ(1.0, oDouble2.toResult(0.0));
  EXPECT_TRUE(oDouble2.toParam(x2, d, true, nullptr)); EXPECT_EQ(2.0, d);
  EXPECT_EQ("test", oDouble2.resultToStr(true));
  EXPECT_EQ("test", oDouble2.paramToStr(true));
}

class CustomDataType
{
public:
  std::string param;
  explicit CustomDataType(const std::string& s) : param(s) {}
};

namespace cras
{

std::string to_string(const CustomDataType& c)
{
  return c.param;
}

DEFINE_CONVERTING_GET_PARAM(CustomDataType, std::string, "units", [](const std::string& s) {return CustomDataType(s);})
}

TEST(GetParamOptions, CustomType)  // NOLINT
{
  // Test support for completely custom datatypes in GetParamOptions.
  GetParamOptions<CustomDataType> o;
  EXPECT_EQ("test", o.toResult("test").param);

  XmlRpc::XmlRpcValue v("tes");
  std::string s;
  EXPECT_TRUE(o.toParam(v, s, false, nullptr));
  EXPECT_EQ("tes", s);

  EXPECT_EQ("te", o.resultToStr(CustomDataType("te")));
  EXPECT_EQ("t", o.paramToStr("t"));

  XmlRpc::XmlRpcValue xml;
  xml["p"] = "p";
  CustomDataType c("");
  const auto res = cras::getParam(cras::XmlRpcValueGetParamAdapter(xml, ""), "p", c);
  EXPECT_EQ("p", res.param);
}

TEST(GetParamOptions, AsType)  // NOLINT
{
  GetParamOptions<double> oDouble;
  oDouble.printMessages = false; oDouble.printDefaultAsWarn = true; oDouble.throwIfConvertFails = true;
  oDouble.allowNestedParams = false; oDouble.paramToStr = &testToStr<double>; oDouble.resultToStr = &testToStr<double>;
  oDouble.toResult = [](const double&) {return 1.0;};
  oDouble.toParam = [](const XmlRpc::XmlRpcValue&, double& v, bool = true, std::list<std::string>* = nullptr) -> bool
  {
    v = 2.0;
    return true;
  };

  auto oBool = oDouble.asType<bool>(
    [](const bool&){return 4.0;},
    &testToStr2<bool>,
    [](const ::XmlRpc::XmlRpcValue& x, bool& v, bool skipNonConvertible, ::std::list<::std::string>* errors)
    {
      v = false;
      return true;
    });

  EXPECT_EQ(typeid(decltype(oBool)), typeid(GetParamOptions<double, bool>));
  EXPECT_EQ(oDouble.printMessages, oBool.printMessages);
  EXPECT_EQ(oDouble.printDefaultAsWarn, oBool.printDefaultAsWarn);
  EXPECT_EQ(oDouble.throwIfConvertFails, oBool.throwIfConvertFails);
  EXPECT_EQ(oDouble.allowNestedParams, oBool.allowNestedParams);
  bool b {true};
  XmlRpc::XmlRpcValue x = false;
  EXPECT_EQ(4.0, oBool.toResult(true));
  EXPECT_TRUE(oBool.toParam(x, b, true, nullptr)); EXPECT_FALSE(b);
  EXPECT_EQ(oDouble.resultToStr(3.0), oBool.resultToStr(3.0));
  EXPECT_EQ("test2", oBool.paramToStr(true));
}

TEST(GetParamResult, Test)  // NOLINT
{
  GetParamResultInfo info;
  EXPECT_FALSE(info.defaultUsed);
  EXPECT_FALSE(info.requiredMissing);
  EXPECT_FALSE(info.convertFailed);
  EXPECT_TRUE(info.message.empty());
  EXPECT_EQ(ros::console::Level::Count, info.messageLevel);

  info.defaultUsed = info.requiredMissing = info.convertFailed = true;
  info.message = "test";
  info.messageLevel = ros::console::Level::Warn;

  GetParamResult<bool> r(true, info);
  EXPECT_TRUE(r.info.defaultUsed);
  EXPECT_TRUE(r.info.requiredMissing);
  EXPECT_TRUE(r.info.convertFailed);
  EXPECT_EQ("test", r.info.message);
  EXPECT_EQ(ros::console::Level::Warn, info.messageLevel);
  EXPECT_EQ(true, r.value);
  EXPECT_EQ(true, r);

  // test the implicit conversion of GetParamResult to the result value
  GetParamResult<ros::Time> r2({1, 0}, info);
  EXPECT_EQ(ros::Time(1, 0), r2.value);
  EXPECT_EQ(ros::Time(1, 0), r2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_param_utils");
  return RUN_ALL_TESTS();
}
