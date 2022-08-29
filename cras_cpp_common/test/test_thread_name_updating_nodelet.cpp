/**
 * \file
 * \brief Unit test for thread_name_updating_nodelet.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <string>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/thread_utils.h>
#include <cras_cpp_common/nodelet_utils/thread_name_updating_nodelet.hpp>

using namespace cras;

/**
 * \brief A mock of the nodelet::Nodelet class that just provides the `getName()` function.
 */
struct MockNodelet
{
  virtual ~MockNodelet() = default;

  /**
   * \brief Get name of the mock.
   * \return Name.
   */
  std::string getName() const
  {
    return this->name;
  }

  //! \brief The name to be returned from `getName()`.
  std::string name {"/nodelet"};
};

/**
 * \brief Test ThreadNameUpdatingNodelet that exposes a public version of `updateThreadName()`.
 */
struct TestNodelet : public cras::ThreadNameUpdatingNodelet<MockNodelet>
{
  void updateThreadNamePublic() const
  {
    cras::ThreadNameUpdatingNodelet<MockNodelet>::updateThreadName();
  }
};

/**
 * \brief Test `updateThreadName()` function.
 */
TEST(ThreadNameUpdatingNodelet, updateThreadName)  // NOLINT
{
  TestNodelet nodelet;
  const auto origName = cras::getThreadName();

  nodelet.updateThreadNamePublic();
  EXPECT_NE(origName, cras::getThreadName());
  EXPECT_EQ("nodelet", cras::getThreadName());

  nodelet.name = "/very/long_nodelet_name_exceeding_15/chars";
  nodelet.updateThreadNamePublic();
  EXPECT_TRUE(cras::startsWith(cras::getThreadName(), "very/"));
  EXPECT_TRUE(cras::endsWith(cras::getThreadName(), "/chars"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
