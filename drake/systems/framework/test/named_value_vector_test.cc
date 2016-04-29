#include "drake/systems/framework/named_value_vector.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/util/eigen_matrix_compare.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(NamedValueVectorTest, Access) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(2, *vector.get_named_value("bar"));
  Eigen::Matrix<int, 2, 1> expected;
  expected << 1, 2;
  const double tolerance = 1e-6;
  EXPECT_TRUE(CompareMatrices(vector.get_value(), expected, tolerance,
                              util::MatrixCompareType::absolute));
}

GTEST_TEST(NamedValueVectorTest, Set) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  vector.set_named_value("bar", 5);
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(5, *vector.get_named_value("bar"));
}

// Tests that setting a name that doesn't exist has no effect.
GTEST_TEST(NamedValueVectorTest, SetInvalidKey) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  vector.set_named_value("baz", 5);
  Eigen::Matrix<int, 2, 1> expected;
  expected << 1, 2;
  const double tolerance = 1e-6;
  EXPECT_TRUE(CompareMatrices(vector.get_value(), expected, tolerance,
                              util::MatrixCompareType::absolute));
}

// Tests that getting a name that doesn't exist returns nullptr.
GTEST_TEST(NamedValueVectorTest, GetInvalidKey) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  EXPECT_EQ(nullptr, vector.get_named_value("baz"));
}

GTEST_TEST(NamedValueVectorTest, Mutate) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  (*vector.get_mutable_value())(1) = 6;
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(6, *vector.get_named_value("bar"));
}

GTEST_TEST(NamedValueVectorTest, SetWholeVector) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  Eigen::Matrix<int, 2, 1> next_value;
  next_value << 3, 4;
  vector.set_value(next_value);
  EXPECT_EQ(3, *vector.get_named_value("foo"));
  EXPECT_EQ(4, *vector.get_named_value("bar"));
}

// Tests that an error is thrown when the NamedValueVector is set to a vector
// of a different size.
GTEST_TEST(NamedValueVectorTest, ReinitializeInvalid) {
  NamedValueVector<int> vector({"foo", "bar"}, {1, 2});
  Eigen::Matrix<int, 3, 1> next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vector.set_value(next_value), std::runtime_error);
}

// Tests that an error is thrown when there are multiple identically
// named elements.
GTEST_TEST(NamedValueVectorTest, InvalidNames) {
  std::unique_ptr<NamedValueVector<int>> vector;
  EXPECT_THROW(vector.reset(new NamedValueVector<int>({"foo", "foo"})),
               std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
