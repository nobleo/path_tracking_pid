#include <gtest/gtest.h>

#include <path_tracking_pid/details/fifo_array.hpp>

namespace
{
using path_tracking_pid::details::FifoArray;

TEST(PathTrackingPidDetailsFifoArray, Initialize)
{
  FifoArray<int, 3> fifo;

  EXPECT_EQ(fifo[0], 0);
  EXPECT_EQ(fifo[1], 0);
  EXPECT_EQ(fifo[2], 0);
}

TEST(PathTrackingPidDetailsFifoArray, Push)
{
  FifoArray<int, 3> fifo;

  fifo.push(42);

  EXPECT_EQ(fifo[0], 42);
  EXPECT_EQ(fifo[1], 0);
  EXPECT_EQ(fifo[2], 0);
}

TEST(PathTrackingPidDetailsFifoArray, Push_UntilFull)
{
  FifoArray<int, 3> fifo;

  fifo.push(42);
  fifo.push(111);
  fifo.push(-4);

  EXPECT_EQ(fifo[0], -4);
  EXPECT_EQ(fifo[1], 111);
  EXPECT_EQ(fifo[2], 42);
}

TEST(PathTrackingPidDetailsFifoArray, Push_BeyondFull)
{
  FifoArray<int, 3> fifo;

  fifo.push(42);
  fifo.push(111);
  fifo.push(-4);
  fifo.push(314);

  EXPECT_EQ(fifo[0], 314);
  EXPECT_EQ(fifo[1], -4);
  EXPECT_EQ(fifo[2], 111);
}

TEST(PathTrackingPidDetailsFifoArray, Reset)
{
  FifoArray<int, 3> fifo;

  fifo.push(42);
  fifo.push(111);
  fifo.push(-4);
  fifo.push(314);

  fifo.reset();

  EXPECT_EQ(fifo[0], 0);
  EXPECT_EQ(fifo[1], 0);
  EXPECT_EQ(fifo[2], 0);
}

TEST(PathTrackingPidDetailsFifoArray, At)
{
  FifoArray<int, 3> fifo;

  fifo.push(42);
  fifo.push(111);
  fifo.push(-4);

  EXPECT_EQ(fifo[0], fifo.at<0>());
  EXPECT_EQ(fifo[1], fifo.at<1>());
  EXPECT_EQ(fifo[2], fifo.at<2>());
}

TEST(PathTrackingPidDetailsFifoArray, OtherType)
{
  FifoArray<double, 3> fifo;

  fifo.push(24.0);
  fifo.push(0.5);
  fifo.push(0.25);

  EXPECT_EQ(fifo[0], 0.25);
  EXPECT_EQ(fifo[1], 0.5);
  EXPECT_EQ(fifo[2], 24.0);
}

TEST(PathTrackingPidDetailsFifoArray, OtherSize)
{
  FifoArray<int, 5> fifo;

  fifo.push(1);
  fifo.push(2);
  fifo.push(3);
  fifo.push(4);
  fifo.push(5);
  fifo.push(6);

  EXPECT_EQ(fifo[0], 6);
  EXPECT_EQ(fifo[1], 5);
  EXPECT_EQ(fifo[2], 4);
  EXPECT_EQ(fifo[3], 3);
  EXPECT_EQ(fifo[4], 2);
}

TEST(PathTrackingPidDetailsFifoArray, Assign)
{
  FifoArray<int, 3> fifo;
  fifo[0] = 1;

  EXPECT_EQ(fifo[0], 1);
  EXPECT_EQ(fifo[1], 0);
  EXPECT_EQ(fifo[2], 0);
}

}  // namespace
