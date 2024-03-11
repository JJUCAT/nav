#include <ros/ros.h>
#include <gtest/gtest.h>
// #include "ggtest/test_file.h" // 待测试文件

// 声明第一个测试例
// TestSuite是测试对象的名称，这个名称是任意的，但是最好具有针对性
// testCase1是测试用例的名称，这个是唯一的，最好说明测试用例的用途
TEST(TestSuite, testCase1) {
  // T t;
  int f = 5;
  ASSERT_EQ(5, f);
}

int main(int argc, char **argv) {
  // 程序运行之后，可以在终端显示测试结果，也会以.xml文件的格式存储到指定路径文件夹，
  // 且.xml文件的名字默认是测试程序的节点的名字，多次运行测试程序，不会覆盖之前的测试文件。
  testing::GTEST_FLAG(output) = "xml:/home/lmr/";
  // 初始化测试器
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_ggnode");
  ros::NodeHandle nh;

  // 开始执行所有的测试，如果全部通过则返回0，否则返回1
  // 此步是必须的，否则不会执行测试
  return RUN_ALL_TESTS();
}