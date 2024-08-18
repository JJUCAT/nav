#include "ros/ros.h"
#include "tf/tf.h"
#include <limits>

#include "eigen3_test/eigen3_test.h"


int main(int argc, char** argv)
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  ros::init(argc, argv, "eigen3_test");
  ros::NodeHandle n("~");

  // Eigen3_Test_ns::TestStorageOrder();
  // Eigen3_Test_ns::TestAlign();
  // Eigen3_Test_ns::TestDynamic();
  // Eigen3_Test_ns::TestResize();
  // Eigen3_Test_ns::TestDefaultTemplate();
  // Eigen3_Test_ns::TestBaseOperation();
  // Eigen3_Test_ns::TestTranspose();
  // Eigen3_Test_ns::TestAdjoint();
  // Eigen3_Test_ns::TestTransform();
  // Eigen3_Test_ns::TestVector();
  // Eigen3_Test_ns::TestVectorOperate();
  // Eigen3_Test_ns::TestVectorDot();
  // Eigen3_Test_ns::TestVectorCross();
  // Eigen3_Test_ns::TestArray();
  // Eigen3_Test_ns::TestArrayOperate();
  Eigen3_Test_ns::TestArrayTransform();

  double hz = 10.f;
  ros::Rate r(hz);
  while(ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
