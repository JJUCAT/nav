#ifndef __EIGEN3_TEST_H__
#define __EIGEN3_TEST_H__


#include <Eigen/Core>
#include <Eigen/Dense>
#include <complex>
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <iostream>


namespace Eigen3_Test_ns {

void TestStorageOrder()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix<int, 3, 3, Eigen::RowMajor> rowmajorm;
  rowmajorm << 0, 1, 2,
               3, 4, 5,
               6, 7, 8;
  ROS_INFO_STREAM("矩阵:" << std::endl << rowmajorm);

  int* rowmajorm_data = rowmajorm.data(); // 获取矩阵系数数据存储地址
  ROS_INFO_STREAM("行优先存储在内存上数据分布:");
  for (size_t i = 0; i < rowmajorm.size(); i++) {
    ROS_INFO_STREAM("index[" << i << "]:" << rowmajorm_data[i]);
  }

  Eigen::Matrix<int, 3, 3, Eigen::ColMajor> colmajorm;
  colmajorm = rowmajorm;
  int* colmajorm_data = colmajorm.data();
  ROS_INFO_STREAM("列优先存储在内存上数据分布:");
  for (size_t i = 0; i < colmajorm.size(); i++) {
    ROS_INFO_STREAM("index[" << i << "]:" << colmajorm_data[i]);
  }
}


void TestAlign()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix<int, 3, 3, Eigen::AutoAlign> autoalignm;
  autoalignm.Random(3, 3);
  ROS_INFO_STREAM("自动对齐矩阵:" << std::endl << autoalignm);

  Eigen::Matrix<int, 3, 3, Eigen::DontAlign> dontalignm;
  dontalignm = autoalignm;
  ROS_INFO_STREAM("不对齐矩阵:" << std::endl << dontalignm);
}


void TestDynamic()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dynamicm(2, 3); // 行列初始化
  dynamicm.Random(2, 3);
  ROS_INFO_STREAM("动态矩阵, 随机赋值:" << std::endl << dynamicm);


  dynamicm << 1.2, 2.3, 3.4,
              4.5, 5.6, 6.7;
  ROS_INFO_STREAM("动态矩阵，赋值:" << std::endl << dynamicm);


  ROS_INFO_STREAM("动态矩阵，系数访问[0,2]:" << dynamicm(0,2));
  dynamicm(0,2) = 233;
  ROS_INFO_STREAM("动态矩阵，系数访问[0,2]:" << dynamicm(0,2));


  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dynamicm44(4, 4);
  ROS_INFO_STREAM("动态矩阵4x4，初始化:" << std::endl << dynamicm44);
  dynamicm44 = dynamicm;
  ROS_INFO_STREAM("动态矩阵4x4，自适应大小:" << std::endl << dynamicm44);
}


void TestResize()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m3x3(3, 3); // 行列初始化
  m3x3.Random(3, 3);
  ROS_INFO_STREAM("矩阵3x3, 随机赋值:" << std::endl << m3x3);

  m3x3.resize(5, 5);
  ROS_INFO_STREAM("调整矩阵大小5x5:" << std::endl << m3x3);

  m3x3.conservativeResize(2, 2);
  ROS_INFO_STREAM("调整矩阵大小2x2:" << std::endl << m3x3);
}


void TestDefaultTemplate()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  // Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>
  Eigen::MatrixXi imx(3, 3);
  ROS_INFO_STREAM("int 动态矩阵 3x3:" << std::endl << imx);

  // Eigen::Matrix<float, 2, 2>
  Eigen::Matrix2f fm2x2;
  ROS_INFO_STREAM("float 固定矩阵 2x2:" << std::endl << fm2x2);

  // Eigen::Matrix<double, Eigen::Dynamic, 2>
  Eigen::MatrixX2d dmx2(3, 2);
  ROS_INFO_STREAM("double 动态行，2列矩阵 3x2:" << std::endl << dmx2);

  // Eigen::Matrix<double, 1, Eigen::Dynamic>
  Eigen::Matrix2Xd dm2x(2, 3);
  ROS_INFO_STREAM("double 2行，动态列矩阵 2x3:" << std::endl << dm2x);
}


void TestBaseOperation()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix3i m0;
  m0 << 2, 1, 3,
        6, 8, 0,
        0, 9, 12;
  ROS_INFO_STREAM("3x3 m0:" << std::endl << m0);

  Eigen::Matrix3i m1;
  m1 << 6, 1, 13,
        2, 0, 4,
        9, 11, 7;
  ROS_INFO_STREAM("3x3 m1:" << std::endl << m1);

  ROS_INFO_STREAM("m0+m1=" << std::endl << m0+m1);
  ROS_INFO_STREAM("3*m0=" << std::endl << 3*m0);
  ROS_INFO_STREAM("m1/2=" << std::endl << m1/2);
}


void TestTranspose()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix3i m;
  m << 2, 1, 3,
       6, 8, 0,
       0, 9, 3;
  ROS_INFO_STREAM("3x3 m:" << std::endl << m);

  Eigen::Matrix3i tm = m.transpose();
  ROS_INFO_STREAM("after transpose, tm:" << std::endl << tm);

  m.transposeInPlace();
  ROS_INFO_STREAM("m.transposeInPlace(), m:" << std::endl << m);
}


void TestAdjoint()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Matrix<std::complex<float>, 2, 2> m;
  m << std::complex<float>(3, 2), std::complex<float>(5, 1),
       std::complex<float>(7, 3), std::complex<float>(4, 2);
  ROS_INFO_STREAM("2x2 complex m:" << std::endl << m);

  Eigen::Matrix<std::complex<float>, 2, 2> cm = m.conjugate();
  ROS_INFO_STREAM("after conjugate, cm:" << std::endl << cm);

  Eigen::Matrix<std::complex<float>, 2, 2> am = m.adjoint();
  ROS_INFO_STREAM("after adjoint, am:" << std::endl << am);

  m.adjointInPlace();
  ROS_INFO_STREAM("m.adjointInPlace(), m:" << std::endl << m);
}


void TestTransform()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::MatrixXf m(3, 3);
  m << 2.3, 4.0, -0.7,
       3.3, 0.0, 1.8,
       0.2, 1.0, -0.5;
  ROS_INFO_STREAM("matrix:" << std::endl << m);

  Eigen::MatrixX3f t(2, 3);
  t << 1.2, 2.3, 3.4,
       4.5, 5.6, 6.7;
  ROS_INFO_STREAM("transform matrix:" << std::endl << t);

  Eigen::MatrixXf tm;
  tm = t*m;
  ROS_INFO_STREAM("t*m=" << std::endl << tm);
}


void TestVector()
{
  setlocale(LC_ALL, "zh_CN.UTF-8"); // 支持 ros 打印中文

  Eigen::Vector3i v(3, 6, 9);
  ROS_INFO_STREAM("列向量3i v 构造初始化指定了系数和行数:" << std::endl << v);

  //! 固定向量类不支持 << 赋值，会导致段错误，但是动态向量类支持。
  Eigen::RowVectorXf u(5);
  u << 1.2, 2.3, 3.4, 4.5, 5.6;
  ROS_INFO_STREAM("动态行向量 u 默认初始化:" << std::endl << u);

  Eigen::VectorXd w(5);
  w.Random(5);
  ROS_INFO_STREAM("列向量 w 随机赋值:" << std::endl << w);
  ROS_INFO_STREAM("列向量[]访问 w[2]:" << std::endl << w[2]);
  w[2] = 12.34;
  ROS_INFO_STREAM("列向量[]修改 w[2]:" << std::endl << w[2]);

}


void TestVectorOperate()
{
  Eigen::Vector3i u(3, 6, 9);
  Eigen::Vector3i v(2, 3, 3);
  ROS_INFO_STREAM("列向量3i u:" << std::endl << u);
  ROS_INFO_STREAM("列向量3i v:" << std::endl << v);
  ROS_INFO_STREAM("u+v=" << std::endl << u+v);

  Eigen::Vector3f fu(2.5, 0, -1.5);
  Eigen::Vector3f fv(5.1, 3.3, 1.3);
  ROS_INFO_STREAM("列向量3f fu:" << std::endl << u);
  ROS_INFO_STREAM("列向量3f fv:" << std::endl << v);
  ROS_INFO_STREAM("fu-fv=" << std::endl << fu-fv);

  Eigen::Vector3d du(0.7, -1.2, -3.5);
  ROS_INFO_STREAM("列向量3d du:" << std::endl << du);
  ROS_INFO_STREAM("列向量3d -3.3 * du=" << std::endl << -3.3*du);
}

void TestVectorDot()
{
  Eigen::Vector3f u0(0.0, 0.3, 0.5);
  Eigen::Vector3f v0(0.5, 0.0, 0.3);
  ROS_INFO_STREAM("u0:" << std::endl << u0);
  ROS_INFO_STREAM("v0:" << std::endl << v0);
  ROS_INFO_STREAM("u0 · v0 = " << u0.dot(v0));
  ROS_INFO_STREAM("v0 · u0 = " << v0.dot(u0));

  Eigen::Vector3f u1(1.2, 2.3, 3.4);
  Eigen::Vector3f v1(5.4, 6.5, 7.6);
  ROS_INFO_STREAM("u1:" << std::endl << u1);
  ROS_INFO_STREAM("v1:" << std::endl << v1);
  ROS_INFO_STREAM("u1 · v1 = " << u1.dot(v1));
}

void TestVectorCross()
{
  Eigen::Vector3f u0(0.0, 0.3, 0.5);
  Eigen::Vector3f v0(0.5, 0.0, 0.3);
  ROS_INFO_STREAM("u0:" << std::endl << u0);
  ROS_INFO_STREAM("v0:" << std::endl << v0);
  ROS_INFO_STREAM("u0 x v0 = " << u0.cross(v0));
  ROS_INFO_STREAM("v0 x u1 = " << v0.cross(u0));

  Eigen::Vector3f u1(1.2, 2.3, 3.4);
  Eigen::Vector3f v1(5.4, 6.5, 7.6);
  ROS_INFO_STREAM("u1:" << std::endl << u1);
  ROS_INFO_STREAM("v1:" << std::endl << v1);
  ROS_INFO_STREAM("u1 x v1 = " << u1.cross(v1));
}


void TestArray()
{
  Eigen::ArrayXf a0(3);
  a0.Random(3);
  ROS_INFO_STREAM("a0:" << a0);

  Eigen::Array33d a1;
  a1(2, 2) = 23.33;
  ROS_INFO_STREAM("a1(2, 2):" << std::endl << a1(2, 2));

  a1 << 1.2, 2.3, 3.4,
        4.5, 5.6, 6.7,
        7.8, 8.9, 9.1;
  ROS_INFO_STREAM("a1:" << std::endl << a1);

  Eigen::Array4i a2;
  a2[2] = 123;
  ROS_INFO_STREAM("a2[2]=" << a2[2]);
}


void TestArrayOperate()
{
  Eigen::Array22f a0;
  a0 << 1.2, -2.3,
        -3.4, 4.5;
  Eigen::Array22f a1;
  a1 << -6.5, 7.6,
        8.7, -9.8;
  ROS_INFO_STREAM("a0= " << std::endl << a0);
  ROS_INFO_STREAM("a1= " << std::endl << a1);
  ROS_INFO_STREAM("a0 + a1 = " << std::endl << a0+a1);
  ROS_INFO_STREAM("a1 + 5.5 = " << std::endl << a1+5.5);
  ROS_INFO_STREAM("a0 * a1 = " << std::endl << a0*a1);
  ROS_INFO_STREAM("a0 * 2.33 = " << std::endl << a0*2.33);
  ROS_INFO_STREAM("a0 / a1 = " << std::endl << a0/a1);
  ROS_INFO_STREAM("a0.abs() = " << std::endl << a0.abs());
  ROS_INFO_STREAM("a1.sqrt() = " << std::endl << a1.sqrt());
  ROS_INFO_STREAM("a0.min(a1) = " << std::endl << a0.min(a1));
}


void TestArrayTransform()
{
  Eigen::Array22f a;
  a << 1.2, -2.3,
       -3.4, 4.5;
  
  Eigen::Matrix2f t;
  t << 0, -1,
       1, 0;

  Eigen::Matrix2f m;
  m = t*a.matrix();
  ROS_INFO_STREAM("数组 a = " << std::endl << a);
  ROS_INFO_STREAM("旋转矩阵 t = " << std::endl << t);
  ROS_INFO_STREAM("旋转后的矩阵 m = t * a.matrix() = " << std::endl << m);
  ROS_INFO_STREAM("m.array() 和 a 中最小系数组成的数组 = " << std::endl << m.array().min(a));
}


void TestBlock()
{
  Eigen::Matrix4f m;
  m << 1.2, -2.3, 3.4, -4.5,
       5.6, -6.7, 7.8, -8.9,
       9.0, -0.1, 1.2, -2.3,
       3.4, -4.5, 5.6, -6.7;
  Eigen::Matrix2f md1122 = m.block(1,1,2,2);
  Eigen::Matrix2f mf1122 = m.block<2,2>(1,1);
  ROS_INFO_STREAM("动态取块，矩阵 m 的 (1,1) 取 2*2 块:" << std::endl << md1122);
  ROS_INFO_STREAM("固定取块，矩阵 m 的 (1,1) 取 2*2 块:" << std::endl << mf1122);

  Eigen::Matrix3f n;
  n << 1.2, -2.3, 3.4,
       -4.5, 5.6, -6.7,
       7.8, -8.9, 9.0;
  Eigen::Matrix2f nd0022 = m.block(0,0,2,2);
  Eigen::Matrix2f mn22 = md1122 + nd0022;
  ROS_INFO_STREAM("动态取块，矩阵 n 的 (0,0) 取 2*2 块:" << std::endl << nd0022);
  ROS_INFO_STREAM("矩阵 m(1,1){2*2} + n(0,0){2*2}" << std::endl << mn22);

  Eigen::Matrix2f md2022 = m.block(2,0,2,2);
  Eigen::Matrix2f nf1122 = n.block<2,2>(1,1);
  ROS_INFO_STREAM("矩阵 m(2,0){2*2} * n(1,1){2*2}" << std::endl << md2022*nf1122);
}


void TestVectorBlock()
{
  Eigen::Vector3f v3f(1.2, -2.3, 3.4);
  Eigen::Vector2f v3fh2 = v3f.head(2);
  Eigen::Vector2f v3ft2 = v3f.tail(2);
  ROS_INFO_STREAM("向量 v3f:" << std::endl << v3f);
  ROS_INFO_STREAM("向量 v3f 取头部 2 个元素:" << std::endl << v3fh2);
  ROS_INFO_STREAM("向量 v3f 取尾部 2 个元素:" << std::endl << v3ft2);

  Eigen::VectorXf v6f(6);
  v6f << 0.1, -1.2, 2.3, -3.4, 4.5, -5.6;
  Eigen::Vector3f v6f33 = v6f.segment(3,3);
  Eigen::Vector3f v6f23 = v6f.segment<3>(2);
  ROS_INFO_STREAM("向量 v6f:" << std::endl << v6f);
  ROS_INFO_STREAM("向量 v6f 动态取块在 3 位置取 3 个元素:" << std::endl << v6f33);
  ROS_INFO_STREAM("向量 v6f 在 2 位置固定取块 3 个元素:" << std::endl << v6f23);
}


void TestVectorSlicingIndexing()
{
  // Eigen::Vector4f v4f(1.2, -2.3, 3.4, -4.5);
  // auto v_elements = v4f(Eigen::seq(1,3));
}


void TestReductions()
{
  Eigen::Matrix4f m;
  m << 1.2, -2.3, 3.4, -4.5,
       5.6, -6.7, 7.8, -8.9,
       9.0, -0.1, 1.2, -2.3,
       3.4, -4.5, 5.6, -6.7;
  ROS_INFO_STREAM("矩阵 m:" << std::endl << m);

  ROS_INFO_STREAM("m.sum():" << m.sum() << std::endl <<
                  "m.prod():" << m.prod() << std::endl <<
                  "m.mean():" << m.mean() << std::endl <<
                  "m.minCoeff():" << m.minCoeff() << std::endl <<
                  "m.maxCoeff():" << m.maxCoeff() << std::endl <<
                  "m.trace():" << m.trace() << std::endl <<
                  "m.squaredNorm():" << m.squaredNorm() << std::endl <<
                  "m.norm():" << m.norm() << std::endl);

  ROS_INFO_STREAM("m.colwise().maxCoeff():" << m.colwise().maxCoeff() << std::endl <<
                  "m.rowwise().sum().minCoeff():" << m.rowwise().sum().minCoeff() << std::endl);
}


void TestBroadcasting()
{
  Eigen::MatrixXf m(2,4);
  m << 1.0, -11.23, 6.9, -0.83,
       -5.6, 2.56, -2.97, 6.56;
  ROS_INFO_STREAM("矩阵 m:" << std::endl << m);

  Eigen::VectorXf n(2);
  n << 2,
       3;
  ROS_INFO_STREAM("向量 n:" << std::endl << n);

  Eigen::Index index;
  (m.colwise() - n).colwise().squaredNorm().minCoeff(&index);
  std::cout << "Nearest neighbour is column " << index << ":" << std::endl;
  std::cout << m.col(index) << std::endl;
}


void TestMap()
{
  int array[8];
  for(int i = 0; i < 8; ++i)
    array[i] = i;
  auto m = Eigen::Map<Eigen::Matrix<int,2,4>>(array);
  ROS_INFO_STREAM("矩阵 array -> m:" << std::endl << m);
  ROS_INFO_STREAM("矩阵 array -> m.colwise().sum()" << std::endl << m.colwise().sum());


  int data[] = {1,2,3,4,5,6,7,8,9};
  new (&m) Eigen::Map<Eigen::Matrix<int,2,4>>(data);
  ROS_INFO_STREAM("矩阵 data -> m:" << std::endl << m);
  ROS_INFO_STREAM("矩阵 data -> m.rowwise().squaredNorm():" << std::endl << m.rowwise().squaredNorm());
}


} // namespace Eigen3_Test_ns


#endif // __EIGEN3_TEST_H__

