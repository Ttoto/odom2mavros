#ifndef YAMLREAD_H
#define YAMLREAD_H

#include "../3rdPartLib/yaml-cpp-0.6.2/include/yaml-cpp/yaml.h"

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<int, 3, 1> Vec3I;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 16, 1> Vec16;
typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 1, 4> Mat1x4;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 3, 4> Mat3x4;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;
typedef Eigen::Matrix<double, 6, 6> Mat6x6;
typedef Eigen::Matrix<double, 9, 9> Mat9x9;
typedef Eigen::Matrix<double, 12, 12> Mat12x12;
typedef Eigen::Matrix<double, 15, 15> Mat15x15;
typedef Eigen::Matrix<double, 15, 6> Mat15x6;
typedef Eigen::Matrix<double, 6, 15> Mat6x15;
typedef Eigen::Matrix<double, 9, 15> Mat9x15;
typedef Eigen::Matrix<double, 15, 12> Mat15x12;
typedef Eigen::Matrix<double, 15, 9> Mat15x9;
typedef Eigen::Matrix<double, 3, 15> Mat3x15;
typedef Eigen::Matrix<double, 15, 3> Mat15x3;
typedef Eigen::Matrix<double, 1, 15> Mat1x15;
typedef Eigen::Matrix<double, 15, 1> Mat15x1;

inline Mat3x3 Mat33FromYaml(string FilePath, string vName)
{
  Mat3x3 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,3,3,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline Mat4x4 Mat44FromYaml(string FilePath, string vName)
{
  Mat4x4 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,4,4,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline double getDoubleVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const double ret = config[vName].as<double>();
  return ret;
}
inline int getIntVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const int ret = config[vName].as<int>();
  return ret;
}
#endif // YAML_EIGEN_H
