#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <dirent.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

struct SnavelyReprojectionError
{
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y)
  {
  }

  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const
  {
    T result[3];
    ceres::AngleAxisRotatePoint(camera, point, result);

    result[0] = result[0] + camera[3];
    result[1] = result[1] + camera[4];
    result[2] = result[2] + camera[5];
    T xp = -result[0] / result[2];
    T yp = -result[1] / result[2];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2 * (camera[7] + camera[8] * r2);
    T predicted_x = camera[6] * distortion * xp;
    T predicted_y = camera[6] * distortion * yp;
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y)
  {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};
struct observation
{
  int camera_id;
  int point_id;
  double observation_x;
  double observation_y;
};
struct Camera
{
  double parameter[9];
};
struct Points
{
  double parameter[3];
};

int main(int argc, char** argv)
{
  
  string filename = "/home/weihao/bal_solver/dataset/problem-16-22106-pre.txt";
  ifstream infile;
  int cn, pn, obn;

  infile.open(filename);

  vector<Camera> camera;
  vector<Points> point;
  vector<observation> ob;
  // printf("gethere2");
  if (infile.is_open())  // 文件打开成功,说明曾经写入过东西
  {
    infile >> cn >> pn >> obn;
    for (int i = 0; i < obn; i++)
    {
      observation temp_ob;
      infile >> temp_ob.camera_id >> temp_ob.point_id >>
          temp_ob.observation_x >> temp_ob.observation_y;
      ob.push_back(temp_ob);
    }
    for (int i = 0; i < cn; i++)
    {
      Camera temp_camera;
      for (int j = 0; j < 9; j++) infile >> temp_camera.parameter[j];
      camera.push_back(temp_camera);
    }
    for (int i = 0; i < pn; i++)
    {
      Points temp_point;
      for (int j = 0; j < 3; j++) infile >> temp_point.parameter[j];
      point.push_back(temp_point);
    }
  }
  // printf("gethere1");
  infile.close();
  ceres::Problem problem;
  for (int i = 0; i < obn; i++)
  {
    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
        ob[i].observation_x, ob[i].observation_y);
    problem.AddResidualBlock(cost_function, NULL,
                             camera[ob[i].camera_id].parameter,
                             point[ob[i].point_id].parameter);
  }
  // printf("gethere");
  ceres::Solver::Options options;
  options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  clock_t start, finish;
  double totaltime;
  start = clock();
  ceres::Solve(options, &problem, &summary);
  finish = clock();
  totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
  cout << "time is :" << totaltime << endl;
  std::cout << summary.FullReport() << "\n";

  return 0;
}
