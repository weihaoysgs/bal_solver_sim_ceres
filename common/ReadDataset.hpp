#ifndef READ_DATASET_HPP
#define READ_DATASET_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

void ReadPoint2dDataset(const std::string &p2d_file,
                        const std::string &p3d_file,
                        std::vector<Eigen::Vector3d> &pts_3d_vec,
                        std::vector<Eigen::Vector2d> &pts_2d_vec,
                        bool print_output)
{
  std::ifstream fp3d(p3d_file);
  std::ifstream fp2d(p2d_file);
  if (!fp3d || !fp2d)
  {
    std::cerr << "Error Open file of fp3d or fp2d!\n";
    return;
  }
  while (!fp3d.eof())
  {
    double pt3d[3] = {0.0};
    for (auto &p : pt3d)
    {
      fp3d >> p;
    }
    Eigen::Vector3d point3d;
    point3d << pt3d[0], pt3d[1], pt3d[2];
    pts_3d_vec.push_back(point3d);
  }
  while (!fp2d.eof())
  {
    double pt2d[2] = {0.0};
    for (auto &p : pt2d)
    {
      fp2d >> p;
    }
    Eigen::Vector2d point2d;
    point2d << pt2d[0], pt2d[1];
    pts_2d_vec.push_back(point2d);
  }
  if (print_output)
  {
    std::for_each(pts_3d_vec.begin(), pts_3d_vec.end(),
                  [](const Eigen::Vector3d &pts_3d) -> void {
                    std::cout << pts_3d.transpose() << "; \t";
                  });
    std::for_each(pts_2d_vec.begin(), pts_2d_vec.end(),
                  [](const Eigen::Vector2d &pts_2d) -> void {
                    std::cout << pts_2d.transpose() << "; \t";
                  });
  }
}

#endif  // READ_DATASET_HPP