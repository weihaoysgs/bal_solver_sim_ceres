#include <fstream>
#include <iostream>
#include <vector>

struct myobservation
{
  int camera_id;
  int point_id;
  double observation[2];
};
struct myCamera
{
  double parameter[9];
};
struct myPoints
{
  double parameter[3];
};
typedef myobservation bal_observation;
typedef myCamera bal_camera_parameter;
typedef myPoints bal_point_parameter;
// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
void WriteToPLYFile(const std::string &filename,
                    const std::vector<bal_camera_parameter> &camera_pose,
                    const std::vector<bal_point_parameter> &points_xyz)
{
  const int num_cameras_ = camera_pose.size();
  const int num_points_ = points_xyz.size();
  std::ofstream of(filename.c_str());

  of << "ply" << '\n'
     << "format ascii 1.0" << '\n'
     << "element vertex " << num_cameras_ + num_points_ << '\n'
     << "property float x" << '\n'
     << "property float y" << '\n'
     << "property float z" << '\n'
     << "property uchar red" << '\n'
     << "property uchar green" << '\n'
     << "property uchar blue" << '\n'
     << "end_header" << std::endl;

  // Export extrinsic data (i.e. camera centers) as green points.

  for (int i = 0; i < num_cameras_; ++i)
  {
    // const double *camera = cameras() + camera_block_size() * i;
    // CameraToAngelAxisAndCenter(camera, angle_axis, center);
    of << camera_pose[i].parameter[3] << ' ' << camera_pose[i].parameter[4]
       << ' ' << camera_pose[i].parameter[5] << " 0 255 0" << '\n';
  }

  // Export the structure (i.e. 3D Points) as white points.
  for (int i = 0; i < num_points_; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      of << points_xyz[i].parameter[j] << ' ';
    }
    of << " 255 255 255\n";
  }
  of.close();
}
