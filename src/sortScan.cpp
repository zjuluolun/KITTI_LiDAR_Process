#include "utils.h"

int main(int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
  std::string path = "../data/000000.bin";
  readPointCloud(point_cloud, path);

  std::vector< pcl::PointCloud<pcl::PointXYZI> > laser_cloud_scans(64);
  sortScan(point_cloud, laser_cloud_scans);

}
