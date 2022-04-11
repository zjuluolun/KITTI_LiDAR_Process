#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>

void sortScan(const pcl::PointCloud<pcl::PointXYZI> point_cloud, std::vector< pcl::PointCloud<pcl::PointXYZI> > &laser_cloud_scans)
{
  int index[65]={0};

  int cloud_size = point_cloud.size();
  int beam_count=0;
  float last_point_angle;
  float back_point_angle;
  bool selected = true;
  int selectcount = 0;
  for (int i=0; i<cloud_size; i++)
  {

    float angle = atan2(point_cloud.points[i].y , -point_cloud.points[i].x ) * 180 / M_PI;   

    if (i==0) last_point_angle = angle, back_point_angle = angle; //handle the first point


    if(selected&&(selectcount<10)) {selectcount++;}
    else selected = false;

    if (((angle-last_point_angle)>=90)&&(~selected)&&((angle-back_point_angle)>=90)) 
    {
      beam_count++ ;
      index[beam_count] = i;
      selected = true;
      selectcount = 0;
    }
    back_point_angle = last_point_angle;
    last_point_angle = angle;
  }
  if(beam_count<63)
  {
    for (int i=beam_count; i<64; i++)
    {
      index[i] = index[beam_count];
    }
      
    std::cout << "scans less than 64" << std::endl;  //beams fly !
  }
  else if(beam_count>=64)
  {
    std::cout << "gg, sort failed" << std::endl; // may fail
    return ;
  }

  index[64] = point_cloud.size()-1;
  int scan_id = 63;
  
  for (int it = 0; it < 64; it++)
  {
    pcl::PointXYZI point;
  	if ( it < 63)
    {
  		for (int i = index[it]; i < index[it+1]; i++)
      {
        point.intensity = scan_id;
        point.x = point_cloud.points[i].y;      
        point.y = point_cloud.points[i].z;
        point.z = point_cloud.points[i].x;
        laser_cloud_scans[scan_id].push_back(point);     
      
      }
    }
 	  else
    {
 		  for (int i = index[it]; i < cloud_size; i++)
      {
        point.intensity = scan_id;
      	point.x = point_cloud.points[i].y;
      	point.y = point_cloud.points[i].z;
      	point.z = point_cloud.points[i].x;
    	  laser_cloud_scans[scan_id].push_back(point);
 	     }
     }
     scan_id--;
  }
}

int readPointCloud(pcl::PointCloud<pcl::PointXYZI> &point_cloud, const std::string filename)
{

  std::ifstream binfile(filename.c_str(),std::ios::binary);
  if(!binfile)
  {
    throw std::runtime_error("file cannot open");
    return -1;
  }
  else
  {
    while(1)
    {
      float s;
      pcl::PointXYZI point;
      binfile.read((char*)&s,sizeof(float));
      if(binfile.eof()) break;
      point.x = s;
      binfile.read((char*)&s,sizeof(float));
      point.y = s;
      binfile.read((char*)&s,sizeof(float));
      point.z = s;
      binfile.read((char*)&s,sizeof(float));
      point.intensity = s;
      point_cloud.push_back(point);
   }
  }
  binfile.close();
  return 1;
}

