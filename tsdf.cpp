// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

int main (int argc, char * argv[])
{
  std::string tsdfDirectory = ".";
  if (argc != 2) {
		std::cout << "usage: " << argv[0] << " <TSDF binary file directory>. File should be named tsdf.bin. Default is current folder."
				<< std::endl;
	} else {
		tsdfDirectory = std::string(argv[1]);
	}
  std::string tsdfName = tsdfDirectory + "/tsdf.bin";
  FILE * fp = fopen(tsdfName.c_str(), "r");
  if(!fp){
    std::cerr << "File not found" << std::endl;
    return 1;
  }
  std::cout << "File found. Processing..." << std::endl;
  std::cout << "Generating TSDF point cloud..." << std::endl;

  std::cout.flush();
  float tsdf_value = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < 512; i++) {
    for (int j = 0; j < 512; j++) {
      for (int k = 0; k < 512; k++) {
        if(fread((void*)(&tsdf_value), sizeof(tsdf_value), 1, fp)) {
          if (tsdf_value > 0) {
            cloud.push_back(pcl::PointXYZ(i, j, k));
          }
        }
      }
    }
  }
  pcl::io::savePCDFileASCII(tsdfDirectory+"/tsdfcloud.pcd",cloud);
  std::cout << "TSDF point cloud generated." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("./tsdfcloud.pcd", *cloud_new);
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is actually rendered
  viewer.showCloud(cloud_new);
  while (!viewer.wasStopped ())
  {

  }
  return 0;
}
