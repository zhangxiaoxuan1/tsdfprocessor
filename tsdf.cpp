// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

int main (int argc, char * argv[])
{
  std::string tsdfDirectory = "";
  if (argc != 2) {
		std::cout << "usage: " << argv[0] << " <TSDF binary file directory>. File should be named tsdf.bin. Default is current folder."
				<< std::endl;
	} else {
		tsdfDirectory = std::string(argv[1]);
	}
  std::string tsdfName = tsdfDirectory + "/tsdf.bin";
  tsdfName += "_tsdf.bin";
  FILE * fp = fopen(tsdfName.c_str(), "r");
  std::cout << "File found. Processing..." << std::endl;
  std::cout << "Generating TSDF point cloud..." << std::endl;

  std::cout.flush();
  float tsdf_value = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < 512; i++) {
    for (int j = 0; j < 512; j++) {
      bool positiveFirst = false;
      for (int k = 0; k < 512; k++) {
        if(fread((void*)(&tsdf_value), sizeof(tsdf_value), 1, fp)) {
          if(k==0 && tsdf_value >= 0) {
            positiveFirst = true;
          }
          // zero crossing?
          if((tsdf_value < 0 && positiveFirst) || (tsdf_value > 0 && !positiveFirst)) {
            cloud.push_back(pcl::PointXYZ(i, j, k));
            break;
          }
        }
      }
    }
  }
  pcl::io::savePCDFileASCII(tsdfDirectory+"/tsdfcloud.pcd",cloud);
  std::cout << "TSDF point cloud generated." << std::endl;
}
