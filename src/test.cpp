#include "rgbd_odometry/rgbd_odometry_core.h"
#include <iostream>

bool readMat( const std::string& filename, cv::Mat& M);
void showImageSideBySide(std::string name, cv::Mat& A, cv::Mat& B, int type);

int main(int argc, char **argv)
{
  // Read in the images
  cv::Mat color_A, depth_A, color_B, depth_B;
  readMat("../images/RGB00420.bin", color_A);
  readMat("../images/D00420.bin", depth_A);
  readMat("../images/RGB00082.bin", color_B);
  readMat("../images/D00082.bin", depth_B);


  // Create an initialize the RGBDOdometryCore
  RGBDOdometryCore vo;
  vo.getImageFunctionProvider()->computeFilterBank();
  vo.getImageFunctionProvider()->initialize(false, ".", "depthmask.cl");
  
  // Set the camera intrisics (Intel RealSense)
  cv::Mat K = (cv::Mat_<double>(3,3) << 
       602.595726, 0, 313.324336, 0, 603.324985, 248.223737, 0, 0, 1);
  vo.setRGBCameraIntrinsics(K);
  

  // Convert depth to a float image with units in meters  
  cv::Mat depthA_scaled, depthB_scaled;
  depth_A.convertTo(depthA_scaled, CV_32FC1, 1e-3);
  depth_B.convertTo(depthB_scaled, CV_32FC1, 1e-3);

  // Show the images, since the .bin files are a bit weird
  showImageSideBySide("color", color_A, color_B, CV_8UC3);
  showImageSideBySide("depth", depth_A, depth_B, CV_16UC1);
  
  // Compute the transform
  Eigen::Matrix4f transform;
  Eigen::Matrix<float, 6,6> cov;
  if (vo.computeRelativePoseDirect(color_A, depthA_scaled, color_B, depthB_scaled, transform, cov, 0, true, 50))
  {
    std::cout << "T = \n" << transform << std::endl;    
    std::cout << "cov = \n" << cov << std::endl;
  }
  else
  {
    std::cout << "Failed" << std::endl;
  }
  cv::waitKey(0);
}


//  Reads a raw binary image
bool readMat( const std::string& filename, cv::Mat& M)
{
  std::ifstream in(filename.c_str(), std::ios::in|std::ios::binary);
  if (!in)
  {
    std::cerr << "Cannot find filename" << std::endl;
    return 0;
  }
  int cols;
  int rows;
  int chan;
  int eSiz;

  // Read header
  in.read((char*)&cols,sizeof(cols));
  in.read((char*)&rows,sizeof(rows));
  in.read((char*)&chan,sizeof(chan));
  in.read((char*)&eSiz,sizeof(eSiz));

  // Determine type of the matrix
  int type = 0;
  switch (eSiz)
  {
  case sizeof(char):
    type = CV_8UC(chan);
    break;
  case sizeof(uint16_t):
    type = CV_16UC(chan);
    break;
  case sizeof(float):
    type = CV_32FC(chan);
    break;
  case sizeof(double):
    type = CV_64FC(chan);
    break;
  }

  // Alocate Matrix.
  M = cv::Mat(rows,cols,type,cv::Scalar(1));

  // Read data.
  if (M.isContinuous())
  {
    in.read((char *)M.data,cols*rows*chan*eSiz);
  }
  else
  {
    return false;
  }
  in.close();
  return true;
}

// Puts the images side by side
void showImageSideBySide(std::string name, cv::Mat& A, cv::Mat& B, int type)
{
  cv::Size szA = A.size();
  cv::Size szB = B.size();
  cv::Mat C(szA.height, szA.width+szB.width, type);
  cv::Mat left(C, cv::Rect(0, 0, szA.width, szA.height));
  A.copyTo(left);
  cv::Mat right(C, cv::Rect(szA.width, 0, szB.width, szB.height));
  B.copyTo(right);
  cv::imshow(name, C);
}