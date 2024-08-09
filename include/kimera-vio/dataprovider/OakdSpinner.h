#pragma once

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <cstdio>
#include <stdlib.h>
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoFrame.h"

namespace VIO {

class OAKdSpinner1{
  public:
  
  OAKdSpinner1();
  
  virtual ~OAKdSpinner1();
  
  bool spin();
  
  std::shared_ptr<dai::DataOutputQueue> groupQueue;
  dai::Device device;
    
  protected:
  
  bool spinOnce();
  
  std::chrono::time_point<std::chrono::_V2::steady_clock, std::chrono::duration<long int, std::ratio<1, 1000000000>>> baseTs;
  
  bool firstTs;
  
 
};

class OAKdSpinner{
  public:
  
  OAKdSpinner();
  
  virtual ~OAKdSpinner();
  
  
    
  protected:
  
  std::shared_ptr<dai::DataOutputQueue> groupQueue;
  
  std::chrono::time_point<std::chrono::_V2::steady_clock, std::chrono::duration<long int, std::ratio<1, 1000000000>>> baseTs;
  
  bool firstTs;
  
 };

 

}
