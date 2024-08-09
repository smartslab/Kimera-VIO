/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LiveDataProvider.h
 * @brief  Parse live data.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 */

#include "kimera-vio/dataprovider/LiveDataProvider.h"

#include <algorithm>  // for max
#include <fstream>
#include <map>
#include <string>
#include <utility>  // for pair<>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <cstdio>
#include <stdlib.h>
#include <iomanip>
#include <ctime>



DEFINE_string(live_dataset_path,
              "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_int64(initial_tt,
             0,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_tt,
             100000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_bool(log_live_gt_data,
            false,
            "Log Euroc ground-truth data to file for later evaluation.");

namespace VIO {

/* -------------------------------------------------------------------------- */
LiveDataProvider::LiveDataProvider(const VioParams& vio_params)
    : DataProviderInterface(),
      vio_params_(vio_params),
      current_tt_(std::numeric_limits<FrameId>::max()),
      imu_measurements_(){
  
  current_tt_ = 0; //This is the FRAME ID!

  
  //OAKD Setup Stuff
  

  auto imuType = device.getConnectedIMU();
  auto imuFirmwareVersion = device.getIMUFirmwareVersion();
  std::cout << "IMU type: " << imuType << ", firmware version: " << imuFirmwareVersion << std::endl;
  
  
  dai::Pipeline pipeline;
  
  
  //auto colorCamera = pipeline.create<dai::node::ColorCamera>();
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto imu = pipeline.create<dai::node::IMU>();
  auto sync = pipeline.create<dai::node::Sync>();
  auto xoutGroup = pipeline.create<dai::node::XLinkOut>();
  auto xoutIMU = pipeline.create<dai::node::XLinkOut>();

  xoutGroup->setStreamName("xout");
  xoutIMU->setStreamName("imu");
  
  //colorCamera->setCamera("color");
  monoLeft->setCamera("left");
  monoRight->setCamera("right");
  monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  
  //std::cout << monoLeft->getFps() <<std::endl;

  //imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 120);
  imu->enableIMUSensor(dai::IMUSensor::LINEAR_ACCELERATION, 400);
  imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 400);
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(20);
  
  

  sync->setSyncThreshold(std::chrono::milliseconds(1));
  sync->setSyncAttempts(0);  // Infinite attempts

  //colorCamera->video.link(sync->inputs["video"]);
  monoLeft->out.link(sync->inputs["left"]);
  monoRight->out.link(sync->inputs["right"]);
  //imu->out.link(sync->inputs["imu"]);
  imu->out.link(xoutIMU->input);

  sync->out.link(xoutGroup->input);
  
  device.startPipeline(pipeline);

  baseTs = std::chrono::time_point<std::chrono::steady_clock>();

  
  
  //setup = false;
  
  //auto groupQueue = device.getOutputQueue("xout", 10, false);
  
  //auto hasMsgs = groupQueue->has();
  //std::cout << "Num Msgs: " << hasMsgs <<std::endl;
  //sleep(1);
  //hasMsgs = groupQueue->has();
  //std::cout << "Num Msgs: " << hasMsgs <<std::endl;
  //sleep(5);
  
}
 

/* -------------------------------------------------------------------------- */
LiveDataProvider::~LiveDataProvider() {
  LOG(INFO) << "LiveDatasetParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool LiveDataProvider::spin() {
  //std::cout << "Spinning!" << std::endl;
  if (true) {
    
    while (!shutdown_ && spinOnce()) {
      if (!vio_params_.parallel_run_) {
        // Return, instead of blocking, when running in sequential mode.
        return true;
      }
    }
  } else {
    LOG(ERROR) << "Live data was not parsed.";
  }
  LOG_IF(INFO, shutdown_) << "LiveDataProvider shutdown requested.";
  return false;
  
}
bool LiveDataProvider::hasData() const {
  
  return 1;
}

/* -------------------------------------------------------------------------- */
bool LiveDataProvider::spinOnce() {
  
  //LOG(INFO) << "Spin Once was called.";
  //int numIMU = 0;
  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);
  auto groupQueue = device.getOutputQueue("xout", 10, false);
  auto IMUQueue = device.getOutputQueue("imu", 50, false);
  
  if (current_tt_ > 50){ 
  while(!groupQueue->has()) {
              
        auto imuData = IMUQueue->get<dai::IMUData>();
        for(auto& packet : imuData->packets) {
            
            auto& acc = packet.acceleroMeter;
            auto& gyr = packet.gyroscope;
            
            auto acceleroTs1 = acc.getTimestampDevice();
            auto gyroTs1 = gyr.getTimestampDevice();
            
            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;
           
            auto thisTs = std::min(acceleroTs, gyroTs);
            
            long ts = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(thisTs).count());
            
            Vector6 imu_accgyr;
            imu_accgyr << acc.z, acc.y, acc.x, gyr.z, gyr.y, gyr.x;
            
            //std::cout << "A(x): " << acc.z << " A(y): " << acc.y << " A(z): " << acc.x << std::endl;
            //LOG(INFO) << "imu_accgyr time: " << ts << std::endl;
            auto imu_meas=ImuMeasurement(ts, imu_accgyr);
            
            //imu_measurements_.push_back(imu_meas);
            imu_single_callback_(imu_meas); 
            //numIMU++;     
        }
     }   
     }
        
        auto groupMessage = groupQueue->get<dai::MessageGroup>();
        auto leftData= groupMessage->get<dai::ImgFrame>("left");
        auto rightData= groupMessage->get<dai::ImgFrame>("right");
        //auto imuData = groupMessage->get<dai::IMUData>("imu");
        auto tDleft=leftData->getTimestampDevice()-baseTs;
        auto tDright=rightData->getTimestampDevice()-baseTs;
        
        //auto TL=static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDleft).count());
       // auto TR=static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDright).count());
        
        //std::cout << "Left time= " << TL << " Right time= " << TR << std::endl;
        
    //    CHECK(left_frame_callback_);
        
      //  ImuStampS stamps;
      //  ImuAccGyrS measures;
        //std::cout << imuData->packets.size() << " imu packets. \n";
      //  for(auto& packet : imuData->packets) {
            
       //     auto& acc = packet.acceleroMeter;
       //     auto& gyr = packet.gyroscope;
            
        //    auto acceleroTs1 = acc.getTimestampDevice();
        //    auto gyroTs1 = gyr.getTimestampDevice();
           
         //   auto acceleroTs = acceleroTs1 - baseTs;
         //   auto gyroTs = gyroTs1 - baseTs;
            
        //    auto thisTs = std::min(acceleroTs, gyroTs);
            
        //    long ts = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(thisTs).count());
            
       //     Vector6 imu_accgyr;
       //     imu_accgyr << acc.z, acc.y, -acc.x, gyr.z, gyr.y, -gyr.x;
            
            //std::cout << "A(x): " << acc.z << " A(y): " << acc.y << " A(z): " << -acc.x << std::endl;
            //LOG(INFO) << "imu_accgyr time: " << ts << std::endl;
       //     auto stamp=ImuStamp(ts);
       //     auto measure=ImuAccGyr(imu_accgyr);
            //std::cout << "after creation \n";
            
      //      ImuStampS tmp(1,stamps.cols()+1);
      //      tmp << stamps, stamp;
      //      stamps=tmp;         
            
      //      ImuAccGyrS tmp2(6,measures.cols()+1);
      //      tmp2 << measures, measure;
      //      measures=tmp2;
            
            //std::cout << "after merge " << stamps << "\n";
            //imu_single_callback_(imu_meas); 
     //   }
       
    //auto imu_meas_multi = ImuMeasurements(stamps, measures);
    //imu_multi_callback_(imu_meas_multi);
    
       
    if (current_tt_ > 50){
        left_frame_callback_(
                std::make_unique<Frame>(current_tt_,
                                        static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDleft).count()),
                                        left_cam_info,
                                        leftData->getCvFrame()));
                                        
        //std::cout << "Left Callback Finished" << std::endl;
        CHECK(right_frame_callback_);
        right_frame_callback_(
                  std::make_unique<Frame>(current_tt_,
                                        static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDleft).count()),
                                          right_cam_info,
                                          rightData->getCvFrame()));
        //std::cout << "Right Callback Finished" << std::endl;
    }
    
  //std::cout << numIMU << std::endl;  
  current_tt_++;     
  return true;
}

void LiveDataProvider::sendImuData() const {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
 // Timestamp previous_timestamp = -1;
 // for (const ImuMeasurement& imu_meas : imu_measurements_) {
   // CHECK_GT(imu_meas.timestamp_, previous_timestamp)
   //     << "Live IMU data is not in chronological order!";
   // previous_timestamp = imu_meas.timestamp_;
   // imu_single_callback_(imu_meas);
  //}
}

/* -------------------------------------------------------------------------- */
void LiveDataProvider::print() const {
  LOG(INFO) << "------------------ ETHDatasetParser::print ------------------\n"
            << "Displaying info for dataset: Live Data";
    
  LOG(INFO) << "-------------------------------------------------------------";
}


}  // namespace VIO
