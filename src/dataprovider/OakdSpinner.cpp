#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <cstdio>
#include <stdlib.h>
#include "kimera-vio/dataprovider/OakdSpinner.h"
#include <unistd.h>

namespace VIO {

OAKdSpinner1::OAKdSpinner1() {

    //device = dai::device;

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
  imu->enableIMUSensor(dai::IMUSensor::LINEAR_ACCELERATION, 500);
  imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 400);
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(20);
  
  

  sync->setSyncThreshold(std::chrono::milliseconds(1));
  sync->setSyncAttempts(-1);  // Infinite attempts

  //colorCamera->video.link(sync->inputs["video"]);
  monoLeft->out.link(sync->inputs["left"]);
  monoRight->out.link(sync->inputs["right"]);
  imu->out.link(xoutIMU->input);

  sync->out.link(xoutGroup->input);
  
  device.startPipeline(pipeline);

  baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();
  firstTs = false;
  //auto groupQueue = device.getOutputQueue("xout", 10, false);
  
}

OAKdSpinner1::~OAKdSpinner1() {
    std::cout << "OakDSpinner destructor called.\n";
}

bool OAKdSpinner1::spin() {
    std::cout << "Spinning. \n";
    if (true){
        while(spinOnce()){
            return true;
        }
    } else {
        return false;
    }
    return true;   
}

bool OAKdSpinner1::spinOnce(){

    std::cout << "Spin Once was called.\n";
    auto groupQueue = device.getOutputQueue("xout", 10, false);
    auto IMUQueue = device.getOutputQueue("imu", 50, false);
    
    
    while(!groupQueue->has()){
        auto imuData = IMUQueue->get<dai::IMUData>();
        for(auto& packet : imuData->packets) {
            //auto& rv = packet.rotationVector;
            auto& acc = packet.acceleroMeter;
            auto& gyr = packet.gyroscope;
            
            auto acceleroTs1 = acc.getTimestamp();
            auto gyroTs1 = gyr.getTimestamp();
            
            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;
            
            auto thisTs = std::min(acceleroTs, gyroTs);
            
            long ts = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(thisTs).count());
            
            Vector6 imu_accgyr;
            imu_accgyr << acc.z, acc.y, acc.x, gyr.z, gyr.y, gyr.x;
            
            std::cout << "A(x): " << acc.z << " A(y): " << acc.y << " A(z): " << acc.x << std::endl;
            LOG(INFO) << "imu_accgyr time: " << ts << std::endl;
                  
        }
    }
    
    auto groupMessage = groupQueue->get<dai::MessageGroup>();
    auto leftData= groupMessage->get<dai::ImgFrame>("left");
    auto rightData= groupMessage->get<dai::ImgFrame>("right");
    auto tDleft=leftData->getTimestamp()-baseTs;
    auto tDright=rightData->getTimestamp()-baseTs;
    
    auto TL=static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDleft).count());
    auto TR=static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(tDright).count());
        
    std::cout << "Left time= " << TL << " Right time= " << TR << std::endl;
    

    return 0;
}



}
