/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LiveDataProvider.h
 * @brief  Parse Live dataset.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <cstdio>
#include <stdlib.h>

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class LiveDataProvider : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(LiveDataProvider);
  KIMERA_POINTER_TYPEDEFS(LiveDataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  LiveDataProvider(const VioParams& vio_params);
  //! Ctor from gflags
  //explicit LiveDataProvider(const VioParams& vio_params);

  virtual ~LiveDataProvider();

 public:
  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

  virtual bool hasData() const override;
  
  dai::Device device;
  /**
   * @brief print Print info about dataset.
   */
  void print() const;

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  virtual bool spinOnce();

  /**
   * @brief parse Parses Live dataset. This is done already in spin() and
   * does not need to be called by the user. Left in public for experimentation.
   */
  void parse();

  /**
   * @brief sendImuData We send IMU data first (before frames) so that the VIO
   * pipeline can query all IMU data between frames.
   */
  void sendImuData() const;
  
  

  

 protected:
  VioParams vio_params_;

  //! Flag to signal if the IMU data has been sent to the VIO pipeline
  bool is_imu_data_sent_ = false;

  const std::string kLeftCamName = "cam0";
  const std::string kRightCamName = "cam1";
  const std::string kImuName = "imu0";
  
  FrameId current_tt_;
  
  
  std::chrono::time_point<std::chrono::steady_clock> baseTs;
  bool firstTs;
  bool setup;

  
  //! Pre-stored imu-measurements
  std::vector<ImuMeasurement> imu_measurements_;

};


}  // namespace VIO
