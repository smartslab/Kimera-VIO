/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OakdDataProvider.h
 * @brief  Parse Oakd dataset.
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

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class OakdDataProvider : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(OakdDataProvider);
  KIMERA_POINTER_TYPEDEFS(OakdDataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  OakdDataProvider(const std::string& oakd_dataset_path,
                    const int& initial_t,
                    const int& final_t,
                    const VioParams& vio_params);
  //! Ctor from gflags
  explicit OakdDataProvider(const VioParams& vio_params);

  virtual ~OakdDataProvider();

 public:
  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

  virtual bool hasData() const override;

  /**
   * @brief print Print info about dataset.
   */
  void print() const;


 public:
  inline std::string getDatasetPath() const {
    return oakd_dataset_path_;
  }
  std::string getDatasetName();

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  virtual bool spinOnce();

  /**
   * @brief parse Parses Oakd dataset. This is done already in spin() and
   * does not need to be called by the user. Left in public for experimentation.
   */
  void parse();

  /**
   * @brief sendImuData We send IMU data first (before frames) so that the VIO
   * pipeline can query all IMU data between frames.
   */
  void sendImuData() const;

  /**
   * @brief parseDataset Parse camera, gt, and imu data if using
   * different Oakd format.
   * @return
   */
  bool parseDataset();

  //! Parsers
  bool parseImuData(const std::string& input_oakd_dataset_path,
                    const std::string& imu_name);

  
  bool parseCameraData(const std::string& cam_name,
                       CameraImageLists* cam_list_i);

  //! Getters.
  /**
   * @brief getLeftImgName returns the img filename given the frame number
   * @param[in] k frame number
   * @param[out] img_name returned filename of the img
   * @return if k is larger than the number of frames, returns false, otw true.
   */
  inline bool getLeftImgName(const size_t& k, std::string* img_name) const {
    return getImgName("cam0", k, img_name);
  }
  /**
   * @brief getLeftImgName returns the img filename given the frame number
   * @param[in] k frame number
   * @param[out] img_name returned filename of the img
   * @return if k is larger than the number of frames, returns false, otw true.
   */
  inline bool getRightImgName(const size_t& k, std::string* img_name) const {
    return getImgName("cam1", k, img_name);
  }

  // Retrieve relative pose between timestamps.
  

  size_t getNumImages() const;
  size_t getNumImagesForCamera(const std::string& camera_name) const;
  /**
   * @brief getImgName returns the img filename given the frame number
   * @param[in] camera_name camera id such as "cam0"/"cam1"
   * @param[in] k frame number
   * @param[out] img_filename returned filename of the img
   * @return if k is larger than the number of frames, returns false, otw true.
   */
  bool getImgName(const std::string& camera_name,
                         const size_t& k,
                         std::string* img_filename) const;

  //! Sanity checks
  bool sanityCheckCameraData(
      const std::vector<std::string>& camera_names,
      std::map<std::string, CameraImageLists>* camera_image_lists) const;

  // Sanity check: nr images is the same for left and right camera
  // Resizes left/right img lists to the minimum number of frames in case of
  // different list sizes.
  bool sanityCheckCamSize(CameraImageLists::ImgLists* left_img_lists,
                          CameraImageLists::ImgLists* right_img_lists) const;

  // Sanity check: time stamps are the same for left and right camera
  bool sanityCheckCamTimestamps(
      const CameraImageLists::ImgLists& left_img_lists,
      const CameraImageLists::ImgLists& right_img_lists,
      const CameraParams& left_cam_info) const;

  //! Utilities
  // Get timestamp of a given pair of stereo images (synchronized).
  Timestamp timestampAtFrame(const FrameId& frame_number);

  // Clip final frame to the number of images in the dataset.
  void clipFinalFrame();

 protected:
  VioParams vio_params_;

  /// Images data.
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  // This matches the names of the folders in the dataset
  std::vector<std::string> camera_names_;
  // Map from camera name to its images
  std::map<std::string, CameraImageLists> camera_image_lists_;

  
  std::string dataset_name_;
  std::string oakd_dataset_path_;

  FrameId current_t_;
  FrameId initial_t_;  // start frame
  FrameId final_t_;    // end frame

  //! Flag to signal when the dataset has been parsed.
  bool dataset_parsed_ = false;
  //! Flag to signal if the IMU data has been sent to the VIO pipeline
  bool is_imu_data_sent_ = false;

  const std::string kLeftCamName = "cam0";
  const std::string kRightCamName = "cam1";
  const std::string kImuName = "imu0";

  //! Pre-stored imu-measurements
  std::vector<ImuMeasurement> imu_measurements_;

};


}  // namespace VIO
