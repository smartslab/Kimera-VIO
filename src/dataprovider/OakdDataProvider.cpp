/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OakdDataProvider.h
 * @brief  Parse Oakd data.
 * @author Antoni Rosinol
 * @author Yun Chang
 * @author Luca Carlone
 */

#include "kimera-vio/dataprovider/OakdDataProvider.h"

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

DEFINE_string(oakd_dataset_path,
              "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_int64(initial_t,
             0,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_t,
             100000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_bool(log_oakd_gt_data,
            false,
            "Log Euroc ground-truth data to file for later evaluation.");

namespace VIO {

/* -------------------------------------------------------------------------- */
OakdDataProvider::OakdDataProvider(const std::string& oakd_dataset_path,
                                     const int& initial_t,
                                     const int& final_t,
                                     const VioParams& vio_params)
    : DataProviderInterface(),
      vio_params_(vio_params),
      oakd_dataset_path_(oakd_dataset_path),
      current_t_(std::numeric_limits<FrameId>::max()),
      initial_t_(initial_t),
      final_t_(final_t),
      imu_measurements_(){
  CHECK(!oakd_dataset_path_.empty())
      << "Dataset path for OakdDataProvider is empty.";

  // Start processing dataset from frame initial_t.
  // Useful to skip a bunch of images at the beginning (imu calibration).
  CHECK_GE(initial_t_, 0);

  // Finish processing dataset at frame final_t.
  // Last frame to process (to avoid processing the entire dataset),
  // skip last frames.
  CHECK_GT(final_t_, 0);

  CHECK_GT(final_t_, initial_t_) << "Value for final_t (" << final_t_
                                 << ") is smaller than value for"
                                 << " initial_t (" << initial_t_ << ").";
  current_t_ = initial_t_;

  // Parse the actual dataset first, then run it.
  if (!shutdown_ && !dataset_parsed_) {
    LOG(INFO) << "Parsing Oakd dataset...";
    parse();
    CHECK_GT(imu_measurements_.size(), 0u);
    dataset_parsed_ = true;
  }
}

/* -------------------------------------------------------------------------- */
OakdDataProvider::OakdDataProvider(const VioParams& vio_params)
    : OakdDataProvider(FLAGS_oakd_dataset_path,
                        FLAGS_initial_t,
                        FLAGS_final_t,
                        vio_params) {}

/* -------------------------------------------------------------------------- */
OakdDataProvider::~OakdDataProvider() {
  LOG(INFO) << "OakdDatasetParser destructor called.";
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::spin() {
  if (dataset_parsed_) {
    if (!is_imu_data_sent_) {
      // First, send all the IMU data. The flag is to avoid sending it several
      // times if we are running in sequential mode.
      if (imu_single_callback_) {
        sendImuData();
      } else {
        LOG(ERROR) << "Imu callback not registered! Not sending IMU data.";
      }
      is_imu_data_sent_ = true;
    }

    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u);
    CHECK_GT(final_t_, initial_t_);
    // We log only the first one, because we may be running in sequential mode.
    LOG_FIRST_N(INFO, 1) << "Running dataset between frame " << initial_t_
                         << " and frame " << final_t_;
    while (!shutdown_ && spinOnce()) {
      if (!vio_params_.parallel_run_) {
        // Return, instead of blocking, when running in sequential mode.
        return true;
      }
    }
  } else {
    LOG(ERROR) << "Oakd dataset was not parsed.";
  }
  LOG_IF(INFO, shutdown_) << "OakdDataProvider shutdown requested.";
  return false;
}

bool OakdDataProvider::hasData() const {
  return current_t_ < final_t_;
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::spinOnce() {
  CHECK_LT(current_t_, std::numeric_limits<FrameId>::max())
      << "Are you sure you've initialized current_t_?";
  if (current_t_ >= final_t_) {
    LOG(INFO) << "Finished spinning Oakd dataset.";
    return false;
  }

  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);
  const bool& equalize_image =
      vio_params_.frontend_params_.stereo_matching_params_.equalize_image_;

  const Timestamp& timestamp_frame_t = timestampAtFrame(current_t_);
  VLOG(10) << "Sending left/right frames k= " << current_t_
           << " with timestamp: " << timestamp_frame_t;

  // TODO(Toni): ideally only send cv::Mat raw images...:
  // - pass params to vio_pipeline ctor
  // - make vio_pipeline actually equalize or transform images as necessary.
  std::string left_img_filename;
  bool available_left_img = getLeftImgName(current_t_, &left_img_filename);
  std::string right_img_filename;
  bool available_right_img = getRightImgName(current_t_, &right_img_filename);
  if (available_left_img && available_right_img) {
    // Both stereo images are available, send data to VIO
    CHECK(left_frame_callback_);
    left_frame_callback_(
        std::make_unique<Frame>(current_t_,
                                timestamp_frame_t,
                                // TODO(Toni): this info should be passed to
                                // the camera... not all the time here...
                                left_cam_info,
                                UtilsOpenCV::ReadAndConvertToGrayScale(
                                    left_img_filename, equalize_image)));
    CHECK(right_frame_callback_);
    right_frame_callback_(
        std::make_unique<Frame>(current_t_,
                                timestamp_frame_t,
                                // TODO(Toni): this info should be passed to
                                // the camera... not all the time here...
                                right_cam_info,
                                UtilsOpenCV::ReadAndConvertToGrayScale(
                                    right_img_filename, equalize_image)));
  } else {
    LOG(ERROR) << "Missing left/right stereo pair, proceeding to the next one.";
  }

  // This is done directly when parsing the Imu data.
  // imu_single_callback_(imu_meas);

  VLOG(10) << "Finished VIO processing for frame k = " << current_t_;
  current_t_++;
  return true;
}

void OakdDataProvider::sendImuData() const {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  Timestamp previous_timestamp = -1;
  for (const ImuMeasurement& imu_meas : imu_measurements_) {
    CHECK_GT(imu_meas.timestamp_, previous_timestamp)
        << "Oakd IMU data is not in chronological order!";
    previous_timestamp = imu_meas.timestamp_;
    imu_single_callback_(imu_meas);
  }
}

/* -------------------------------------------------------------------------- */
void OakdDataProvider::parse() {
  VLOG(100) << "Using dataset path: " << oakd_dataset_path_;
  // Parse the dataset (ETH format).
  parseDataset();
  if (VLOG_IS_ON(1)) print();

}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::parseImuData(const std::string& input_oakd_dataset_path,
                                     const std::string& imuName) {
  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  //#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
  // a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
  std::string filename_data =
      input_oakd_dataset_path + "/mav0/" + imuName + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  LOG_IF(FATAL, !fin.is_open()) << "Cannot open file: " << filename_data;

  // Skip the first line, containing the header.
  std::string line;
  std::getline(fin, line);

  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  double stdDelta = 0;
  double imu_rate_maxMismatch = 0;
  double maxNormAcc = 0, maxNormRotRate = 0;  // only for debugging
  Timestamp previous_timestamp = -1;

  // Read/store imu measurements, line by line.
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    gtsam::Vector6 gyr_acc_data;
    for (int i = 0u; i < gyr_acc_data.size() + 1u; i++) {
      int idx = line.find_first_of(',');
      if (i == 0) {
        timestamp = std::stoll(line.substr(0, idx));
      } else {
        gyr_acc_data(i - 1) = std::stod(line.substr(0, idx));
      }
      line = line.substr(idx + 1);
    }
    CHECK_GT(timestamp, previous_timestamp)
        << "Oakd IMU data is not in chronological order!";
    Vector6 imu_accgyr;
    // Acceleration first!
    imu_accgyr << gyr_acc_data.tail(3), gyr_acc_data.head(3);

    double normAcc = gyr_acc_data.tail(3).norm();
    if (normAcc > maxNormAcc) maxNormAcc = normAcc;

    double normRotRate = gyr_acc_data.head(3).norm();
    if (normRotRate > maxNormRotRate) maxNormRotRate = normRotRate;

    //! Store imu measurements
    imu_measurements_.push_back(ImuMeasurement(timestamp, imu_accgyr));

    if (previous_timestamp != -1) {
      sumOfDelta += (timestamp - previous_timestamp);
      double deltaMismatch =
          std::fabs(static_cast<double>(
                        timestamp - previous_timestamp -
                        vio_params_.imu_params_.nominal_sampling_time_s_) *
                    1e-9);
      stdDelta += std::pow(deltaMismatch, 2);
      imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
      deltaCount += 1u;
    }
    previous_timestamp = timestamp;
  }

  // Converted to seconds.
  VLOG(10) << "IMU rate: "
           << (static_cast<double>(sumOfDelta) /
               static_cast<double>(deltaCount)) *
                  1e-9
           << '\n'
           << "IMU rate std: "
           << std::sqrt(stdDelta / static_cast<double>(deltaCount - 1u)) << '\n'
           << "IMU rate max mismatch: " << imu_rate_maxMismatch << '\n'
           << "Maximum measured rotation rate (norm):" << maxNormRotRate << '\n'
           << "Maximum measured acceleration (norm): " << maxNormAcc;
  fin.close();

  return true;
}


/* -------------------------------------------------------------------------- */
bool OakdDataProvider::parseDataset() {
  // Parse IMU data.
  CHECK(parseImuData(oakd_dataset_path_, kImuName));

  // Parse Camera data.
  CameraImageLists left_cam_image_list;
  CameraImageLists right_cam_image_list;
  parseCameraData(kLeftCamName, &left_cam_image_list);
  if (VLOG_IS_ON(1)) left_cam_image_list.print();
  parseCameraData(kRightCamName, &right_cam_image_list);
  if (VLOG_IS_ON(1)) right_cam_image_list.print();
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  camera_names_.push_back(kLeftCamName);
  camera_names_.push_back(kRightCamName);
  // WARNING Use [x] not .at() because we are adding entries that do not exist.
  camera_image_lists_[kLeftCamName] = left_cam_image_list;
  camera_image_lists_[kRightCamName] = right_cam_image_list;
  // CHECK(sanityCheckCameraData(camera_names_, &camera_image_lists_));

  
  return true;
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::parseCameraData(const std::string& cam_name,
                                        CameraImageLists* cam_list_i) {
  CHECK_NOTNULL(cam_list_i)
      ->parseCamImgList(oakd_dataset_path_ + "/mav0/" + cam_name, "data.csv");
  return true;
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::sanityCheckCameraData(
    const std::vector<std::string>& camera_names,
    std::map<std::string, CameraImageLists>* camera_image_lists) const {
  CHECK_NOTNULL(camera_image_lists);
  CHECK_GT(vio_params_.camera_params_.size(), 0u);
  CHECK_EQ(vio_params_.camera_params_.size(), 2u);
  const auto& left_cam_info = vio_params_.camera_params_.at(0);
  auto& left_img_lists = camera_image_lists->at(camera_names.at(0)).img_lists_;
  auto& right_img_lists = camera_image_lists->at(camera_names.at(1)).img_lists_;
  return sanityCheckCamSize(&left_img_lists, &right_img_lists) &&
         sanityCheckCamTimestamps(
             left_img_lists, right_img_lists, left_cam_info);
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::sanityCheckCamSize(
    CameraImageLists::ImgLists* left_img_lists,
    CameraImageLists::ImgLists* right_img_lists) const {
  CHECK_NOTNULL(left_img_lists);
  CHECK_NOTNULL(right_img_lists);
  size_t nr_left_cam_imgs = left_img_lists->size();
  size_t nr_right_cam_imgs = right_img_lists->size();
  if (nr_left_cam_imgs != nr_right_cam_imgs) {
    LOG(WARNING) << "Different number of images in left and right camera!\n"
                 << "Left: " << nr_left_cam_imgs << "\n"
                 << "Right: " << nr_right_cam_imgs;
    size_t nrCommonImages = std::min(nr_left_cam_imgs, nr_right_cam_imgs);
    left_img_lists->resize(nrCommonImages);
    right_img_lists->resize(nrCommonImages);
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::sanityCheckCamTimestamps(
    const CameraImageLists::ImgLists& left_img_lists,
    const CameraImageLists::ImgLists& right_img_lists,
    const CameraParams& left_cam_info) const {
  double stdDelta = 0.0;
  double frame_rate_maxMismatch = 0.0;
  size_t deltaCount = 0u;
  for (size_t i = 0; i < left_img_lists.size(); i++) {
    if (i > 0) {
      deltaCount++;
      const Timestamp& timestamp = left_img_lists.at(i).first;
      const Timestamp& previous_timestamp = left_img_lists.at(i - 1).first;
      // TODO(TONI): this looks horrible.
      double deltaMismatch =
          std::fabs(static_cast<double>(timestamp - previous_timestamp -
                                        left_cam_info.frame_rate_) *
                    1e-9);
      stdDelta += pow(deltaMismatch, 2);
      frame_rate_maxMismatch = std::max(frame_rate_maxMismatch, deltaMismatch);
    }

    LOG_IF(FATAL, left_img_lists.at(i).first != right_img_lists.at(i).first)
        << "Different timestamp for left and right image!\n"
        << "left: " << left_img_lists.at(i).first << '\n'
        << "right: " << right_img_lists.at(i).first << '\n'
        << " for image " << i << " of " << left_img_lists.size();
  }

  CHECK_NE(deltaCount - 1, 0);
  LOG(INFO) << "nominal frame rate: " << left_cam_info.frame_rate_ << '\n'
            << "frame rate std: "
            << std::sqrt(stdDelta / static_cast<double>(deltaCount - 1u))
            << '\n'
            << "frame rate maxMismatch: " << frame_rate_maxMismatch;
  return true;
}

std::string OakdDataProvider::getDatasetName() {
  if (dataset_name_.empty()) {
    // Find and store actual name (rather than path) of the dataset.
    size_t found_last_slash = oakd_dataset_path_.find_last_of("/\\");
    std::string oakd_dataset_path_tmp = oakd_dataset_path_;
    dataset_name_ = oakd_dataset_path_tmp.substr(found_last_slash + 1);
    // The dataset name has a slash at the very end
    if (found_last_slash >= oakd_dataset_path_tmp.size() - 1) {
      // Cut the last slash.
      oakd_dataset_path_tmp = oakd_dataset_path_tmp.substr(0, found_last_slash);
      // Repeat the search.
      found_last_slash = oakd_dataset_path_tmp.find_last_of("/\\");
      // Try to pick right name.
      dataset_name_ = oakd_dataset_path_tmp.substr(found_last_slash + 1);
    }
    LOG(INFO) << "Dataset name: " << dataset_name_;
  }
  return dataset_name_;
}




/* -------------------------------------------------------------------------- */
size_t OakdDataProvider::getNumImages() const {
  CHECK_GT(camera_names_.size(), 0u);
  const std::string& left_cam_name = camera_names_.at(0);
  const std::string& right_cam_name = camera_names_.at(0);
  size_t n_left_images = getNumImagesForCamera(left_cam_name);
  size_t n_right_images = getNumImagesForCamera(right_cam_name);
  CHECK_EQ(n_left_images, n_right_images);
  return n_left_images;
}

/* -------------------------------------------------------------------------- */
size_t OakdDataProvider::getNumImagesForCamera(
    const std::string& camera_name) const {
  const auto& iter = camera_image_lists_.find(camera_name);
  CHECK(iter != camera_image_lists_.end());
  return iter->second.getNumImages();
}

/* -------------------------------------------------------------------------- */
bool OakdDataProvider::getImgName(const std::string& camera_name,
                                   const size_t& k,
                                   std::string* img_filename) const {
  CHECK_NOTNULL(img_filename);
  const auto& iter = camera_image_lists_.find(camera_name);
  CHECK(iter != camera_image_lists_.end());
  const auto& img_lists = iter->second.img_lists_;
  if (k < img_lists.size()) {
    *img_filename = img_lists.at(k).second;
    return true;
  } else {
    LOG(ERROR) << "Requested image #: " << k << " but we only have "
               << img_lists.size() << " images.";
  }
  return false;
}

/* -------------------------------------------------------------------------- */
Timestamp OakdDataProvider::timestampAtFrame(const FrameId& frame_number) {
  CHECK_GT(camera_names_.size(), 0);
  //CHECK_LT(frame_number,
  //         camera_image_lists_.at(camera_names_[0]).img_lists_.size());
  return camera_image_lists_.at(camera_names_[0])
      .img_lists_[frame_number]
      .first;
}

void OakdDataProvider::clipFinalFrame() {
  // Clip final_t_ to the total number of images.
  const size_t& nr_images = getNumImages();
  if (final_t_ > nr_images) {
    LOG(WARNING) << "Value for final_t, " << final_t_ << " is larger than total"
                 << " number of frames in dataset " << nr_images;
    final_t_ = nr_images;
    LOG(WARNING) << "Using final_t = " << final_t_;
  }
  CHECK_LE(final_t_, nr_images);
}
/* -------------------------------------------------------------------------- */
void OakdDataProvider::print() const {
  LOG(INFO) << "------------------ ETHDatasetParser::print ------------------\n"
            << "Displaying info for dataset: " << oakd_dataset_path_;
  // For each of the 2 cameras.
  CHECK_EQ(vio_params_.camera_params_.size(), camera_names_.size());
  for (size_t i = 0; i < camera_names_.size(); i++) {
    LOG(INFO) << "\n"
              << (i == 0 ? "Left" : "Right")
              << " camera name: " << camera_names_[i] << ", with params:\n";
    vio_params_.camera_params_.at(i).print();
    camera_image_lists_.at(camera_names_[i]).print();
  }
  
  LOG(INFO) << "-------------------------------------------------------------";
}


}  // namespace VIO
