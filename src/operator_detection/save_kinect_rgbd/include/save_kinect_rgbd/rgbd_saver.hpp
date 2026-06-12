#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

namespace save_kinect_rgbd
{

class RgbdSaver
{
public:
  explicit RgbdSaver(const std::string & output_directory);

  /// Throws std::runtime_error if a write fails.
  void save_pair(const cv::Mat & rgb_image, const cv::Mat & depth_image) const;

  int experiment_number() const;
  const std::string & rgb_path() const;
  const std::string & depth_path() const;

private:
  static int find_next_number(const std::string & directory);

  int experiment_number_;
  std::string rgb_path_;
  std::string depth_path_;
};

}  // namespace save_kinect_rgbd