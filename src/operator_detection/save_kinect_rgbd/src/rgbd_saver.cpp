#include "save_kinect_rgbd/rgbd_saver.hpp"

#include <algorithm>
#include <filesystem>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace save_kinect_rgbd
{

static const std::string RGB_FILENAME_STEM = "kinect_rgb_";
static const std::string DEPTH_FILENAME_STEM = "kinect_depth_";
static constexpr int JPEG_QUALITY = 95;
static constexpr int PNG_COMPRESSION_LEVEL = 3;

RgbdSaver::RgbdSaver(const std::string & output_directory)
: experiment_number_(find_next_number(output_directory)),
  rgb_path_(
    output_directory + '/' + RGB_FILENAME_STEM + std::to_string(experiment_number_) + ".jpg"),
  depth_path_(
    output_directory + '/' + DEPTH_FILENAME_STEM + std::to_string(experiment_number_) + ".png")
{
}

void RgbdSaver::save_pair(const cv::Mat & rgb_image, const cv::Mat & depth_image) const
{
  if (!cv::imwrite(rgb_path_, rgb_image, {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY})) {
    throw std::runtime_error("Failed to write RGB image to " + rgb_path_);
  }
  if (
    !cv::imwrite(depth_path_, depth_image, {cv::IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION_LEVEL}))
  {
    throw std::runtime_error("Failed to write depth image to " + depth_path_);
  }
}

int RgbdSaver::experiment_number() const
{
  return experiment_number_;
}

const std::string & RgbdSaver::rgb_path() const
{
  return rgb_path_;
}

const std::string & RgbdSaver::depth_path() const
{
  return depth_path_;
}

int RgbdSaver::find_next_number(const std::string & directory)
{
  namespace fs = std::filesystem;

  if (!fs::exists(directory)) {
    return 1;
  }

  std::regex pattern(RGB_FILENAME_STEM + "(\\d+)\\.jpg");
  std::vector<int> used_numbers;

  for (const auto & entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string filename = entry.path().filename().string();
    std::smatch match;
    if (std::regex_match(filename, match, pattern)) {
      used_numbers.push_back(std::stoi(match[1]));
    }
  }

  if (used_numbers.empty()) {
    return 1;
  }

  return *std::max_element(used_numbers.begin(), used_numbers.end()) + 1;
}

}  // namespace save_kinect_rgbd