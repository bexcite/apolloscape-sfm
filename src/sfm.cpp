
#include <algorithm>

#include "cv_gl/utils.hpp"
#include "cv_gl/sfm.h"

#include <boost/filesystem.hpp>




#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>



// ========== SfM3D =============
void SfM3D::AddImages(const std::vector<ImageData>& camera1_images,
                 const std::vector<ImageData>& camera2_images,
                 const bool make_pairs, const int look_back) {
  assert(camera1_images.size() == camera2_images.size());
  const int first_index = image_data_.size();
  for (size_t i = 0; i < camera1_images.size(); ++i) {
    image_data_.push_back(camera1_images[i]);
    image_data_.push_back(camera2_images[i]);

    CameraInfo camera1_info, camera2_info;

    camera1_info.intr = intrinsics_[camera1_images[i].camera_num - 1];
    camera1_info.rotation_angles[0] = camera1_images[i].coords[0];
    camera1_info.rotation_angles[1] = camera1_images[i].coords[1];
    camera1_info.rotation_angles[2] = camera1_images[i].coords[2];
    camera1_info.translation[0] = camera1_images[i].coords[3];
    camera1_info.translation[1] = camera1_images[i].coords[4];
    camera1_info.translation[2] = camera1_images[i].coords[5];

    camera2_info.intr = intrinsics_[camera2_images[i].camera_num - 1];
    camera2_info.rotation_angles[0] = camera2_images[i].coords[0];
    camera2_info.rotation_angles[1] = camera2_images[i].coords[1];
    camera2_info.rotation_angles[2] = camera2_images[i].coords[2];
    camera2_info.translation[0] = camera2_images[i].coords[3];
    camera2_info.translation[1] = camera2_images[i].coords[4];
    camera2_info.translation[2] = camera2_images[i].coords[5];

    cameras_.push_back(camera1_info);
    cameras_.push_back(camera2_info);
  }

  if (make_pairs) {
    // Add-hoc method to find pairs if the sorted and look back
    for (int i = first_index; i < image_data_.size(); i += 2) {
      image_pairs_.push_back({ i, i + 1 });
      for (int j = i - 2; j >= std::max(first_index, i - look_back * 2); j -= 2) {
        image_pairs_.push_back({ i, j });
        image_pairs_.push_back({ i, j + 1 });
      }
    }
    for (auto ip : image_pairs_) {
      std::cout << "pair = " << ip << std::endl;
    }
  }
}

void SfM3D::ExtractFeatures() {
  std::cout << "SfM3D: Extract Features\n";

  images_.clear();
  image_features_.clear();

  assert(image_data_.size() == cameras_.size());
  for (size_t i = 0; i < image_data_.size(); ++i) {
    ImageData& im_data = image_data_[i];
    boost::filesystem::path full_image_path = boost::filesystem::path(im_data.image_dir)
        / boost::filesystem::path(im_data.filename);
    cv::Mat img = cv::imread(full_image_path.string().c_str());
    std::cout << "full_image_path = " << full_image_path.string() << std::endl;
    Features features;
    ::ExtractFeatures(img, features);
    images_.push_back(img);
    image_features_.push_back(features);
  }
  

}

void SfM3D::Print(std::ostream& os) const {
  os << "SfM3D: intrinsics_.size = " << intrinsics_.size() << ", "
     << "image_data_.size = " << image_data_.size() << ", "
     << "image_pairs_.size = " << image_pairs_.size() << ", "
     << "cameras_.size = " << cameras_.size() << ", "
     << "images_.size = " << images_.size() << ", "
     << "image_features_.size = " << image_features_.size()
     << "mats.size = " << mats.size();
    //  << std::endl;
}

std::ostream& operator<<(std::ostream& os, const SfM3D& sfm) {
  sfm.Print(os);
  return os;
}

 