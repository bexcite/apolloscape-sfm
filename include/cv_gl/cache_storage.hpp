// Copyright Pavlo 2018
#ifndef CV_GL_CACHE_STORAGE_H_
#define CV_GL_CACHE_STORAGE_H_

#include <boost/filesystem.hpp>

#include <string>
// #include <cereal/cereal.hpp>
// #include <cereal/types/vector.hpp>

#include "cv_gl/serialization.hpp"
#include "cv_gl/sfm_common.h"

#include <cereal/archives/binary.hpp>


#define CACHE_FEATURES_DIR  "features"
#define CACHE_MATCHES_DIR  "matches"

class CacheStorage {
public:
  explicit CacheStorage() : cache_dir_{"_features_cache"} {
    Init();
  }
  explicit CacheStorage(std::string cache_dir)
      : cache_dir_{cache_dir} {
    Init();
  }

  bool GetFeatures(const std::string& img_path, Features& features) {
    // std::cout << "GetFeatures: " << img_path << std::endl;
    boost::filesystem::path p(img_path);
    // std::cout << "p.size = " << p.size() << std::endl;
    // std::cout << "p.stem = " << p.stem() << std::endl;
    auto camera_path = p.parent_path();
    auto record_path = camera_path.parent_path();
    std::string feature_file = p.stem().string() + ".f";
    boost::filesystem::path cache_file(cache_dir_);
    cache_file /= CACHE_FEATURES_DIR / record_path.stem()
        / camera_path.stem() / feature_file;
    // std::cout << "cache_file = " << cache_file << std::endl;
    if (boost::filesystem::exists(cache_file)
        && boost::filesystem::is_regular_file(cache_file)) {
      // Open and de-serialize features
      std::ifstream file(cache_file.string(), std::ios::binary);
      cereal::BinaryInputArchive archive(file);
      archive(features);
      return true;
    }
    // std::cout << "pc2.stem = " << pc2.stem() << std::endl;
    // for (auto c : p) {
    //   std::cout << "c = " << c << std::endl;
    // }
    return false;
  }

  void SaveFeatures(const std::string& img_path, const Features& features) {
    // std::cout << "SaveFeatures: " << img_path << std::endl;
    boost::filesystem::path p(img_path);
    auto camera_path = p.parent_path();
    auto record_path = camera_path.parent_path();
    std::string feature_file = p.stem().string() + ".f";
    boost::filesystem::path cache_file_dir(cache_dir_);
    cache_file_dir /= CACHE_FEATURES_DIR / record_path.stem()
        / camera_path.stem();
    // std::cout << "cache_file_dir = " << cache_file_dir << std::endl;
    if (!boost::filesystem::exists(cache_file_dir)) {
      boost::filesystem::create_directories(cache_file_dir);
    }
    boost::filesystem::path cache_file = cache_file_dir / feature_file;
    if (boost::filesystem::exists(cache_file)
        && boost::filesystem::is_regular_file(cache_file)) {
      // Open and de-serialize features
      std::cout << "CacheFile EXISTS - STRANGE!!!!\n";
      boost::filesystem::remove(cache_file);
    }

    // Store Features
    std::ofstream file(cache_file.string(), std::ios::binary);
    cereal::BinaryOutputArchive archive(file);
    archive(features);
  }

  bool GetImageMatches(const ImageData& im_data1, 
                       const ImageData& im_data2, 
                       Matches& matches) {

    // std::cout << "GetMatches: " << im_data1 << ", "
    //           << im_data2 << std::endl;

    assert(::IsPairInOrder(im_data1, im_data2));

    // Create match file <record>_<cam_num>_<file_name>
    auto im1_stem = boost::filesystem::path(im_data1.filename).stem();
    auto im2_stem = boost::filesystem::path(im_data2.filename).stem();

    std::string cache1_name = im_data1.record + "_"
                            + std::to_string(im_data1.camera_num) + "_"
                            + im1_stem.string();
    std::string cache2_name = im_data2.record + "_"
                            + std::to_string(im_data2.camera_num) + "_"
                            + im2_stem.string();

    // std::cout << "cache1_name = " << cache1_name << std::endl;
    // std::cout << "cache2_name = " << cache2_name << std::endl;

    // Concatenate in lexicographical order
    std::string cache_name = cache1_name + "_" + cache2_name + ".m";
    // std::cout << "cache_name = " << cache_name << std::endl;

    boost::filesystem::path cache_file_dir(cache_dir_);
    cache_file_dir /= CACHE_MATCHES_DIR;
    // std::cout << "cache_file_dir = " << cache_file_dir << std::endl;

    boost::filesystem::path cache_file(cache_file_dir);
    cache_file /= cache_name;
    // std::cout << "cache_file = " << cache_file << std::endl;
    
    if (boost::filesystem::exists(cache_file)
        && boost::filesystem::is_regular_file(cache_file)) {
      // Open and de-serialize matches
      // std::cout << "restore matches from CACHE" << std::endl;
      std::ifstream file(cache_file.string(), std::ios::binary);
      cereal::BinaryInputArchive archive(file);
      archive(matches);
      return true;
    }

    return false;

  }

  void SaveImageMatches(const ImageData& im_data1,
                        const ImageData& im_data2,
                        const Matches& matches) {

    // std::cout << "SaveMatches: " << im_data1 << ", "
    //           << im_data2 << std::endl;

    assert(::IsPairInOrder(im_data1, im_data2));

    // Create match file <record>_<cam_num>_<file_name>
    auto im1_stem = boost::filesystem::path(im_data1.filename).stem();
    auto im2_stem = boost::filesystem::path(im_data2.filename).stem();

    std::string cache1_name = im_data1.record + "_"
                            + std::to_string(im_data1.camera_num) + "_"
                            + im1_stem.string();
    std::string cache2_name = im_data2.record + "_"
                            + std::to_string(im_data2.camera_num) + "_"
                            + im2_stem.string();

    // std::cout << "cache1_name = " << cache1_name << std::endl;
    // std::cout << "cache2_name = " << cache2_name << std::endl;

    // Concatenate in lexicographical order
    std::string cache_name = cache1_name + "_" + cache2_name + ".m";
    // std::cout << "cache_name = " << cache_name << std::endl;

    boost::filesystem::path cache_file_dir(cache_dir_);
    cache_file_dir /= CACHE_MATCHES_DIR;
    // std::cout << "cache_file_dir = " << cache_file_dir << std::endl;

    boost::filesystem::path cache_file(cache_file_dir);
    cache_file /= cache_name;
    // std::cout << "cache_file = " << cache_file << std::endl;

    if (!boost::filesystem::exists(cache_file_dir)) {
      boost::filesystem::create_directories(cache_file_dir);
    }
    if (boost::filesystem::exists(cache_file)
        && boost::filesystem::is_regular_file(cache_file)) {
      // Open and de-serialize features
      std::cout << "CacheFile for Matches EXISTS - STRANGE!!!!\n";
      boost::filesystem::remove(cache_file);
    }

    // Store Matches
    std::ofstream file(cache_file.string(), std::ios::binary);
    cereal::BinaryOutputArchive archive(file);
    archive(matches);

  }


private:
  std::string cache_dir_;
  void Init() {
    // Create cache dir if needed
    std::cout << "Cache Storage: Init\n";
    boost::filesystem::create_directories(cache_dir_);
  }

};


#endif  // CV_GL_CACHE_STORAGE_H_
