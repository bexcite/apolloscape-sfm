
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "cv_gl/utils.h"
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
    // Add-hoc method to find pairs if they sorted cameras moving backwards
    for (int i = first_index; i < image_data_.size(); i += 2) {
      image_pairs_.push_back({ i, i + 1 });
      for (int j = i - 2; j >= std::max(first_index, i - look_back * 2); j -= 2) {
        image_pairs_.push_back({ i, j });
        image_pairs_.push_back({ i, j + 1 });
      }
    }
    std::cout << "pairs.size = " << image_pairs_.size() << std::endl;

    // TODO: Look for conics intersections?
    if (first_index > 0) {

      // Add pair based on distance < 11m * n (works for apolloscape case only)
      for (int i = 0; i < first_index - 1; ++i) {
        for (int j = first_index; j < cameras_.size(); ++j) {
          double d = ::GetCamerasDistance(cameras_[i], cameras_[j]);
          if (d < look_back * 11.0) {
            image_pairs_.push_back({ i, j });
            // std::cout << "d = " << d << ": added pair ["
            //           << i << ", " << j << "]"
            //           << std::endl;
          }
        }
      }

    }

    // for (auto ip : image_pairs_) {
    //   std::cout << "pair = " << ip << std::endl;
    // }
  }
}

void SfM3D::ExtractFeatures() {
  std::cout << "SfM3D: Extract Features\n";

  images_resized_.clear();
  image_features_.clear();

  assert(image_data_.size() == cameras_.size());


  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  // bool tst = false;

  
  int capacity = std::max(
      static_cast<int>(std::thread::hardware_concurrency() - 2), 1);
  std::cout << "image_data_.size = " << image_data_.size() << std::endl;
  capacity = std::min(capacity, static_cast<int>(image_data_.size()));
  std::cout << "Extract features concurrency = " << capacity << std::endl;

  image_features_.resize(image_data_.size());
  images_resized_.resize(image_data_.size());

  std::atomic<int> next_idx(0);
  std::vector<std::thread> extractor_threads;

  std::mutex cout_mu;
  std::mutex acc_mu;
  

  for (int i = 0; i < capacity; ++i) {
    auto extractor = [this, &next_idx, &cout_mu](int thread_id) {
      int idx;
      while ((idx = next_idx++) < image_data_.size()) {

        std::stringstream ss;
        
        ss << "[th:" << thread_id << "] Extract " << idx << " out of " 
           << image_data_.size();
        

        ImageData& im_data = image_data_[idx];
        boost::filesystem::path full_image_path = boost::filesystem::path(im_data.image_dir)
        / boost::filesystem::path(im_data.filename);
        cv::Mat img = ::LoadImage(im_data);

        Features features;

        // TODO: Refactor to use ImageData
        if (!cache_storage.GetFeatures(full_image_path.string(),
                                      features)) {
          ss << ": extracted ...";
          ::ExtractFeatures(img, features);
          cache_storage.SaveFeatures(full_image_path.string(),
                                    features);
          ss << " cached ...";
        } else {
          ss << ": restored from cache";
        }
        ss << std::endl;

        cv::resize(img, img, cv::Size(), resize_scale, resize_scale);
        images_resized_[idx] = img;
        image_features_[idx] = features;

        cout_mu.lock();
        std::cout << ss.str();
        cout_mu.unlock();

      }
    };
    extractor_threads.push_back(std::thread(extractor, i));
  }

  for(int i = 0; i < capacity; ++i) {
    extractor_threads[i].join();
  }
  
  auto t1 = high_resolution_clock::now();
  auto dur = duration_cast<microseconds>(t1 - t0);
  std::cout << "EXTRACT_FEATURES_TIME = " << dur.count() / 1e+6 << std::endl;


  // ShowFeatures(128); // Records008
  // for (int i = 0; i < image_features_.size(); i += 2) {
  //   ShowFeatures(i);
  // }
  
}

void SfM3D::MatchImageFeatures(const int skip_thresh,
                               const double max_line_dist,
                               const bool use_cache) {
  assert(image_features_.size() > 1);
  if (image_pairs_.empty()) {
    // Fall back case when pair are not generated on AddImages
    GenerateAllPairs();
  }

  // Check duplicates
  // for (int i = 0; i < image_pairs_.size(); ++i) {
  //   if (image_pairs_[i].first > image_pairs_[i].second) {
  //     std::cout << "swap " << i << " : " << image_pairs_[i] << std::endl;
  //     std::swap(image_pairs_[i].first, image_pairs_[i].second);
  //   }
  // }

  // std::set<ImagePair> ipset(image_pairs_.begin(), image_pairs_.end());

  // std::sort(image_pairs_.begin(), image_pairs_.end());
  // for (int i = 0; i < image_pairs_.size() - 1; ++i) {
  //   if (image_pairs_[i] == image_pairs_[i+1]) {
  //     std::cout << "dup " << image_pairs_[i] << std::endl;
  //   }
  // }

  // return;

  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  
  int total_matched_points = 0;
  int skipped_matches = 0;
  int filtered_by_distance = 0;

  
  int capacity = std::max(
      static_cast<int>(std::thread::hardware_concurrency() / 2 - 2), 1);
  std::cout << "image_pairs_.size = " << image_pairs_.size() << std::endl;
  capacity = std::min(capacity, static_cast<int>(image_pairs_.size()));
  std::cout << "Concurrency = " << capacity << std::endl;


  std::atomic<int> next_idx(0);
  std::vector<std::thread> matcher_threads;

  std::mutex cout_mu;
  std::mutex acc_mu;

  // capacity = 2;

  for (int i = 0; i < capacity; ++i) {
    auto matcher = [this, &next_idx, &cout_mu, &acc_mu,
                    &skipped_matches, &total_matched_points,
                    &filtered_by_distance,
                    skip_thresh, use_cache, max_line_dist
                    ](int thread_id) {
      int idx;
      while ((idx = next_idx++) < image_pairs_.size()) {

        // cout_mu.lock();
        // std::cout << "thread " << thread_id << " : "
        //           << "process " << idx 
        //           << " = " << image_pairs_[idx]
        //           << std::endl;
        // cout_mu.unlock();

        
        ImagePair ip = image_pairs_[idx];

        int img_first = ip.first;
        int img_second = ip.second;

        if (!IsPairInOrder(img_first, img_second)) {
          std::swap(img_first, img_second);
        }

        ImageData& im_data1 = image_data_[img_first];
        ImageData& im_data2 = image_data_[img_second];
        Features& features1 = image_features_[img_first];
        Features& features2 = image_features_[img_second];
        CameraInfo& camera_info1 = cameras_[img_first];
        CameraInfo& camera_info2 = cameras_[img_second];

        Matches matches;
        matches.image_index.first = img_first;
        matches.image_index.second = img_second;

        bool from_cache = true;
        // bool tst = true;
        if (use_cache) {
          if(!cache_storage.GetImageMatches(im_data1, im_data2, matches)) {
            from_cache = false;
            ::ComputeLineKeyPointsMatch(features1, camera_info1, features2, camera_info2, matches);
            // Save to Cache
            cache_storage.SaveImageMatches(im_data1, im_data2, matches);
          }
        } else {
          from_cache = false;
          ::ComputeLineKeyPointsMatch(features1, camera_info1, features2, camera_info2, matches);
        }

        int msize = matches.match.size();

        // ShowMatchesLineConstraints(matches, max_line_dist);

        // Filter Matches base on Line Distance
        FilterMatchByLineDistance(features1, camera_info1,
                                  features2, camera_info2,
                                  matches, max_line_dist);

        filtered_by_distance += msize - matches.match.size();

        // Restore indexes to the current run
        matches.image_index.first = img_first;
        matches.image_index.second = img_second;    

        // cout_mu.lock();
        // std::cout << "thread " << thread_id << " : ";
        // std::cout << "Match: "
        //           << "(" << img_first << ", " << img_second << ")"
        //           << " [" << (from_cache ? "R" : "C") << "]"
        //           << ": matches.size = " << matches.match.size()
        //           << std::endl;
        // cout_mu.unlock();
                  // << std::endl;

        // Don't add empty or small matches
        if (matches.match.size() < skip_thresh) { 
          // std::lock_guard<std::mutex> lck(acc_mu);
          acc_mu.lock();
          ++skipped_matches;
          acc_mu.unlock();
          
          cout_mu.lock();
          std::cout << "[th:" << thread_id << "] ";
          std::cout << "Mtch:"
                  // << " (" << img_first << "," << img_second << ")"
                  << " " << idx << " out of " << image_pairs_.size()
                  << " [" << (from_cache ? "R" : "C") << "]"
                  << ": matches.size = " << matches.match.size();
                  // << std::endl;
          std::cout << ", skipped ...\n";
          cout_mu.unlock();
          continue;
        }


        acc_mu.lock();
        image_matches_.push_back(matches);

        // put index into matches_index_
        auto p1 = std::make_pair(img_first, img_second);
        auto p2 = std::make_pair(img_second, img_first);
        matches_index_.insert(
            std::make_pair(p1, image_matches_.size() - 1));
        matches_index_.insert(
            std::make_pair(p2, image_matches_.size() - 1));

        int mid = image_matches_.size() - 1;
        int tmp = total_matched_points;

        // Connect keypoints and images for a quick retrieval later
        for (int i = 0; i < matches.match.size(); ++i) {
          IntPair p1 = std::make_pair(
              matches.image_index.first,
              matches.match[i].queryIdx);
          IntPair p2 = std::make_pair(
              matches.image_index.second, 
              matches.match[i].trainIdx);
          // std::cout << "union: " << p1 << ", " << p2 << std::endl;
          ccomp_.Union(p1, p2);
        }

        acc_mu.unlock();

        cout_mu.lock();
        total_matched_points += matches.match.size();
        std::cout << "[th:" << thread_id << "] ";
        std::cout << "Mtch:"
                  // << " (" << img_first << "," << img_second << ")"
                  << " " << idx << " out of " << image_pairs_.size()
                  << " [" << (from_cache ? "R" : "C") << "]"
                  << ": matches.size = " << matches.match.size();
                  // << std::endl;
        std::cout << ", total_matched_points = " << total_matched_points 
                  << ", id: " << image_matches_.size() - 1
                  << std::endl;
        // std::cout << "thread " << thread_id << " : waiting\n";
        cout_mu.unlock();
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      }
      // cout_mu.lock();
      // std::cout << "thread " << thread_id << " : FINISHED! " << std::endl;
      // cout_mu.unlock();
    };
    matcher_threads.push_back(std::thread(matcher, i));
  }

  for (int i = 0; i < capacity; ++i) {
    matcher_threads[i].join();
  }  
  

  // Show Matches
  // for (int i = 0; i < image_matches_.size(); ++i) {
  //   std::cout << "Show matches: " << i 
  //             << ", match.size = " << image_matches_[i].match.size()
  //             << std::endl; 
  //   ShowMatches(image_matches_[i]);
  //   // ShowMatchesLineConstraints(image_matches_[i], max_line_dist);
  // }
  

  
  // int most_match = 0;
  // int most_match_id = -1;
  /*
  for (const ImagePair& ip : image_pairs_) {
    int img_first = ip.first;
    int img_second = ip.second;

    if (!IsPairInOrder(img_first, img_second)) {
      // std::cout << "SWAP pair " << img_first << ", " << img_second << std::endl;
      std::swap(img_first, img_second);
      // std::cout << "SWAP res " << img_first << ", " << img_second << std::endl;
    }

    ImageData& im_data1 = image_data_[img_first];
    ImageData& im_data2 = image_data_[img_second];
    Features& features1 = image_features_[img_first];
    Features& features2 = image_features_[img_second];
    CameraInfo& camera_info1 = cameras_[img_first];
    CameraInfo& camera_info2 = cameras_[img_second];

    Matches matches;
    matches.image_index.first = img_first;
    matches.image_index.second = img_second;

    bool from_cache = true;
    // bool tst = true;
    if(!cache_storage.GetImageMatches(im_data1, im_data2, matches)) {
      from_cache = false;
      ::ComputeLineKeyPointsMatch(features1, camera_info1, features2, camera_info2, matches);
      // Save to Cache
      cache_storage.SaveImageMatches(im_data1, im_data2, matches);
    } 
    //  else {
    //   std::cout << "Matches restored from CACHE!!!\n";
    // } 

    // Restore indexes to the current run
    matches.image_index.first = img_first;
    matches.image_index.second = img_second;    

    std::cout << "Match: "
              << "(" << img_first << ", " << img_second << ")"
              << " [" << (from_cache ? "R" : "C") << "]"
              << ": matches.size = " << matches.match.size();
              // << std::endl;

    // std::cout << "!atch: "
    //               << "(" << matches.image_index.first << ", "
    //               << matches.image_index.second << ")"
    //               << ": matches.size = " << matches.match.size();
              // << std::endl;

    // Don't add empty or small matches
    if (matches.match.size() < skip_thresh) { 
      ++skipped_matches;
      std::cout << ", skipped ...\n";
      continue;
    }

    image_matches_.push_back(matches);

    // put index into matches_index_
    auto p1 = std::make_pair(img_first, img_second);
    auto p2 = std::make_pair(img_second, img_first);
    matches_index_.insert(
        std::make_pair(p1, image_matches_.size() - 1));
    matches_index_.insert(
        std::make_pair(p2, image_matches_.size() - 1));

    // find most match for debug only
    // if (matches.match.size() > most_match) {
    //   most_match = matches.match.size();
    //   most_match_id = image_matches_.size() - 1;
    // }

    total_matched_points += matches.match.size();
    std::cout << ", total_matched_points = " << total_matched_points 
              << ", id: " << image_matches_.size() - 1
              << std::endl;

    // Connect keypoints and images for a quick retrieval later
    for (int i = 0; i < matches.match.size(); ++i) {
      IntPair p1 = std::make_pair(
          matches.image_index.first,
          matches.match[i].queryIdx);
      IntPair p2 = std::make_pair(
          matches.image_index.second, 
          matches.match[i].trainIdx);
      // std::cout << "union: " << p1 << ", " << p2 << std::endl;
      ccomp_.Union(p1, p2);
    }

  }
  */
  

  std::cout << "total_matched_points = " << total_matched_points << std::endl;
  // std::cout << "most_match = " 
  //           << image_matches_[most_match_id].image_index.first 
  //           << " - " << image_matches_[most_match_id].image_index.second
  //           << ", " << most_match
  //           << " (id: " << most_match_id << ")"
  //           << std::endl;
  std::cout << "skipped_matches = " << skipped_matches << std::endl;
  std::cout << "filtered_by_distance = " << filtered_by_distance << std::endl;
  std::cout << "image_matches_.size = " << image_matches_.size() << std::endl;

  auto t1 = high_resolution_clock::now();
  auto dur = duration_cast<microseconds>(t1 - t0);
  std::cout << "match_features_time = " << dur.count() / 1e+6 << std::endl;

}

int SfM3D::FindMaxSizeMatch(const bool within_todo_views) const {

  int max_match = 0;
  int max_match_id = -1;
  for (int i = 0; i < image_matches_.size(); ++i) {
    const Matches& m = image_matches_[i];
    if (within_todo_views 
        && todo_views_.find(m.image_index.first) == todo_views_.end()) {
      continue;
    }
    if (within_todo_views 
        && todo_views_.find(m.image_index.second) == todo_views_.end()) {
      continue;
    }
    if (m.match.size() > max_match) {
      max_match = m.match.size();
      max_match_id = i;
    }
  }

  return max_match_id;

}

void SfM3D::InitReconstruction() {
  std::cout << "Init Reconstruction\n";

  assert(image_matches_.size() > 0);

  // Init with all images
  for (size_t i = 0; i < image_features_.size(); ++i) {
    todo_views_.insert(i);
  }

  // Find the match with the most points
  int most_match_id = FindMaxSizeMatch();
  std::cout << "most_match = " 
            << image_matches_[most_match_id].image_index.first 
            << " - " << image_matches_[most_match_id].image_index.second
            << ", " << image_matches_[most_match_id].match.size()
            << " (id: " << most_match_id << ")"
            << std::endl;

  // ShowMatches(image_matches_[most_match_id]);

  int first_id = image_matches_[most_match_id].image_index.first;
  int second_id = image_matches_[most_match_id].image_index.second;

  map_.clear();

  TriangulatePointsFromViews(first_id, second_id, map_);

  // for (auto& wp : map_) {
  //   std::cout << "mp: " << wp << std::endl;
  // }

  CombineMapComponents(map_, max_merge_dist);

  OptimizeMap(map_);

  std::cout << "init map size = " << map_.size() << std::endl;
  
  // Add used images
  used_views_.insert(first_id);
  used_views_.insert(second_id);

  // Remove used from todo
  todo_views_.erase(first_id);
  todo_views_.erase(second_id);

  // for (auto& wp : map_) {
  //   std::cout << "mp: " << wp << std::endl;
  // }

}

void SfM3D::TriangulatePointsFromViews(const int first_id, 
                                  const int second_id, 
                                  Map3D& map) {

  assert(IsPairInOrder(first_id, second_id));

  
  auto find = matches_index_.find({first_id, second_id});
  if (find == matches_index_.end()) {
    std::cerr << "No match to triangulate for views (" << first_id << ", "
              << second_id << ")\n";
  }
  int match_index = find->second;
  // std::cout << "match_index = " << match_index << std::endl;


  // == Triangulate Points =====
  std::vector<cv::Point2f> points1f, points2f;
  ::KeyPointsToPointVec(image_features_[first_id].keypoints, 
                        image_features_[second_id].keypoints,
                        image_matches_[match_index].match, 
                        points1f, points2f);

  cv::Mat points3d;
  ::TriangulatePoints(cameras_[first_id], points1f, 
                    cameras_[second_id], points2f, 
                    points3d);

  // Backprojection error
  cv::Mat proj1 = ::GetProjMatrix(cameras_[first_id]);
  cv::Mat proj2 = ::GetProjMatrix(cameras_[second_id]);

  std::vector<double> errs1 = ::GetReprojectionErrors(points1f, proj1, points3d);
  std::vector<double> errs2 = ::GetReprojectionErrors(points2f, proj2, points3d);

  // Z distance of points from camera
  std::vector<double> cam1_zdist = ::GetZDistanceFromCamera(cameras_[first_id],
                                                            points3d);
  std::vector<double> cam2_zdist = GetZDistanceFromCamera(cameras_[second_id],
                                                          points3d);

  // for (size_t i = 0; i < cam1_zdist.size(); ++i) {
  //   std::cout << "p = " << i << " : " << cam1_zdist[i] << std::endl;
  // }
  // << [end front back]

  // std::cout << "AFTER points3d.row(0) = " << points3d.row(0) << std::endl << std::flush;
  // std::cout << "AFTER points3d.size = " << points3d.rows << ", " << points3d.cols 
  //           << std::endl << std::flush;

  double rep_err1 = 0.0;
  double rep_err2 = 0.0;


  // Show distances from camera
  /*
  std::vector<std::pair<int, double> > izs(cam1_zdist.size());
  for (int i = 0; i < cam1_zdist.size(); ++i) {
    izs[i] = std::make_pair(i, cam1_zdist[i]);
  }
  std::sort(izs.begin(), izs.end(), [](std::pair<int, double>& a1,
      std::pair<int, double>& a2) { return a1.second > a2.second; });
  for (int i = 0; i < izs.size(); ++i) {
    if (cam1_zdist[izs[i].first] > 100.0 || cam2_zdist[izs[i].first] > 100.0) {
      std::cout << i << ": zd1 = " << cam1_zdist[izs[i].first] 
                << ", zd2 = " << cam2_zdist[izs[i].first] 
                << " (" << izs[i].first << ")"
                << " out of " << izs.size() 
                << std::endl;
    }
  }
  */
  

  for (size_t i = 0; i < errs1.size(); ++i) {
    rep_err1 += errs1[i];
    rep_err2 += errs2[i];

    

    // sqrt(errs1[i]*errs1[i] + errs2[i]*errs2[i]) < 2.0
    // errs1[i] > 1.0 || errs2[i] > 1.0           // reprojection error too big
    double d = sqrt(errs1[i]*errs1[i] + errs2[i]*errs2[i]);
    if (   errs1[i] > repr_error_thresh    // reprojection error too big
        || errs2[i] > repr_error_thresh  // reprojection error too big
        || cam1_zdist[i] < 0 || cam2_zdist[i] < 0  // points on the back of the camera
        || cam1_zdist[i] > 100.0 || cam2_zdist[i] > 100.0  // points too far from the camera
        || points3d.at<float>(i, 2) < 38.0) {      // it's out of the ground
      // std::cout << i << " [SKIP] : errs1, errs2 = " << errs1[i]
      //           << ", " << errs2[i] 
      //           << ", d = " << d
      //           << ", z = " << points3d.at<float>(i, 2)
      //           << std::endl;
      continue;
    }

    // Add point to the map
    WorldPoint3D wp;
    wp.pt = cv::Point3d(
        points3d.at<float>(i, 0),
        points3d.at<float>(i, 1),
        points3d.at<float>(i, 2));
    wp.views[image_matches_[match_index].image_index.first] 
        = image_matches_[match_index].match[i].queryIdx;
    wp.views[image_matches_[match_index].image_index.second] 
        = image_matches_[match_index].match[i].trainIdx;
    std::pair<int, int> vk = std::make_pair(
          image_matches_[match_index].image_index.first,
          image_matches_[match_index].match[i].queryIdx);
    wp.component_id = ccomp_.Find(vk);
    map.push_back(wp);
      
    // }
  }

  // for (auto& wp : map) {
  //   std::cout << "mp: " << wp << std::endl;
  // }


  // std::cout << "rep_err1 = " << rep_err1 << std::endl;
  // std::cout << "rep_err2 = " << rep_err2 << std::endl;
  // std::cout << "map.size = " << map.size() << std::endl;

  // double all_error;
  // all_error = ::GetReprojectionError(map, cameras_, image_features_);
  // std::cout << "all_error = " << all_error << std::endl;

}

void SfM3D::OptimizeMap(Map3D& map) {
  // std::cout << "SfM: Optimize Map, map.size = " << map.size() << std::endl;
  double all_error;
  // all_error = ::GetReprojectionError(map, cameras_, image_features_);
  // std::cout << ", err_before = " << all_error;
  // == Optimize Bundle ==
  // TODO!!!!!!!!!!!!!
  ::OptimizeBundle(map, cameras_, image_features_);
  // all_error = ::GetReprojectionError(map, cameras_, image_features_);
  // std::cout << ", err_after = " << all_error << std::endl;
}



void SfM3D::ReconstructNextView(const int next_img_id) {

  assert(todo_views_.count(next_img_id) > 0);

  std::cout << "====> Process img_id = " << next_img_id << " (";
  std::cout << "todo_views.size = " << todo_views_.size();
  std::cout <<  ")"; 

  Map3D view_map;

  // Pairwise use next_img_id and used_views
  for (auto view_it = used_views_.begin(); view_it != used_views_.end(); ++view_it) {
    int view_id = (* view_it);
    // std::cout << "==> Triangulate pair = " << next_img_id << ", "
    //           << view_id << std::endl;

    int first_id = next_img_id;
    int second_id = view_id;

    // std::cout << "first, second = " << first_id << ", "
    //           << second_id << std::endl;

    auto m = matches_index_.find(std::make_pair(first_id, second_id));
    if (m == matches_index_.end()) {
      // std::cout << "skip pair ...\n";
      continue;
    }

    // std::cout << "found = (" << m->first.first << ", " << m->first.second
    //           << "), -> " << m->second 
    //           << std::endl;

    int matches_id = m->second;

    // for (int i = 0; i < image_matches_.size(); ++i) {
    //   Matches& mm = image_matches_[i];
    //   std::cout << "m: " << i
    //           << " (" << mm.image_index.first << ", "
    //           << mm.image_index.second << ")"
    //           << ", matches.size = " << mm.match.size() << std::endl;  
    // }

    Matches& matches = image_matches_[matches_id];
    // std::cout << "matches_id = " << matches_id
    //           << " (" << matches.image_index.first << ", "
    //           << matches.image_index.second << ")"
    //           << ", matches.size = " << matches.match.size() << std::endl;
    
    // Get the right order
    if (matches.image_index.first != first_id) {
      std::swap(first_id, second_id);
      // std::cout << "SWAP: first, second = " << first_id << ", "
      //           << second_id << std::endl;
      // std::cout << "IMAGE_INDEX: first, second = " << matches.image_index.first << ", "
      //         << matches.image_index.second << std::endl;
    }

    // == Triangulate Points =====
    TriangulatePointsFromViews(first_id, second_id, view_map);

    // std::cout << "rep_err1 = " << rep_err1 << std::endl;
    // std::cout << "rep_err2 = " << rep_err2 << std::endl;
    // std::cout << "view_map.size = " << view_map.size() << std::endl;

    // for (auto& wp : view_map) {
    //   std::cout << "view_mp: " << wp << std::endl;
    // }

  }

  std::cout << ", view_map = " << view_map.size();

  // TEST: Looks like almost the same result (same errors and map points)
  // it's likely due to already precise location of the camera and keypoints
  // filtering using epipolar line constraints
  // OptimizeMap(view_map);

  /*
  double all_error;
  all_error = GetReprojectionError(view_map, cameras, image_features);
  // std::cout << "view_error = " << all_error << std::endl;
  // == Optimize Bundle ==
  OptimizeBundle(view_map, cameras, image_features);
  all_error = GetReprojectionError(view_map, cameras, image_features);
  // std::cout << "view_error = " << all_error << std::endl;      
  */

  map_mutex.lock();
  // ::MergeToTheMap(map_, view_map, ccomp_);
  // ::MergeToTheMapImproved(map_, view_map, ccomp_);
  ::MergeAndCombinePoints(map_, view_map, max_merge_dist);
  map_mutex.unlock();

  std::cout << ", map = " << map_.size();
  


  /*
  all_error = GetReprojectionError(map, cameras, image_features);
  std::cout << "map_error = " << all_error << std::endl;
  // == Optimize Bundle ==
  OptimizeBundle(map, cameras, image_features);
  all_error = GetReprojectionError(map, cameras, image_features);
  std::cout << "map_error = " << all_error << std::endl;
  */

  // OptimizeMap(map_);

  // std::cout << std::endl;

  used_views_.insert(next_img_id);

  todo_views_.erase(next_img_id);

  // ++vis_version_;
  // map_update_.notify_one();
  EmitMapUpdate();

}

void SfM3D::ReconstructNextViewPair(const int first_id, const int second_id) {
  assert(IsPairInOrder(first_id, second_id));

  std::cout << "NEXT PAIR RECONSTRUCT: " << first_id
            << " - " << second_id;

  Map3D view_map;

  TriangulatePointsFromViews(first_id, second_id, view_map);
  std::cout << ", view_map = " << view_map.size();

  map_mutex.lock();
  // ::MergeToTheMap(map_, view_map, ccomp_);
  // ::MergeToTheMapImproved(map_, view_map, ccomp_);
  ::MergeAndCombinePoints(map_, view_map, max_merge_dist);
  std::cout << ", map = " << map_.size();
  map_mutex.unlock();

  // OptimizeMap(map_);

  std::cout << std::endl;

  used_views_.insert(first_id);
  used_views_.insert(second_id);

  todo_views_.erase(first_id);
  todo_views_.erase(second_id);

  // ++vis_version_;
  // map_update_.notify_one();
  EmitMapUpdate();
}

void SfM3D::ReconstructAll() {

  using namespace std::chrono;

  double total_time = 0.0;
  int total_views = 0;

  int last_optimitzation_cnt = map_.size();
  bool need_optimization = false;

  int cnt = 0;

  while (todo_views_.size() > 0) {

    /*
    if (cnt == 3) {
      // Test visualization
      std::cout << "Quit early! - optimize\n";
      std::cout << "\nOPTIMIZING on " << map_.size() << " ...\n\n";
      OptimizeMap(map_);
      std::cout << "\nOPTIMIZATION DONE! (" << map_.size() << ") \n\n";
      break;
    }
    */

    // std::cout << "TODO_VIEWS: ";
    // for (auto v : todo_views_) {
    //   std::cout << v << ",";
    // }
    // std::cout << std::endl;

    // check finish flag and finish)
    if (proc_status_.load() == FINISH) {
      std::cout << "FINISJ!!!\n";
      break;
    }

    int todo_size = todo_views_.size();

    auto t0 = high_resolution_clock::now();


    if (map_.size() - last_optimitzation_cnt > 40000 && need_optimization) {
      std::cout << "\nOPTIMIZING on " << map_.size() << " ...\n\n";
      OptimizeMap(map_);
      std::cout << "\nOPTIMIZATION DONE! (" << map_.size() << ") \n\n";
      last_optimitzation_cnt = map_.size();
      need_optimization = false;
    }


    // int next_img_id1 = ::GetNextBestView(map_, todo_views_,
    //     ccomp_, image_matches_, matches_index_);

    int next_img_id = ::GetNextBestViewByViews(map_, todo_views_,
        used_views_, image_matches_, matches_index_);

    auto t1 = high_resolution_clock::now();
    auto dur_gnbv = duration_cast<microseconds>(t1 - t0).count() / 1e+6;
    std::cout << ">>>>>>>> dur_gnbv = " << dur_gnbv 
              << ", next_img_id = " << next_img_id 
              // << ", next_img_id1 = " << next_img_id1 
              << std::endl;

    


    if (next_img_id < 0) {
      // Didn't find connected views, so proceed with the best pair left

      int most_match_id = FindMaxSizeMatch(true);

      if (most_match_id < 0) {
        std::cerr << "ERROR: matches not found for left imgs ";
        for (auto v : todo_views_) {
          std::cerr << v << ",";
        }
        std::cerr << std::endl;
        todo_views_.clear();
        break;
      }

      std::cout << "OTHER::: most_match = " 
                << image_matches_[most_match_id].image_index.first 
                << " - " << image_matches_[most_match_id].image_index.second
                << ", " << image_matches_[most_match_id].match.size()
                << " (id: " << most_match_id << ")"
                << std::endl;
      

      auto t2 = high_resolution_clock::now();
      auto dur_fmsm = duration_cast<microseconds>(t2 - t1).count() / 1e+6;
      std::cout << ", dur_fmsm = " << dur_fmsm;

      int first_id = image_matches_[most_match_id].image_index.first;
      int second_id = image_matches_[most_match_id].image_index.second;

      ReconstructNextViewPair(first_id, second_id);

      auto t3 = high_resolution_clock::now();
      auto dur_rnvp = duration_cast<microseconds>(t3 - t2).count() / 1e+6;
      std::cout << ", dur_rnvp = " << dur_rnvp;

      // continue;
    } else {
      ReconstructNextView(next_img_id);
      auto t4 = high_resolution_clock::now();
      auto dur_rnv = duration_cast<microseconds>(t4 - t1).count() / 1e+6;
      std::cout << ", dur_rnv = " << dur_rnv;
    }

    auto t5 = high_resolution_clock::now();
    auto dur_vr = duration_cast<microseconds>(t5 - t0).count() / 1e+6;
    std::cout << ", view_time = " << dur_vr; // << std::endl;

    total_time += dur_vr;
    total_views += todo_size - todo_views_.size();
    double view_avg = total_time / total_views;
    std::cout << "\nview_avg = " << view_avg;
    std::cout << "\nt_left = " << view_avg * todo_views_.size();
    
    std::cout << std::endl;

    need_optimization = true;

    ++cnt;

    // if (cnt == 2)
    //   break;

  }


  if (need_optimization) {
    std::cout << "\nOPTIMIZING ALLLL ....\n\n";
    OptimizeMap(map_);

    

    // OptimizeMap(map_);

    std::cout << "\nOPTIMIZATION DONE! \n\n";
  }

  // map_update_.notify_one();
  EmitMapUpdate();

}

void SfM3D::PrintFinalStats() {

  map_mutex.lock();

  // Counts points view nums
  std::map<int, int> map_counts;
  for (auto& wp: map_) {
    int n = wp.views.size();
    auto mc_it = map_counts.find(n);
    if (mc_it != map_counts.end()) {
      mc_it->second++;
    } else {
      map_counts.insert(std::make_pair(n, 1));
    }
  }

  std::cout << "Map counts:\n";
  for (auto mc: map_counts) {
    std::cout << mc.first << " : " << mc.second << std::endl;
  }


  double all_error = ::GetReprojectionError(map_, cameras_, image_features_);

  map_mutex.unlock();

  std::cout << "FINAL_error = " << all_error << std::endl;
  std::cout << "USED_views = " << used_views_.size()
            << " out of " << image_features_.size() << std::endl;
  std::cout << "FINAL_map.size = " << map_.size() << " points" << std::endl;


  // ::RemoveOutliersByError(map_, cameras_, image_features_, 0.05);
  // std::cout << "map res size = " << map_.size() << std::endl;


  std::vector<double> errs = ::GetReprojectionErrors(map_, cameras_, image_features_);

  typedef std::pair<int, double> ErrEl;
  std::vector<ErrEl> errsi(errs.size());
  for (int i = 0; i < errs.size(); ++i) {
    errsi[i] = std::make_pair(i, errs[i]);
  }

  std::sort(errsi.rbegin(), errsi.rend(), [](const ErrEl& a, const ErrEl& b) {
    return a.second < b.second;
  });

  std::cout << "=== Top errors:" << std::endl;
  for (int i = 0; i < 20; ++i) {
    std::cout << i << ": " << errsi[i].second
              << ", " << map_[errsi[i].first].views.size()
              << std::endl;
  }

  // count errs in bins
  int num_bins = 20;
  double min_err = (* std::min_element(errs.begin(), errs.end()));
  double max_err = (* std::max_element(errs.begin(), errs.end()));
  std::map<int, int> count_bins;
  for (int i = 0; i < num_bins; ++i) {
    count_bins.insert(std::make_pair(i, 0));
  }

  // TODO: TEST on THIS
  // make && ./bin/3d_recon --records="1,2" --matches_line_dist_thresh=10.0 --matches_num_thresh=7 --sfm_repr_error_thresh=10.0 --sfm_max_merge_dist=5.0 --noviz
  // idx = 20
  double bin_width = (max_err - min_err) / num_bins;
  // std::cout << "bin_width = " << bin_width << std::endl;
  for (auto e: errs) {
    int idx = static_cast<int>(floor((e - min_err) / bin_width));

    if (idx >= num_bins) {
      idx = num_bins - 1;
    }
    
    // if (e > 30.0) {
    //   std::cout << e << " -> " << idx << std::endl;
    //   std::cout << "idx raw0 = " << (e - min_err) / bin_width << std::endl;
    //   std::cout << "idx raw1 = " << floor((e - min_err) / bin_width) << std::endl;
    // }
    ++count_bins[idx];
    // std::cin.get();
  }

  
  std::cout << "=== Errors distribution:" << std::endl;
  for (int i = 0; i < std::min(20, num_bins); ++i) {
    std::cout << i << " [" << (bin_width * (i + 1)) << "]: " 
              << count_bins[i] << std::endl;
  }

  
  
}

bool SfM3D::GetMapPointsVec(std::vector<Point3DColor>& glm_points) {

  // if(!map_mutex.try_lock()) return false;

  // std::cout << "\n VIS_version (gmpv) = " <<  vis_version_.load() << std::endl;

  glm_points.clear();

  // std::cout << "\n>> reduce map ratio = "
  //           << map_points_ratio_ << std::endl;
  // Map3D map = ::ReduceMapByError(map_, cameras_, image_features_, map_points_ratio_);

  // ::RemoveOutliersByError(map_, cameras_, image_features_, 0.5);

  // if (retain_ratio < 1.0) {
  //   int steps = 0;
  //   int start_size = map.size();
  //   int end_size = map.size() * retain_ratio;
  //   while ()
  // }

  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  // std::cout << "GET MAP POINTS VEC <<<<<<!!!!!!!!!!! = " << map_.size();
  for (auto& wp: map_) {
    Point3DColor p3d;
    glm::vec3 v(wp.pt.x, wp.pt.y, wp.pt.z);
    p3d.pt = v;

    // p3d.color = glm::vec3(0.0);
    bool first = true;
    double orig_angle = 0.0;
    for (auto& view : wp.views) {
      int img_id = view.first;

      Point3DColor p3dc;

      cv::KeyPoint kp = image_features_[img_id].keypoints[view.second];
      if (first) {
        orig_angle = kp.angle;
        first = false;
      }
      // std::cout << "[" << img_id << "]: "
      //           << "kp.size = " << kp.size << ", "
      //           << "kp.angle = " << kp.angle << ", "
      //           << "orig_angle = " << orig_angle << ", "
      //           << "kp.pt = " << kp.pt << ""
      //           << std::endl;

      ::GetKeyPointColors(
        images_resized_[img_id],
        kp,
        p3dc, true, kp.angle - orig_angle, resize_scale);

      // std::cout << " oo: " << glm::to_string(p3dc.color) << std::endl;
      // std::cout << " tl: " << glm::to_string(p3dc.color_tl) << std::endl;
      // std::cout << " tr: " << glm::to_string(p3dc.color_tr) << std::endl;
      // std::cout << " br: " << glm::to_string(p3dc.color_br) << std::endl;
      // std::cout << " bl: " << glm::to_string(p3dc.color_bl) << std::endl;

      p3d.color += p3dc.color;
      p3d.color_tl += p3dc.color_tl;
      p3d.color_tr += p3dc.color_tr;
      p3d.color_br += p3dc.color_br;
      p3d.color_bl += p3dc.color_bl;
    }
    p3d.color = p3d.color / static_cast<float>(wp.views.size());
    p3d.color_bl = p3d.color_bl / static_cast<float>(wp.views.size());
    p3d.color_br = p3d.color_br / static_cast<float>(wp.views.size());
    p3d.color_tr = p3d.color_tr / static_cast<float>(wp.views.size());
    p3d.color_tl = p3d.color_tl / static_cast<float>(wp.views.size());
    // std::cout << "soo: " << glm::to_string(p3d.color) << std::endl;
    // std::cout << "stl: " << glm::to_string(p3d.color_tl) << std::endl;
    // std::cout << "str: " << glm::to_string(p3d.color_tr) << std::endl;
    // std::cout << "sbr: " << glm::to_string(p3d.color_br) << std::endl;
    // std::cout << "sbl: " << glm::to_string(p3d.color_bl) << std::endl;

    glm_points.push_back(p3d);
  }

  
  // Add errors to the points
  std::vector<double> errs = ::GetReprojectionErrors(map_, cameras_, image_features_);
  for (int i = 0; i < errs.size(); ++i) {
    glm_points[i].err = errs[i];
  }

  std::sort(glm_points.begin(), glm_points.end(), [](const Point3DColor& p1,
      const Point3DColor& p2) {
    return p1.err < p2.err;
  });

  // for (int i = 0; i < glm_points.size(); ++i) {
  //   if (i > 10) break;
  //   std::cout << i << ": err " << glm_points[i].err << std::endl;
  // }
  

  // map_mutex.unlock();

  auto t1 = high_resolution_clock::now();
  auto dur_gmpv = duration_cast<microseconds>(t1 - t0).count() / 1e+6;
  // std::cout << "\nGET_MAP_POINTS_TIME = " << dur_gmpv << std::endl;

  return true;
}

bool SfM3D::GetMapCamerasWithPointsVec(MapCameras& map_cameras) {

  // if (!map_mutex.try_lock()) return false;

  // std::cout << "\n VIS_version (gmcwpv) = " <<  vis_version_.load() << std::endl;

  map_cameras.clear();

  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  for (auto& wp: map_) {

    Point3DColor p3d;

    glm::vec3 v(wp.pt.x, wp.pt.y, wp.pt.z);
    p3d.color = glm::vec3(0.0);
    for (auto& view : wp.views) {
      int img_id = view.first;
      glm::vec3 v_color = ::GetGlmColorFromImage(
        images_resized_[img_id],
        image_features_[img_id].keypoints[view.second],
        resize_scale);
      // std::cout << "o: " << glm::to_string(v_color) << std::endl;
      p3d.color += v_color;
    }
    p3d.pt = v;
    p3d.color = p3d.color / static_cast<float>(wp.views.size());

    for (auto& vw: wp.views) {
      int cam_id = vw.first;
      int point_id = vw.second;
      auto p = std::make_pair(point_id, p3d);
      auto it = map_cameras.find(cam_id);
      if (it != map_cameras.end()) {
        it->second.push_back(p);
      } else {
        std::vector<std::pair<int, Point3DColor> > vec = {p};
        map_cameras.insert(std::make_pair(cam_id, vec));
      }
    }
  }

  // map_mutex.unlock();

  auto t1 = high_resolution_clock::now();
  auto dur_gmc = duration_cast<microseconds>(t1 - t0).count() / 1e+6;
  // std::cout << "\nGET_MAP_CAMERAS_TIME = " << dur_gmc << std::endl;

  return true;

}

void SfM3D::GetMapPointsAndCameras(std::vector<Point3DColor>& glm_points,
                              MapCameras& map_cameras,
                              int& last_version) {

  std::unique_lock<std::mutex> lck(map_mutex);
  // map_update_.wait(lck);
  map_update_.wait(lck, [&]() { return last_version < vis_version_.load(); }); //return last_version < vis_version_.load();
  std::cout << "\n>> GET_MAP_POINTS_AND_CAMERAS !!!!!!!!!!!!!!!!!!"
            << std::endl;

  GetMapPointsVec(glm_points);
  GetMapCamerasWithPointsVec(map_cameras);

  last_version = vis_version_.load();


}

cv::Mat SfM3D::GetImage(int cam_id, bool full_size) const {
  if (!full_size) {
    return images_resized_[cam_id].clone();
  } else {
    return ::LoadImage(image_data_[cam_id]);
  }
  
}

CameraInfo SfM3D::GetCameraInfo(int cam_id) const {
  return cameras_[cam_id];
}

cv::KeyPoint SfM3D::GetKeypoint(int cam_id, int point_id) const {
  return image_features_[cam_id].keypoints[point_id];
}


void SfM3D::GenerateAllPairs() {
  if (!image_pairs_.empty()) return;
  for (int i = 0; i < ImageCount() - 1; ++i) {
    for (int j = i + 1; j < ImageCount(); ++j) {
      image_pairs_.push_back({i, j});
    }
  }
}

int SfM3D::ImageCount() const {
  return image_data_.size();
}

int SfM3D::MapSize() const {
  return map_.size();
}

void SfM3D::SetProcStatus(SfMStatus proc_status) {
  proc_status_.store(proc_status);

  // ++vis_version_;
  // map_update_.notify_all();
  EmitMapUpdate();
}

bool SfM3D::IsFinished() {
  return proc_status_.load() == SfM3D::FINISH;
}

void SfM3D::EmitMapUpdate() {
  ++vis_version_;
  map_update_.notify_all();
}

bool SfM3D::IsPairInOrder(const int p1, const int p2) {
  const ImageData& im_data1 = image_data_[p1];
  const ImageData& im_data2 = image_data_[p2];
  return ::IsPairInOrder(im_data1, im_data2);
}

void SfM3D::RestoreImages() {

  if (!images_resized_.empty()) {
    std::cout << "Images exist. Skip RestoreImages.\n";
    return;
  }

  // images_resized_.clear();

  assert(image_data_.size() == cameras_.size());

  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  
  std::atomic<int> next_idx(0);
  std::vector<std::thread> resize_threads;

  images_resized_.resize(image_data_.size());

  int capacity = std::max(
      static_cast<int>(std::thread::hardware_concurrency() - 2), 1);
  std::cout << "image_data_.size = " << image_data_.size() << std::endl;
  capacity = std::min(capacity, static_cast<int>(image_data_.size()));
  std::cout << "Concurrency = " << capacity << std::endl;

  std::mutex cout_mu;

  for (int i = 0; i < capacity; ++i) {
    auto resizer = [this, &next_idx, &cout_mu](int thread_id) {
      int idx;
      while ((idx = next_idx++) < image_data_.size()) {

        cout_mu.lock();
        std::cout << "[th:" << thread_id << "] Restore image " << idx << " out of " << image_data_.size()
              <<std::endl;    
        cout_mu.unlock();

        ImageData& im_data = image_data_[idx];
        cv::Mat img = ::LoadImage(im_data, resize_scale);
        images_resized_[idx] = img;

      }
    };
    resize_threads.push_back(std::thread(resizer, i));
  }

  for(int i = 0; i < capacity; ++i) {
    resize_threads[i].join();
  }

  auto t1 = high_resolution_clock::now();
  auto dur = duration_cast<microseconds>(t1 - t0);
  std::cout << "\nRESTORE_IMAGES_TIME = " << dur.count() / 1e+6 << std::endl;


};

void SfM3D::ClearImages() {
  images_resized_.clear();
}

void SfM3D::ShowFeatures(int img_id) {
  // Show Image Features
  int first_id = img_id;
  int second_id;
  if (img_id % 2 == 0) {
    second_id =  img_id + 1;
  } else {
    second_id = first_id;
    first_id = second_id - 1;
  }

  std::cout << "Show Features: " << first_id << ","
            << second_id << std::endl;
  
  // int show_id = 0;
  std::vector<cv::KeyPoint> points1, points2;
  points1 = image_features_[first_id].keypoints;
  points2 = image_features_[second_id].keypoints;
  int win_x = 0;
  int win_y = 100;
  double win_scale = resize_scale;
  // == Show Camera Images (Keypoints) ==
  cv::Mat img1_points, img2_points, img_matches;
  DrawKeypointsWithResize(images_resized_[first_id], points1, img1_points, win_scale);
  DrawKeypointsWithResize(images_resized_[second_id], points2, img2_points, win_scale);

  std::cout << "features1.size = " << points1.size() << std::endl;
  std::cout << "features2.size = " << points2.size() << std::endl;


  // Debug: Show points
  ImShow("img2", img2_points);
  cv::moveWindow("img2", win_x, win_y);
  ImShow("img1", img1_points);
  cv::moveWindow("img1", win_x + img2_points.size().width, win_y);
  cv::waitKey();
}

void SfM3D::ShowMatches(const Matches& matches) {

  int first_id = matches.image_index.first;
  int second_id = matches.image_index.second;

  ::ImShowMatchesWithResize(images_resized_[first_id],
                            image_features_[first_id].keypoints, 
                            images_resized_[second_id],
                            image_features_[second_id].keypoints, 
                            matches.match,
                            resize_scale,
                            10, 10);
  cv::waitKey();
  cv::destroyAllWindows();

}

void SfM3D::ShowMatchesLineConstraints(const Matches& matches,
                                       const double line_dist) {
  cv::Mat fund;
  int first_id = matches.image_index.first;
  int second_id = matches.image_index.second;
  CalcFundamental(cameras_[first_id], cameras_[second_id], fund);

  // double line_dist = 0.0;

  // cv::Mat img1 = images_resized_[first_id].clone();
  // cv::Mat img2 = images_resized_[second_id].clone();

  for (int i = 0; i < matches.match.size(); ++i) {
    auto match = matches.match[i];
    cv::Mat points2(3, 1, CV_64F);
    points2.at<double>(0, 0) = image_features_[second_id].keypoints[match.trainIdx].pt.x;
    points2.at<double>(1, 0) = image_features_[second_id].keypoints[match.trainIdx].pt.y;
    points2.at<double>(2, 0) = 1.0;

    cv::Mat points1(1, 3, CV_64F);
    points1.at<double>(0, 0) = image_features_[first_id].keypoints[match.queryIdx].pt.x;
    points1.at<double>(0, 1) = image_features_[first_id].keypoints[match.queryIdx].pt.y;
    points1.at<double>(0, 2) = 1.0;

    cv::Mat kp_l2 = fund * points2;
    double a = kp_l2.at<double>(0);
    double b = kp_l2.at<double>(1);
    double d = sqrt(a*a + b*b);

    cv::Mat dd_mat = points1 * kp_l2 / d;
    double dd = abs(dd_mat.at<double>(0));

    if (dd > line_dist) {
      // Draw point & line
      cv::Mat img1 = images_resized_[first_id].clone();
      cv::Mat img2 = images_resized_[second_id].clone();

      std::vector<cv::Point2f> line_pts;
      GetLineImagePoints(kp_l2, line_pts, img1.size().width / resize_scale, img1.size().height / resize_scale);

      std::cout << i << ": dd = " << dd
                << std::endl;

      cv::drawMarker(img2, cv::Point2f(points2.at<double>(0, 0) * resize_scale,
                                       points2.at<double>(1, 0) * resize_scale),
                     cv::Scalar(100.0, 100.0, 255.0), cv::MARKER_CROSS,
                     15, 2);

      cv::drawMarker(img1, cv::Point2f(points1.at<double>(0, 0) * resize_scale,
                                       points1.at<double>(0, 1) * resize_scale),
                     cv::Scalar(100.0, 100.0, 255.0), cv::MARKER_CROSS,
                     15, 2);

      cv::line(img1, 
               line_pts[0] * resize_scale, 
               line_pts[1] * resize_scale, 
               cv::Scalar(0.0, 0.0, 255.0), 1);

      int win_x = 10;
      int win_y = 10;
      // Debug: Show points
      ImShow("img2", img2);
      cv::moveWindow("img2", win_x, win_y);
      ImShow("img1", img1);
      cv::moveWindow("img1", win_x + img2.size().width, win_y);
      cv::waitKey();
    }

  }

  // int win_x = 10;
  // int win_y = 10;
  // // Debug: Show points
  // ImShow("img2", img2);
  // cv::moveWindow("img2", win_x, win_y);
  // ImShow("img1", img1);
  // cv::moveWindow("img1", win_x + img2.size().width, win_y);
  // cv::waitKey();


}

void SfM3D::Print(std::ostream& os) const {
  os << "SfM3D: intrinsics_.size = " << intrinsics_.size() << ", "
     << "image_data_.size = " << image_data_.size() << ", "
     << "image_pairs_.size = " << image_pairs_.size() << ", "
     << "cameras_.size = " << cameras_.size() << ", "
     << "images_.size = " << images_.size() << ", "
     << "images_resized_.size = " << images_resized_.size() << ", "
     << "image_features_.size = " << image_features_.size() << ", "
     << "image_matches_.size = " << image_matches_.size() << ", "
     << "used_views_.size = " << used_views_.size() << ", "
     << "todo_views_.size = " << todo_views_.size() << ", "
     << "map.size = " << map_.size();
    //  << std::endl;
}

std::ostream& operator<<(std::ostream& os, const SfM3D& sfm) {
  sfm.Print(os);
  return os;
}

 