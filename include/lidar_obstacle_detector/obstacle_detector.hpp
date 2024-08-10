/* obstacle_detector.hpp

 * Copyright (C) 2021 SS47816

 * Implementation of 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#pragma once

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>  
#include <pcl/filters/conditional_removal.h>  

#include <algorithm>
#include <ctime>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "lidar_obstacle_detector/box.hpp"

namespace lidar_obstacle_detector {
template <typename PointT>
class ObstacleDetector { //障碍物检测
 public: 
  ObstacleDetector();
  virtual ~ObstacleDetector();

  // ****************** Detection ***********************
  /*
  filterCloud
  点云滤波器
  读取点云cloud 分辨率filter_res 分割最小点min_pt 和分割最大点max_pt
  */
  typename pcl::PointCloud<PointT>::Ptr filterCloud(
      const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
      const float filter_res, const Eigen::Vector4f &min_pt,
      const Eigen::Vector4f &max_pt);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  segmentPlane(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
               const int max_iterations, const float distance_thresh);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering(
      const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
      const float cluster_tolerance, const int min_size, const int max_size);

  Box axisAlignedBoundingBox(
      const typename pcl::PointCloud<PointT>::ConstPtr &cluster, const int id);

  Box pcaBoundingBox(const typename pcl::PointCloud<PointT>::Ptr &cluster,
                     const int id);

  // ****************** Tracking ***********************
  void obstacleTracking(const std::vector<Box> &prev_boxes,
                        std::vector<Box> *curr_boxes,
                        const float displacement_thresh,
                        const float iou_thresh);

 private:
  // ****************** Detection ***********************
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  separateClouds(const pcl::PointIndices::ConstPtr &inliers,
                 const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

  // ****************** Tracking ***********************
  bool compareBoxes(const Box &a, const Box &b, const float displacement_thresh,
                    const float iou_thresh);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(
      const std::vector<Box> &prev_boxes, const std::vector<Box> &curr_boxes,
      const float displacement_thresh, const float iou_thresh);

  // Connection Matrix
  std::vector<std::vector<int>> connectionMatrix(
      const std::vector<std::vector<int>> &connection_pairs,
      std::vector<int> *left, std::vector<int> *right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(const int i,
                     const std::vector<std::vector<int>> &connection_matrix,
                     std::vector<bool> *right_connected,
                     std::vector<int> *right_pair);

  // Customized Hungarian Algorithm
  std::vector<int> hungarian(
      const std::vector<std::vector<int>> &connection_matrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Box> &Boxes, const int id);
};



// constructor:
template <typename PointT>
ObstacleDetector<PointT>::ObstacleDetector() {}

// de-constructor:
template <typename PointT>
ObstacleDetector<PointT>::~ObstacleDetector() {}


//点云滤波器,使用体素滤波器降低点云的分辨率，对点云进行裁剪，返回体素格式的点云。
template <typename PointT>
/*filter_res 体素网格的大小
  min_pt ROI的最小区域
  max_pt ROI的最大区域
*/
typename pcl::PointCloud<PointT>::Ptr ObstacleDetector<PointT>::filterCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const float filter_res, const Eigen::Vector4f &min_pt,
    const Eigen::Vector4f &max_pt) {
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size
  pcl::VoxelGrid<PointT> vg; //声明点云的体素 vg
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>); //创建一个指向pcl::PointCloud<PointT>类型的智能指针cloud_filtered，用于存储体素化后的点云。
  vg.setInputCloud(cloud);
  vg.setLeafSize(filter_res, filter_res, filter_res); //在x y z上三个方向大小一致的立方体
  vg.filter(*cloud_filtered); //执行体素化操作

  // Cropping the ROI
  typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>); //创建一个存储ROI区域点云的智能指针
  pcl::CropBox<PointT> region(true);
  region.setMin(min_pt);
  region.setMax(max_pt);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_roi);


  
  // Removing the car roof region， 去除车顶区域，这里去除了roi区域中的所有车顶部分。在机器人中这里屏蔽看看
  /*
  std::vector<int> indices;
  //这里因为是对于室内环境进行处理，所以需要去掉这个功能
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_roi);
  roof.filter(indices);
  //将感兴趣的点保存在indices中
  


  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto &point : indices) inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_roi);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_roi);
  //将roi区域中的indices点去除掉
  */
  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time =
  // std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
  // start_time); std::cout << "filtering took " << elapsed_time.count() << "
  // milliseconds" << std::endl;

  return cloud_roi;
} //点云滤波器



//分离障碍物点与地面点
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::separateClouds(
    const pcl::PointIndices::ConstPtr &inliers,
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
  //声明障碍物和地面点的智能指针
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr ground_cloud(
      new pcl::PointCloud<PointT>());

  // Pushback all the inliers into the ground_cloud 将左右点放入地面点中
  for (int index : inliers->indices) {
    ground_cloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstacle_cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return std::pair<typename pcl::PointCloud<PointT>::Ptr,
                   typename pcl::PointCloud<PointT>::Ptr>(obstacle_cloud,
                                                          ground_cloud);
}





/*地面点提取程序
cloud是一个指向pcl::PointCloud<PointT>对象的常量指针，表示输入的点云；
max_iterations是分割算法的最大迭代次数；
distance_thresh是点与平面模型之间的最大距离阈值。
函数返回一个std::pair，包含两个指向pcl::PointCloud<PointT>对象的指针，分别表示障碍物点云和地面点云。
*/
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::segmentPlane(  
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const int max_iterations, const float distance_thresh) {
  // Time segmentation process
  // const auto start_time = std::chrono::steady_clock::now();

  // Find inliers for the cloud.

  //添加地面高度限制

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //声明了一个平面模型

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_thresh);


  //设置地面高度过滤函数
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  

  //添加过滤器
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
  // 设置条件：z坐标必须小于或等于输入参数 
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, distance_thresh)));

  
  condrem.setInputCloud(cloud); 
  condrem.setCondition(range_cond); 
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);  
      
    // 应用过滤条件并获取结果  
  condrem.filter(*filtered_cloud);

  
  // Segment the largest planar component from the input cloud


  /*
  使用平面模型对地面点进行分割，室内暂时不需要使用地面点进行分割
  seg.setInputCloud(filtered_cloud);
  seg.segment(*inliers, *coefficients); //使用平面模型对点云进行分割
  if (inliers->indices.empty()) {
    std::cout << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }
  */
  

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time =
  // std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
  // start_time); std::cout << "plane segmentation took " <<
  // elapsed_time.count() << " milliseconds" << std::endl;


  // 遍历过滤后的点云，将索引添加到 inliers 中  
  for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {  
      inliers->indices.push_back(i);
  }  

  return separateClouds(inliers, cloud);
}





/*
点云的聚类函数

*/
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ObstacleDetector<PointT>::clustering(
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const float cluster_tolerance, const int min_size, const int max_size) {
  // Time clustering process
  // const auto start_time = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec; //使用欧式距离进行聚类
  ec.setClusterTolerance(cluster_tolerance); //聚类的阈值，判断是否属于同一类
  ec.setMinClusterSize(min_size); //聚类点的最小数量
  ec.setMaxClusterSize(max_size); //聚类点的最大数量
  ec.setSearchMethod(tree); //使用kdTree来进行快速搜索
  ec.setInputCloud(cloud); 
  ec.extract(cluster_indices); //聚类的结果存储在cluster_indices


  /*
  遍历聚类索引cluster_indices，为每个聚类创建一个新的点云对象，并将这些点云对象添加到clusters向量中
  */
  for (auto &getIndices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>); //存储当前遍历到的聚类中的所有点。

    for (auto &index : getIndices.indices)
      cluster->points.push_back(cloud->points[index]); //索引对应的点从输入点云cloud中复制到新的点云对象cluster中。

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster); //将新的点云对象cluster添加到clusters向量中。clusters向量就包含了所有聚类的点云。
  }

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time =
  // std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
  // start_time); std::cout << "clustering took " << elapsed_time.count() << "
  // milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}





/*
计算点云的碰撞框的坐标
碰撞盒的表示方式
*/
template <typename PointT>
Box ObstacleDetector<PointT>::axisAlignedBoundingBox(
    const typename pcl::PointCloud<PointT>::ConstPtr &cluster, const int id) {
  // Find bounding box for one of the clusters
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);  //获取聚类的最大坐标和最小坐标

  const Eigen::Vector3f position((max_pt.x + min_pt.x) / 2,
                                 (max_pt.y + min_pt.y) / 2,
                                 (max_pt.z + min_pt.z) / 2);  //获取碰撞盒的中心坐标
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y),
                                  (max_pt.z - min_pt.z));

  return Box(id, position, dimension);
}




//通过PCA分析来计算一个更紧密的、与点云数据主成分方向对齐的边界盒。
template <typename PointT>
Box ObstacleDetector<PointT>::pcaBoundingBox(
    const typename pcl::PointCloud<PointT>::Ptr &cluster, const int id) {
  // Compute the bounding box height (to be used later for recreating the box)
  //使用主成分分析来计算碰撞盒的中心点，与直接创建碰撞盒不同，PCA方法将点云聚类压平到x-y平面上，之后重新创建碰撞盒。
  PointT min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  const float box_height = max_pt.z - min_pt.z;
  // const float box_z = (max_pt.z + min_pt.z) / 2;

  // Compute the cluster centroid
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cluster, pca_centroid);

  // Squash the cluster to x-y plane with z = centroid z
  for (size_t i = 0; i < cluster->size(); ++i) {
    cluster->points[i].z = pca_centroid(2);
  }



  // Compute principal directions & Transform the original cloud to PCA
  // coordinates
  //创建一个新的点云pca_projected_cloud，用于存储PCA变换后的点云。然后，使用pcl::PCA类对原始聚类进行PCA分析，并将结果投影到新的点云上。
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *pca_projected_cloud);

  const auto eigen_vectors = pca.getEigenVectors();

  // Get the minimum and maximum points of the transformed cloud.
  pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
  const Eigen::Vector3f meanDiagonal =
      0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf quaternion(
      eigen_vectors);  // Quaternions are a way to do rotations 使用PCA分析得到的特征向量来创建一个四元数（quaternion），这个四元数表示了将边界盒从PCA坐标系旋转回原始坐标系所需的旋转。
                       // https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f position =
      eigen_vectors * meanDiagonal + pca_centroid.head<3>();
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y),
                                  box_height);

  return Box(id, position, dimension, quaternion);
}




// ************************* Tracking ***************************
//据障碍物在连续帧之间的位置和大小变化来跟踪障碍物
template <typename PointT>
void ObstacleDetector<PointT>::obstacleTracking(
    const std::vector<Box> &prev_boxes, std::vector<Box> *curr_boxes,
    const float displacement_thresh, const float iou_thresh) {
  // Tracking (based on the change in size and displacement between frames)

  if (curr_boxes->empty() || prev_boxes.empty()) {
    return;
  } else {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, *curr_boxes,
                                           displacement_thresh, iou_thresh); //上一帧的障碍物盒子列表prev_boxes、当前帧的障碍物盒子列表curr_boxes、位移阈值displacement_thresh和交并比阈值iou_thresh

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix =
        connectionMatrix(connection_pairs, &pre_ids, &cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j) {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);

      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j];  // right and matches has the same size
      const auto cur_index = searchBoxIndex(*curr_boxes, cur_id);

      if (pre_index > -1 && cur_index > -1) {
        // change the id of the current box to the same as the previous box
        (*curr_boxes)[cur_index].id = prev_boxes[pre_index].id;
      }
    }
  }
}





template <typename PointT>
bool ObstacleDetector<PointT>::compareBoxes(const Box &a, const Box &b,
                                            const float displacement_thresh,
                                            const float iou_thresh) {
  // Percetage Displacements ranging between [0.0, +oo] 比较两个障碍物盒子a和b是否相似，基于它们的位移和尺寸差异
  const float dis =
      sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) +
           (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) +
           (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim =
      std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim =
      std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  // Dimension similiarity values between [0.0, 1.0]
  const float x_dim =
      2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim =
      2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim =
      2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh &&
      y_dim <= iou_thresh && z_dim <= iou_thresh) {
    return true;
  } else {
    return false;
  }
}





template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(
    const std::vector<Box> &prev_boxes, const std::vector<Box> &curr_boxes,
    const float displacement_thresh, const float iou_thresh) {
  std::vector<std::vector<int>> connection_pairs;

  for (auto &prev_box : prev_boxes) {
    for (auto &curBox : curr_boxes) {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (this->compareBoxes(curBox, prev_box, displacement_thresh,
                             iou_thresh)) {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  return connection_pairs;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::connectionMatrix(
    const std::vector<std::vector<int>> &connection_pairs,
    std::vector<int> *left, std::vector<int> *right) {
  // Hash the box ids in the connection_pairs to two vectors(sets), left and
  // right
  for (auto &pair : connection_pairs) {
    const bool left_found = std::any_of(left->begin(), left->end(),
                                        [pair](int i) { return i == pair[0]; });
    if (!left_found) left->push_back(pair[0]);
    const bool right_found = std::any_of(
        right->begin(), right->end(), [pair](int j) { return j == pair[1]; });
    if (!right_found) right->push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(
      left->size(), std::vector<int>(right->size(), 0));

  for (auto &pair : connection_pairs) {
    int left_index = -1;
    for (int i = 0; i < left->size(); ++i) {
      if ((*left)[i] == pair[0]) left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right->size(); ++i) {
      if ((*right)[i] == pair[1]) right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}


/*
hungarianFind函数，实现二分图左右两侧寻找最适合的匹配节点。
左侧表示当前帧的障碍物，右侧表示前一帧的障碍物。
*/
template <typename PointT>
bool ObstacleDetector<PointT>::hungarianFind(
    const int i, const std::vector<std::vector<int>> &connection_matrix,
    std::vector<bool> *right_connected, std::vector<int> *right_pair) {
  for (int j = 0; j < connection_matrix[0].size(); ++j) {
    if (connection_matrix[i][j] == 1 && (*right_connected)[j] == false) {
      (*right_connected)[j] = true;

      if ((*right_pair)[j] == -1 ||
          hungarianFind((*right_pair)[j], connection_matrix, right_connected,
                        right_pair)) {
        (*right_pair)[j] = i;
        return true;
      }
    }
  }

  return false;
}


/*
hungarian匹配算法入口
用于匹配连续帧之间检测的障碍物
*/
template <typename PointT>
std::vector<int> ObstacleDetector<PointT>::hungarian(
    const std::vector<std::vector<int>> &connection_matrix) {
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i) {
    if (hungarianFind(i, connection_matrix, &right_connected, &right_pair))
      count++;
  }

  std::cout << "For: " << right_pair.size()
            << " current frame bounding boxes, found: " << count
            << " matches in previous frame! " << std::endl;

  return right_pair;
}

template <typename PointT>
int ObstacleDetector<PointT>::searchBoxIndex(const std::vector<Box> &boxes,
                                             const int id) {
  for (int i = 0; i < boxes.size(); i++) {
    if (boxes[i].id == id) return i;
  }

  return -1;
}

}  // namespace lidar_obstacle_detector
