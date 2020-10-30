// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    region.setInputCloud(cloudRegion);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> &inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles (new pcl::PointCloud<PointT>), cloud_ground (new pcl::PointCloud<PointT>);
    
    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloud_ground->points.push_back(point);
        else
            cloud_obstacles->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_ground);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    uint n = cloud->points.size();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> cluster_indices;
    std::vector<bool> processed(n, false);
    
    // Build KD-Tree
    KdTree<PointT>* tree = new KdTree<PointT>;
    tree->buildTree(cloud->points);

    // Nearest Neighbors Search
    for (uint i=0; i<n; ++i)
    {   if (!processed[i]) {
          std::vector<int> nearest;
          KDTreeClustering(i, cloud, tree, clusterTolerance, nearest, processed);
          if ((nearest.size() >= minSize) && (nearest.size() <= maxSize))
            cluster_indices.push_back(nearest);
        }
    }

    // Indices to Points
    for (std::vector<int> indice_group : cluster_indices)
    {   
        typename pcl::PointCloud<PointT>::Ptr clustered_cloud (new pcl::PointCloud<PointT>);
        for (int idx : indice_group)
            clustered_cloud->push_back((*cloud)[idx]);
        clustered_cloud->width = clustered_cloud->size ();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;
        clusters.push_back(clustered_cloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

/* __________________________________
 *          
 *   RANSAC ALGO FOR SEGMENTATION
 * __________________________________
 */
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    PointT p[3];

  for (int t=0; t<maxIterations; ++t)
  {
    std::unordered_set<int> inliers;
    // insert three points
    
    for (int i=0; i<3; ++i) {
      int idx = rand() % cloud->points.size();
      p[i] =cloud->points[idx];
    }
    
    // calculate normal vector
    float a = (p[1].y-p[0].y)*(p[2].z-p[0].z)-(p[1].z-p[0].z)*(p[2].y-p[0].y);
    float b = (p[1].z-p[0].z)*(p[2].x-p[0].x)-(p[1].x-p[0].x)*(p[2].z-p[0].z);
    float c = (p[1].x-p[0].x)*(p[2].y-p[0].y)-(p[1].y-p[0].y)*(p[2].x-p[0].x);
    float d = -(a*p[0].x + b*p[0].y + c*p[0].z);
    float norm_n = std::sqrt(a*a + b*b + c*c);
    
    float distance;
    for (int i=0; i<cloud->points.size(); ++i)
    { if (inliers.count(i)>0)
        continue;
      auto point = cloud->points[i];
      float distance = std::fabs(a*point.x + b*point.y + c*point.z + d) / norm_n;
      if (distance <= distanceTol)
        inliers.insert(i);
    }
    if (inliers.size() > inliersResult.size())
        inliersResult = inliers;
  }
    
    return inliersResult;
}

/* __________________________________
 *          
 *   KD-TREE FOR CLUSTERING
 * __________________________________
 */

template<typename PointT>
void ProcessPointClouds<PointT>::KDTreeClustering(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, std::vector<int> &cluster, std::vector<bool> &processed)
{ processed[idx] = true;
  cluster.push_back(idx);
  std::vector<int> nearby_points = tree->search(cloud->points[idx], distanceTol);
  
  for (int indice:nearby_points)
  {   
      if (!processed[indice])
        KDTreeClustering(indice, cloud, tree, distanceTol, cluster, processed);
  }
}