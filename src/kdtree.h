/**
	KD-Tree template for PCL
    @author Thinh Lu
    @version 1.0 10/29/20 
*/

#include "./render/render.h"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>

// Note structure for KD-Tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT setPoint, int setId)
	:	point(setPoint), id(setId), left(NULL), right(NULL)
	{}
};

// KD-Tree 3D Template for PCL
template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}


/**
 * Find new median and assign that PointT to the given node.
 * Two new sub-indices (left & right) are created and passed to the next recursive calls
 * to build the left & right branches of the subtree.
 * 
 * @param node the root node of this subtree (where median will be assigned to)
 * @param points reference to the point list
 * @param indices remaining indices that belongs to this subtree
 * @param depth level of the current node, decides which dimension will be used for sorting.
 */
	void buildTreeHelper(Node<PointT>** node, std::vector<PointT, Eigen::aligned_allocator<PointT>> &points, std::vector<int> &indices, uint depth)
	{	if (indices.size()==1)
		{	*node = new Node<PointT>(points[indices.front()], indices.front());
			return;
		}
		auto comparator = [&points, &depth](int a_idx, int b_idx) { return points[a_idx].data[depth%3] < points[b_idx].data[depth%3]; };
		std::sort(indices.begin(), indices.end(), comparator);
		int mid = indices.size()/2;
		int median_idx = indices[mid];

		*node = new Node<PointT>(points[median_idx], median_idx);
		if (mid > 0) {
			std::vector<int> left_indices(indices.begin(), indices.begin()+mid);
			buildTreeHelper(&((*node)->left), points, left_indices, depth+1);
		}
		if (mid+1 < indices.size()) 
		{	std::vector<int> right_indices(indices.begin()+(mid+1), indices.end());
			buildTreeHelper(&((*node)->right), points, right_indices, depth+1);
		}
	}

/**
 * Build a balanced KDTree with PointCloud data
 * @param points reference to the point list
 */
	void buildTree(std::vector<PointT, Eigen::aligned_allocator<PointT>> &points)
	{
		std::vector<int> indices(points.size());
		std::iota(indices.begin(), indices.end(), 0);
		buildTreeHelper(&root, points, indices, 0);
	}

/**
 * Return a list of point indices that are within distance of target
 * @param target target point to find nearest neighbors
 * @param node current node to check euclidian distance with target
 * @param depth level of the current node, decides which dimension will be considered when searching left & right branch.
 * @param distanceTol distance tolerance for KD-Tree nearest neighbor search
 */
	void searchHelper(PointT target, Node<PointT>* node, uint depth, float distanceTol, std::vector<int> &ids)
	{
		 
		if (node != NULL) {
			if (  ((node->point.x>=(target.x-distanceTol)) && (node->point.x<=(target.x+distanceTol))) && 
			   	  ((node->point.y>=(target.y-distanceTol)) && (node->point.y<=(target.y+distanceTol))) &&
				  ((node->point.z>=(target.z-distanceTol)) && (node->point.z<=(target.z+distanceTol)))
			   )
			{	
				float distance = std::sqrt(std::pow(node->point.x-target.x, 2.0)+std::pow(node->point.y-target.y, 2.0)+std::pow(node->point.z-target.z, 2.0));
				if (distance<=distanceTol)
					ids.push_back(node->id);
			}
			int cd = depth % 3;

			if ((target.data[cd]-distanceTol)<node->point.data[cd]){
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if ((target.data[cd]+distanceTol)>node->point.data[cd]){
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

/**
 * Find nearest neighbors of the target point
 * @param target target PointT
 * @param distanceTol distance tolerance
 */
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}	

};