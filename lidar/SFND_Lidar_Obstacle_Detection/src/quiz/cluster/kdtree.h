/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	// Also ok if we pass by pointer instead of passing by reference
	void InsertHelper(Node *&node, std::vector<float> point, int id, int depth)
	{

		// base case
		if (node == NULL)
		{
			node = new Node(point, id);
			return;
		}

		// recursive case

		// Comparison of x or y (or others) value depends on depth of tree
		uint coordIdx = depth % 2; // For 2D (x-axis = 0, y-axis = 1)

		if (point[coordIdx] < (node->point[coordIdx]))
		{
			InsertHelper(node->left, point, id, depth + 1);
		}

		else
		{
			InsertHelper(node->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		InsertHelper(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> SearchHelper(Node *&node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids)
	{
		int coordIdx = depth % 2; // for 2D

		// Base 1
		if (node == NULL)
		{
			return ids;
		}

		// Base 2 - node within bounding box
		else if (node->point[0] <= (target[0] + distanceTol) && node->point[0] >= (target[0] - distanceTol) &&
				 node->point[1] <= (target[1] + distanceTol) && node->point[1] >= (target[1] - distanceTol))
		{
			float distToTarget = CalcDistToTarget(node->point, target);

			// Add node's ID only if node's distance is within distance tolerance
			if (distToTarget <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		// Recursive 1 - if node's cutting axis is intersecting bounding box
		if (node->point[coordIdx] <= (target[coordIdx] + distanceTol) && node->point[coordIdx] >= (target[coordIdx] - distanceTol))
		{
			SearchHelper(node->left, target, distanceTol, depth + 1, ids);
			SearchHelper(node->right, target, distanceTol, depth + 1, ids);
		}

		// Recursive 2a - if node's cutting axis is NOT intersecting bounding box - min bound
		else if (node->point[coordIdx] < (target[coordIdx] - distanceTol))
		{
			SearchHelper(node->right, target, distanceTol, depth + 1, ids);
		}

		// Recursive 2b - if node's cutting axis is NOT intersecting bounding box - max bound
		else
		{
			SearchHelper(node->left, target, distanceTol, depth + 1, ids);
		}

		return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		SearchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}

	float CalcDistToTarget(std::vector<float> node, std::vector<float> target)
	{
		float sumSquared = 0;
		for (size_t i = 0; i < node.size(); i++)
		{
			sumSquared += ((node[i] - target[i]) * (node[i] - target[i]));
		}
		double dist = sqrt(sumSquared);
		return dist;
	}
};
