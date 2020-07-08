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
	void insertHelper(Node *&node, std::vector<float> point, int id, int depth)
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
			insertHelper(node->left, point, id, depth + 1);
		}

		else
		{
			insertHelper(node->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertHelper(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
