/* \author Ragu Manjegowda */
/* \adopted from Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

struct KdTree
{
    Node* root;

    KdTree() : root(NULL) {}

    void insertHelper(Node*& node, size_t depth, std::vector<float> point, int id)
    {
        if (node == nullptr)
        {
            node = new Node(point, id);
            return;
        }

        int currentDepth = depth % 3;

        if (point[currentDepth] <= node->point[currentDepth])
        {
            insertHelper(node->left, ++depth, point, id);
        }
        else
        {
            return insertHelper(node->right, ++depth, point, id);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        // Done: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the
        // root

        insertHelper(root, 0, point, id);
    }

    void searchHelper(Node* node,
                      int depth,
                      std::vector<int>& ids,
                      std::vector<float> target,
                      float distanceTol)
    {
        if (node == nullptr)
        {
            return;
        }

        int ind = depth % 3;

        if (abs(node->point[ind] - target[ind]) > distanceTol)
        {
            if (node->point[ind] >= target[ind])
            {
                searchHelper(node->left, ++depth, ids, target, distanceTol);
            }
            else
            {
                searchHelper(node->right, ++depth, ids, target, distanceTol);
            }
        }
        else
        {
            float dis = sqrt(pow(node->point[0] - target[0], 2) +
                             pow(node->point[1] - target[1], 2) +
                             pow(node->point[2] - target[2], 2));

            if (dis < distanceTol)
            {
                ids.push_back(node->id);
            }

            searchHelper(node->left, ++depth, ids, target, distanceTol);
            searchHelper(node->right, ++depth, ids, target, distanceTol);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, 0, ids, target, distanceTol);
        return ids;
    }
};
