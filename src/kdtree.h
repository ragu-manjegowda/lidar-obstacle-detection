/* \author Ragu Manjegowda */

#ifndef CUSTOM_KD_TREE_H_
#define CUSTOM_KD_TREE_H_

#include "./render/render.h"

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

template <typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree() : root(NULL) {}

    void insertHelper(Node<PointT>*& node, size_t depth, PointT point, int id)
    {
        if (node == nullptr)
        {
            node = new Node<PointT>(point, id);
            return;
        }

        int currentDepth = depth % 3;

        switch (currentDepth)
        {
            case 0:
                if (point.x <= node->point.x)
                {
                    insertHelper(node->left, ++depth, point, id);
                }
                else
                {
                    return insertHelper(node->right, ++depth, point, id);
                }
                break;
            case 1:
                if (point.y <= node->point.y)
                {
                    insertHelper(node->left, ++depth, point, id);
                }
                else
                {
                    return insertHelper(node->right, ++depth, point, id);
                }
                break;
            case 2:
                if (point.z <= node->point.z)
                {
                    insertHelper(node->left, ++depth, point, id);
                }
                else
                {
                    return insertHelper(node->right, ++depth, point, id);
                }
                break;
            default:
                break;
        }
    }

    void insert(PointT point, int id)
    {
        // Done: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the
        // root

        insertHelper(root, 0, point, id);
    }

    void searchHelper(Node<PointT>* node,
                      int depth,
                      std::vector<int>& ids,
                      PointT target,
                      float distanceTol)
    {
        if (node == nullptr)
        {
            return;
        }

        int ind = depth % 3;

        float dist;
        float pointDimVal;
        float targetDimVal;
        if (ind == 0)
        {
            pointDimVal = node->point.x;
            targetDimVal = target.x;
            dist = node->point.x - target.x;
        }
        else if (ind == 1)
        {
            pointDimVal = node->point.y;
            targetDimVal = target.y;
            dist = node->point.y - target.y;
        }
        else if (ind == 2)
        {
            pointDimVal = node->point.z;
            targetDimVal = target.z;
            dist = node->point.z - target.z;
        }

        if (abs(dist) > distanceTol)
        {
            if (pointDimVal >= targetDimVal)
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
            float dis =
                sqrt(pow(node->point.x - target.x, 2) + pow(node->point.y - target.y, 2) +
                     pow(node->point.z - target.z, 2));

            if (dis < distanceTol)
            {
                ids.push_back(node->id);
            }

            searchHelper(node->left, ++depth, ids, target, distanceTol);
            searchHelper(node->right, ++depth, ids, target, distanceTol);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, 0, ids, target, distanceTol);
        return ids;
    }
};
#endif /* CUSTOM_KD_TREE_H_ */
