#include <iostream>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#ifndef POINT_H
    #define POINT_H
    #include <project4/point.h>
#endif

class rrtTree
{
private:
    struct node
    {
        int idx;
        point rand;
        point location;
        double cost;    // For RRT*
        int idx_parent;
    }*root;

    int count;
    point x_init, x_goal;

    cv::Mat map;
    cv::Mat* dynamic_map_ptr;
    double map_origin_x, map_origin_y;
    double res;
    node *ptrTable[20000];

    cv::Mat addMargin(cv::Mat map, int margin);
    void visualizeTree();
    void addVertex(point x_new, point x_rand, int idx_near);
    int nearestNeighbor(point x_rand);
    bool isCollision(point x1, point x2);
    point randomState(double x_max, double x_min, double y_max, double y_min);
    point newState(int idx_near, point x_rand, double MaxStep);

    // TODO
    // Declare member functions need to implement RRT-Star
    std::vector<int> nearNeighbors(point, double);
    void addVertexAndCost(point x_new, point x_rand, int idx_min, double cost);
    void changeEdge(int idx, int idx_parent, double c);
    double getC(point, point);
    void optimizePath();    // RRT*-smart implementation. Probabily unused?

public:
    rrtTree();
    rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin);
    ~rrtTree();

    int generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);
    std::vector<point> backtracking();

    // Updated part for RRT-Star
    int generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);

    void visualizeTree(std::vector<point> path);

    int checkPoint(point tocheck);

    void setDynamicMap(cv::Mat* dm);
    void resetDynamicMap();
    cv::Mat map_original;
};
