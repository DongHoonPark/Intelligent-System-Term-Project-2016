#include <project4/control.h>
#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project4/point.h>
#endif

class purePursuit{
public:
    purePursuit();

    //this function makes control input using arguments which are current robot position and current following goal.
    //x_goal means look-ahead point.
    control get_control(point x_robot, point x_goal);
    bool is_reached(point x_robot, point x_goal);

private:
    //"control" is data type defined by TA, and it contains variable v, w which are linear and angular velocity, respectively.
    control ctrl;
    const double threshold_th1 = M_PI/3;
    const double threshold_th2 = M_PI/8;
    const double threshold_point = 0.2;
};
