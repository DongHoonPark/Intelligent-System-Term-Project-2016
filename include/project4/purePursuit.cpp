#include <project4/purePursuit.h>

purePursuit::purePursuit(){

}


#define STD_V 0.30  // max speed is 1.2 m/s
#define STD_W1 0.50  // max angular speed is 5.235988 rad/sec (=300 degree/sec)
#define STD_W2 0.50


control purePursuit::get_control(point x_robot, point x_goal){


    /* TO DO
     *
     * implement purepursuit algorithm
     *
    */
    double rel_x = (x_goal.x - x_robot.x)*cos(x_robot.th) + (x_goal.y - x_robot.y)*sin(x_robot.th);
    double rel_y = -(x_goal.x - x_robot.x)*sin(x_robot.th) + (x_goal.y - x_robot.y)*cos(x_robot.th);
    double l_sqr = pow(rel_x,2) + pow(rel_y,2);
    if(fabs(rel_y) < 0.2 && rel_x > 0){   // in this case, it's better to go straight
        ctrl.v = STD_V;
        ctrl.w = 0.0;
        return ctrl;
    }
    double rel_th = atan(rel_y/rel_x);
    if(rel_x < 0) rel_y > 0? rel_th += M_PI: rel_th -= M_PI;
    if(fabs(rel_th) > M_PI/15.0){
        ctrl.v = (ctrl.v > 0.1? (ctrl.v - 0.1) :0.0);
        ctrl.w = (-(rel_th > 0) + (rel_th < 0)) * STD_W1;// -0.50*rel_th;
        return ctrl;
    } else {
        ctrl.v = STD_V;
        ctrl.w = (-(rel_th > 0) + (rel_th < 0)) * STD_W2;// -ctrl.v * 2.0 * rel_y / l_sqr;
    }
return ctrl;
}

bool purePursuit::is_reached(point x_robot, point x_goal) {
    auto diff = sqrt((x_robot.x - x_goal.x)*(x_robot.x - x_goal.x) + (x_robot.y - x_goal.y)*(x_robot.y - x_goal.y));
    if(diff < this->threshold_point){
        return true;
    }
    else{
        return false;
    }
}
