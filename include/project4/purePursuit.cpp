#include <project4/purePursuit.h>

purePursuit::purePursuit(){

}

control purePursuit::get_control(point x_robot, point x_goal){


    /* TO DO
     *
     * implement purepursuit algorithm
     *
    */
    auto goal_th =  atan2(x_goal.x - x_robot.x, x_goal.y - x_robot.y);
    auto diff_th = (goal_th + x_robot.th - M_PI_2) ;

    if(diff_th > M_PI){
        diff_th = diff_th - 2*M_PI;
    }
    if(diff_th < -M_PI){
        diff_th = diff_th + 2*M_PI;
    }


    if(fabs(diff_th) > this->threshold_th1){
        //turn fast
        this->ctrl.w = M_PI/6 * diff_th/fabs(diff_th) ;
        this->ctrl.v = 0.0;
    }
    else if(fabs(diff_th) > this->threshold_th2){
        //turn slow
        this->ctrl.w = M_PI/8 * diff_th/fabs(diff_th);
        this->ctrl.v = 0.0;
    }
    else{
        //persuit
        this->ctrl.w = M_PI/15 * diff_th/fabs(diff_th);
        this->ctrl.v = 0.45;
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
