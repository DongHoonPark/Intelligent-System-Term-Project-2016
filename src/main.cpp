
//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#define MODE_DYNAMIC 1


#include <unistd.h>
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <project4/purePursuit.h>
#include <project4/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pwd.h>
#include <image_transport/image_transport.h>

//map spec
cv::Mat map;
cv::Mat dynamic_map; //use this variable at dynamic mapping
cv::Mat display_map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

void dynamic_mapping();
bool robot_pose_check();

//way points
std::vector<point> waypoints;
int waypoint_idx = 0;

point goalpoint;

//path
std::vector<point> path_RRT;
rrtTree* rrt;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//FSM state
int state;

//function definition
bool isCollision();
bool isReallyCollision(int idx);
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);

auto robot_pose_pitch = 0.0;
auto robot_pose_roll = 0.0;

int look_ahead_idx = 0;
auto back_step_flag = 0;


class DynamicMapper{
public:
	DynamicMapper(cv::Mat& dynamic_map){
		this->dynamic_map = dynamic_map;
	}
	void dynamic_mapping();
	cv::Mat dynamic_map;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1);
    ros::Subscriber gazebo_camera_sub = n.subscribe("/camera/depth/points",1,callback_points);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

// Load Map
    auto userpw = getpwuid(getuid());
//    char* user = getlogin();
    char* user = userpw->pw_name;

    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project4/src/ground_truth_map_sin2.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = (map.cols == 0)? 800: map.cols;
    map_x_range = (map.rows == 0)? 800: map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;
    printf("Load map\n");

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    //Temp goalpoint
    goalpoint = waypoints[1];
    dynamic_map = map.clone();
    display_map = map.clone();

    cv::Mat dynamic_map_circledetection = dynamic_map.clone();
    cv::Mat dynamic_map_linedetection = dynamic_map.clone();

    if(MODE_DYNAMIC){
        cv::GaussianBlur(dynamic_map, dynamic_map_circledetection, cv::Size(3, 3), 2, 2 );
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles( dynamic_map_circledetection, circles, CV_HOUGH_GRADIENT, 1, 8, 50, 15, 0, 10 );

        for( size_t i = 0; i < circles.size(); i++ ){
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

            // circle center
            cv::circle( dynamic_map, center, 10, cv::Scalar(255,255,255), CV_FILLED);
         }

        cv::Canny(dynamic_map, dynamic_map_linedetection, 200, 1000, 3);
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(dynamic_map_linedetection, lines, 1, CV_PI/180, 40, 20, 10 );
        for( size_t i = 0; i < lines.size(); i++ ){
            cv::Point pt1, pt2;
            pt1.x = lines[i][0];
            pt1.y = lines[i][1];
            pt2.x = lines[i][2];
            pt2.y = lines[i][3];

            cv::line( dynamic_map, pt1, pt2, cv::Scalar(0,0,0),15);
        }
    }

    // Real-time mapping consts.
    int dm_iter = 0;
//    DynamicMapper dynamic_mapper(dynamic_map);
//    boost::function<void()> th_func = boost::bind(&DynamicMapper::dynamic_mapping, &dynamic_mapper);
//    boost::thread th(th_func);
//    th.join();

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    ros::Rate control_rate(20);


//    cv::namedWindow( "debug path" );
    cv::namedWindow( "dm" );    // Create a window for display.
    cv::startWindowThread();
    cv::imshow( "dm", display_map );
//    cv::imshow( "debug path", display_map );
    cv::waitKey(1000);          // Wait for a keystroke in the window


//    image_transport::ImageTransport it(n);
//    image_transport::Publisher dynamic_map_pub = it.advertise("/dynamic_map", 100);
//    cv_bridge::CvImage dynamic_map_msg;
//    dynamic_map_msg.header.frame_id = "image";
//    dynamic_map_msg.encoding = sensor_msgs::image_encodings::MONO8;
//    dynamic_map_msg.header.stamp = ros::Time::now();
//    dynamic_map_msg.image = map;
//    auto msg = dynamic_map_msg.toImageMsg();
//    dynamic_map_pub.publish(msg);
//    ros::spin();


    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
            //visualize path
//            for(int i = 0; i < path_RRT.size(); i++){
//                gazebo_msgs::SpawnModel model;
//                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
//                        std::string("<link name=\"ball\">") +
//                        std::string("<inertial>") +
//                        std::string("<mass value=\"1.0\" />") +
//                        std::string("<origin xyz=\"0 0 0\" />") +
//                        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
//                        std::string("</inertial>") +
//                        std::string("<visual>") +
//                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
//                        std::string("<geometry>") +
//                        std::string("<sphere radius=\"0.09\"/>") +
//                        std::string("</geometry>") +
//                        std::string("</visual>") +
//                        std::string("<collision>") +
//                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
//                        std::string("<geometry>") +
//                        std::string("<sphere radius=\"0\"/>") +
//                        std::string("</geometry>") +
//                        std::string("</collision>") +
//                        std::string("</link>") +
//                        std::string("<gazebo reference=\"ball\">") +
//                        std::string("<mu1>10</mu1>") +
//                        std::string("<mu2>10</mu2>") +
//                        std::string("<material>Gazebo/Blue</material>") +
//                        std::string("<turnGravityOff>true</turnGravityOff>") +
//                        std::string("</gazebo>") +
//                        std::string("</robot>");
//
//                std::ostringstream ball_name;
//                ball_name << i;
//                model.request.model_name = ball_name.str();
//                model.request.reference_frame = "world";
//                model.request.initial_pose.position.x = path_RRT[i].x;
//                model.request.initial_pose.position.y = path_RRT[i].y;
//                model.request.initial_pose.position.z = 1.2;
//                model.request.initial_pose.orientation.w = 0.0;
//                model.request.initial_pose.orientation.x = 0.0;
//                model.request.initial_pose.orientation.y = 0.0;
//                model.request.initial_pose.orientation.z = 0.0;
//
//                gazebo_spawn.call(model);
//
//                ros::spinOnce();
//                ros::Rate(200).sleep();
//            }
//            printf("Spawn path\n");

            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.4;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "RosAria";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");

            state = RUNNING;
        } break;

        case RUNNING: {
            //TODO
            /*
             * copy your code from previous project2
             */
            if(isCollision()){
                setcmdvel(-0.1,0);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                ros::Duration(1).sleep();

                dynamic_mapping();
                if(isReallyCollision(look_ahead_idx))
                	   state = PATH_PLANNING;
                ros::spinOnce();
                cv::circle(dynamic_map,
                           cv::Point(
                                   (int)(robot_pose.y / 0.05 + map_origin_y),
                                   (int)(robot_pose.x / 0.05 + map_origin_x)
                           ),
                           12,
                           cv::Scalar(255, 255, 255),
                           CV_FILLED);

//                cv::imshow("dm", display_map);
//                cv::waitKey(3);

//                state = PATH_PLANNING;
            }
            else{
                if(pure_pursuit.is_reached(robot_pose, path_RRT[look_ahead_idx])){

                    look_ahead_idx++;
                    if(look_ahead_idx == path_RRT.size()){
                        waypoint_idx++;
                        if(waypoint_idx == waypoints.size()){
                            state = FINISH;
                        }
                        else{
                            setcmdvel(0,0);
                            cmd_vel_pub.publish(cmd_vel);
                            goalpoint = waypoints[waypoint_idx];
                            state = PATH_PLANNING;
                        }
                    }
                }
                else{
                	auto ctrl = pure_pursuit.get_control(robot_pose, path_RRT[look_ahead_idx]);

                	setcmdvel(ctrl.v, ctrl.w);
                	cmd_vel_pub.publish(cmd_vel);

                	if(++dm_iter == 8){
                		dynamic_mapping();
                		dm_iter = 0;
                	}
                	rrt->setDynamicMap(&dynamic_map);
                	std::vector<point> path_RRT_remain(path_RRT.begin()+look_ahead_idx-1, path_RRT.end());
                	if( !rrt->checkPathValidity(path_RRT_remain) ){
                		setcmdvel(0,0);
                		cmd_vel_pub.publish(cmd_vel);
                		state = PATH_PLANNING;
                	}
                }
            }

            /*
             * add transition part from RUNNING to PATH_PLANNING
             * when meeting obstacle
             */

            ros::spinOnce();

            cv::circle(dynamic_map,
                       cv::Point(
                               (int)(robot_pose.y / 0.05 + map_origin_y),
                               (int)(robot_pose.x / 0.05 + map_origin_x)
                       ),
                       10,
                       cv::Scalar(255, 255, 255),
                       CV_FILLED);
            cv::circle(display_map,
                       cv::Point(
                               (int)(robot_pose.y / 0.05 + map_origin_y),
                               (int)(robot_pose.x / 0.05 + map_origin_x)
                       ),
                       10,
                       cv::Scalar(255, 255, 255),
                       CV_FILLED);

            cv::imshow("dm", display_map);
            cv::waitKey(3);
            control_rate.sleep();
            ROS_INFO("%d", look_ahead_idx);
        } break;

        case PATH_PLANNING: {
            //TODO
            /*
             * do dynamic mapping from kinect data
             * pop up the opencv window
             * after drawing the dynamic map, transite the state to RUNNING state
             */
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);

            ros::spinOnce();
            control_rate.sleep();
            generate_path_RRT();
            state = RUNNING;
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);
            running = false;

            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
        //ROS_INFO("%f", robot_pose_pitch);
        //printf("curr state : %d\ncurr robot pos : %.2f,%.2f\ncurr robot vel : %.2f,%.2f\n",state,robot_pose.x,robot_pose.y,cmd_vel.linear.x,cmd_vel.angular.z);
    }

    return 0;
}

void generate_path_RRT()
{
    //TODO
    /*
     * copy your code from previous project2
     */
    point current_pos;

    if(state == INIT){
        current_pos.x = waypoints[0].x;
        current_pos.y = waypoints[0].y;
    }
    else{
        current_pos.x = robot_pose.x;
        current_pos.y = robot_pose.y;
    }

    rrt = new rrtTree(current_pos, goalpoint, dynamic_map, map_origin_x, map_origin_y, res, 12);
    rrt->setDynamicMap(&dynamic_map);

    while(true){
        ros::spinOnce();
        current_pos.x = robot_pose.x;
        current_pos.y = robot_pose.y;
        auto rrtResult = rrt->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 10000, 2.5);
        if(rrtResult == 0){
            break;
        }
        else if(rrtResult == 2){
            ros::spinOnce();
            current_pos.x = robot_pose.x;
            current_pos.y = robot_pose.y;
                    cv::circle(dynamic_map,
                    cv::Point(
                           (int)(current_pos.y / 0.05 + map_origin_y),
                           (int)(current_pos.x / 0.05 + map_origin_x)
                    ),
                    20,
                    cv::Scalar(255, 0, 255),
                    CV_FILLED);

//            cv::imshow("dm", dynamic_map);
//            cv::waitKey(30);
            setcmdvel(-0.1,0);
        }
        rrt = new rrtTree(current_pos, goalpoint, dynamic_map, map_origin_x, map_origin_y, res, 12);
        rrt->setDynamicMap(&dynamic_map);
    };


    auto result = rrt->backtracking();

    while(look_ahead_idx < path_RRT.size()){
        path_RRT.pop_back();
    }
    path_RRT.insert(path_RRT.end(), result.begin(), result.end());
    /*
    cv::Mat display_map_with_path = display_map.clone();
    for(auto i=0; i<path_RRT.size(); i++){
        cv::circle(display_map_with_path,
            cv::Point(
                (int)(path_RRT[i].y / 0.05 + map_origin_y),
                (int)(path_RRT[i].x / 0.05 + map_origin_x)
             ),
            10,
            cv::Scalar(0, 0, 255),
            CV_FILLED);
    }
    cv::imshow("debug path", display_map_with_path);
    cv::waitKey(30);
    */
}

void set_waypoints()
{
    // scenario 1 sample way points
    point waypoint_candid[3];
    waypoint_candid[0].x = -6.0;
    waypoint_candid[0].y = 0.0;
    waypoint_candid[1].x = 3.0;
    waypoint_candid[1].y = 1.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 7.0;
    int order[] = {0,1,2};
    int order_size = 3;

//    point waypoint_candid[3];
//    waypoint_candid[0].x = -5.0;
//    waypoint_candid[0].y = -4.0;
//    waypoint_candid[1].x = 5.0;
//    waypoint_candid[1].y = 6.0;
//    waypoint_candid[2].x = -8.0;
//    waypoint_candid[2].y = 8.0;
//    int order[] = {0,1,2};
//    int order_size = 3;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
            auto q = msgs->pose[i].orientation;
            robot_pose_pitch = asin(2*(q.x*q.z - q.w*q.y));
            robot_pose_roll = atan2(2*(q.z*q.y - q.x*q.w), 1-2*(q.y*q.y + q.x*q.x));
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

bool isCollision()
{
    //TODO
    /*
     * obstacle emerge in front of robot -> true
     * other wise -> false
     */
    if(!robot_pose_check()){
        ROS_INFO("robot pos is illegal!");
        setcmdvel(-0.1,0);
        return false;
    }
    auto collision_point = 0;
    pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter;
    for(pc_iter = point_cloud.points.begin(); pc_iter < point_cloud.points.end(); pc_iter++){
        // robot_frame(x,y,z) = (pc_iter->z, -(pc_iter->x), -(pc_iter->y))
        // return true if an obstacle is close enough to the robot's face
        if(pc_iter->z  < 0.7){
            if(fabs(pc_iter->x) < 0.8 && pc_iter->y < 1.0 && pc_iter->y >0.3){
                 collision_point++;
            }
        }
    }
    if(collision_point > 10){
        return true;
    }
    return false;
}

bool isReallyCollision(int idx){
    // Pioneer has radius about 40cm(= 8px).
    int pixel_xrange = 8;
    int pixel_yrange = 8;

    auto x1_x_idx = (int)(robot_pose.x / res + map_origin_x);
    auto x1_y_idx = (int)(robot_pose.y / res + map_origin_y);
    auto x2_x_idx = (int)(path_RRT[idx].x / res + map_origin_x);
    auto x2_y_idx = (int)(path_RRT[idx].y / res + map_origin_y);

    double diff_x = x1_x_idx - x2_x_idx;
    double diff_y = x1_y_idx - x2_y_idx;
    double len = sqrt(diff_x*diff_x + diff_y*diff_y);
    int pnum = (int)(len);
    if(pnum < 0){
        return false;
    }
    else{
    	for(auto i=0; i< pnum; i++){
    		auto sample_x = x1_x_idx + (int)((x2_x_idx - x1_x_idx) * 1.0 * i / pnum);
           auto sample_y = x1_y_idx + (int)((x2_y_idx - x1_y_idx) * 1.0 * i / pnum);
           for(auto j=0; j < pixel_xrange; j++){
               for(auto k=0; k < pixel_yrange; k++){
                   auto row = sample_x - pixel_xrange/2 + j;
                   auto col = sample_y - pixel_yrange/2 + k;
                   if(dynamic_map.cols == 0 || dynamic_map.rows == 0){
                       printf("Error! (map.rows or map.cols is 0)\n"); return true;
                      }
                   auto pixel = dynamic_map.at<uchar>(row, col);
                   if(pixel != 255){
                       return true;
                   }
               }
           }
    	}
		return false;
    }
}

void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}

void dynamic_mapping()
{
    //TODO
    /*
    * draw dynamic map using variable dynamic_map and variable point_cloud
    */
    if(!robot_pose_check()){
        ROS_INFO("robot pos is illegal!");
        setcmdvel(-0.1,0);
        return;
    }
//    ros::Duration(0.25).sleep();
//    ros::spinOnce();
    pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter;
    for(pc_iter = point_cloud.points.begin(); pc_iter < point_cloud.points.end(); pc_iter++){
        // Kinect frame => Grid map frame
        if(pc_iter->x != NAN && pc_iter->z != NAN){
            if(pc_iter->x > 0 && pc_iter->x < 10.0 && pc_iter->y <0 && pc_iter->y > -10.0) {
                int pos_x = (int) (
                        (cos(robot_pose.th) * (pc_iter->z) + sin(robot_pose.th) * (pc_iter->x) + robot_pose.x) / res +
                        map_origin_x);
                int pos_y = (int) (
                        (sin(robot_pose.th) * (pc_iter->z) - cos(robot_pose.th) * (pc_iter->x) + robot_pose.y) / res +
                        map_origin_y);
                //dynamic_map.at<uchar>(pos_x, pos_y) = 0;
                cv::circle(dynamic_map,
                           cv::Point(
                                   pos_y,
                                   pos_x
                           ),
                           2,
                           cv::Scalar(0, 0, 255),
                           CV_FILLED);
                cv::circle(display_map,
                           cv::Point(
                                   pos_y,
                                   pos_x
                           ),
                           2,
                           cv::Scalar(0, 0, 255),
                           CV_FILLED);
            }
        }
    }

    cv::imshow("dm", display_map);
    cv::waitKey(10);
}

bool robot_pose_check(){
    if(fabs(robot_pose_pitch) < M_PI/24
       && fabs(robot_pose_roll) < M_PI/24){
        return true;
    }
    else{
        return false;
    }
}

