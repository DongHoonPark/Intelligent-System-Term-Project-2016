
//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
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
#include <pwd.h>
#include <image_transport/image_transport.h>

//map spec
cv::Mat map;
cv::Mat dynamic_map; //use this variable at dynamic mapping
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
bool isNewObject(double, double);

//way points
std::vector<point> waypoints;
int waypoint_idx = 0;

point goalpoint;

//path
std::vector<point> path_RRT;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//FSM state
int state;

//function definition
int isCollision();
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);

auto robot_pose_pitch = 0.0;
auto robot_pose_roll = 0.0;

int look_ahead_idx = 0;

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
                      std::string("/catkin_ws/src/project4/src/ground_truth_map_sin1.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
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

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    ros::Rate control_rate(10);


//    cv::namedWindow( "debug path" );
//    cv::namedWindow( "dm" );// Create a window for display.
//    cv::imshow( "dm", dynamic_map );
//    cv::imshow( "debug path", dynamic_map );
//    cv::waitKey(1000);                                          // Wait for a keystroke in the window


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
            for(int i = 0; i < path_RRT.size(); i++){
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;

                gazebo_spawn.call(model);

                ros::spinOnce();
                ros::Rate(200).sleep();
            }
            printf("Spawn path\n");

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

            dynamic_map = map.clone();

            state = RUNNING;
        } break;

        case RUNNING: {
            //TODO
            /*
             * copy your code from previous project2
             */
        	int collision_side = isCollision();
            if(collision_side){
                setcmdvel(-0.1,0);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                ros::Rate(0.5).sleep();
                setcmdvel(0, collision_side*0.2);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                ros::Rate(1).sleep();
                state = PATH_PLANNING;
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
                }
            }

            /*
             * add transition part from RUNNING to PATH_PLANNING
             * when meeting obstacle
             */

            ros::spinOnce();
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

            dynamic_mapping();

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

    auto rrt = new rrtTree(current_pos, goalpoint, dynamic_map, map_origin_x, map_origin_y, res, 12);

    while(rrt->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 2000, 2.5)==-1){
        ros::spinOnce();
        current_pos.x = robot_pose.x;
        current_pos.y = robot_pose.y;
        dynamic_map = map.clone();
        rrt = new rrtTree(current_pos, goalpoint, dynamic_map, map_origin_x, map_origin_y, res, 12);
    };

    auto result = rrt->backtracking();

    while(look_ahead_idx < path_RRT.size()){
        path_RRT.pop_back();
    }
    path_RRT.insert(path_RRT.end(), result.begin(), result.end());
    cv::Mat dynamic_map_with_path = dynamic_map.clone();
    for(auto i=0; i<path_RRT.size(); i++){
        cv::circle(dynamic_map_with_path,
            cv::Point(
                (int)(path_RRT[i].y / 0.05 + map_origin_y),
                (int)(path_RRT[i].x / 0.05 + map_origin_x)
             ),
            10,
            cv::Scalar(0, 0, 255),
            CV_FILLED);
    }
    cv::imshow("debug path", dynamic_map_with_path);
    cv::waitKey(30);
}

void set_waypoints()
{
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -7.0;
    waypoint_candid[1].x = -3.0;
    waypoint_candid[1].y = -3.0;
    waypoint_candid[2].x = -8.0;
    waypoint_candid[2].y = 8.0;
    waypoint_candid[3].x = 8.0;
    waypoint_candid[3].y = 8.0;
    int order[] = {0,1,2,3};
    int order_size = 4;

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

int isCollision()
{
    //TODO
    /*
     * obstacle emerge in front of robot -> true
     * other wise -> false
     */
    if(!robot_pose_check()){
        ROS_INFO("robot pos is illegal!");
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter;
    for(pc_iter = point_cloud.points.begin(); pc_iter < point_cloud.points.end(); pc_iter++){
        // robot_frame(x,y,z) = (pc_iter->z, -(pc_iter->x), -(pc_iter->y))
        // return true if an obstacle is close enough to the robot's face
        if(pc_iter->z  < 0.7){
            if(fabs(pc_iter->x) < 1.0 && pc_iter->y < 2.0 && pc_iter->y >0.3){
                if(isNewObject(pc_iter->z, -(pc_iter->x)))
                	   return (pc_iter->x > 0)? -1: 1;
            }
        }
    }
    return 0;
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
        return;
    }
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
                dynamic_map.at<uchar>(pos_x, pos_y) = 0;
            }
        }
    }

    cv::imshow("dm", dynamic_map);
    cv::waitKey(10);
}

bool robot_pose_check(){
    if(fabs(robot_pose_pitch) < M_PI/12
       && fabs(robot_pose_roll) < M_PI/12){
        return true;
    }
    else{
        return false;
    }
}

bool isNewObject(double x, double y){
	auto row = (int)((robot_pose.x + x*cos(robot_pose.th) + y*sin(robot_pose.th))/res + map_origin_x);
	auto col = (int)((robot_pose.y + x*sin(robot_pose.th) - y*cos(robot_pose.th))/res + map_origin_y);
	if(map.at<uchar>(row, col) == 255)
		return true;
	else
		return false;
}
