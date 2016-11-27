#include "rrtTree.h"

rrtTree::rrtTree(){
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(30);

}

void rrtTree::visualizeTree(std::vector<point> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map_original, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(x_init.y/res + map_origin_y)), (int)(Res*(x_init.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(x_goal.y/res + map_origin_y)), (int)(Res*(x_goal.x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        x1 = cv::Point((int)(Res*(ptrTable[i]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[i]->location.x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(ptrTable[idx_parent]->location.y/res + map_origin_y)), (int)(Res*(ptrTable[idx_parent]->location.x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        x1 = cv::Point((int)(Res*(path[i-1].y/res + map_origin_y)), (int)(Res*(path[i-1].x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }

    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(30);

}



// TODO
// 1. Copy your implementation of member functions in Project Assignment #2
// 2. Implement generateRRTst

int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep)
{
    // TODO 2.
}



void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    // TODO
    node* node_parent = this->ptrTable[idx_near];
    if(isCollision(node_parent->location, x_new)){
        return;
    }
    auto dx = node_parent->location.x - x_new.x;
    auto dy = node_parent->location.y - x_new.y;
    auto len = sqrt(dx*dx + dy*dy);
    if(len < 1){
        return;
    }
    this->ptrTable[this->count] = new node;
    node* node_toadd = this->ptrTable[this->count];
    node_toadd->idx_parent = idx_near;
    node_toadd->idx = this->count;
    node_toadd->location = x_new;
    node_toadd->rand = x_rand;
    this->count++;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    // TODO
    bool is_rrt_done = false;
    auto iter = 0;
    while(!is_rrt_done){
        iter++;
        if(iter > 10000){
            //reset and replanning
            this->count = 0;
            iter = 0;
        }

        point x_rand = randomState(x_max, x_min, y_max, y_min);

        int idx_near = nearestNeighbor(x_rand);
        point x_new = newState(idx_near, x_rand, MaxStep);
        auto count_tmp = this->count;
        addVertex(x_new, x_rand, idx_near);

        double dx = this->x_goal.x - x_new.x;
        double dy = this->x_goal.y - x_new.y;
        auto len = sqrt(dx*dx + dy*dy);

        if(len < 1.0 && count_tmp!=this->count){
            is_rrt_done = true;
            addVertex(x_goal, x_rand, idx_near);
        }

    }

//    int cnt = 0;
//    while(1){
//        /*
//        point x1, x2;
//        x1.x=+1.1; x1.y=-7.2; x2.x=+1.2; x2.y=-7.1;
//        if(isCollision(x1, x2)) printf("ok.\n");
//        */
//        point x_rand = (rand() % 5 == 0)? x_goal: randomState(x_max, x_min, y_max, y_min);
//        int idx_near = nearestNeighbor(x_rand);
//        point x_new = newState(idx_near, x_rand, MaxStep);
//        if(!isCollision(x_new, ptrTable[idx_near]->location)) continue;
//        addVertex(x_new, x_rand, idx_near);
//        // Break if the closest vertex is close enough to x_goal
//        if(++cnt == K){
//            int nearest = nearestNeighbor(x_goal);
//            point x_nearest = ptrTable[nearest]->location;
//            /*
//            double dist = sqrt(pow((x_goal.x-x_nearest.x), 2) + pow((x_goal.y-x_nearest.y), 2));
//            printf("\t\tdist = %f\n", dist);
//            if(dist < 0.04) return 0;
//            */
//            if((x_goal.x - x_nearest.x == 0) && (x_goal.y - x_nearest.y == 0)) return 0;
//            else break;
//        }
//    }
//    while(1){
//        point x_rand = (rand() % 5 == 0)? x_goal: randomState(x_max, x_min, y_max, y_min);
//        int idx_near = nearestNeighbor(x_rand);
//        point x_new = newState(idx_near, x_rand, MaxStep);
//        if(!isCollision(x_new, ptrTable[idx_near]->location)) continue;
//        addVertex(x_new, x_rand, idx_near);
//        // Break if the latest vertex is close enough to x_goal
//        point x_nearest = ptrTable[count-1]->location;
//        if((x_goal.x - x_nearest.x == 0) && (x_goal.y - x_nearest.y == 0)) return 0;
//        if(++cnt == 20000) break;
//    }
//    return -1;

}


point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    // TODO
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0,1);

    point random_point;
    random_point.x = x_min + (x_max - x_min) * dist(gen);
    random_point.y = y_min + (y_max - y_min) * dist(gen);

    return random_point;
}


point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    // TODO
    point x_near = this->ptrTable[idx_near]->location;
    point x_new;
    auto dx = x_near.x - x_rand.x;
    auto dy = x_near.y - x_rand.y;

    auto len = sqrt(dx*dx + dy*dy);

    if(len < MaxStep){
        return x_rand;
    }
    else{
        double prop_a = MaxStep/len;
        double prop_b = (len - MaxStep)/len;
        x_new.x = x_near.x*prop_b + x_rand.x*prop_a ;
        x_new.y = x_near.y*prop_b + x_rand.y*prop_a ;
        return x_new;
    }
}


int rrtTree::nearestNeighbor(point x_rand) {
    // TODO
    auto min_len = 1000000.0;
    auto min_idx = 0;
    for(auto i=0; i < this->count; i++){
        auto dx = this->ptrTable[i]->location.x - x_rand.x;
        auto dy = this->ptrTable[i]->location.y - x_rand.y;
        auto len = sqrt(dx*dx + dy*dy);
        if(len < min_len){
            min_len = len;
            min_idx = this->ptrTable[i]->idx;
        }
    }
    return min_idx;
}

bool rrtTree::isCollision(point x1, point x2) {
    // TODO
    //Pioneer has about 40cm radius, 8px
    int pixel_xrange = 3;
    int pixel_yrange = 3;


    auto x1_x_idx = (int)(x1.x / 0.05 + this->map_origin_x);
    auto x1_y_idx = (int)(x1.y / 0.05 + this->map_origin_y);
    auto x2_x_idx = (int)(x2.x / 0.05 + this->map_origin_x);
    auto x2_y_idx = (int)(x2.y / 0.05 + this->map_origin_y);

    double diff_x = x1_x_idx - x2_x_idx;
    double diff_x_abs = fabs(diff_x);

    double diff_y = x1_y_idx - x2_y_idx;
    double diff_y_abs = fabs(diff_y);

    double len = sqrt(diff_x*diff_x + diff_y*diff_y);
    int pnum = (int)(len / 0.05);
    if(pnum < 0){
        return false;
    }
    else{
        for(auto i=0; i< pnum; i++){
            auto sample_x = (int)(((double)x1_x_idx) + (x2_x_idx - x1_x_idx) * i / pnum);
            auto sample_y = (int)(((double)x1_y_idx) + (x2_y_idx - x1_y_idx) * i / pnum);

            for(auto j=0; j < pixel_xrange; j++){
                for(auto k=0; k < pixel_yrange; k++){

                    auto pixel = this->map.at<uchar>(sample_x - pixel_xrange/2 + j, sample_y - pixel_yrange + k);
                    if(pixel != 255){
                        return true;
                    }
                }
            }

        }
        return false;
    }
}


std::vector<point> rrtTree::backtracking(){
    // TODO
    std::vector<point> point_set;
    auto point_idx_now = this->ptrTable[this->count-1]->idx;
    while(point_idx_now != 0){
        node* node_toprocess = this->ptrTable[point_idx_now];
        point_set.push_back(node_toprocess->location);
        point_idx_now = node_toprocess->idx_parent;
    }
    point_set.push_back(this->root->location);
    std::reverse(point_set.begin(), point_set.end());
    return point_set;
}
