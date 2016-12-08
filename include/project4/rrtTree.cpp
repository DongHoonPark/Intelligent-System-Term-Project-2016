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
//    this->map = addMargin(map, margin);
    this->map = map.clone();
    this->map = map.clone();
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->cost = 0;
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
    cv::waitKey(100);

}



// TODO
// 1. Copy your implementation of member functions in Project Assignment #2
// 2. Implement generateRRTst

void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    // TODO
    node* node_parent = this->ptrTable[idx_near];
    if(isCollision(node_parent->location, x_new)){
        return;
    }
    auto dx = node_parent->location.x - x_new.x;
    auto dy = node_parent->location.y - x_new.y;
    auto len = sqrt(dx*dx + dy*dy);
    if(len < 0.7){
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
    auto generate_fail = 0;
    while(!is_rrt_done){
        iter++;
        if(iter > 60000){
            //reset and replanning
            this->count = 0;
            iter = 0;
            generate_fail++;
            if(generate_fail > 5){
                return -1;
            }
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
            return 0;
        }
    }
}


int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep){
    // TODO
    /* card(V) = size of V, the set of vertices
     * d = dimension of point
     * gamma = Heuristic parameter
     */
    double gamma = 3.0;
    auto iter = 0;
    auto generate_fail = 0;
    if(checkPoint(this->root->location)!=0 ){

        cv::circle(this->map,
                   cv::Point(
                           (int)(this->root->location.y / 0.05 + map_origin_y),
                           (int)(this->root->location.x / 0.05 + map_origin_x)
                   ),
                   12,
                   cv::Scalar(255, 255, 255),
                   CV_FILLED);
        return 2;
    }
    cv::imshow("dm", *(this->dynamic_map_ptr));
    cv::waitKey(30);
    while(1){
        if(++iter > 60000){
            if(++generate_fail > 3)
                return -1;
            this->count = 1;
            iter = 0;
        }
        point x_rand = (iter % 10 == 0)? this->x_goal: randomState(x_max, x_min, y_max, y_min);

        int idx_nearest = nearestNeighbor(x_rand);
        point x_new = newState(idx_nearest, x_rand, MaxStep);
        if(isCollision(x_new, this->ptrTable[idx_nearest]->location)) continue;

        // RRT*
        // Find neighbor points s.t. |X_near[i] - x_new| < radius & no collision
        std::vector<int> X_nears = nearNeighbors(x_new, std::min(MaxStep, gamma*pow((log(this->count)/(this->count)),1/2)));
        std::vector<double> X_nears_c;       // Remember c(X_near_i, x_new)
        for(auto i=0; i < X_nears.size(); ++i)
            X_nears_c.push_back(getC(this->ptrTable[X_nears[i]]->location, x_new));
        // Connect along a minimum-cost path
        int idx_min = idx_nearest;
        auto c_min = this->ptrTable[idx_nearest]->cost + getC(this->ptrTable[idx_nearest]->location, x_new);
        for(auto i=0; i<X_nears.size(); ++i){
            auto tmp_c_min = this->ptrTable[X_nears[i]]->cost + X_nears_c[i];
            if(tmp_c_min < c_min){
                idx_min = X_nears[i];
                c_min = tmp_c_min;
            }
         }
        addVertexAndCost(x_new, x_rand, idx_min, c_min);
        // Rewire the tree
        for(auto i=0; i<X_nears.size(); ++i){
            if(this->ptrTable[this->count - 1]->cost + X_nears_c[i] < this->ptrTable[X_nears[i]]->cost)
                changeEdge(X_nears[i], (this->count - 1), X_nears_c[i]);
         }
        //printf("[RRT*] trial #: %d, iteration: %d, # of points: %d\n", generate_fail, iter, this->count);

        if(this->count >= K){
            int idx_near_goal = (this->count == K)? nearestNeighbor(this->x_goal): (this->count - 1);
            point x_nearest = this->ptrTable[idx_near_goal]->location;
            double dx = this->x_goal.x - x_nearest.x;
            double dy = this->x_goal.y - x_nearest.y;
            auto len = sqrt(dx*dx + dy*dy);
            if(len < MaxStep){
                addVertexAndCost(this->x_goal, this->x_goal, idx_near_goal, len);
                return 0;
            }
        }
    }
    return -1;
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
    int pixel_xrange = 2;
    int pixel_yrange = 2;

    auto x1_x_idx = (int)(x1.x / (this->res) + this->map_origin_x);
    auto x1_y_idx = (int)(x1.y / (this->res) + this->map_origin_y);
    auto x2_x_idx = (int)(x2.x / (this->res) + this->map_origin_x);
    auto x2_y_idx = (int)(x2.y / (this->res) + this->map_origin_y);

    double diff_x = x1_x_idx - x2_x_idx;
    double diff_y = x1_y_idx - x2_y_idx;
    double len = sqrt(diff_x*diff_x + diff_y*diff_y);
    int pnum = (int)(len);//(len / (this->res));
    if(pnum < 0){
        return false;
    }
    else{
        for(auto i=0; i< pnum; i++){
            auto sample_x = x1_x_idx + ((x2_x_idx - x1_x_idx) * 1.0 * i / (double)pnum);
            auto sample_y = x1_y_idx + ((x2_y_idx - x1_y_idx) * 1.0 * i / (double)pnum);
            for(auto j=0; j < pixel_xrange; j++){
                for(auto k=0; k < pixel_yrange; k++){
                    auto row = (int)(sample_x - pixel_xrange/2.0 + j);
                    auto col = (int)(sample_y - pixel_yrange/2.0 + k);
                    if(this->map.cols == 0 || this->map.rows == 0){
                    	   printf("Error! (map.rows or map.cols is 0)\n"); return false;
                       }
                    auto pixel = this->map.at<uchar>(row, col);
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
    point_set = optimizePath(point_set);
    return point_set;
}


// RRT*
void rrtTree::addVertexAndCost(point x_new, point x_rand, int idx_min, double cost) {
    // TODO
    this->ptrTable[this->count] = new node;
    node* node_toadd = this->ptrTable[this->count];
    node_toadd->idx_parent = idx_min;
    node_toadd->idx = this->count;
    node_toadd->location = x_new;
    node_toadd->rand = x_rand;
    node_toadd->cost = cost;
    this->count++;
}

double rrtTree::getC(point x1, point x2){
    /* Default cost of a point: distance between a point and its parent
     * In general, Cost(p) = Cost(parent) + c(p, parent)
     * This function returns c(x1,x2)
     */
    return sqrt((x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y));
}

void rrtTree::changeEdge(int idx, int idx_parent, double c){
    this->ptrTable[idx]->idx_parent = idx_parent;
    this->ptrTable[idx]->cost = this->ptrTable[idx_parent]->cost + c;
    // c = getC(ptrTable[idx]->location, ptrTable[idx_parent]->location);
}

std::vector<int> rrtTree::nearNeighbors(point x_new, double radius){
    std::vector<int> result;
    for(auto i=0; i < this->count; i++){
        auto dx = x_new.x - this->ptrTable[i]->location.x;
        auto dy = x_new.y - this->ptrTable[i]->location.y;
        auto len = sqrt(dx*dx + dy*dy);
        if(len < radius){
            if(!isCollision(ptrTable[i]->location, x_new)){
                result.push_back(i);
            }
        }
    }
//    printf("\n");
    /* L2 norm 대신 x_new+(+-radius, +-radius) 범위의 모든 point를 리턴
    for(int i=count-1; i>-1; i--){
        point tmp_x = ptrTable[i]->location;
        if(radius > max(tmp_x.x, x_new.x) - min(tmp_x.x, x_new.x)))
            result.push_back(ptrTable[i]->location);
    }
    for(int j=result.size(); j>-1; j--){
        if(!(radius > max(result[j].y, x_new.y) - min(result[j].y, x_new.y))))
            result.erase(result.begin()+j);
    }
    */
    return result;
}

std::vector<point> rrtTree::optimizePath(std::vector<point> path){
//	int iter = this->count - 1;
//	int iter_parent = ptrTable[iter]->idx_parent;
//	while(iter_parent != 0){
//		int iter_parent = ptrTable[iter]->idx_parent;
//		int iter_gp = ptrTable[iter_parent]->idx_parent;
//		if(!isCollision(ptrTable[iter]->location, ptrTable[iter_gp]->location)){
//			changeEdge(iter, iter_gp, getC(ptrTable[iter]->location, ptrTable[iter_gp]->location));
//			iter = iter_parent;
//		}
//	}
    for(auto iter=path.begin(); iter!=path.end()-2;){
        if(isCollision(*iter.base(), *(iter+2).base())){
            iter++;
        }
        else{
            path.erase(iter+1);
        }
    }
    return path;
}

void rrtTree::setDynamicMap(cv::Mat *dynamic_map) {
    this->dynamic_map_ptr = dynamic_map;
    //this->map = addMargin(*dynamic_map,8);
    this->map = dynamic_map->clone();
}

void rrtTree::resetDynamicMap() {
    *(this->dynamic_map_ptr) = this->map_original.clone();
}

int rrtTree::checkPoint(point check) {
    //check some point and return error code if it is occupied
    /*
     * error code description
     * code 0 : no error
     * code 1 : point occupied
     * code 2 : near point (<3px is occupied)
     * code 3 : point is not occupied but it is isolated by occupied points
     *
     */

    auto near = 2;

    auto x_idx = (int)(check.x / (this->res) + this->map_origin_x);
    auto y_idx = (int)(check.y / (this->res) + this->map_origin_y);
    if(this->map.at<uchar>(x_idx, y_idx) != 255) return 1;
    else {
        for(auto i=0; i<near; i++){
            for(auto j=0; j<near; j++){
                if(this->map.at<uchar>(x_idx-near/2 + i, y_idx-near/2 + j) != 255) return 2;
            }
        }
    }

    return 0;
}

bool rrtTree::checkPathValidity(std::vector<point> path) {
    for(auto i=0; i<path.size()-1; i++){
        if(isCollision(path[i], path[i+1])){
            return false;
        }
    }
    return true;
}