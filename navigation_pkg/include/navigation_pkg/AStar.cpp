//
// Created by stdcat on 3/4/21.
//

#include "AStar.h"

void AStar::tracePoint(Node *cur) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = "AStar_trace";
    marker.id = pointId++;
    marker.color.a = 1;
    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 255;

    marker.scale.x = map->reso();
    marker.scale.y = map->reso();
    marker.scale.z = map->reso();


    Vector2d _v = map->getXY_from_GridXY(cur->coordinate);
    marker.pose.position.x = _v.x();
    marker.pose.position.y = _v.y();
    marker.pose.position.z = map->reso()/2;

    pub.publish(marker);
}

AStar::AStar(GridMap_t *init_map):
    pointId(0),
    end_n(NULL)
{
    nh.param("AStar/checkSize", checkSize_A, 0.8);
    nh.param("AStar/longCheckSize", longCheckSize_A, 1.8);
    nh.param("AStar/minAng", minAng_A, 5.0);



    pub = nh.advertise<visualization_msgs::Marker>("/visualization/AStar_trace", 1000);
    pathMsg.header.frame_id = "world";
    pathMsg.poses.clear();
    map = init_map;

//    closeList.clear();
    mapNode.clear();

    for (int i = 0; i < map->size(); ++i) {
        mapNode.push_back(new Node(map->getGridXY_from_GridIndex(i)));
        mapNode[i]->index = i;
    }
}

void AStar::reset() {
#if usePriorityQueue
    while (!openList.empty()) openList.pop();
#elif useVectorSort
    openList.clear();
#endif
    for (int i = 0; i < mapNode.size(); ++i) {
        mapNode[i]->h = 0;
        mapNode[i]->g = 0;
        mapNode[i]->father = NULL;
        mapNode[i]->state = FREE_NODE;
    }
}

bool AStar::checkBound(Node *node) {
    if ( node->coordinate.x() < 0 || node->coordinate.y() < 0 ||
        node->coordinate.x() >= map->width() || node->coordinate.y() >= map->height()
    ) return false;
    else return true;
}

bool AStar::checkBound(int x, int y) {
    if ( x < 0 || y < 0 ||
        x >= map->width() || y >= map->height()
    ) return false;
    else return true;
}

bool AStar::unWalk(int x, int y) {
    //out of bound or there is a block, then it's unwalkable
    if (!checkBound(x,y)) return true;
    if (map->get_Data_from_GridXY(x,y) == 1||
        map->get_Data_from_GridXY(x,y) == 2
    ) return true;
    return false;
}

void AStar::extend(Node *current) {
    goTo(current->coordinate.x() - 1,current->coordinate.y(),current,WeightW);
	goTo(current->coordinate.x() + 1,current->coordinate.y(),current,WeightW);
	goTo(current->coordinate.x(),current->coordinate.y() + 1,current,WeightW);
	goTo(current->coordinate.x(),current->coordinate.y() - 1,current,WeightW);
	goTo(current->coordinate.x() - 1,current->coordinate.y() + 1,current,WeightWH);
	goTo(current->coordinate.x() - 1,current->coordinate.y() - 1,current,WeightWH);
	goTo(current->coordinate.x() + 1,current->coordinate.y() - 1,current,WeightWH);
	goTo(current->coordinate.x() + 1,current->coordinate.y() + 1,current,WeightWH);

}

nav_msgs::Path AStar::getAStarPath(Vector2d start_p, Vector2d end_p) {
    pathMsg.poses.clear();

    int sIndex = map->get_gridIndex_from_XY(start_p);
    int eIndex = map->get_gridIndex_from_XY(end_p);
    if (sIndex == -1 || eIndex == -1){
        ROS_WARN("Out Of Bound");
        return pathMsg;
    }

    Node * sNode = mapNode.at(sIndex);
    Node * eNode = mapNode.at(eIndex);

    if (search(sNode, eNode)){
        ROS_WARN("Find the Path");
    }
    else{
        reset();
        ROS_WARN("Failed to the Path");
    }

    return pathMsg;
}

void AStar::countH(Node *sNode, Node *eNode) {
    /* h! */
    double curh = sqrt(pow(sNode->coordinate.x() - eNode->coordinate.x(), 2) + pow(sNode->coordinate.y() - eNode->coordinate.y(), 2))*WeightW;

//    double curh = fabs(sNode->coordinate.x() - eNode->coordinate.x()) * WeightW + fabs(sNode->coordinate.y() - eNode->coordinate.y()) * WeightW;

    sNode->h = curh;
}

bool AStar::search(Node *sNode, Node *eNode) {
    if ( !checkBound(sNode) || !checkBound(eNode) ) return false;

    this->end_n = eNode;
    sNode->father = NULL;
    sNode->state = IN_OPENLIST;
    countH(sNode, eNode);   //h in f = g + h
#if usePriorityQueue
    openList.push(sNode);
#elif useVectorSort
    openList.push_back(sNode);
#endif
#if isVisual
    tracePoint(sNode);
#endif
    bool isReach = false;
    Node *current;
    while (!openList.empty()){
#if usePriorityQueue
        current = openList.top();
#elif useVectorSort
        current = openList[0];
#endif
        if (current->index == end_n->index){
            getPath(current);
            reset();
            isReach = true;
            break;
        }
#if usePriorityQueue
        openList.pop();
#elif useVectorSort
        openList.erase(openList.begin());
#endif
        current->state = IN_CLOSELIST;
        extend(current);
#if useVectorSort
        sort(openList.begin(),openList.end(),cmp);
#endif
    }

    return isReach;
}

void AStar::goTo(int x, int y, Node *father, double g) {
    if (unWalk(x,y)) return;
    if ( map->get_Data_from_GridXY(x,y) == 3){
        g *= 2;
    }
    int index = map->get_gridIndex_from_GridXY(x,y);
    Node *current = mapNode[index];
    if (current->state == IN_CLOSELIST) return;

    if (current->state == IN_OPENLIST) {
        if (current->g > (father->g + g)) {
            current->g = father->g + g;
            current->father = father;
#if isVisual
            tracePoint(current);
#endif
        }
    }
    else{
            current->father = father;
            current->g = father->g + g;
            countH(current, end_n);
            current->state = IN_OPENLIST;
#if usePriorityQueue
            openList.push(current);
#elif useVectorSort
            openList.push_back(current);
#endif
#if isVisual
            tracePoint(current);
#endif
    }
}

void AStar::getPath(Node *current) {
    if (current->father != NULL) getPath(current->father);
    geometry_msgs::PoseStamped ps;
    Vector2d _v = map->getXY_from_GridXY(current->coordinate);
    ps.pose.position.x = _v.x();
    ps.pose.position.y = _v.y();
    ps.pose.position.z = 2*map->reso();

    pathMsg.poses.push_back(ps);
}

nav_msgs::Path AStar::curPath() {
    return pathMsg;
}

vector<Vector3d> AStar::getWayPoints(int kernelSize, double miu, double sigma){
    vector<Vector3d> points;

    points.clear();
    if(pathMsg.poses.size() == 0) return points;
    
    //generate gaussian kernel
    static double *kernel = new double[kernelSize];
    for(int i = 0; i < kernelSize; i++){
        kernel[i] = 1/(sqrt(2*pi)*sigma)*exp(-pow((i-(kernelSize-1)/2)/sigma, 2));
    }
    //filter way point
    Vector3d pointTmp;
    double kernelSumTmp;
    for(int i = 0; i < pathMsg.poses.size(); i+=5){
        kernelSumTmp = 0.0;
        pointTmp.x() = 0.0;
        pointTmp.y() = 0.0;
        pointTmp.z() = 0.0;
        for(int j = -(kernelSize-1)/2; j < (kernelSize+1)/2; j++){
            if(i+j>=0 && i+j<pathMsg.poses.size()){
                pointTmp.x() += kernel[(kernelSize-1)/2+j] * pathMsg.poses[i+j].pose.position.x;
                pointTmp.y() += kernel[(kernelSize-1)/2+j] * pathMsg.poses[i+j].pose.position.y;
                kernelSumTmp += kernel[(kernelSize-1)/2+j];
            }
        }
        pointTmp.x() /= kernelSumTmp;
        pointTmp.y() /= kernelSumTmp;
        points.emplace_back(pointTmp);
    }
    return points;
}

vector<Vector3d> AStar::getWayPoints(){
    vector<Vector3d> points;

    points.clear();
    bool first = false;
    if (pathMsg.poses.size() == 0) return points;

    double checkSize = checkSize_A;
    double longCheckSize = longCheckSize_A;
    double minAng = minAng_A*pi/180.0;
    int checklengh = int(checkSize/map->reso())+1;
    int allsize = pathMsg.poses.size();

    if (allsize < checklengh){
        Vector3d p1(pathMsg.poses[0].pose.position.x, pathMsg.poses[0].pose.position.y, 0);
        Vector3d p2(pathMsg.poses[allsize-1].pose.position.x, pathMsg.poses[allsize-1].pose.position.y, 0);
        points.emplace_back(p1);
        points.emplace_back(p2);
        ROS_WARN("not enough");
        return  points;
    }
    Vector3d checkPoint(pathMsg.poses[0].pose.position.x, pathMsg.poses[0].pose.position.y, 0);
    Vector3d lastCheckPoint(0,0,0);
    points.emplace_back(checkPoint);
    int lastIndex;

    for (int i = 0; i < allsize-checklengh; i++) {
        double dis = sqrt(pow(pathMsg.poses[i].pose.position.y - checkPoint.y(),2)+pow(pathMsg.poses[i].pose.position.x - checkPoint.x(),2));
//        if ( sqrt(pow(pathMsg.poses[i].pose.position.y - pathMsg.poses[allsize-1].pose.position.y,2)+pow(pathMsg.poses[i].pose.position.x - pathMsg.poses[allsize-1].pose.position.x,2))
//                < checkSize
//                )break;
        if(dis < checkSize
                ) {
            continue;
        }
        if (!first) {
            lastCheckPoint = checkPoint;
            checkPoint.x() = pathMsg.poses[i].pose.position.x;
            checkPoint.y() = pathMsg.poses[i].pose.position.y;

            points.emplace_back(checkPoint);
            lastIndex = i;
            first = true;
            continue;
        }
        else if (fabs(
                atan2( pathMsg.poses[i].pose.position.y - checkPoint.y(), pathMsg.poses[i].pose.position.x - checkPoint.x() ) -
                atan2( checkPoint.y() - lastCheckPoint.y(), checkPoint.x() - lastCheckPoint.x() )
                ) < minAng
                && dis < longCheckSize
                ) continue;
        bool ok = false;
        if(points.size() >= 2){
            double dirs = atan2(pathMsg.poses[i].pose.position.y - checkPoint.y(), pathMsg.poses[i].pose.position.x - checkPoint.x() );
            int maxL = (int)(sqrt(pow(pathMsg.poses[i].pose.position.y - checkPoint.y(),2)+pow(pathMsg.poses[i].pose.position.x - checkPoint.x(),2))/map->reso());
            ROS_WARN("maxl:%d", maxL);
            for (int j = 0; j < maxL; ++j) {
                int index = map->get_gridIndex_from_GridXY(checkPoint.x()+i*cos(dirs), checkPoint.x()+i*sin(dirs));
                if ((*map)[index] != 3 || (*map)[index]!=0){
                    ok = true;
                }
            }
        }
        if(!ok) continue;
        if(fabs(
                atan2( pathMsg.poses[i].pose.position.y - checkPoint.y(), pathMsg.poses[i].pose.position.x - checkPoint.x() ) -
                atan2( checkPoint.y() - lastCheckPoint.y(), checkPoint.x() - lastCheckPoint.x() )
                ) > 60.0*pi/180.0){

            lastCheckPoint = checkPoint;
            checkPoint.x() = (pathMsg.poses[i].pose.position.x + checkPoint.x())/2;
            checkPoint.y() = (pathMsg.poses[i].pose.position.y + checkPoint.y())/2;
//            checkPoint.x() = pathMsg.poses[i].pose.position.x;
//            checkPoint.y() = pathMsg.poses[i].pose.position.y;
//


            points.emplace_back(checkPoint);
            checkPoint.x() = pathMsg.poses[i].pose.position.x;
            checkPoint.y() = pathMsg.poses[i].pose.position.y;
            lastIndex = i;
        }
        else{
            lastCheckPoint = checkPoint;
            checkPoint.x() = pathMsg.poses[i].pose.position.x;
            checkPoint.y() = pathMsg.poses[i].pose.position.y;
            lastIndex = i;
            points.emplace_back(checkPoint);
        }

    }

    checkPoint.x() = pathMsg.poses[allsize-1].pose.position.x;
    checkPoint.y() = pathMsg.poses[allsize-1].pose.position.y;
    points.emplace_back(checkPoint);
    ROS_WARN("psize:%d", points.size());

    return points;


//    vector<Vector3d> points;
//    points.clear();
//
//    if(pathMsg.poses.size() == 0) return points;
//
//    int path_length = (int)pathMsg.poses.size();
//    int piece_size = 8;
//    int points_num = path_length/piece_size;
//    if (points_num == 0) points_num = 2;
//    int _t = path_length/points_num;
//
//    if(_t == 0){
//        _t = 1;
//    }
//
//    bool fi = false;
//    Vector3d point_s(pathMsg.poses[0].pose.position.x,pathMsg.poses[0].pose.position.y,0);
//    points.push_back(point_s);
//    for(int p = 0; p < points_num; p++){
//        double x_sum=0;
//        double y_sum=0;
//
//        //Vector3d point1(pathMsg.poses[p * _t].pose.position.x, pathMsg.poses[p * _t].pose.position.y, 0);
//        //points.push_back(point1);
//        for(int t=0; t < _t; t++){
//            x_sum += pathMsg.poses[p * _t + t].pose.position.x;
//            y_sum += pathMsg.poses[p * _t + t].pose.position.y;
//        }
//        if (!fi){
//            fi = true;
//            continue;
//        }
//        Vector3d point(x_sum/_t, y_sum/_t,0);
//        points.push_back(point);
//    }
//    Vector3d point_end(pathMsg.poses[path_length-1].pose.position.x,pathMsg.poses[path_length-1].pose.position.y,0);
//    points.push_back(point_end);
//    return points;

}

//void AStar::resetMap(GridMap_t newMap) {
//    map = newMap;
//
//    for (int i = 0; i < map.size(); ++i) {
//        delete mapNode[i];
//        mapNode.push_back(new Node(map.getGridXY_from_GridIndex(i)));
//        mapNode[i]->index = i;
//    }
//}