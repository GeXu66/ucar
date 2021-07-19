#include "navigation_pkg/common_include.h"
#include "navigation_pkg/AStar.h"
#include "navigation_pkg/am_traj.hpp"
#include "navigation_pkg/WayPoint.h"
#include "navigation_pkg/pid.h"
#include "navigation_pkg/config.h"
#include <thread>

#define PLAN_ONCE 0 //1 for plan once, 0 for realtime plan
#define VISUALIZATION 0

Config cfg;
ros::Publisher vel_pub, wp_pub, astarwp_pub, mapinflate_pub, astar_pub, path_pub;

geometry_msgs::Pose curPos;
geometry_msgs::Twist curTwist;
ros::Time sTime, globalStart;

GridMap_t *gridMap;
AStar *astar = NULL;
vector<Vector3d> astarWPS;
AmTraj::Ptr amTrajPtr;
Trajectory traj;

Vector2d globalDst;
Vector2d globalOri;

bool isSet = false; //is the robot ready for running
bool replanning = false;
bool showMap = false;
bool stop = false;
bool isAStarNull = true;
bool isRun = false;

double getTotalTime(Trajectory traj_){
    double totalTime = 0.0;
    for(int i = 0; i < traj_.getDurations().size(); i++){
        totalTime += traj_.getDurations()[i];
    }
    return totalTime;
}

void publishTrajToNav(Trajectory traj_){
    nav_msgs::Path path_msg;
    path_msg.poses.clear();
    double cur_t = 0.0;
    
    while(cur_t < getTotalTime(traj_)){
        cur_t += 0.01;
        Vector3d cur = traj_.getPos(cur_t);
        Vector3d cVel = traj_.getVel(cur_t);
        geometry_msgs::PoseStamped ps;

        ps.pose.position.x = cur.x();
        ps.pose.position.y = cur.y();
        ps.pose.position.z = cur.z();
        path_msg.poses.push_back(ps);
    }
    ROS_WARN("There %ld sample points of traj", path_msg.poses.size());
    path_msg.header.frame_id = "world";
    path_pub.publish(path_msg);
}

void visualWayPoint(ros::Publisher pub, navigation_pkg::WayPoint wp){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "waypoint_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.03;
    marker.scale.y = 0.08;
    marker.scale.z = 0.1;

    marker.color.a = 1;
    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 0;

    geometry_msgs::Point p1,p2;
    p1.x = wp.pos.x;
    p1.y = wp.pos.y;
    p1.z = 0;

    p2.x = p1.x + wp.vel.x;
    p2.y = p1.y + wp.vel.y;
    p2.z = 0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    pub.publish(marker);
}

void visualAStarWayPoint(ros::Publisher pub, vector<Vector3d> wps){
    visualization_msgs::MarkerArray markers;
    int _id = 0;
    for (int i = 0; i < wps.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "AStar_waypoint";
        marker.id = _id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1;
        marker.color.r = 255;
        marker.color.g = 0;
        marker.color.b = 0;

        marker.pose.position.x = wps[i].x();
        marker.pose.position.y = wps[i].y();
        marker.pose.position.z = 0;

        markers.markers.push_back(marker);
    }
    pub.publish(markers);
}

//generate from beginning
void generate_Traj(vector<Vector3d> wayPoints){
    Vector3d zerosVec(0.0, 0.0, 0.0);
    ros::Time t1 = ros::Time::now();
    traj = amTrajPtr->genOptimalTrajDTC(wayPoints, zerosVec, zerosVec, zerosVec, zerosVec);
    ros::Time t2 = ros::Time::now();
    ROS_WARN("Finish generate traj with time: %lf ms", (double)((t2-t1).toSec())*1e3);
    ROS_WARN("Traj total duration: %lf s", getTotalTime(traj));
}

//generate from mid
void generate_Traj(vector<Vector3d> wayPoints, Vector3d vel, Vector3d acc){
    Vector3d zerosVec(0.0, 0.0, 0.0);
    ros::Time t1 = ros::Time::now();
    traj = amTrajPtr->genOptimalTrajDTC(wayPoints, vel, acc, zerosVec, zerosVec);
    ros::Time t2 = ros::Time::now();
    ROS_WARN("Finish genrate traj with time: %lf ms", (double)((t2-t1).toSec())*1e3);
    ROS_WARN("Traj total duration: %lf s", getTotalTime(traj));
}

void keyboard_callback(const std_msgs::Char::ConstPtr &msg){
    if(msg->data == 's'){
	while(isAStarNull);
        ROS_WARN("----Start----");
        Vector2d startVector2d(cfg.globalSttX, cfg.globalSttY);
        nav_msgs::Path ph = astar->getAStarPath(startVector2d, globalDst);
        
        ROS_WARN("There are %ld AStar points.", ph.poses.size());
        if(ph.poses.size() != 0){
            astarWPS = astar->getWayPoints(cfg.gaussianKernelSize, cfg.gaussianKernelMiu, cfg.gaussianKernelSigma);
            ROS_WARN("There are %ld way points.", astarWPS.size());
            visualAStarWayPoint(astarwp_pub, astarWPS);
            astar_pub.publish(ph);

            Vector3d zeroV3d(0.0, 0.0, 0.0);
            generate_Traj(astarWPS, zeroV3d, zeroV3d);
            replanning = false;
            sTime = ros::Time::now();
        }
        isSet = true;
        showMap = true;
        publishTrajToNav(traj);
    }
    else if(msg->data == 'q'){
	stop = 1;
    }
    else if(msg->data == 'w'){
        stop = 0;
    }
    else if(msg->data == 'd'){
        ;
    }
    else if(msg->data == 'a'){
        isRun = true;
        sTime = ros::Time::now();
        globalStart = sTime;
    }
}

void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    if(astar == NULL) return;
    curPos = msg->pose.pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double yaw, pitch, roll;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    curPos.position.z = yaw;

    //curTwist = msg->twist.twist;
    /*********************************/
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    ;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    gridMap = new GridMap_t(
        msg->info.width,
        msg->info.height,
        msg->info.resolution,
        msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->data,
        I_ROUND);
    gridMap->inflate(cfg.inflation_r);

    if(astar == NULL) {astar = new AStar(gridMap); isAStarNull = false;}
}

void timer_callback(const ros::TimerEvent &evt){
    static navigation_pkg::WayPoint curWP, lastWP;
    geometry_msgs::Twist velMsg;

    if(gridMap != NULL && showMap) gridMap->pubMsg(mapinflate_pub);

    if(!isSet)return; //If it's not ready, do nothing.
    
    if(stop){
    	velMsg.linear.x = 0;
    	velMsg.linear.y = 0;
    	velMsg.angular.z = 0;
    	vel_pub.publish(velMsg);
    	return;
    }

    ros::Duration dur = ros::Time::now() - sTime;
    double dur_t = dur.toSec();
    if(traj.getPieceNum() != 0) dur_t = dur_t>traj.getTotalDuration()? traj.getTotalDuration():dur_t;
    //refresh dur_t to current time, and if dur_t is more than the total time of the traj, set a limit

    //reach the dst
    if(sqrt(pow(globalDst.y() - curPos.position.y, 2) +
            pow(globalDst.x() - curPos.position.x, 2)) < 0.2){
        static int first = 0; //only the first-time reach counts
        if(first == 0){
            first++;
            ros::Time endT = ros::Time::now();
            ROS_WARN("Reach Time: %lf", (endT - globalStart).toSec());
            //print out the reach time
        }
        velMsg.linear.x = 0;
        velMsg.linear.y = 0;
        velMsg.angular.z = 0;
        vel_pub.publish(velMsg);
    }
    //replanning
    else if(replanning){
        //initialize the velocity for replanning
        if(sqrt(pow(curTwist.linear.x, 2) + pow(curTwist.linear.y, 2)) > 0.3 
            || curTwist.angular.z > 0.1){
            velMsg.linear.x = 0.3 * velMsg.linear.x / sqrt(pow(curTwist.linear.x, 2) + pow(curTwist.linear.y, 2));
            velMsg.linear.y = 0.3 * velMsg.linear.y / sqrt(pow(curTwist.linear.x, 2) + pow(curTwist.linear.y, 2));
            velMsg.angular.z = 0;
            vel_pub.publish(velMsg);
        }
        //replanning
        else{
            ROS_WARN("Replanning.....");
            Vector2d _v2d(curPos.position.x, curPos.position.y);
            nav_msgs::Path ph = astar->getAStarPath(_v2d, globalDst);
            if(ph.poses.size() != 0){
                astarWPS = astar->getWayPoints(cfg.gaussianKernelSize, cfg.gaussianKernelMiu, cfg.gaussianKernelSigma);
                visualAStarWayPoint(astarwp_pub,astarWPS);

                Vector3d _v3dV(0.3*cos(curPos.position.z), 0.3*sin(curPos.position.z), 0.0);
                Vector3d zeroV3d(0.0, 0.0, 0.0);
                generate_Traj(astarWPS, _v3dV, zeroV3d);
                replanning = false;
                sTime = evt.current_real;
            }
        }
    }
    //control part
    
    else if(isRun){
        Vector3d goalVel = traj.getVel(dur_t);
        Vector3d goalPos = traj.getPos(dur_t);
        Vector3d goalAcc = traj.getAcc(dur_t);
        dur_t = (dur_t - cfg.aheadTime < 0) ? 0 : (dur_t - cfg.aheadTime);
        Vector3d curGoalVel = traj.getVel(dur_t);
        Vector3d curGoalAcc = traj.getAcc(dur_t);
        
        lastWP = curWP;

        curWP.vel.x = goalVel.x();
        curWP.vel.y = goalVel.y();
        curWP.vel.z = goalVel.z();
        curWP.pos.x = goalPos.x();
        curWP.pos.y = goalPos.y();
        curWP.pos.z = goalPos.z();
        curWP.accel.x = goalPos.x();
        curWP.accel.y = goalPos.y();
        curWP.accel.z = goalPos.z();

        /*
        double errPosX = curWP.pos.x - curPos.position.x;
        double errPosY = curWP.pos.y - curPos.position.y;

        static double errAng_1 = 0;
        double tarAng = atan2(curWP.vel.y, curWP.vel.x);
        double errAng = tarAng - curPos.position.z;

        if (errAng > pi) errAng = 2*pi - errAng;
        if (errAng < -pi) errAng = -2*pi + errAng;

        errAng_1 = cfg.angularIDecay * errAng_1 + errAng;

        double velX = cfg.linearKPP * curWP.vel.x + cfg.linearKP * errPosX;
        double velY = cfg.linearKPP * curWP.vel.y + cfg.linearKP * errPosY;
        velMsg.angular.z = cfg.angularKP * errAng + cfg.angularKI * errAng_1;

        velMsg.linear.x = velX * cos(curPos.position.z) + velY * sin(curPos.position.z);
        velMsg.linear.y = velY * cos(curPos.position.z) - velX * sin(curPos.position.z);
        */
        static double errPos_1 = 0.0;
        static double errPos_2 = 0.0;
        static double curPIDVel = 0.0;
        double errPos = sqrt(pow(curWP.pos.x - curPos.position.x, 2) + pow(curWP.pos.y - curPos.position.y, 2));
        double dPID = cfg.linearKP*((errPos-errPos_1) + (cfg.linearTD/cfg.loopT)*(errPos-2*errPos_1+errPos_2) + (cfg.loopT/cfg.linearTI)*errPos);
        curPIDVel += dPID;
        errPos_2 = errPos_1;
        errPos_1 = errPos;
        
        double curVel = sqrt(pow(curGoalVel.x(), 2) + pow(curGoalVel.y(), 2));
        
        double vel = cfg.linearKPP * curVel + (1 - cfg.linearKPP) * curPIDVel;
        if(vel > 2 * cfg.maxVel) vel = 2 * cfg.maxVel;
        if(vel < -2 * cfg.maxVel) vel = -2 * cfg.maxVel;
        

        static double errAng_2 = 0.0;
        static double errAng_1 = 0.0;
        static double curPIDAng = 0.0;
        double tarAng = atan2(curWP.pos.y-curPos.position.y, curWP.pos.x-curPos.position.x);
        double errAng = tarAng - curPos.position.z;
        if (errAng > pi) errAng = errAng - 2*pi;
        if (errAng < -pi) errAng = errAng + 2*pi;
        dPID = cfg.angularKP*((errAng-errAng_1) + (cfg.angularTD/cfg.loopT)*(errAng-2*errAng_1+errAng_2) + (cfg.loopT/cfg.angularTI)*errAng);
        curPIDAng += dPID;
        errAng_2 = errAng_1;
        errAng_1 = errAng;

        double curAngVel = -(curGoalAcc.x()*sin(curPos.position.z)-curGoalAcc.y()*cos(curPos.position.z))/curVel;
        
        double angVel = cfg.angularKPP*curAngVel+(1-cfg.angularKPP)*curPIDAng;
        if(angVel > 1.0) angVel = 1.0;
        if(angVel < -1.0) angVel = -1.0;
        

        velMsg.linear.x = vel;
        velMsg.linear.y = 0.0;
        velMsg.linear.z = 0.0;
        velMsg.angular.x = 0.0;
        velMsg.angular.y = 0.0;
        velMsg.angular.z = angVel;

        vel_pub.publish(velMsg);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "real_time_tracker");
    ros::NodeHandle nh;
    cfg.set(nh);

    globalDst.x() = cfg.globalDstX;
    globalDst.y() = cfg.globalDstY;
    globalOri.x() = cfg.globalOriX;
    globalOri.y() = cfg.globalOriY;

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    wp_pub = nh.advertise<visualization_msgs::Marker>("/visualization/wp_pub", 10);
    astarwp_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/astarwp_pub", 10);
    mapinflate_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/map_inflate", 10);
    astar_pub = nh.advertise<nav_msgs::Path>("/visualization/astar_pub", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/visualization/global_path", 10);

    //ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("laser", 10, laser_callback);
    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, odom_callback);
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, map_callback);
    ros::Subscriber keyboard_sub = nh.subscribe<std_msgs::Char>("keyboard_pub", 10, keyboard_callback);
    
    amTrajPtr.reset(new AmTraj);
    amTrajPtr->init(
        cfg.timeWeight, 
        cfg.accWeight, 
        cfg.jerkWeight,
        cfg.maxVel,
        cfg.maxAcc,
        cfg.maxIts,
        cfg.eps);
    ROS_WARN("Traj optimizer init successfully");

    ros::Timer controlThread = nh.createTimer(ros::Duration(1.0/cfg.loopHz), timer_callback);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
