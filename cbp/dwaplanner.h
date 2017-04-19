#ifndef _DWA_PLANNER_H
#define _DWA_PLANNER_H

//c++ lib
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <vector>
#include <deque>
#include <queue>
#include <iterator>
#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <cassert>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <algorithm>

#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/binomial_heap.hpp>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//other lib

#include <bitset>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "visualize.h"
#include "reeds_shepp.h"
#include "dubins.h"
#include "functions.h"

#include "ivmap/ivmapmsgobj.h"
#include "ivpathplanner/path.h"

using namespace std;
using namespace cv;

#define SIMBLOCK 0

typedef struct VEHICLESTATUS
{
    // dm
    float x;
    float y;
    // rad
    float heading;
    // m/s
    float linear_velocity;
    // rad/s
    float angular_velocity;

    /**
     * \brief The overload of the operator "=="
     *
     */
    bool operator== (const VEHICLESTATUS &rhs)
    {
        return (fabs(heading - rhs.heading) <= 1e-2 && fabs(x - rhs.x) <= 1e-1 && fabs(y - rhs.y) <= 1e-1 &&
            fabs(linear_velocity - rhs.linear_velocity) <= 1e-1 && fabs(angular_velocity - rhs.angular_velocity) <= 1e-1);
    }

    VEHICLESTATUS& operator= (const VEHICLESTATUS &rhs)
    {
        this->angular_velocity = rhs.angular_velocity;
        this->linear_velocity = rhs.linear_velocity;
        this->x = rhs.x;
        this->y = rhs.y;
        this->heading = rhs.heading;
        return *this;
    }
} sVehicleStatus;

/**  
* @brief 函数类   
 * @brief 作为hash_map的比较函数 
 * @brief 查找的时候不同的key往往可能会用到相同的hash值  
*/
template <typename T>
struct hash_compare
{
    bool operator()(const T &lhs, const T &rhs) const
    {
        return (lhs.heading == rhs.heading) && (lhs.x == rhs.x) && (lhs.y == rhs.y);
    }
};

/** 
 * @brief 模板的特例化 
 * @brief 函数类  
 * @brief 作为hash_map的hash函数   
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"   
 */
namespace std
{
template <>
    struct hash<sVehicleStatus>
    {
        inline size_t operator()(const sVehicleStatus &vs) const
        {
            std::hash<float> hash_fn;
            return size_t(hash_fn(vs.x + vs.y + vs.heading));
        }
    };
}

/** 
 * @brief 函数类  
 * @brief 作为hash_map的hash函数   
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"   
 */
template <typename T>
class hash_key
{
public:
    size_t operator()(const T &str) const
    {
        std::hash<float> hash_fn;
        return size_t(hash_fn(str.x + str.y + str.heading));
    }
};

typedef struct KINEMATIC
{
    // m/s
    float max_linear_velocity;
    // ras/s
    float max_angular_velocity;
    // m/s^2
    float max_linear_acceleration;
    // m/s^3
    float maximum_acceleration;
    // m/s^3
    float maximum_decceleration;
    //rad/s^2
    float max_angular_acceleration;
    // m/s^2
    float min_turn_radius;
    // m/s^2
    float centrifugal_acceleration; // Multiply the gravitational acceleration
} sKinematic;

typedef struct EVALPARAM
{
    // heading
    float heading_coeff;
    // dist to the obstacle
    float clearance_coeff;
    // breaking distance
    float velocity_coeff;
    // This is the multiply coeff
    float multiply_coeff;
} sEvalparam;

typedef struct VEHICLEELEMENT
{
    /*Unit: decimetre*/
    float length;
    float width;
    float frontoverhang;
    float backoverhang;
    float wheelbase;
    float headwheel;
    float tailwheel;
    /*turnangle, Unit: degree*/

    /*Left wheel steer angle*/
    float innerangle;
    /*Right wheel steer angle*/
    float outerangle;
    //>= 2.5 dm
    float outersafedis;
    float innersafedis;
} sVehicleElem;

typedef struct ROADELEMENT
{
    /*Unit: decimetre*/
    float roadwidthmin;
    float turnradiusmin;
    float turnradiusmax;
    float roadouterradius;
    float roadinnerradius;
} sRoadElem;

// The usage of unordered_map is marvelous
template <typename Val>
using unordered_t = std::unordered_map<sVehicleStatus, Val, hash_key<sVehicleStatus>, hash_compare<sVehicleStatus>>;

using sorted_tree = std::map<float, functions::PtsTree<geometry_msgs::Point32>::ytree>;

class dwaplanner
{
public:
    dwaplanner(ros::NodeHandle nh);

    dwaplanner();

    ~dwaplanner();

    std::vector<geometry_msgs::Point32> bsplinecurve(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt);

    void calculateradius(sVehicleElem *info, sRoadElem *rtn);

    inline float CalcHeadingEval(sVehicleStatus predictvs, sVehicleStatus goalvs);

    inline float CalcBreakingDist(sVehicleStatus predictvs, sVehicleStatus goalvs);
    
    int collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree& obspts);

    int collisiondetection(ivpathplanner::path ego, sorted_tree &obspts);

    int DynamicWindowApproach(sVehicleStatus vs, sVehicleStatus goalvs);

    functions::PriorityNode<int, float>::PQElement Evaluation(std::vector<cv::Point3f> evaldb);
    
    sVehicleStatus evolvedastar(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus>& came_from,
     unordered_t<float>& cost_so_far, unordered_t<std::vector<sVehicleStatus>>& pathtable, sorted_tree sortedobjs);

    float globalpath(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt, std::vector<geometry_msgs::Point32>& pts);

    float heuristic(sVehicleStatus cur, sVehicleStatus goal);

    sVehicleStatus hybridastar(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
     unordered_t<float> &cost_so_far, unordered_t<std::vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs);

    void NormalizeEval(vector<vector<sVehicleStatus>> path, std::vector<cv::Point3f> &evaldb);


    /**
    * @brief Return the end point of the generated path
    */
    std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
    GenerateTrajectory(sVehicleStatus currentvs, sKinematic sk);

    std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
    lookuptable(sVehicleStatus currentvs, sKinematic sk);

    void obstacleprocess(std::vector<geometry_msgs::Point32> obs, sorted_tree& rtnobs);

    void obstacleprocess(ivmap::ivmapmsgobj objs, sorted_tree& rtnobs);
    
    void obstaclemodify(ivmap::ivmapmsgobj objs, sorted_tree &rtnobs);

    int reachgoal(std::vector<sVehicleStatus> road, sVehicleStatus goalvs);
    
    std::vector<sVehicleStatus>
    reconstruct_path(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
       sVehicleStatus last);

    // The use of stl set, can get the end() and begin() of the sorted elements!!!
    // And it is more efficient!!!
    functions::PriorityNode<cv::Point3f, float> reEvaluation(std::vector<cv::Point3f> evaldb);

    void run();

    void test();

    inline float todegree(float rad);

    inline float toradian(float deg);

    int pruning(std::vector<sVehicleStatus> motion, functions::PtsTree<geometry_msgs::Point32>::ytree obspts);

    ivpathplanner::path pathinterface(ivpathplanner::path rd, ivmap::ivmapmsgobj objs);

    void vehicleinfoinit(sVehicleElem *info);

private:
    sVehicleElem vehicleinfo;
    sRoadElem laneinfo;

    sEvalparam evalinfo;
    //Time diff
    float deltat;
    float stepsize;
    // degrees
    float yawtolerance;
    // m
    float disttolerance;
    // m/s
    float speedtolerance;
    float precosth;
    int count;
    bool REVERSE;
    tf::TransformBroadcaster pathbr;
    tf::Transform pathtr;
    tf::Quaternion pathquater;
    std::unordered_map<int, int> samplenum;
    std::vector<std::vector<sVehicleStatus>> trajectorys;
    std::vector<sVehicleStatus> trajectory;
    std::vector<cv::Point3f> evalpts;
    std::vector<geometry_msgs::Point32> obstacle;
    
    sorted_tree sortedobs;

    std::vector<geometry_msgs::Point32> mtest;

    //Computance index
    sVehicleStatus initialstatus;
    sVehicleStatus updatingstatus;
    sVehicleStatus goalstatus;
    sKinematic dyn;

    std::unique_ptr<visualize_pose> visdwa_pose;
    std::unique_ptr<visualize_path> visdwa_path;
    std::unique_ptr<visualize_marker> visdwa_marker;
    std::unique_ptr<visualize_polygon> visdwa_polygon;
    std::unique_ptr<visualize_path> vishybrid_path;
};

#endif
