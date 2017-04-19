#ifndef IVCBP_FUNCTIONS
#define IVCBP_FUNCTIONS

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
#include <limits>
#include <utility>

#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/binomial_heap.hpp>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
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


namespace functions
{

static const float radius = 3.2;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.0;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;

/**
 * @brief The boost::heap::binomial_heap implementaion of the priority_queue
 * @brief template pair and template priority_queue and template node
 */
template <typename T, typename priority_t>
struct PriorityNode
{
    typedef std::pair<T, priority_t> PQElement;

    /**
    * \brief The overload of the operator "()"
    *
    */
    struct CompareNode
    {
        bool operator()(const PQElement lhs, const PQElement rhs) const
        {
            return (lhs.second > rhs.second);
        }
    };

    //The default priorityqueue is big top element heap!!!
    typedef boost::heap::priority_queue<PriorityNode> boostPriorityQueue;
    typedef boost::heap::binomial_heap<PQElement, boost::heap::compare<CompareNode>> boostPriority;
    typedef std::priority_queue<PQElement, std::vector<PQElement>, CompareNode> standardPriority;

    boostPriority elements;
    PQElement node;

    PriorityNode() {}

    PriorityNode(PQElement ref)
        : node(ref) {}

    PriorityNode(T a, priority_t b)
    {
        node.first = a;
        node.second = b;
    }

    /**
     * \brief The overload of the operator "<"
     *
     */
    friend bool operator<(const PriorityNode lhs, const PriorityNode rhs)
    {
        return (lhs.node.second > rhs.node.second);
    }
};

/**
 * @brief The boost::heap::priority_queue implementaion of the priority_queue
 */
template <typename T, typename priority_t>
class CostNode
{
  public:
    typedef boost::heap::priority_queue<CostNode> boostPriorityQueue;
    typedef std::pair<T, priority_t> PQElement;
    PQElement node;

    CostNode() {}

    CostNode(PQElement ref)
        : node(ref) {}

    CostNode(T a, priority_t b)
    {
        node.first = a;
        node.second = b;
    }

    ~CostNode() {}

    /**
     * \brief The overload of the operator "<"
     *
     */
    friend bool operator<(const CostNode lhs, const CostNode rhs)
    {
        return (lhs.node.second < rhs.node.second);
    }

    struct CompareNode
    {
        bool operator()(const CostNode lhs, const CostNode rhs) const
        {
            return (lhs.node.second < rhs.node.second);
        }
    };
};

/**
 * @brief The multeset obstacles tree
 */
template <typename T>
class PtsTree
{
  public:
    PtsTree() {}
    PtsTree(T ref)
        : elem(ref) {}
    ~PtsTree() {}
    T elem;
    struct CompareLeafX
    {
        bool operator()(const T lhs, const T rhs) const
        {
            return lhs.x < rhs.x;
        }
    };
    struct CompareLeafY
    {
        bool operator()(const T lhs, const T rhs) const
        {
            return lhs.y < rhs.y;
        }
    };
    typedef std::multiset<T, CompareLeafX> xtree;
    typedef std::multiset<T, CompareLeafY> ytree;
};

static std::tuple<geometry_msgs::Point32, geometry_msgs::Point32> carmodel(geometry_msgs::Point32 ego, geometry_msgs::Polygon &polygon)
{
    polygon.points.clear();
    float front_lon = 1.2;
    float back_lon = 0.4;
    float half_lat = 0.6;
    geometry_msgs::Point32 pt, ld, ru;
    std::set<float> cmpx, cmpy;
    float vyaw = ego.z + M_PI / 2.0;
    pt.x = ego.x + front_lon * cos(ego.z) - half_lat * cos(vyaw);
    pt.y = ego.y + front_lon * sin(ego.z) - half_lat * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x + front_lon * cos(ego.z) + half_lat * cos(vyaw);
    pt.y = ego.y + front_lon * sin(ego.z) + half_lat * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x - back_lon * cos(ego.z) + half_lat * cos(vyaw);
    pt.y = ego.y - back_lon * sin(ego.z) + half_lat * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x - back_lon * cos(ego.z) - half_lat * cos(vyaw);
    pt.y = ego.y - back_lon * sin(ego.z) - half_lat * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    ld.x = *(cmpx.begin());
    ld.y = *(cmpy.begin());
    ru.x = *(cmpx.rbegin());
    ru.y = *(cmpy.rbegin());
    auto rtn = std::forward_as_tuple(ld, ru);
    return rtn;
}

static void vArrayCreate(std::vector<std::vector<unsigned char>> &array, int m, int n)
{
    std::vector<unsigned char> temparr;
    temparr.assign(n, 0);
    array.assign(m, temparr);
    return;
}

static bool IsInsideFootprint(geometry_msgs::Point32 pt, geometry_msgs::Polygon &bounding_polygon)
{
    int counter = 0;
    int i;
    double xinters;
    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
    int N = bounding_polygon.points.size();

    p1 = bounding_polygon.points.at(0);
    for (i = 1; i <= N; i++)
    {
        p2 = bounding_polygon.points.at(i % N);
        if (pt.y > std::min<float>(p1.y, p2.y))
        {
            if (pt.y <= std::max<float>(p1.y, p2.y))
            {
                if (pt.x <= std::max<float>(p1.x, p2.x))
                {
                    if (p1.y != p2.y)
                    {
                        xinters = (pt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if (p1.x == p2.x || pt.x <= xinters)
                            counter++;
                    }
                }
            }
        }
        p1 = p2;
    }
    if (counter % 2 == 0)
        return false;
    else
        return true;
}

static std::string getcurrentdir()
{
    std::string dirname = __FILE__;
    boost::filesystem::path pathname(dirname);
    pathname.remove_filename();
    dirname = pathname.string();
    return dirname;
}

static float rad2Deg(float radian)
{
    if (radian > 2 * M_PI)
    {
        radian -= M_PI;
    }
    else if (radian < 0.0)
    {
        radian += 2 * M_PI;
    }
    return radian * 180 / M_PI;
}

static float deg2Rad(float degree)
{
    if (degree > 360.0)
    {
        degree -= 360.0;
    }
    else if (degree < 0.0)
    {
        degree += 360.0;
    }
    return M_PI * degree / 180.0;
}

static float normalizeradian(float rad)
{
  if (rad < 0.0)
  {
    rad += 2 * M_PI;
  }
  if (rad > 2 * M_PI)
  {
    rad -= 2 * M_PI;
  }
  return rad;
}

static double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

// generateGaussianNoise box_muller algorithm
static float boxmuller(float m, float sd)
{
    const double epsilon = std::numeric_limits<double>::min();
    //Get random number
    srand((unsigned)time(NULL));
    float u1 = rand() * (1.0 / RAND_MAX);
    float u2 = rand() * (1.0 / RAND_MAX);

    while(u2 <= epsilon)
    {
        u2 = rand() * (1.0 / RAND_MAX);
    }

    float r = sqrt(-2.0 * log(u2));
    float theta = 2 * M_PI * u1;
    float z = r * cos(theta);

    return (m + z * sd);
}



static void diffinsert(const std::vector<geometry_msgs::Point> &xythetaPath, std::vector<geometry_msgs::Point> *spline)
{
    geometry_msgs::Point temp;
    for (int i = 0; i < xythetaPath.size() - 1; ++i)
    {
        temp.x = xythetaPath.at(i).x;
        temp.y = xythetaPath.at(i).y;
        temp.z = xythetaPath.at(i).z;
        spline->push_back(temp);
        if (fabs(xythetaPath.at(i).x - xythetaPath.at(i + 1).x) > 0.12 ||
            fabs(xythetaPath.at(i).y - xythetaPath.at(i + 1).y) > 0.12)
        {
            if (fabs(xythetaPath.at(i).x - xythetaPath.at(i + 1).x) < 0.18 ||
                fabs(xythetaPath.at(i).y - xythetaPath.at(i + 1).y) < 0.18)
            {
                temp.x = (xythetaPath.at(i + 1).x + 2 * xythetaPath.at(i).x) / 3.0;
                temp.y = (xythetaPath.at(i + 1).y + 2 * xythetaPath.at(i).y) / 3.0;
                temp.z = (xythetaPath.at(i + 1).z + 2 * xythetaPath.at(i).z) / 3.0;
                spline->push_back(temp);
                temp.x = (2 * xythetaPath.at(i + 1).x + xythetaPath.at(i).x) / 3.0;
                temp.y = (2 * xythetaPath.at(i + 1).y + xythetaPath.at(i).y) / 3.0;
                temp.z = (2 * xythetaPath.at(i + 1).z + xythetaPath.at(i).z) / 3.0;
                spline->push_back(temp);
            }
            else if (fabs(xythetaPath.at(i + 1).x - xythetaPath.at(i).x) < 0.24 ||
                     fabs(xythetaPath.at(i + 1).y - xythetaPath.at(i).y) < 0.24)
            {
                temp.x = (xythetaPath.at(i + 1).x + 3 * xythetaPath.at(i).x) / 4.0;
                temp.y = (xythetaPath.at(i + 1).y + 3 * xythetaPath.at(i).y) / 4.0;
                temp.z = (xythetaPath.at(i + 1).z + 3 * xythetaPath.at(i).z) / 4.0;
                spline->push_back(temp);
                temp.x = (2 * xythetaPath.at(i + 1).x + 2 * xythetaPath.at(i).x) / 4.0;
                temp.y = (2 * xythetaPath.at(i + 1).y + 2 * xythetaPath.at(i).y) / 4.0;
                temp.z = (2 * xythetaPath.at(i + 1).z + 2 * xythetaPath.at(i).z) / 4.0;
                spline->push_back(temp);
                temp.x = (3 * xythetaPath.at(i + 1).x + xythetaPath.at(i).x) / 4.0;
                temp.y = (3 * xythetaPath.at(i + 1).y + xythetaPath.at(i).y) / 4.0;
                temp.z = (3 * xythetaPath.at(i + 1).z + xythetaPath.at(i).z) / 4.0;
                spline->push_back(temp);
            }
            else if (fabs(xythetaPath.at(i).x - xythetaPath.at(i + 1).x) < 0.36 ||
                     fabs(xythetaPath.at(i).y - xythetaPath.at(i + 1).y) < 0.36)
            {
                temp.x = (xythetaPath.at(i + 1).x + 4 * xythetaPath.at(i).x) / 5.0;
                temp.y = (xythetaPath.at(i + 1).y + 4 * xythetaPath.at(i).y) / 5.0;
                temp.z = (xythetaPath.at(i + 1).z + 4 * xythetaPath.at(i).z) / 5.0;
                spline->push_back(temp);
                temp.x = (2 * xythetaPath.at(i + 1).x + 3 * xythetaPath.at(i).x) / 5.0;
                temp.y = (2 * xythetaPath.at(i + 1).y + 3 * xythetaPath.at(i).y) / 5.0;
                temp.z = (2 * xythetaPath.at(i + 1).z + 3 * xythetaPath.at(i).z) / 5.0;
                spline->push_back(temp);
                temp.x = (3 * xythetaPath.at(i + 1).x + 2 * xythetaPath.at(i).x) / 5.0;
                temp.y = (3 * xythetaPath.at(i + 1).y + 2 * xythetaPath.at(i).y) / 5.0;
                temp.z = (3 * xythetaPath.at(i + 1).z + 2 * xythetaPath.at(i).z) / 5.0;
                spline->push_back(temp);
                temp.x = (4 * xythetaPath.at(i + 1).x + 1 * xythetaPath.at(i).x) / 5.0;
                temp.y = (4 * xythetaPath.at(i + 1).y + 1 * xythetaPath.at(i).y) / 5.0;
                temp.z = (4 * xythetaPath.at(i + 1).z + 1 * xythetaPath.at(i).z) / 5.0;
                spline->push_back(temp);
            }
            else
            {
                temp.x = (xythetaPath.at(i + 1).x + 5 * xythetaPath.at(i).x) / 6.0;
                temp.y = (xythetaPath.at(i + 1).y + 5 * xythetaPath.at(i).y) / 6.0;
                temp.z = (xythetaPath.at(i + 1).z + 5 * xythetaPath.at(i).z) / 6.0;
                spline->push_back(temp);
                temp.x = (2 * xythetaPath.at(i + 1).x + 4 * xythetaPath.at(i).x) / 6.0;
                temp.y = (2 * xythetaPath.at(i + 1).y + 4 * xythetaPath.at(i).y) / 6.0;
                temp.z = (2 * xythetaPath.at(i + 1).z + 4 * xythetaPath.at(i).z) / 6.0;
                spline->push_back(temp);
                temp.x = (3 * xythetaPath.at(i + 1).x + 3 * xythetaPath.at(i).x) / 6.0;
                temp.y = (3 * xythetaPath.at(i + 1).y + 3 * xythetaPath.at(i).y) / 6.0;
                temp.z = (3 * xythetaPath.at(i + 1).z + 3 * xythetaPath.at(i).z) / 6.0;
                spline->push_back(temp);
                temp.x = (4 * xythetaPath.at(i + 1).x + 2 * xythetaPath.at(i).x) / 6.0;
                temp.y = (4 * xythetaPath.at(i + 1).y + 2 * xythetaPath.at(i).y) / 6.0;
                temp.z = (4 * xythetaPath.at(i + 1).z + 2 * xythetaPath.at(i).z) / 6.0;
                spline->push_back(temp);
                temp.x = (5 * xythetaPath.at(i + 1).x + 1 * xythetaPath.at(i).x) / 6.0;
                temp.y = (5 * xythetaPath.at(i + 1).y + 1 * xythetaPath.at(i).y) / 6.0;
                temp.z = (5 * xythetaPath.at(i + 1).z + 1 * xythetaPath.at(i).z) / 6.0;
                spline->push_back(temp);
            }
        }
    }
    temp.x = xythetaPath.back().x;
    temp.y = xythetaPath.back().y;
    temp.z = xythetaPath.back().z;
    spline->push_back(temp);
}





}//The end of the namespace functions!!!

#endif
