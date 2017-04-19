#include "dwaplanner.h"

dwaplanner::dwaplanner(ros::NodeHandle nh)
    : deltat(0.6), stepsize(0.15), yawtolerance(30.0), disttolerance(0.5), precosth(0.0), count(0), REVERSE(false) //Holy shit, this is so bad!!!
{

  visdwa_pose = std::unique_ptr<visualize_pose>(new visualize_pose("/dwa"));
  visdwa_path = std::unique_ptr<visualize_path>(new visualize_path("/dwa"));
  visdwa_marker = std::unique_ptr<visualize_marker>(new visualize_marker("/dwa"));
  visdwa_polygon = std::unique_ptr<visualize_polygon>(new visualize_polygon("/dwa"));
  vishybrid_path = std::unique_ptr<visualize_path>(new visualize_path("/hybridastar"));

  for (int i = 0; i <= 20; ++i)
  {
    samplenum.emplace(i, i);
  }
  // heading evaluation
  evalinfo.heading_coeff = 0.2;
  // clearance evaluation
  evalinfo.clearance_coeff = 0.2;
  // speed evaluation
  evalinfo.velocity_coeff = 20.0;
  evalinfo.multiply_coeff = 10;

  initialstatus.x = 0.0;
  initialstatus.y = 0.0;
  initialstatus.heading = toradian(0.0);
  initialstatus.linear_velocity = 3.0;
  initialstatus.angular_velocity = toradian(0.0);

  updatingstatus = initialstatus;

  goalstatus.x = 18.0;
  goalstatus.y = 18.0;
  goalstatus.heading = toradian(90.0);
  goalstatus.angular_velocity = toradian(0.0);
  goalstatus.linear_velocity = 0.0;

  dyn.max_linear_velocity = 5.0;
  //This is the recommended centrifugal_acceleration
  dyn.centrifugal_acceleration = 1.5;
  dyn.max_linear_acceleration = 2.0;
  dyn.min_turn_radius = functions::radius;

  geometry_msgs::Point32 test;
  for (int i = 0; i <= 10; ++i)
  {
    test.x = i;
    test.y = 2;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 0; i <= 10; ++i)
  {
    test.x = i;
    test.y = -2;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 14; i <= 24; ++i)
  {
    test.x = i;
    test.y = 2;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 14; i <= 24; ++i)
  {
    test.x = i;
    test.y = -2;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 2; i <= 10; ++i)
  {
    test.x = 10;
    test.y = i;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 2; i <= 10; ++i)
  {
    test.x = 10;
    test.y = -i;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 2; i <= 10; ++i)
  {
    test.x = 14;
    test.y = i;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  for (int i = 2; i <= 10; ++i)
  {
    test.x = 14;
    test.y = -i;
    mtest.push_back(test);
    obstacle.push_back(test);
  }
  test.x = 7;
  test.y = 0;
  mtest.push_back(test);
  obstacle.push_back(test);
  test.x = 3;
  test.y = -1;
  mtest.push_back(test);
  obstacle.push_back(test);
  // obstacleprocess(obstacle, sortedobs);
  std::cout << "The size of the sortedobs: " << sortedobs.size() << std::endl;
}

dwaplanner::dwaplanner()
{
}

dwaplanner::~dwaplanner() {}

std::vector<geometry_msgs::Point32> dwaplanner::bsplinecurve(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt)
{
  int step = 20;
  geometry_msgs::Point32 temp, temppt, prept;
  std::deque<geometry_msgs::Point32> controlvec;
  std::vector<geometry_msgs::Point32> bspline;
  float dis = sqrt(pow(startpt.x - goalpt.x, 2) + pow(startpt.y - goalpt.y, 2));
  // if (dis <= 0.5)
  //   return bspline;
  step = std::ceil(dis / 0.3);
  // ROS_WARN_STREAM("The bspline step: " << step);
  float g0, g1, g2, g3, t = 0;
  float offset = 0.5;
  offset = std::max(fabs(startpt.x - goalpt.x), fabs(startpt.y - goalpt.y)) / 3.0;

  controlvec.push_back(startpt);
  temp.x = startpt.x + offset * cos(startpt.z);
  temp.y = startpt.y + offset * sin(startpt.z);
  controlvec.push_back(temp);
  temp.x = goalpt.x - offset * cos(goalpt.z);
  temp.y = goalpt.y - offset * sin(goalpt.z);
  controlvec.push_back(temp);
  controlvec.push_back(goalpt);

  temp.x = 2 * controlvec.at(0).x - controlvec.at(1).x;
  temp.y = 2 * controlvec.at(0).y - controlvec.at(1).y;
  temp.z = 0;
  controlvec.push_front(temp);

  int num = controlvec.size();

  temp.x = 2 * controlvec.at(num - 1).x - controlvec.at(num - 2).x;
  temp.y = 2 * controlvec.at(num - 1).y - controlvec.at(num - 2).y;
  temp.z = 0;
  controlvec.push_back(temp);

  prept = startpt;
  for (int a = 0; a < controlvec.size() - 3; a++)
  {
    t = 0;
    for (int b = 0; b <= step; b++)
    {
      // t += 1.0 / step;
      g0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
      g1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
      g2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
      g3 = (t * t * t) / 6.0;
      t += 1.0 / step;
      temppt.x = controlvec.at(a).x * g0 + controlvec.at(a + 1).x * g1 + controlvec.at(a + 2).x * g2 +
                 controlvec.at(a + 3).x * g3;
      temppt.y = controlvec.at(a).y * g0 + controlvec.at(a + 1).y * g1 + controlvec.at(a + 2).y * g2 +
                 controlvec.at(a + 3).y * g3;
      temppt.z = atan2(temppt.y - prept.y, temppt.x - prept.x);
      prept.x = temppt.x;
      prept.y = temppt.y;
      prept.z = temppt.z;

      // Avoiding the overlapped point and it is quite essential
      if (b == 0)
        continue;
      bspline.push_back(temppt);
    }
  }
  return bspline;
}

void dwaplanner::calculateradius(sVehicleElem *info, sRoadElem *rtn)
{
  memset(rtn, 0, sizeof(*rtn));
  double od = info->wheelbase / tan(info->outerangle);
  double oa = info->wheelbase / sin(info->outerangle);
  /*One method*/
  double ae = info->frontoverhang * sin(info->outerangle);
  double l1 = od - info->width;
  double l2 = oa;
  double l3 = oa + ae;
  double l = l3 - l1;
  ROS_INFO_STREAM("Minimun turn radius: " << l3 << "; Minimum road width: " << l);
  /*Another method*/
  rtn->turnradiusmax = l3;
  double r = sqrt(rtn->turnradiusmax * rtn->turnradiusmax) - (info->tailwheel + info->width) / 2.0;
  double r2 = r - info->innersafedis;
  double rr = sqrt((info->wheelbase + info->frontoverhang) * (info->wheelbase + info->frontoverhang) + (r + info->width) * (r + info->width));
  double r0 = rr + info->outersafedis;
  double w = r0 - r2;
  ROS_INFO_STREAM("Road outer radius: " << r0 << "; Road inner radius: " << r2 << "; Minimum road width: " << w);
  rtn->roadouterradius = r0;
  rtn->roadinnerradius = r2;
  rtn->roadwidthmin = w;
  rtn->turnradiusmin = r + info->width / 2.0;
  ROS_INFO_STREAM("rtn->turnradiusmin: " << rtn->turnradiusmin);
  return;
}

inline float dwaplanner::CalcHeadingEval(sVehicleStatus predictvs, sVehicleStatus goalvs)
{
  return (180.0 - fabs(todegree(goalvs.heading) - todegree(predictvs.heading)));
  //   return fabs(todegree(goalvs.heading) - todegree(predictvs.heading));
}

inline float dwaplanner::CalcBreakingDist(sVehicleStatus predictvs, sVehicleStatus goalvs)
{
  // return (sqrt((goalvs.x - predictvs.x) * (goalvs.x - predictvs.x) + \
  //       (goalvs.y - predictvs.y) * (goalvs.y - predictvs.y)));
  ReedsSheppStateSpace reeds_shepp(2.5);
  double q0[3] = {predictvs.x, predictvs.y, predictvs.heading};
  double q1[3] = {goalvs.x, goalvs.y, goalvs.heading};
  return (reeds_shepp.distance(q0, q1));
}

int dwaplanner::collisiondetection(ivpathplanner::path ego, sorted_tree &obspts)
{
  geometry_msgs::Point32 p, q, tp;
  geometry_msgs::Polygon poly;
  if (obspts.size() == 0)
  {
    return 0;
  }
  for (auto tt : ego.points)
  {
    tp.x = tt.x;
    tp.y = tt.y;
    tp.z = tt.angle * M_PI / 180.0;
    std::tie(p, q) = functions::carmodel(tp, poly);
    auto ha = obspts.lower_bound(p.x);
    if (!obspts.count(ha->first) || ha->first > q.x)
    {
      continue;
    }
    else
    {
      for (; ha->first <= q.x && ha != obspts.end(); ++ha)
      {
        auto hi = obspts[ha->first].lower_bound(p);
        if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
        {
          continue;
        }
        else
        {
          for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
          {
            if (functions::IsInsideFootprint(*hi, poly))
            {
              return 1;
            }
          }
        }
      }
    }
  }
  return 0;
}

int dwaplanner::collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree &obspts)
{
  geometry_msgs::Point32 p, q, tp;
  geometry_msgs::Polygon poly;
  if (obspts.size() == 0)
  {
    return 0;
  }
  for (auto tt : ego)
  {
    tp.x = tt.x;
    tp.y = tt.y;
    tp.z = tt.heading;
    std::tie(p, q) = functions::carmodel(tp, poly);
    auto ha = obspts.lower_bound(p.x);
    if (!obspts.count(ha->first) || ha->first > q.x)
    {
      continue;
    }
    else
    {
      for (; ha->first <= q.x && ha != obspts.end(); ++ha)
      {
        auto hi = obspts[ha->first].lower_bound(p);
        if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
        {
          continue;
        }
        else
        {
          for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
          {
            if (functions::IsInsideFootprint(*hi, poly))
            {
              return 1;
            }
          }
        }
      }
    }
  }
  return 0;
}

int dwaplanner::DynamicWindowApproach(sVehicleStatus vs, sVehicleStatus goalvs)
{
  //   trajectory.clear();
  GenerateTrajectory(vs, dyn);
  NormalizeEval(trajectorys, evalpts);
  functions::PriorityNode<int, float>::PQElement rtn = Evaluation(evalpts);
  //   functions::PriorityNode<cv::Point3f, float> rtn = reEvaluation(evalpts);
  updatingstatus = trajectorys.at(rtn.first).back();
  //Three simulating modes!!!
  std::vector<geometry_msgs::Point32> singlepath;
  for (int i = 0; i < trajectory.size(); ++i)
  {
    geometry_msgs::Point32 temp;
    temp.x = trajectory.at(i).x;
    temp.y = trajectory.at(i).y;
    temp.z = trajectory.at(i).heading;
    singlepath.push_back(temp);
  }
  visdwa_path->publishpath(singlepath);

  std::vector<std::vector<geometry_msgs::Point32>> multipath;
  for (int i = 0; i < trajectorys.size(); ++i)
  {
    std::vector<geometry_msgs::Point32> tempv;
    for (int j = 0; j < trajectorys.at(i).size(); ++j)
    {
      geometry_msgs::Point32 temp;
      temp.x = trajectorys[i][j].x;
      temp.y = trajectorys[i][j].y;
      temp.z = trajectorys[i][j].heading;
      tempv.push_back(temp);
    }
    multipath.push_back(tempv);
  }
  visdwa_marker->publishmultipath(multipath, rtn.first);
  trajectory.insert(trajectory.end(), trajectorys.at(rtn.first).begin(), trajectorys.at(rtn.first).end());
}

functions::PriorityNode<int, float>::PQElement dwaplanner::Evaluation(std::vector<cv::Point3f> evaldb)
{
  //The stl priority_queue and boost binomial_queue;
  std::priority_queue<functions::PriorityNode<int, float>::PQElement> refqueue;
  functions::PriorityNode<int, float> rtnqueue;
  for (int i = 0; i < evaldb.size(); ++i)
  {
    functions::PriorityNode<int, float>::PQElement temp;
    //Store the index
    temp.first = i;
    //Store the cost
    temp.second = evalinfo.multiply_coeff * (evalinfo.heading_coeff * evaldb.at(i).x + evalinfo.velocity_coeff * evaldb.at(i).z);
    refqueue.push(temp);
    rtnqueue.elements.push(temp);
  }
  return rtnqueue.elements.top();
  //   return refqueue.top();
}

sVehicleStatus dwaplanner::evolvedastar(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
                                        unordered_t<float> &cost_so_far, unordered_t<vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs)
{
  int remark = 0;
  sVehicleStatus current;
  functions::PriorityNode<sVehicleStatus, float> frontier;
  frontier.elements.emplace(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;
  while (!frontier.elements.empty())
  {
    current = frontier.elements.top().first;
    frontier.elements.pop();
    if (collisiondetection(pathtable[current], sortedobjs))
    {
      continue;
    }
    if (sqrt(pow(current.x - goal.x, 2) + pow(current.x - goal.x, 2)) <= 0.2 || remark >= 150)
    {
      break;
    }
    remark++;
    for (auto next : lookuptable(current, dyn))
    {
      auto nextpath = std::get<1>(next);
      sVehicleStatus nextvs = std::get<0>(next);
      float new_cost = cost_so_far[current] + std::get<2>(next) * 0.15; // Update the G-value here!!!
      // Only if the successor is not in the close table
      if (!came_from.count(nextvs))
      {
        // If the successor is not in the open table, If the successor is in the open table
        if (!cost_so_far.count(nextvs) || (new_cost < cost_so_far[nextvs]))
        {
          float temph = heuristic(nextvs, goal);
          float priority = new_cost + temph;
          frontier.elements.emplace(nextvs, priority);
          cost_so_far[nextvs] = new_cost;
          came_from[nextvs] = current;
          pathtable[nextvs] = nextpath;
        }
      }
    }
  }
  return current;
}

std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
dwaplanner::GenerateTrajectory(sVehicleStatus currentvs, sKinematic sk)
{
  trajectorys.clear();

  float line_speed = 0.0;
  float angular_speed = 0.0;
  int velocity_num = 6;
  int angular_num = 5;
  float tp1, tp2, tp3;
  // linear_velocity, angular_velocity, and G cost

  std::vector<std::tuple<float, float, float>> speedarray;
  std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>> rtn;

  // velocity and omega sampling
  line_speed = sk.max_linear_acceleration * deltat / velocity_num;

  /*    the reversing mode sampling     */
  if (REVERSE)
  {
    std::cout << "Reversing mode!!!" << std::endl;
  }
  /*    the normal mode sampling    */
  else
  {
    // for (int i = -velocity_num; i <= velocity_num; ++i)
    for (int i = -1; i <= 1; ++i)
    {
      tp1 = currentvs.linear_velocity + i * line_speed;
      if (tp1 < 1e-2 || tp1 > sk.max_linear_velocity)
        continue;
      if (tp1 <= 0.5)
      {
        angular_num = samplenum[3] - 1;
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        angular_speed = sk.max_angular_acceleration * deltat / angular_num;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = currentvs.angular_velocity + angular_speed * j;
          if (fabs(tp2) > sk.max_angular_velocity || fabs(tp2 * tp1) > sk.centrifugal_acceleration)
            // if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else if (tp1 <= 1.0)
      {
        angular_num = samplenum[5] - 1;
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        angular_speed = sk.max_angular_acceleration * deltat / angular_num;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = currentvs.angular_velocity + angular_speed * j;
          if (fabs(tp2) > sk.max_angular_velocity || fabs(tp2 * tp1) > sk.centrifugal_acceleration)
            // if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else if (tp1 <= 3.0)
      {
        angular_num = samplenum[5] - 1;
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        angular_speed = sk.max_angular_acceleration * deltat / angular_num;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = currentvs.angular_velocity + angular_speed * j;
          if (fabs(tp2) > sk.max_angular_velocity || fabs(tp2 * tp1) > sk.centrifugal_acceleration)
            // if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else
      {
        angular_num = samplenum[5] - 1;
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        angular_speed = sk.max_angular_acceleration * deltat / angular_num;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = currentvs.angular_velocity + angular_speed * j;
          if (fabs(tp2) > sk.max_angular_velocity || fabs(tp2 * tp1) > sk.centrifugal_acceleration)
            // if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
    }
  }
  // The trajectory samping process
  for (auto spd : speedarray)
  {
    std::vector<sVehicleStatus> path;
    sVehicleStatus tempvs;
    geometry_msgs::Point32 temp;
    tempvs = currentvs;
    path.push_back(tempvs);
    tempvs.linear_velocity = std::get<0>(spd);
    tempvs.angular_velocity = std::get<1>(spd);
    float dis = tempvs.linear_velocity * deltat;
    float diffnum = std::ceil(dis / stepsize);
    float timestep = deltat / diffnum;
    for (float a = 0.0; a <= deltat; a += timestep)
    {
      // Vehicle state sampling
      tempvs.heading += tempvs.angular_velocity * timestep;
      if (std::get<1>(spd) < 1e-2)
      {
        tempvs.x += tempvs.linear_velocity * timestep * cos(tempvs.heading);
        tempvs.y += tempvs.linear_velocity * timestep * sin(tempvs.heading);
      }
      else
      {
        //The approximate motion equations differ from the essay!!!
        //@brief refer to the <<Probabilistic robotics>>, table 5.3, 5.9
        tempvs.x += (tempvs.linear_velocity / tempvs.angular_velocity) *
                    (sin(tempvs.heading + tempvs.angular_velocity * timestep) - sin(tempvs.heading));
        tempvs.y += (tempvs.linear_velocity / tempvs.angular_velocity) *
                    (-cos(tempvs.heading + tempvs.angular_velocity * timestep) + cos(tempvs.heading));
      }
      path.push_back(tempvs);
    }
    trajectorys.push_back(path);
    float cg = fabs(tempvs.linear_velocity - currentvs.linear_velocity) +
               fabs(tempvs.heading - goalstatus.heading) * 0.5 +
               fabs(tempvs.heading - currentvs.heading);
    // rtn.emplace_back(std::forward_as_tuple(path.back(), path, cg));
    rtn.emplace_back(std::forward_as_tuple(path.back(), path, std::get<2>(spd) + cg));
    // rtn.emplace_back(std::forward_as_tuple(path.back(), path, std::get<2>(spd)));
  }
#if 0
  std::vector<std::vector<geometry_msgs::Point32>> multipath;
  for (int i = 0; i < trajectorys.size(); ++i)
  {
    std::vector<geometry_msgs::Point32> tempv;
    for (int j = 0; j < trajectorys.at(i).size(); ++j)
    {
      geometry_msgs::Point32 temp;
      temp.x = trajectorys[i][j].x;
      temp.y = trajectorys[i][j].y;
      temp.z = trajectorys[i][j].heading;
      tempv.push_back(temp);
    }
    multipath.push_back(tempv);
  }
  visdwa_marker->publishmultipath(multipath, 0);
#endif
  return rtn;
}

//This is a new distance heuristic method!!!
float dwaplanner::globalpath(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt, std::vector<geometry_msgs::Point32> &pts)
{
  geometry_msgs::Point32 crspt, temp, center;
  geometry_msgs::Vector3 connect, vec;
  float radius;
  float lenstep = 0.15;
  float thetastep;
  float minradius = functions::radius;
  float tempradian;
  float length;
  float refpos1, refpos2;
  float symbol1, symbol2;
  float radiandiff = acos(cos(startpt.z) * cos(goalpt.z) + sin(startpt.z) * sin(goalpt.z));

  connect.x = goalpt.x - startpt.x;
  connect.y = goalpt.y - startpt.y;
  // Determins the goal to start position
  symbol1 = cos(startpt.z) * connect.y - sin(startpt.z) * connect.x;
  // Determins the goal to the linkline position
  symbol2 = cos(goalpt.z) * connect.y - sin(goalpt.z) * connect.x;
  // Normalize the symbol
  symbol1 = (symbol1 > 1e-3) ? 1.0 : ((symbol1 < -1e-3) ? -1.0 : 0.0);
  symbol2 = (symbol2 > 1e-3) ? 1.0 : ((symbol2 < -1e-3) ? -1.0 : 0.0);

  if (fabs(symbol1) < 1e-3)
  {
    float dir = cos(startpt.z) * cos(goalpt.z) + sin(startpt.z) * sin(goalpt.z);
    // Collinear and in the same direction
    if (dir > 0.0)
    {
      float d = 0.0;
      vec.x = (goalpt.x - startpt.x) >= 0.0 ? 1.0 : -1.0;
      vec.y = (goalpt.y - startpt.y) >= 0.0 ? 1.0 : -1.0;
      float dis = sqrt(pow(goalpt.x - startpt.x, 2) + pow(goalpt.y - startpt.y, 2));
      while (d <= dis)
      {
        temp.x = startpt.x + vec.x * d * cos(startpt.z);
        temp.y = startpt.y + vec.y * d * sin(startpt.z);
        temp.z = startpt.z;
        pts.push_back(temp);
        d += lenstep;
      }
      length = dis;
    }
    else
    // Collinear and in the opposite direction and needs to be implemented later
    {
    }
  }
  else
  {
    // This is the normal case
    if ((symbol2 > 1e-3 && symbol1 < -1e-3) || (symbol2 < -1e-3) && (symbol1 > 1e-3))
    {
      // Startline
      float c1 = sin(startpt.z) * startpt.x - cos(startpt.z) * startpt.y;
      // Goalline
      float c2 = sin(goalpt.z) * goalpt.x - cos(goalpt.z) * goalpt.y;
      // Get the crossing point
      float denominator = sin(startpt.z) * cos(goalpt.z) - cos(startpt.z) * sin(goalpt.z);
      crspt.x = (c1 * cos(goalpt.z) - c2 * cos(startpt.z)) / denominator;
      crspt.y = (c1 * sin(goalpt.z) - c2 * sin(startpt.z)) / denominator;
      float includedradian = (M_PI - radiandiff) / 2.0;
      float dis1 = sqrt(pow(startpt.x - crspt.x, 2) + pow(startpt.y - crspt.y, 2));
      float dis2 = sqrt(pow(goalpt.x - crspt.x, 2) + pow(goalpt.y - crspt.y, 2));
      float dis = fabs(dis1 - dis2);

      // The judgement is essential, determines if the crosspoint is in front of startpt or not
      float flag = (crspt.x - startpt.x) * cos(startpt.z) + (crspt.y - startpt.y) * sin(startpt.z);
      if (flag * (dis1 - dis2) > 1e-3)
      {
        radius = tan(includedradian) * dis2;
        thetastep = lenstep / radius;
        tempradian = goalpt.z - symbol1 * M_PI / 2.0;
        center.x = goalpt.x - cos(tempradian) * radius;
        center.y = goalpt.y - sin(tempradian) * radius;
        float radian = 0.0;
        float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
        while (radian <= radiansum)
        {
          tempradian -= symbol1 * thetastep;
          temp.x = center.x + radius * cos(tempradian);
          temp.y = center.y + radius * sin(tempradian);
          temp.z = goalpt.z - symbol1 * radian;
          radian += thetastep;
          pts.push_back(temp);
        }
        std::reverse(pts.begin(), pts.end());
        float d = 0.0;
        std::vector<geometry_msgs::Point32> array;
        while (d <= dis)
        {
          temp.x = startpt.x + d * cos(startpt.z);
          temp.y = startpt.y + d * sin(startpt.z);
          temp.z = startpt.z;
          d += lenstep;
          array.push_back(temp);
        }
        pts.insert(pts.begin(), array.begin(), array.end());
        length = dis + radiandiff * radius;
      }
      else if (flag * (dis2 - dis1) > 1e-3)
      {
        radius = tan(includedradian) * dis1;
        thetastep = lenstep / radius;
        tempradian = startpt.z - symbol1 * M_PI / 2.0;
        center.x = startpt.x - cos(tempradian) * radius;
        center.y = startpt.y - sin(tempradian) * radius;
        float radian = 0.0;
        float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
        while (radian <= radiansum)
        {
          tempradian += symbol1 * thetastep;
          temp.x = center.x + radius * cos(tempradian);
          temp.y = center.y + radius * sin(tempradian);
          temp.z = startpt.z + symbol1 * radian;
          radian += thetastep;
          pts.push_back(temp);
        }
        float d = 0.0;
        std::vector<geometry_msgs::Point32> array;
        while (d <= dis)
        {
          temp.x = goalpt.x - d * cos(goalpt.z);
          temp.y = goalpt.y - d * sin(goalpt.z);
          temp.z = goalpt.z;
          d += lenstep;
          array.push_back(temp);
        }
        std::reverse(array.begin(), array.end());
        pts.insert(pts.end(), array.begin(), array.end());
        length = dis + radiandiff * radius;
      }
      else
      {
        radius = tan(includedradian) * dis1;
        thetastep = lenstep / radius;
        tempradian = startpt.z - symbol1 * M_PI / 2.0;
        center.x = startpt.x - cos(tempradian) * radius;
        center.y = startpt.y - sin(tempradian) * radius;
        float radian = 0.0;
        float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
        while (radian <= radiansum)
        {
          tempradian += symbol1 * thetastep;
          temp.x = center.x + radius * cos(tempradian);
          temp.y = center.y + radius * sin(tempradian);
          temp.z = startpt.z + symbol1 * radian;
          radian += thetastep;
          pts.push_back(temp);
          length = dis + radiandiff * radius;
        }
      }
    }
  }
  return length;
}

//This H-value updating function bind the dubins_curve and the reeds_shepp as heuristic!!!
float dwaplanner::heuristic(sVehicleStatus cur, sVehicleStatus goal)
{
  float dubinsCost, reedsSheppCost, disCost;
  float dubins_radius = functions::radius;
  geometry_msgs::Point32 p1, p2;
  std::vector<geometry_msgs::Point32> arr;
  p1.x = cur.x;
  p1.y = cur.y;
  p1.z = cur.heading;
  p2.x = goal.x;
  p2.y = goal.y;
  p2.z = goal.heading;
  // cur.heading = std::floor(cur.heading * 36.0 / M_PI) * M_PI / 36.0;
  // goal.heading = std::floor(goal.heading * 36.0 / M_PI) * M_PI / 36.0;
  double q0[3] = {cur.x, cur.y, cur.heading};
  double q1[3] = {goal.x, goal.y, goal.heading};
  auto cb = [](double q[3], double x, void *user_data) -> int {
    return 1;
  };

  dubins_curve::DubinsPath dubins_path;
  dubins_curve::dubins_init(q0, q1, dubins_radius, &dubins_path);
  dubins_curve::dubins_path_sample_many(&dubins_path, cb, 0.2, nullptr);
  dubinsCost = dubins_curve::dubins_path_length(&dubins_path);

  //     ReedsSheppStateSpace reeds_shepp(dubins_radius);
  //     reedsSheppCost = reeds_shepp.distance(q0, q1);

  // disCost = globalpath(p1, p2, arr);

  // return std::min(reedsSheppCost, dubinsCost);

  return dubinsCost;
}

//The kernel function of this god-like algorithm
sVehicleStatus dwaplanner::hybridastar(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
                                       unordered_t<float> &cost_so_far, unordered_t<std::vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs)
{
  int remark = 0;
  sVehicleStatus current;
  functions::PriorityNode<sVehicleStatus, float> frontier;
  frontier.elements.emplace(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0.0;
  while (!frontier.elements.empty())
  {
    current = frontier.elements.top().first;
    frontier.elements.pop();
    //This algorithm is like holy shit, and perfectly wonderful!!!
    if (collisiondetection(pathtable[current], sortedobjs))
    {
      continue;
    }
    if (sqrt(pow(current.x - goal.x, 2) + pow(current.x - goal.x, 2)) <= 0.2 || remark >= 150)
    {
      break;
    }
    remark++;
    for (auto next : GenerateTrajectory(current, dyn))
    {
      //Compare the algorithm efficiency!!!
      auto nextpath = std::get<1>(next);
      sVehicleStatus nextvs = std::get<0>(next);
      float new_cost = cost_so_far[current] + std::get<2>(next) * 0.1;
      // Only if the successor is not in the close table
      if (!came_from.count(nextvs))
      {
        // If the successor is not in the open table, If the successor is in the open table
        if (!cost_so_far.count(nextvs) || (new_cost < cost_so_far[nextvs]))
        {
          float temph = heuristic(nextvs, goal);
          float priority = new_cost + temph;
          precosth = temph;
          frontier.elements.emplace(nextvs, priority);
          cost_so_far[nextvs] = new_cost;
          came_from[nextvs] = current;
          pathtable[nextvs] = nextpath;
        }
      }
    }
  }
  //std::cout << "The loop sequences >>>>>>>>>>>> " << remark << std::endl;
  return current;
}

std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
dwaplanner::lookuptable(sVehicleStatus currentvs, sKinematic sk)
{
  trajectorys.clear();
  float line_speed = 0.0;
  int velocity_num = 6;
  // Evering angular_num change the radian by 5.0 degrees!!!
  int angular_num = 5;
  float tp1, tp2, tp3;
  std::vector<std::tuple<float, float, float>> speedarray;
  std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>> rtn;

  // velocity and omega sampling
  line_speed = sk.max_linear_acceleration * deltat / velocity_num;

  /*    the reversing mode sampling     */
  if (REVERSE)
  {
    std::cout << "Reversing mode!!!" << std::endl;
  }
  /*    the normal mode sampling    */
  else
  {
    // for (int i = -velocity_num; i <= velocity_num; ++i)
    for (int i = -1; i <= 1; ++i)
    {
      tp1 = currentvs.linear_velocity + i * line_speed;
      if (tp1 < 1e-2 || tp1 > sk.max_linear_velocity)
        continue;
      if (tp1 <= 0.5)
      {
        angular_num = samplenum[3];
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = j * 5.0 * M_PI / 180.0 / deltat;
          if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else if (tp1 <= 1.0)
      {
        angular_num = samplenum[5];
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = j * 5.0 * M_PI / 180.0 / deltat;
          if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else if (tp1 <= 3.0)
      {
        angular_num = samplenum[6];
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = j * 5.0 * M_PI / 180.0 / deltat;
          if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
      else
      {
        angular_num = samplenum[7];
        sk.max_angular_velocity = fabs(tp1) / sk.min_turn_radius;
        sk.max_angular_acceleration = sk.max_angular_velocity / deltat;
        for (int j = -angular_num; j <= angular_num; ++j)
        {
          tp2 = j * 5.0 * M_PI / 180.0 / deltat;
          if (fabs(tp2) > sk.max_angular_velocity)
            continue;
          speedarray.emplace_back(std::forward_as_tuple(tp1, tp2, fabs(tp1 * deltat) * (functions::penaltyTurning + fabs(j) * 0.1)));
        }
      }
    }
  }
  // The trajectory samping process
  for (auto spd : speedarray)
  {
    std::vector<sVehicleStatus> path;
    sVehicleStatus tempvs;
    geometry_msgs::Point32 temp;
    tempvs = currentvs;
    path.push_back(tempvs);
    tempvs.linear_velocity = std::get<0>(spd);
    tempvs.angular_velocity = std::get<1>(spd);
    float dis = tempvs.linear_velocity * deltat;
    float diffnum = std::ceil(dis / stepsize);
    float timestep = deltat / diffnum;
    for (float a = 0.0; a <= deltat; a += timestep)
    {
      // Vehicle state sampling
      tempvs.heading += tempvs.angular_velocity * timestep;
      if (std::get<1>(spd) < 1e-2)
      {
        tempvs.x += tempvs.linear_velocity * timestep * cos(tempvs.heading);
        tempvs.y += tempvs.linear_velocity * timestep * sin(tempvs.heading);
      }
      else
      {
        //The approximate motion equations differ from the essay!!!
        //@brief refer to the <<Probabilistic robotics>>, table 5.3, 5.9
        tempvs.x += (tempvs.linear_velocity / tempvs.angular_velocity) *
                    (sin(tempvs.heading + tempvs.angular_velocity * timestep) - sin(tempvs.heading));
        tempvs.y += (tempvs.linear_velocity / tempvs.angular_velocity) *
                    (-cos(tempvs.heading + tempvs.angular_velocity * timestep) + cos(tempvs.heading));
      }
      path.push_back(tempvs);
    }
    trajectorys.push_back(path);
    float cg = fabs(tempvs.linear_velocity - currentvs.linear_velocity) +
               fabs(tempvs.heading - goalstatus.heading) * 0.5 +
               fabs(tempvs.heading - currentvs.heading);
    // rtn.emplace_back(std::forward_as_tuple(path.back(), path, cg));
    rtn.emplace_back(std::forward_as_tuple(path.back(), path, std::get<2>(spd) + cg));
  }
#if 0
  std::vector<std::vector<geometry_msgs::Point32>> multipath;
  for (int i = 0; i < trajectorys.size(); ++i)
  {
    std::vector<geometry_msgs::Point32> tempv;
    for (int j = 0; j < trajectorys.at(i).size(); ++j)
    {
      geometry_msgs::Point32 temp;
      temp.x = trajectorys[i][j].x;
      temp.y = trajectorys[i][j].y;
      temp.z = trajectorys[i][j].heading;
      tempv.push_back(temp);
    }
    multipath.push_back(tempv);
  }
  visdwa_marker->publishmultipath(multipath, 0);
#endif
  return rtn;
}

void dwaplanner::NormalizeEval(std::vector<std::vector<sVehicleStatus>> path, std::vector<cv::Point3f> &evaldb)
{
  evaldb.clear();
  // Heading, clearance, distance evaluation parameters
  float totalyaw;
  float totaldist;
  float totalvel;
  for (int i = 0; i < path.size(); ++i)
  {
    totalyaw += CalcHeadingEval(path.at(i).back(), goalstatus);
    totalvel += CalcBreakingDist(path.at(i).back(), goalstatus);
  }
  for (int i = 0; i < path.size(); ++i)
  {
    cv::Point3f temp;
    // Normalize heading evaluation parameter
    //     temp.x = path.size() * CalcHeadingEval(path.at(i).back(), goalstatus) / totalyaw;
    //     // Normalize distance evaluation parameter
    //     temp.z = -path.size() * CalcBreakingDist(path.at(i).back(), goalstatus) / totalvel;

    temp.x = path.size() * CalcHeadingEval(path.at(i).back(), goalstatus);
    // Normalize distance evaluation parameter
    temp.z = -path.size() * CalcBreakingDist(path.at(i).back(), goalstatus);

    evaldb.push_back(temp);
  }
  return;
}

void dwaplanner::obstacleprocess(std::vector<geometry_msgs::Point32> obs, sorted_tree &rtnobs)
{
  for (auto a : obs)
  {
    rtnobs[a.x].insert(a);
  }
  return;
}

void dwaplanner::obstacleprocess(ivmap::ivmapmsgobj objs, sorted_tree &rtnobs)
{
  geometry_msgs::Point32 tt;
  for (auto a : objs.obj)
  {
    a.width += 0.8;
    a.length += 0.8;
    float widthseg = a.width / std::ceil(a.width / 0.50);
    float lengthseg = a.length / std::ceil(a.length / 0.50);
    for (float x = a.x - a.length / 2.0; x <= a.x + a.length / 2.0; x += lengthseg)
    {
      for (float y = a.y - a.width / 2.0; y <= a.y + a.width / 2.0; y += widthseg)
      {
        tt.x = x;
        tt.y = y;
        rtnobs[tt.x].insert(tt);
      }
    }
  }
  return;
}

void dwaplanner::obstaclemodify(ivmap::ivmapmsgobj objs, sorted_tree &rtnobs)
{
  geometry_msgs::Point32 tt;
  for (auto a : objs.obj)
  {
    float widthseg = a.width / std::ceil(a.width / 0.50);
    float lengthseg = a.length / std::ceil(a.length / 0.50);
    for (float x = a.x - a.length / 2.0; x <= a.x + a.length / 2.0; x += lengthseg)
    {
      for (float y = a.y - a.width / 2.0; y <= a.y + a.width / 2.0; y += widthseg)
      {
        tt.x = x;
        tt.y = y;
        rtnobs[tt.x].insert(tt);
      }
    }
  }
  return;
}

/**
  * @brief
  * @param
  */
int dwaplanner::reachgoal(std::vector<sVehicleStatus> road, sVehicleStatus goalvs)
{
  float dist = sqrt((road.back().x - goalvs.x) * (road.back().x - goalvs.x) + (road.back().y - goalvs.y) * (road.back().y - goalvs.y));
  float theta = todegree(fabs(road.back().heading - goalvs.heading));
  if (dist <= disttolerance && theta <= yawtolerance)
  {
    return 1;
  }
  return 0;
}

std::vector<sVehicleStatus>
dwaplanner::reconstruct_path(sVehicleStatus start, sVehicleStatus goal,
                             unordered_t<sVehicleStatus> &came_from, sVehicleStatus last)
{
  std::vector<sVehicleStatus> path;
  sVehicleStatus current = last;
  path.push_back(current);
  while (!(current == start))
  {
    current = came_from[current];
    path.push_back(current);
  }
  //   path.push_back(start);
  std::reverse(path.begin(), path.end());
  //   path.push_back(goal);
  return path;
}

functions::PriorityNode<cv::Point3f, float> dwaplanner::reEvaluation(std::vector<cv::Point3f> evaldb)
{
  std::multiset<functions::PriorityNode<cv::Point3f, float>> refset;
  for (int i = 0; i < evaldb.size(); ++i)
  {
    functions::PriorityNode<cv::Point3f, float> temp;
    //Store the index
    temp.node.first = evaldb.at(i);
    //Store the cost
    temp.node.second = evalinfo.multiply_coeff * (evalinfo.heading_coeff * evaldb.at(i).x + evalinfo.velocity_coeff * evaldb.at(i).z);
    refset.insert(temp);
  }
  return *refset.rbegin();
}

void dwaplanner::run()
{
#if SIMBLOCK
  GenerateTrajectory(initialstatus, dyn);
  std::vector<std::vector<geometry_msgs::Point32>> multipath;
  for (int i = 0; i < trajectorys.size(); ++i)
  {
    std::vector<geometry_msgs::Point32> tempv;
    for (int j = 0; j < trajectorys.at(i).size(); ++j)
    {
      geometry_msgs::Point32 temp;
      temp.x = trajectorys[i][j].x;
      temp.y = trajectorys[i][j].y;
      temp.z = trajectorys[i][j].heading;
      tempv.push_back(temp);
    }
    multipath.push_back(tempv);
  }
  visdwa_marker->publishmultipath(multipath, 0);
#else

  if (visdwa_pose->updatestate())
  {
    trajectory.clear();
    geometry_msgs::PoseStamped temps;
    geometry_msgs::PoseStamped tempg;
    visdwa_pose->setstart(temps);
    initialstatus.x = temps.pose.position.x;
    initialstatus.y = temps.pose.position.y;
    initialstatus.heading = tf::getYaw(temps.pose.orientation);
    updatingstatus = initialstatus;
    visdwa_pose->setgoal(tempg);
    goalstatus.x = tempg.pose.position.x;
    goalstatus.y = tempg.pose.position.y;
    goalstatus.heading = tf::getYaw(tempg.pose.orientation);
  }
#endif
  return;
}

void dwaplanner::test()
{
  // ROS_WARN_STREAM(functions::generateGaussianNoise(1.0, 0.2));

  std::vector<geometry_msgs::Point32> array;
  geometry_msgs::Point32 p1, p2;

  if (visdwa_pose->updatestate())
  {
    trajectory.clear();
    geometry_msgs::PoseStamped tempg;
    updatingstatus = initialstatus;
    visdwa_pose->setgoal(tempg);
    goalstatus.x = tempg.pose.position.x;
    goalstatus.y = tempg.pose.position.y;
    goalstatus.heading = tf::getYaw(tempg.pose.orientation);
    goalstatus.angular_velocity = 0.0;
    goalstatus.linear_velocity = 0.0;
  }
  p1.x = initialstatus.x;
  p1.y = initialstatus.y;
  p1.z = initialstatus.heading;
  p2.x = goalstatus.x;
  p2.y = goalstatus.y;
  p2.z = goalstatus.heading;

  globalpath(p1, p2, array);
  visdwa_marker->publishobstacle(array, 4, 0.05);

#if 1
  geometry_msgs::Point32 ego;
  geometry_msgs::Polygon poly;
  ego.x = goalstatus.x;
  ego.y = goalstatus.y;
  ego.z = goalstatus.heading;
  functions::carmodel(ego, poly);
  visdwa_polygon->publishpolygon(poly);
  // visdwa_marker->publishobstacle(mtest, 3, 0.2);

  std::vector<sVehicleStatus> rtnpath;
  sVehicleStatus lst;
  unordered_t<sVehicleStatus> came_from;
  unordered_t<float> cost_so_far;
  unordered_t<std::vector<sVehicleStatus>> pathtable;
  // lst = evolvedastar(initialstatus, goalstatus, came_from, cost_so_far, pathtable, sortedobs);
  lst = hybridastar(initialstatus, goalstatus, came_from, cost_so_far, pathtable, sortedobs);
  // ROS_WARN_STREAM("came_from.size(): " << came_from.size());
  rtnpath = reconstruct_path(initialstatus, goalstatus, came_from, lst);
  std::vector<geometry_msgs::Point32> singlepath;
  for (int i = 0; i < rtnpath.size(); ++i)
  {
    for (auto val : pathtable[rtnpath.at(i)])
    {
      geometry_msgs::Point32 temp;
      temp.x = val.x;
      temp.y = val.y;
      temp.z = val.heading;
      singlepath.push_back(temp);
    }
  }
  vishybrid_path->publishpath(singlepath);
#endif
}

inline float dwaplanner::todegree(float rad)
{
  if (rad - 2 * M_PI > 0.0)
  {
    rad -= 2 * M_PI;
  }
  else if (rad < 0.0)
  {
    rad += 2 * M_PI;
  }
  return rad * 180.0 / M_PI;
}

inline float dwaplanner::toradian(float deg)
{
  if (deg - 360.0 > 0.0)
  {
    deg -= 360.0;
  }
  else if (deg < 0.0)
  {
    deg += 360.0;
  }
  return deg * M_PI / 180.0;
}

// This algorithm is awesome, but the revised collision detection algorithm works better!!!
int dwaplanner::pruning(std::vector<sVehicleStatus> motion, functions::PtsTree<geometry_msgs::Point32>::ytree obspts)
{
  geometry_msgs::Point32 ego, p, q;
  geometry_msgs::Polygon poly;
  if (obspts.size() == 0)
  {
    return 0;
  }
  for (auto tt : motion)
  {
    ego.x = tt.x;
    ego.y = tt.y;
    ego.z = tt.heading;
    functions::PtsTree<geometry_msgs::Point32>::xtree obs;
    std::tie(p, q) = functions::carmodel(ego, poly);
    auto ha = obspts.lower_bound(p);
    //     std::cout<<*ha;
    if (!obspts.count(*ha) || (*ha).y > q.y)
    {
      continue;
    }
    else
    {
      for (; (*ha).y <= q.y && ha != obspts.end(); ++ha)
      {
        // std::cout<<*ha;
        obs.insert(*ha);
      }
    }
    auto hi = obs.lower_bound(p);

    if (!obspts.count(*hi) || (*hi).x > q.x)
    {
      continue;
    }
    else
    {
      //Here comes the final collision detection module!!!
      for (; (*hi).x <= q.x && hi != obs.end(); ++hi)
      {
        if (functions::IsInsideFootprint(*hi, poly))
        {
          return 1;
        }
      }
    }
  }
  return 0;
}

//Need to be modified!!!
ivpathplanner::path dwaplanner::pathinterface(ivpathplanner::path rd, ivmap::ivmapmsgobj objs)
{
  if (objs.obj.size() == 0)
  {
    return rd;
  }
  // ROS_ERROR_STREAM("rd.points.size(): "<<rd.points.size());
  int cc = 2 * rd.points.size() / 3;
  if (cc <= 10)
  {
    return rd;
  }
  goalstatus.x = rd.points.at(cc).x;
  goalstatus.y = rd.points.at(cc).y;
  goalstatus.heading = rd.points.at(cc).angle * M_PI / 180.0;
  goalstatus.linear_velocity = rd.points.at(cc).velocity;

  geometry_msgs::Point32 ego;
  geometry_msgs::Polygon poly;
  ego.x = goalstatus.x;
  ego.y = goalstatus.y;
  ego.z = goalstatus.heading;

  //Dispaly the car pose in the destination
  //Testing module
  functions::carmodel(ego, poly);
  visdwa_polygon->publishpolygon(poly);

  sorted_tree rtnobs;
  obstacleprocess(objs, rtnobs);

  std::vector<sVehicleStatus> rtnpath;
  sVehicleStatus lst;
  unordered_t<sVehicleStatus> came_from;
  unordered_t<float> cost_so_far;
  unordered_t<std::vector<sVehicleStatus>> pathtable;
  lst = hybridastar(initialstatus, goalstatus, came_from, cost_so_far, pathtable, rtnobs);
  rtnpath = reconstruct_path(initialstatus, goalstatus, came_from, lst);

  ivpathplanner::path ivpath;
  for (int i = 0; i < rtnpath.size(); ++i)
  {
    for (auto val : pathtable[rtnpath.at(i)])
    {
      ivpathplanner::pathpoint temp;
      temp.x = val.x;
      temp.y = val.y;
      temp.angle = val.heading * 180.0 / M_PI;
      // std::cout<<"temp.angle: "<<temp.angle<<std::endl;
      temp.velocity = val.linear_velocity;
      ivpath.points.push_back(temp);
    }
  }
  // Visulization testing module
  std::vector<geometry_msgs::Point32> singlepath;
  for (int i = 0; i < rtnpath.size(); ++i)
  {
    for (auto val : pathtable[rtnpath.at(i)])
    {
      geometry_msgs::Point32 temp;
      temp.x = val.x;
      temp.y = val.y;
      temp.z = val.heading;
      singlepath.push_back(temp);
    }
  }

  geometry_msgs::Point32 st, gt;
  std::vector<geometry_msgs::Point32> bc;
  auto lk = ivpath.points.back();
  st.x = lk.x;
  st.y = lk.y;
  st.z = lk.angle * M_PI / 180.0;
  gt.x = goalstatus.x;
  gt.y = goalstatus.y;
  gt.z = goalstatus.heading;

  bc = bsplinecurve(st, gt);
  //This for loop is size safe
  for (auto val : bc)
  {
    ivpathplanner::pathpoint temp;
    temp.x = val.x;
    temp.y = val.y;
    temp.angle = val.z * 180.0 / M_PI;
    // std::cout<<"temp.angle: "<<temp.angle<<std::endl;
    ivpath.points.push_back(temp);
  }
  singlepath.insert(singlepath.end(), bc.begin(), bc.end());

  vishybrid_path->publishpath(singlepath);

  return ivpath;
}

void dwaplanner::vehicleinfoinit(sVehicleElem *info)
{
  memset(info, 0, sizeof(*info));
  info->length = 45.15;
  info->width = 17.25;
  info->frontoverhang = 9.0;
  info->backoverhang = 9.0;
  info->wheelbase = 26.00;
  info->headwheel = 9.00;
  info->tailwheel = 10.15;
  info->innerangle = 39.5 * (M_PI / 180.0);
  info->outerangle = 33.5 * (M_PI / 180.0);
  info->outersafedis = 3.0;
  info->innersafedis = 3.0;
}
