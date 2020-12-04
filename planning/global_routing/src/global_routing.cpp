#include "global_routing.h"
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <chrono>

#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <autoware_lanelet2_msgs/MapBin.h>
/*蓝色是全局路径规划   ，绿色是动态规划*/
/*只要终点方向正确，就能生存路径，起点方向没关系*/
namespace global_routing
{ // namespace global_routing

  /*******************************
 * 1.调用构造函数，初始化参数与函数，发布初始化与订阅，执行copy_with_high_definition_map解析建立地图
 * 2.执行GlobalRouting::start_pose_call_back与GlobalRouting::goal_pose_call_back  定义起点终点位置
 * 3. 执行soultion(start_pose_, goal_pose_)   总的规划函数 ,包含执行以下函数
 *            3.1  get_shortest_path_according_to_pose(start_pose, goal_pose)  》getClosestLanelet(start_pose, map_,
 * &start_lanelet) 3.2  get_path_points() 3.3  average_interpolation() 优化轨迹 4.析构函数
 * ******************************/
  /*定义起点位置*/
  void GlobalRouting::start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) //  geometry_msgs/PoseWithCovariance表示带有不确定性的一个空间位姿
  {
    // std::cout<<"start_pose_call_back"<<std::endl;
    // std::cout << "x:  " << msg->pose.pose.position.x << std::endl;     //获取rivz上面的x点坐标
    // std::cout << "y:  " << msg->pose.pose.position.y << std::endl;     //获取rivz上面的x点坐标
    // std::cout << "o1:  " << msg->pose.pose.orientation.x << std::endl; //获取rivz上面的y点坐标
    // std::cout << "o2:  " << msg->pose.pose.orientation.y << std::endl; //获取rivz上面的y点坐标
    // std::cout << "o3:  " << msg->pose.pose.orientation.z << std::endl; //获取rivz上面的y点坐标
    // std::cout << "z:  " << msg->pose.pose.position.z << std::endl; //获取rivz上面的x点坐标
    // std::cout << "origin_utm_point_.x:  " << origin_utm_point_.x << std::endl;
    // std::cout << "origin_utm_point_.y:  " << origin_utm_point_.y << std::endl;
    start_pose_.position.x = msg->pose.pose.position.x;
    start_pose_.position.y = msg->pose.pose.position.y;
    start_pose_.orientation.x = msg->pose.pose.orientation.x;
    start_pose_.orientation.y = msg->pose.pose.orientation.y;
    start_pose_.orientation.z = msg->pose.pose.orientation.z;
    start_pose_.orientation.w = msg->pose.pose.orientation.w;
    start_pp.x = msg->pose.pose.position.x;
    start_pp.y = msg->pose.pose.position.y;
    is_start_pose_set = true;
    //判断起点方向是否正确
    if (is_direction_Valid(start_pose_))
    {
      is_vehical_start = true;
      publish_vehical_start_pose(start_pose_);
    }
    else
    {
      ROS_WARN("vehical start is not correct!");
      is_vehical_start = false;
    }
  }
  /*定义终点位置*/
  void GlobalRouting::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg) // geometry_msgs::PoseStamped包含坐标系和时间戳信息的空间位姿
  {
    // std::cout<<"goal_pose_call_back"<<std::endl;
    // std::cout << "x:  " << msg->pose.position.x << std::endl;     //获取rivz上面的x点坐标
    // std::cout << "y:  " << msg->pose.position.y << std::endl;     //获取rivz上面的y点坐标
    // std::cout << "o1:  " << msg->pose.orientation.x << std::endl; //获取rivz上面的y点坐标
    // std::cout << "o2:  " << msg->pose.orientation.y << std::endl; //获取rivz上面的y点坐标
    // std::cout << "o3:  " << msg->pose.orientation.z << std::endl; //获取rivz上面的y点坐标
    // std::cout << "z:  " << msg->pose.position.z << std::endl;           //获取rivz上面的y点坐标
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    goal_pose_.orientation.x = msg->pose.orientation.x;
    goal_pose_.orientation.y = msg->pose.orientation.y;
    goal_pose_.orientation.z = msg->pose.orientation.z;
    goal_pose_.orientation.w = msg->pose.orientation.w;
    stop_pp.x = msg->pose.position.x;
    stop_pp.y = msg->pose.position.y;
    is_goal_pose_set = true;
    //判断终点方向是否正确
    if (is_direction_Valid(goal_pose_))
    {
      is_vehical_stop = true;
    }
    else
    {
      ROS_WARN("vehical stop is not correct!");
      is_vehical_stop = false;
    }
    /*如果开始起点和终点已经设置，还没设置就为false*/
    if (is_start_pose_set == true && is_goal_pose_set == true)
    {
      soultion(start_pose_, goal_pose_); //获取最佳全局路径
      if (is_vehical_cango == true)
      {
        marker_pub_.publish(getTrajPathMarker()); //发布蓝色带路径
      }
      is_received_goal_ = true;
    }
  }
  /*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
  void GlobalRouting::odom_call_back(const nav_msgs::Odometry &odom)
  {
    // std::cout<<"void Globa#include <geometry_msgs/PoseStamped.h>
    sub_odom_ = odom;
    static double last_yaw;
    double this_yaw;
    this_yaw = tf::getYaw(odom.pose.pose.orientation);
    if ((this_yaw - last_yaw) < -M_PI)
    {
      this_yaw += M_PI * 2.0;
    }
    else if ((this_yaw - last_yaw) > M_PI)
    {
      this_yaw -= M_PI * 2.0;
    }
    car_state.yaw = this_yaw;
    last_yaw = this_yaw;
  }

  /*默认构造函数：规划函数,初始化参数*/
  GlobalRouting::GlobalRouting(void)
  {
    ros::NodeHandle private_nh; // NodeHandle:创建自己的命名空间,为这个进程的节点创建一个句柄。
    ros::NodeHandle gr_nh;      // NodeHandle:创建自己的命名空间,为这个进程的节点创建一个句柄。
    // subscribe<为订阅的消息类型>(主题, 队列大小, 回调函数地址,
    // 为回调函数所处的类),使用类内函数时，将第四个参数替换成this：

    //回调函数
    odom_sub_ = gr_nh.subscribe("odom", 10, &GlobalRouting::odom_call_back, this);                                    //订阅vehicle_control.cpp的里程计
    start_pose_subscriber_ = gr_nh.subscribe("/initialpose", 10, &GlobalRouting::start_pose_call_back, this);         //订阅
    goal_pose_subscriber_ = gr_nh.subscribe("/move_base_simple/goal", 10, &GlobalRouting::goal_pose_call_back, this); //订阅
    // 实例化，路点发布初始化
    waypoints_pub_ = private_nh.advertise<nav_msgs::Path>("waypoints", 10);                                //路径点发布第一种方式
    waypoints_vis_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);              //路径点发布第二种方式
    marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("Marker", 10);                          //车轨发布
    lanelets_id_pub_ = private_nh.advertise<planning_msgs::lanelets_id>("lanelets_id", 10);                //车道的id
    control_data_pub_ = private_nh.advertise<geometry_msgs::Vector3>("control_data", 10);                  //控制数据
    vehicle_start_pose_pub_ = private_nh.advertise<planning_msgs::vehicle_pose>("vehicle_start_pose", 10); //发布车的起点位置

    // 参数获取
    ros::param::get("hdmap_file_path", this->file_path_);           // ros::param::get()获取参数“hdmap_file_path”的value，写入到file_path_上
    ros::param::get("origin_pos_lat", this->origin_gps_point_.lat); // ros::param::get()获取参数“origin_pos_lat”的value，写入到origin_gps_point_.lat上
    ros::param::get("origin_pos_lon", this->origin_gps_point_.lon); // ros::param::get()获取参数“origin_pos_lon”的value，写入到origin_gps_point_.lon上
    // std::cout<<"path: "<< this->file_path_ <<std::endl;
    // std::cout<<"lat: "<< this->origin_gps_point_.lat <<std::endl;
    // std::cout<<"lon: "<< this->origin_gps_point_.lon <<std::endl;

    //起始点与终点未设置
    is_start_pose_set = false;
    is_goal_pose_set = false;
    is_vehical_cango = false; //判断车是否可行，不能逆向行使，初始化不可行
    //车的初始值设置
    this->VEHICLE_VEL = 1; //车速度
    this->STANLEY_K = 0.3; // stanly算法的K值
    /*解析高精地图*/
    // 地图路径
    copy_with_high_definition_map(file_path_, origin_gps_point_); //
    // visual可视化
    traj_path_marker_.header.frame_id = "/base_link"; // frame_id设置为/base_link
    traj_path_marker_.ns = "lines";                   //命名空间（ns）和id是用来给这个marker创建一个唯一的名字的
    traj_path_marker_.id = 0;                         //命名空间（ns）和id是用来给这个marker创建一个唯一的名字的
    traj_path_marker_.header.stamp = ros::Time::now();
    //这个就是指定我们会发送哪种形状过去。如果不确定，我们就把它设定成shape，这里我们设置成线条，即LINE_STRIP
    traj_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    //这个是用来指定我们要对marker做什么的，有ADD和DELETE两个选项，ADD其实是创建或者修改的意思
    traj_path_marker_.action = visualization_msgs::Marker::ADD;
    //指定了标记的规模，对于基本形状，1表示在这边长度是1米。
    traj_path_marker_.scale.x = 4;
    traj_path_marker_.scale.y = 1;
    traj_path_marker_.scale.z = 1;
    traj_path_marker_.pose.position.x = 0;
    traj_path_marker_.pose.position.y = 0;
    traj_path_marker_.pose.position.z = 0;
    traj_path_marker_.pose.orientation.x = 0.0;
    traj_path_marker_.pose.orientation.y = 0.0;
    traj_path_marker_.pose.orientation.z = 0.0;
    traj_path_marker_.pose.orientation.w = 1.0; //设置了marker的姿态，这里我们把方向设置为身份方向（w=0）
    // marker的颜色就像指定 std_msgs/ColorRGBA。每个数字介于0-1之间。最后一个a(alpha)表示透明度，0表示完全透明。
    traj_path_marker_.color.r = 0.0;
    traj_path_marker_.color.g = 0.0;
    traj_path_marker_.color.b = 1.0; //蓝色显示
    traj_path_marker_.color.a = 0.5;
    traj_path_marker_.lifetime = ros::Duration();
    //指定marker在被自动清除前可以逗留多长时间。这里
    // ros::Duration()表示不自动删除。如果在lifetime结束之前有新内容到达了，它就会被重置。
    // traj_path_marker_.lifetime =ros::Duration();

    // 轨迹多项式参数求解
    // 输入：路径点 和 时间分配
    polynomial_solver_ = new MinimumJerk; // MinimumJerk是用于轨迹规划的类，求解多项式曲线

    // 标志位：
    is_received_goal_ = false;    // is_received_goal_：是否选择了终点
    is_update_next_path_ = false; // is_update_next_path_: 是否完成路径的生成
    is_vehical_start_set = false;
    is_begin_goal_ = false;
    /*创建两个线程：
 1.GlobalRouting::thread_generate_trajectory
 2.GlobalRouting::thread_routing
 */
    generate_trj_thread_ = new boost::thread(boost::bind(&GlobalRouting::thread_generate_trajectory, this)); //用来生成轨迹
    routing_thread_ = new boost::thread(boost::bind(&GlobalRouting::thread_routing, this));                  //用来路径规划
  }

  /*析构函数：释放线程*/
  GlobalRouting::~GlobalRouting(void)
  {
    generate_trj_thread_->interrupt();
    generate_trj_thread_->join();
    delete generate_trj_thread_;
    routing_thread_->interrupt();
    routing_thread_->join();
    delete generate_trj_thread_;
  }

  /*以下是调用的功能函数*/

  //复制高精地图，就是加载出里面的数据
  //加载真实地图的时候需要考虑坐标系转换，一般是地理系与平面系的转换，Lanelet中推荐转换的地理系是UTM系。
  void GlobalRouting::copy_with_high_definition_map(std::string &file_path, lanelet::GPSPoint &origin_gps_point)
  {
    // Projection：提供全球地理坐标系到局部平面坐标系的准换
    lanelet::projection::MGRSProjector projector; // //地图原点latlon
    map_ = load(file_path, projector);            //加载地图
    // 建立规则
    // TrafficRules：根据不同的road user类型和国家，来解释相应的交通规则。
    trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);       //示例的交规在德国
    pedestrainRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian); // 行人交规

    // 建立路线图
    // Routing：根据交通规则，创建路由图，来确定精准的行驶路线。
    routingGraph_ = routing::RoutingGraph::build(*map_, *trafficRules);

    //存储lanelet的id到map_lanelet_id_
    LaneletLayer &get_lanelets = map_->laneletLayer; //引用
    for (LaneletLayer::iterator it = get_lanelets.begin(); it != get_lanelets.end(); it++)
    {
      // std::cout << (*it).id() << std::endl;
      map_lanelet_id_.id.push_back((*it).id());
    }
    lanelets_id_pub_.publish(getMapLaneletId()); //发布高精地图的Lanelet的车道id
  }

  // 目前使用的规划函数，传入起点终点，输出一条最优路径
  void GlobalRouting::soultion(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose)
  {
    // 在高精地图上搜索最优路线
    if (!get_shortest_path_according_to_pose(start_pose, goal_pose)) //调用get_shortest_path_according_to_pose函数，判断起点终点是否在车道上，是的话继续
    {
      return;
    }
    if (is_vehical_start == true && is_vehical_stop == true)
    {
      is_vehical_cango = true;
    }
    // 获取路线点
    get_path_points();
    back_vehical(shortestPath_, start_pose, goal_pose);
    // 优化路线点
    // Eigen::MatrixXd hdmap_way_points,hdmap_way_points已经从   get_path_points()中定义值
    traj_opti_.average_interpolation(hdmap_way_points, path_point_after_interpolation_, 2.0, 5); //  traj_opti_.average_interpolation：加权平均优化，输出path_point_after_interpolation_
    traj_path_array_after_interpolation_.poses.clear();                                          //先清空
    traj_path_array_after_interpolation_.header.frame_id = "/base_link";
    traj_path_array_after_interpolation_.header.stamp = ros::Time::now();
    for (int i = 0; i < path_point_after_interpolation_.rows(); i++) //从行开始遍历
    {
      geometry_msgs::Pose point_ppose; //定义临时变量，获取x,y,z
      point_ppose.position.x = path_point_after_interpolation_(i, 0);
      point_ppose.position.y = path_point_after_interpolation_(i, 1);
      point_ppose.position.z = path_point_after_interpolation_(i, 2);
      /*根据两点计算tan:-180~180，用来判断方向，朝向*/
      double angle;                                          //定义角度
      if (i == (path_point_after_interpolation_.rows() - 1)) //如果是末点
      {
        angle = atan2(path_point_after_interpolation_(i, 1) - path_point_after_interpolation_(i - 1, 1),
                      path_point_after_interpolation_(i, 0) - path_point_after_interpolation_(i - 1, 0));
      }
      else //如果不是末点
      {
        angle = atan2(path_point_after_interpolation_(i + 1, 1) - path_point_after_interpolation_(i, 1),
                      path_point_after_interpolation_(i + 1, 0) - path_point_after_interpolation_(i, 0));
      }
      point_ppose.orientation = tf::createQuaternionMsgFromYaw(angle); ////只通过绕z的旋转角度计算四元数，用于平面小车。返回四元数
      traj_path_array_after_interpolation_.poses.push_back(point_ppose);
    }
    // for (int i = 0; i < traj_path_array_after_interpolation_.poses.size(); i++)
    // {
    //   std::cout << " traj_path_array_after_interpolation_:"
    //             << "(" << traj_path_array_after_interpolation_.poses[i].position.x << "," <<
    //             traj_path_array_after_interpolation_.poses[i].position.y << ")" << std::endl;
    // }
  }

  //判断定义的终点和起点最近的车道是否存在
  bool GlobalRouting::get_shortest_path_according_to_pose(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose)
  {
    bool has_obtained_start_lanelet = getClosestLanelet(start_pose, map_, &start_lanelet);
    bool has_obtained_goal_lanelet = getClosestLanelet(goal_pose, map_, &goal_lanelet);
    // 判断定义的终点和起点最近的车道是否存在
    if (has_obtained_start_lanelet == false || has_obtained_goal_lanelet == false)
    {
      std::cout << "no closest lanelet !!!" << std::endl;
      return false;
    }
    //如果以上车道存在，则去获取最短路径，调用lanelet2内置的路径规划，路径是车道，蓝色部分
    /*
    RoutingGraph::getRoute(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId, bool withLaneChanges)
    routingCostId路由成本模块的ID，用于确定最短路径
    withLaneChanges如果为false，则最短路径将不包含车道更改，
    并且路线将仅包含在不更改车道的情况下可到达的车道
    */
    Optional<routing::Route> route = routingGraph_->getRoute(start_lanelet, goal_lanelet, 0, true);
    if (!route)
    {
      std::cout << "no path" << std::endl;
      return false;
    }
    shortestPath_ = route->shortestPath();

    // const auto left_relations = routingGraph_->leftRelations(start_lanelet);
    // std::string zz = lanelet::routing::relationToString(lanelet::routing::RelationType::Right);
    // std::cout << "zz:" << zz << std::endl;
    // std::string zzz = lanelet::routing::relationToString(left_relations[0].relationType);
    // std::cout << "zzz:" << zzz << std::endl;

    routingGraph_->checkValidity(); //自检：该机制可验证图的所有部分是否处于健全状态。
    return true;
  }

  //判断车的行使方向
  bool GlobalRouting::isInLanes(const lanelet::Lanelet &lanelet, const lanelet::ConstPoint3d &point)
  {
    //检查目标是否在车道上以适当的角度
    const auto distance = boost::geometry::distance(lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    return distance < th_distance;
  }
  constexpr double GlobalRouting::normalizeRadian(const double rad, const double min_rad, const double max_rad)
  {
    const auto value = std::fmod(rad, 2 * M_PI);
    if (min_rad < value && value <= max_rad)
      return value;
    else
      return value - std::copysign(2 * M_PI, value);
  }
  bool GlobalRouting::is_direction_Valid(geometry_msgs::Pose &pose)
  {
    lanelet::Lanelet close_lanelet;
    bool has_obtained_lanelet = getClosestLanelet(pose, map_, &close_lanelet);
    const auto lanelet_pt = lanelet::utils::conversion::toLaneletPoint(pose.position);
    if (has_obtained_lanelet == false)
    {
      return false;
    }
    //目标点
    if (isInLanes(close_lanelet, lanelet_pt))
    {
      const auto lane_yaw = lanelet::utils::getLaneletAngle(close_lanelet, pose.position); //lane_yaw
      const auto set_yaw = tf2::getYaw(pose.orientation);
      const auto angle_diff = normalizeRadian(lane_yaw - set_yaw, -M_PI, M_PI);
      // std::cout << "goal_xaw: " << goal_pose.orientation.x << std::endl;
      // std::cout << "goal_yaw: " << goal_pose.orientation.y << std::endl;
      // std::cout << "goal_zaw: " << goal_pose.orientation.z << std::endl;
      // std::cout << "goal_waw: " << goal_pose.orientation.w << std::endl;
      // std::cout << "goal_w: " << goal_yaw << std::endl;
      constexpr double th_angle = M_PI / 4; //45度
      if (std::abs(angle_diff) < th_angle)
      {
        return true;
      }
    }
    // // 获取起点左右边界
    // // Optional<lanelet::ConstLanelet> adjacentLefts = routingGraph_->adjacentLeft(start_lanelet);
    // lanelet::LineString3d lefts = start_lanelet.leftBound();
    // // std::cout << "lefts.id(): " << lefts.id() << std::endl;
    // lanelet::LineString3d rights = start_lanelet.rightBound();
    // //std::cout << "rights.id(): " << rights.id() << std::endl;
    // for (Point3d &p : lefts)
    // {
    //   //遍历边界的point点
    //   //std::cout << "左边: " << p << std::endl;
    // }
    // for (Point3d &p : rights)
    // {
    //   //遍历边界的point点
    //   //  std::cout << "右边: " << p << std::endl;
    // }
  }

  //获得路径点
  void GlobalRouting::get_path_points()
  {
    //输出全局路径
    generate_global_routing(shortestPath_, start_pose_, goal_pose_);
  }

  //判断是否通过Lanelet获得了最近车道lane，传入：位姿坐标， lanelet地图，最近车道，距离阈值
  bool GlobalRouting::getClosestLanelet(const geometry_msgs::Pose &search_pose, const lanelet::LaneletMapPtr &lanelet_map_ptr_, lanelet::Lanelet *closest_lanelet, double distance_thresh)
  {
    lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);
    std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet = lanelet::geometry::findNearest(
        lanelet_map_ptr_->laneletLayer, search_point, 1); // lanelet::geometry::findNearest，
    if (nearest_lanelet.empty())
    {
      std::cout << "Failed to find the closest lane!" << std::endl;
      return false;
    }
    // nearest_lanelet.front().first存储的是距离
    // nearest_lanelet.front().second存储的是车道
    if (nearest_lanelet.front().first > distance_thresh)
    {
      std::cout << "Closest lane is too far away!" << std::endl;
      // std::cout<< "search point: " << toString(search_pose) << std::endl
      // std::cout<< "lane id: " << nearest_lanelet.front().second.id();
      return false;
    }
    *closest_lanelet = nearest_lanelet.front().second;
    // std::cout << "lane id: " << nearest_lanelet.front().second.id();
    // std::cout << nearest_lanelet.front().second << std::endl;
    return true;
  }
  /*
这些函数是接口函数
一般类成员变量定义为 私有的， 外部不能直接获取到 私有成员变量，就通过接口函数来获取
*/
  geometry_msgs::PoseArray &GlobalRouting::getTrajPathArray(void)
  {
    return traj_path_array_;
  }
  nav_msgs::Path &GlobalRouting::getTrajPath(void)
  {
    return traj_path_;
  }
  nav_msgs::Path &GlobalRouting::getPath(void)
  {
    return path_points_;
  }
  geometry_msgs::PoseArray &GlobalRouting::getStorePathArray(void)
  {
    return all_path_array_;
  }
  geometry_msgs::PoseArray &GlobalRouting::getStorePathArrayAfterInterpolation(void)
  {
    return traj_path_array_after_interpolation_;
  }
  visualization_msgs::Marker &GlobalRouting::getTrajPathMarker(void)
  {
    return traj_path_marker_;
  }
  planning_msgs::lanelets_id &GlobalRouting::getMapLaneletId(void)
  {
    return map_lanelet_id_;
  }
  Eigen::MatrixXd &GlobalRouting::getMatrixPathPointafterInterpolation(void)
  {
    return path_point_after_interpolation;
  }
  Eigen::MatrixXd &GlobalRouting::getMatrixPathPointAfterInterpolation(void)
  {
    return path_point_after_interpolation_;
  }
  Eigen::MatrixXd &GlobalRouting::gethdmap_way_points(void)
  {
    return hdmap_way_points;
  }

  //找到下一个最近路经点,用于车跟踪轨迹
  int GlobalRouting::find_nearest_point_index(const geometry_msgs::PoseArray &pose_array, Point3d_s &pose)
  {
    int index = 0;
    double min_val = 1000000;
    for (int i = 0; i < pose_array.poses.size(); i++)
    {
      double val = abs(pose_array.poses[i].position.x - pose.x) + abs(pose_array.poses[i].position.y - pose.y) +
                   abs(pose_array.poses[i].position.z - pose.z);
      if (val < min_val)
      {
        min_val = val; //更新
        index = i;
      }
    }
    //返回最近点的下标位置
    return index;
  }
  //找到下一个最近路经点，用于全局路径的切割
  int GlobalRouting::Find_nearest_point(const geometry_msgs::PoseArray &pose_array, Point3d_s &pose)
  {
    int index = 0;
    double min_val = 1000000;
    double x2 = pose.x;
    double y2 = pose.y;
    for (int i = 0; i < pose_array.poses.size(); i++)
    {
      double x1 = pose_array.poses[i].position.x;
      double y1 = pose_array.poses[i].position.y;
      double val = sqrt((fabs(x1 - x2)) * (fabs(x1 - x2)) + (fabs(y1 - y2)) * (fabs(y1 - y2)));
      if (val < min_val)
      {
        min_val = val; //更新
        index = i;
      }
    }
    //返回最近点的下标位置
    return index;
  }
  /*主要是用来控制车开的角度
  传入的参数：
 * trj_point_array：路径点，odom：里程计信息源，包含速度信息和坐标，tar_index：目标index，out_turn_agl：输出的转向,
 * out_index：输出的index
 * */
  void GlobalRouting::control_vehicle(const geometry_msgs::PoseArray &trj_point_array, nav_msgs::Odometry &odom, int tar_index, double &out_turn_agl, int &out_index)
  {
    double e_lat;
    double xt = 0;
    double yt = 0;
    double zt = 0;
    Point3d_s p;
    p.x = odom.pose.pose.position.x;
    p.y = odom.pose.pose.position.y;
    p.z = odom.pose.pose.position.z;
    int index = find_nearest_point_index(trj_point_array, p); //调用find_nearest_point_index()函数，找下一个路径点
    //  获取车的YAW
    geometry_msgs::Quaternion qqq = trj_point_array.poses[index].orientation;
    double point_yaw = tf::getYaw(qqq);
    double car_yaw = tf::getYaw(odom.pose.pose.orientation);
    // std::cout<<"point_yaw: "<< point_yaw <<std::endl;
    // std::cout << "car_yaw: " << car_yaw << std::endl;

    //四元数转欧拉角
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    // double roll, pitch, car_yaw;//定义存储r\p\y的容器
    // tf::Matrix3x3(quat).getRPY(roll, pitch, car_yaw);//进行转换
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;
    xt = trj_point_array.poses[index].position.x;
    yt = trj_point_array.poses[index].position.y;
    zt = trj_point_array.poses[index].position.z;
    // std::cout<<"odom  "<<"x: "<<x<<"  y: "<<y<<"  z: "<<z<<std::endl;
    // std::cout<<"trj  "<<"x: "<<xt<<"  y: "<<yt<<"  z: "<<zt<<std::endl;

    // 计算横向误差,类似frenet
    double fx = (yt - y) * cos(point_yaw) - (xt - x) * sin(point_yaw); //横向误差
    if (fx > 0)
    {
      e_lat = sqrt((xt - x) * (xt - x) + (yt - y) * (yt - y) + (zt - z) * (zt - z));
    }
    else
    {
      e_lat = -sqrt((xt - x) * (xt - x) + (yt - y) * (yt - y) + (zt - z) * (zt - z));
    }
    // stanly算法的实现
    double test_val = atan2(STANLEY_K * e_lat, VEHICLE_VEL);
    // std::cout<<"e_lat"<<e_lat<<std::endl;
    // std::cout<<"test_val"<<test_val<<std::endl;
    //使得角度在-180到180之间
    double yaw_diff = point_yaw - car_yaw; //航向偏差
    if (yaw_diff > M_PI)
    {
      yaw_diff = -2.0 * M_PI + yaw_diff;
    }
    else if (yaw_diff < -M_PI)
    {
      yaw_diff = 2.0 * M_PI + yaw_diff;
    }
    // std::cout<<"yaw_diff  "<<yaw_diff<<std::endl;
    //转向角控制率
    double turn_angle;
    turn_angle = (yaw_diff + atan2(STANLEY_K * e_lat, VEHICLE_VEL));
    // 限制车轮转角 [-60, 60]
    if (turn_angle > M_PI / 2.0)
    {
      turn_angle = M_PI / 2.0;
    }
    else if (turn_angle < -M_PI / 2.0)
    {
      turn_angle = -M_PI / 2.0;
    }

    //输出
    out_turn_agl = turn_angle;
    out_index = index;
  }

  // 发布车辆在路径的起始位置,传入起点
  void GlobalRouting::publish_vehical_start_pose(const geometry_msgs::Pose &start_pose)
  {
    is_vehical_start_set = false;
    // get 起始位置的yaw
    double set_vehicle_yaw;
    set_vehicle_yaw = tf2::getYaw(start_pose.orientation);
    // get vehicle start pose
    vehicle_start_pose_.x = start_pose.position.x;
    vehicle_start_pose_.y = start_pose.position.y;
    vehicle_start_pose_.yaw = set_vehicle_yaw;
    //发布车的起点位置
    vehicle_start_pose_pub_.publish(vehicle_start_pose_);
    is_vehical_start_set = true;
  }
  //判断车是否到了路径的尽头
  bool GlobalRouting::vehical_stop(const geometry_msgs::PoseArray &trj_point_array, nav_msgs::Odometry &odom)
  {
    //获取车的里程位置
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;
    //获取路径点的最后一个位置
    double xt = 0;
    double yt = 0;
    double zt = 0;
    xt = trj_point_array.poses[trj_point_array.poses.size() - 1].position.x;
    yt = trj_point_array.poses[trj_point_array.poses.size() - 1].position.y;
    zt = trj_point_array.poses[trj_point_array.poses.size() - 1].position.z;
    //判断如果两点坐标接近
    if ((abs(x - xt) <= 2) && (abs(y - yt) <= 2) && (abs(z - zt) <= 2))
    {
      return true;
    }
    return false;
  }

  //输出全局规划的路线：点到点，蓝色带：Marker输出。
  void GlobalRouting::generate_global_routing(routing::LaneletPath &shortestPath, geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose)
  {
    // 1.初始化清零
    start_path_array_.poses.clear();
    start_path_array_.header.frame_id = "/base_link";
    start_path_array_.header.stamp = ros::Time::now();
    stop_path_array_.poses.clear();
    stop_path_array_.header.frame_id = "/base_link";
    stop_path_array_.header.stamp = ros::Time::now();
    after_start_path_array_.poses.clear();
    after_start_path_array_.header.frame_id = "/base_link";
    after_start_path_array_.header.stamp = ros::Time::now();
    after_stop_path_array_.poses.clear();
    after_stop_path_array_.header.frame_id = "/base_link";
    after_stop_path_array_.header.stamp = ros::Time::now();
    all_path_array_.poses.clear();
    all_path_array_.header.frame_id = "/base_link";
    all_path_array_.header.stamp = ros::Time::now();
    stores_path_array_.poses.clear();
    stores_path_array_.header.frame_id = "/base_link";
    stores_path_array_.header.stamp = ros::Time::now();
    after_start_path_array.poses.clear();
    after_start_path_array.header.frame_id = "/base_link";
    after_start_path_array.header.stamp = ros::Time::now();
    after_stop_path_array.poses.clear();
    after_stop_path_array.header.frame_id = "/base_link";
    after_stop_path_array.header.stamp = ros::Time::now();
    // 2.获取起点和终点所在的车段id
    get_start_path = shortestPath[0];
    get_stop_path = shortestPath[shortestPath.size() - 1];
    // 3.遍历起点和终点对应车段的centerline的路径点，并分别存储于start_path_array_与stop_path_array_

    lanelet::ConstLineString3d start_center_line;
    lanelet::ConstLineString3d stop_center_line;
    start_center_line = get_start_path.centerline();
    stop_center_line = get_stop_path.centerline();
    for (int ci = 0; ci < start_center_line.size(); ci++)
    {
      point_posess.position.x = start_center_line[ci].x() - origin_utm_point_.x;
      point_posess.position.y = start_center_line[ci].y() - origin_utm_point_.y;
      point_posess.position.z = start_center_line[ci].z() - origin_utm_point_.z;
      start_path_array_.poses.push_back(point_posess);
    }
    for (int ci = 0; ci < stop_center_line.size(); ci++)
    {
      point_posess.position.x = stop_center_line[ci].x() - origin_utm_point_.x;
      point_posess.position.y = stop_center_line[ci].y() - origin_utm_point_.y;
      point_posess.position.z = stop_center_line[ci].z() - origin_utm_point_.z;
      stop_path_array_.poses.push_back(point_posess);
    }
    // 4. 找出离起点和终点最近的点  st_pp，sp_pp，
    int index1 = 10000;
    int index2 = 10000;
    index1 = Find_nearest_point(start_path_array_, start_pp);
    index2 = Find_nearest_point(stop_path_array_, stop_pp);
    // std::cout << "index2 " << index2 << std::endl;
    st_pp.x = start_path_array_.poses[index1].position.x;
    st_pp.y = start_path_array_.poses[index1].position.y;
    st_pp.z = start_path_array_.poses[index1].position.z;
    sp_pp.x = stop_path_array_.poses[index2].position.x;
    sp_pp.y = stop_path_array_.poses[index2].position.y;
    sp_pp.z = stop_path_array_.poses[index2].position.z;
    // 5.如果车段大于2条，则获取其他的车段路径点,存储于stores_path_array_。
    if (shortestPath_.size() > 2)
    {
      // 获取中心线
      ConstLanelet a_Lanelet1;
      lanelet::ConstLineString3d center_ls1;
      for (int pi = 1; pi < shortestPath_.size() - 1; pi++)
      {
        a_Lanelet1 = shortestPath_[pi];
        center_ls1 = a_Lanelet1.centerline();
        for (int ci = 0; ci < center_ls1.size(); ci++)
        {
          point_posess.position.x = center_ls1[ci].x() - origin_utm_point_.x;
          point_posess.position.y = center_ls1[ci].y() - origin_utm_point_.y;
          point_posess.position.z = center_ls1[ci].z() - origin_utm_point_.z;
          stores_path_array_.poses.push_back(point_posess);
        }
      }
    }
    // std::cout << " shortestPath_.size():" << shortestPath_.size() << std::endl;
    // 6.获取最近点的前一个点和后一个点，并进行切割。输出after_start_path_array_是切割后的
    if (shortestPath_.size() == 0)
    {
      return;
    }
    else if (shortestPath_.size() == 1) //注意了，这种情况有可能要考虑倒车
    {
      traj_opti_.trajectory_start_cutting(start_path_array_, after_start_path_array, index1, st_pp, start_pp);
      traj_opti_.trajectory_stop_cutting(stop_path_array_, after_stop_path_array, index2, sp_pp, stop_pp);
      traj_opti_.trajectory_same_cutting(start_path_array_, after_start_path_array, after_stop_path_array,
                                         after_start_path_array_, index1, st_pp, start_pp, index2, sp_pp, stop_pp);
    }
    else if (shortestPath_.size() > 1)
    {
      traj_opti_.trajectory_start_cutting(start_path_array_, after_start_path_array_, index1, st_pp, start_pp);
      traj_opti_.trajectory_stop_cutting(stop_path_array_, after_stop_path_array_, index2, sp_pp, stop_pp);
    }
    // 7.获取全部路径
    //如果只有一条，则只放起点的
    for (int i = 0; i < after_start_path_array_.poses.size(); i++)
    {
      point_posess.position.x = after_start_path_array_.poses[i].position.x;
      point_posess.position.y = after_start_path_array_.poses[i].position.y;
      point_posess.position.z = after_start_path_array_.poses[i].position.z;
      all_path_array_.poses.push_back(point_posess);
    }
    //如果大于两条，则把终点的轨迹也放进去
    if (shortestPath_.size() > 2)
    {
      for (int i = 0; i < stores_path_array_.poses.size(); i++)
      {
        point_posess.position.x = stores_path_array_.poses[i].position.x;
        point_posess.position.y = stores_path_array_.poses[i].position.y;
        point_posess.position.z = stores_path_array_.poses[i].position.z;
        all_path_array_.poses.push_back(point_posess);
      }
    }
    //如果有两条，则把终点的轨迹也放进去
    if (shortestPath_.size() > 1)
    {
      for (int i = 0; i < after_stop_path_array_.poses.size(); i++)
      {
        point_posess.position.x = after_stop_path_array_.poses[i].position.x;
        point_posess.position.y = after_stop_path_array_.poses[i].position.y;
        point_posess.position.z = after_stop_path_array_.poses[i].position.z;
        all_path_array_.poses.push_back(point_posess);
      }
    }
    // ROS 数据 转换成 矩阵数据
    hdmap_way_points = Eigen::MatrixXd::Zero(all_path_array_.poses.size(), 3); //初始化零矩阵
    for (int i = 0; i < all_path_array_.poses.size(); i++)
    {
      hdmap_way_points(i, 0) = all_path_array_.poses[i].position.x;
      hdmap_way_points(i, 1) = all_path_array_.poses[i].position.y;
      hdmap_way_points(i, 2) = all_path_array_.poses[i].position.z;
    }
    //获取点
    traj_path_marker_.points.clear();                 //先清空
    for (int i = 0; i < hdmap_way_points.rows(); i++) //从行开始遍历
    {
      geometry_msgs::Point p;
      p.x = hdmap_way_points(i, 0);
      p.y = hdmap_way_points(i, 1);
      p.z = hdmap_way_points(i, 2);
      //发布车道轨迹，轨迹存放于traj_path_marker_.points，rviz看到的是蓝色
      traj_path_marker_.points.push_back(p);
    }
  }

  //倒车或掉头到达目的地判断,弯道不能倒车
  void GlobalRouting::back_vehical(routing::LaneletPath &shortestPath, geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose)
  {
    ////获取最短路径的车道的所有lanelet
    // lanelet::ConstLanelets path_lanelets;
    // for (const auto &llt : shortestPath)
    // {
    //   path_lanelets.push_back(llt);
    // }
    ////判度lanelet是否具有某种属性
    // for (auto lli = path_lanelets.begin(); lli != path_lanelets.end(); lli++)
    // {
    //   lanelet::ConstLanelet ll = *lli;
    //   if (!ll.hasAttribute(std::string("turn_direction")))
    //   {
    //   }
    // }
    ////获取地图道路类型
    // lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(map_);
    // lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);//车行道
    // lanelet::ConstLanelets crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);//人行横道
    //lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);//人行通道
    // //输出车道的属性，类型要是 lanelet::Lanelet，不能是 lanelet::ConstLanelet
    // std::cout << "attributes: " << start_lanelet.attributes()[AttributeName::Subtype] << std::endl;

    // 判断起点和终点是否在统一直线道路
    // 判断起点和终点是否比较近
    //判断是否有属性turn_direction，没有的话说明是直线
    //获取起点和终点的tan，是否和道方向相反
    double angle = std::atan2(goal_pose.position.y - start_pose.position.y, goal_pose.position.x - start_pose.position.x);
    // std::cout << "angle: " << angle << std::endl;
    const auto lane_yaw = lanelet::utils::getLaneletAngle(start_lanelet, start_pose.position);
    const auto angle_diff = normalizeRadian(lane_yaw - angle, -M_PI, M_PI);
    constexpr double th_angle = M_PI / 2; //90度
                                          // std::cout << "lane_yaw: " << lane_yaw << std::endl;
                                          // std::cout << "angle_diff: " << std::abs(angle_diff) << std::endl;
    std::cout << "start_lanelet.id: " << start_lanelet.id() << std::endl;
    if (start_lanelet.id() == goal_lanelet.id())
    {
      double distance = sqrt(pow((goal_pose.position.y - start_pose.position.y), 2) + pow((goal_pose.position.x - start_pose.position.x), 2));
      if (distance <= 30)
      {
        if ((!start_lanelet.hasAttribute(std::string("turn_direction"))) && (!goal_lanelet.hasAttribute(std::string("turn_direction"))))
        {
          if (std::abs(angle_diff) > th_angle)
          {
            std::cout << "可倒车" << std::endl;
          }
        }
      }
    }
  }

  /*两个线程函数跑循环函数*/
  //输出轨迹
  void GlobalRouting::thread_generate_trajectory(void)
  {
    //线程加锁
    /*
    boost::unique_lock<boost::recursive_mutex> lock(routing_mutex_);
    。。。
    ros::spinOnce();
    lock.unlock();
    */
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    int POINTS_NUM; //存储   每段的点数量
    double vel = 2; //时间分配的速度
    double acc = 1; //时间分配的加速度
    int next_path_index = 0;
    while (n.ok())
    {
      //  如果接受到目标点&&如果每段路经点大于等于POINTS_NUM*2,is_received_goal_ == true 是执行完solution后才满足,is_vehical_cango == true是道路可行
      if (is_received_goal_ == true && is_vehical_cango == true && 1)
      {
        if ((path_point_after_interpolation_.rows() > 20) &&
            is_begin_goal_ == false) //如果轨迹点大于20个，每一段10个点，最后一段少于10个点的时候另作处理
        {
          is_update_next_path_ = false;
          POINTS_NUM = 20;
          Eigen::MatrixXd sub_path = Eigen::MatrixXd::Zero(POINTS_NUM, 3); //零矩阵
          Eigen::MatrixXd time_allo;                                       //时间矩阵
          // next_path_index += POINTS_NUM - 1，-1是为了保持连续性,以上一次取的子矩阵的最后一个为新的起点
          for (next_path_index = 0; next_path_index < (path_point_after_interpolation_.rows() - 2 * POINTS_NUM);
               next_path_index += POINTS_NUM - 1)
          {
            auto start = std::chrono::high_resolution_clock::now(); //计时开始
            sub_path = path_point_after_interpolation_.block(next_path_index, 0, POINTS_NUM,
                                                             3); // matrix.block(i,j, p, q) : 表示返回从矩阵(i,
                                                                 // j)开始，每行取p个元素，每列取q个元素
            traj_opti_.acc_time_allocation(sub_path, vel, acc, time_allo);
            polynomial_solver_->solution(sub_path, time_allo);                    //求解多项式
            traj_path_ = polynomial_solver_->get_trajectory_points();             //赋值轨迹
            traj_path_array_ = polynomial_solver_->get_trajectory_points_array(); //赋值轨迹
            waypoints_pub_.publish(getTrajPath());                                //发布绿色线路径
            waypoints_vis_pub_.publish(getStorePathArray());                      //发布红色线路径
            auto finish = std::chrono::high_resolution_clock::now();              //计时结束
            // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << "ms\n";
            // //输出计时
          }
          //补全剩下的点
          int path_index = path_point_after_interpolation_.rows() - next_path_index;
          sub_path = path_point_after_interpolation_.block(next_path_index, 0, path_index, 3); // matrix.block(i,j, p, q) : 表示返回从矩阵(i,
                                                                                               // j)开始，每行取p个元素，每列取q个元素
          traj_opti_.acc_time_allocation(sub_path, vel, acc, time_allo);
          polynomial_solver_->solution(sub_path, time_allo);                    //求解多项式
          traj_path_ = polynomial_solver_->get_trajectory_points();             //赋值轨迹
          traj_path_array_ = polynomial_solver_->get_trajectory_points_array(); //赋值轨迹
          waypoints_pub_.publish(getTrajPath());                                //发布绿色线路径
          waypoints_vis_pub_.publish(getStorePathArray());                      //发布红色线路径
          is_begin_goal_ = true;                                                // for循环之后，路径已经生成完毕了
        }
        else if ((path_point_after_interpolation_.rows() <= 20) && is_begin_goal_ == false) //如果轨迹点大于20个，每一段10个点，最后一段少于10个点的时候另作处理
        {
          is_update_next_path_ = false;
          POINTS_NUM = path_point_after_interpolation_.rows();
          Eigen::MatrixXd sub_path = Eigen::MatrixXd::Zero(POINTS_NUM, 3);       //零矩阵
          Eigen::MatrixXd time_allo;                                             //时间矩阵
          sub_path = path_point_after_interpolation_.block(0, 0, POINTS_NUM, 3); // matrix.block(i,j, p, q) : 表示返回从矩阵(i, j)开始，每行取p个元素，每列取q个元素
          traj_opti_.acc_time_allocation(sub_path, vel, acc, time_allo);
          polynomial_solver_->solution(sub_path, time_allo);                    //求解多项式
          traj_path_ = polynomial_solver_->get_trajectory_points();             //赋值轨迹
          traj_path_array_ = polynomial_solver_->get_trajectory_points_array(); //赋值轨迹
          waypoints_pub_.publish(getTrajPath());                                //发布绿色线路径
          waypoints_vis_pub_.publish(getStorePathArray());                      //发布红色线路径
          is_begin_goal_ = true;                                                // for循环之后，路径已经生成完毕了
        }
        is_update_next_path_ = true;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  //开车
  void GlobalRouting::thread_routing(void)
  {
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    Point3d_s start_pose;
    start_pose.x = 0;
    start_pose.y = 0;
    start_pose.z = 0;
    Point3d_s tar_pose;
    double turn_angle = 0;
    int tar_index__ = 0;
    int out_index__ = 0;
    int count = 30;
    while (n.ok())
    {
      // is_update_next_path_ == true指生成路径后
      // is_vehical_start_set ==false指车已在路径起点准备好
      //is_vehical_cango == true是道路可行
      if (is_received_goal_ == true && is_update_next_path_ == true && is_vehical_start_set == true && is_vehical_cango == true && 1)
      {
        /*
    //线程加锁
    。。。
    ros::spinOnce();
    lock.unlock();
    */
        int index = find_nearest_point_index(traj_path_array_, start_pose);                 //最近的点的标
        control_vehicle(traj_path_array_, sub_odom_, tar_index__, turn_angle, out_index__); //调用车的控制程序
        is_vehical_stop_set = vehical_stop(traj_path_array_, sub_odom_);
        geometry_msgs::Vector3 msg;
        msg.y = turn_angle;               //转角
        if (is_vehical_stop_set == false) //未到达终点
        {
          msg.x = VEHICLE_VEL; //速度
        }
        if (is_vehical_stop_set == true) //已经到达终点
        {
          msg.x = 0; //速度
        }
        count--;
        if (count < 0)
        {
          control_data_pub_.publish(msg); //发布车的控制
        }
        else
        {
          geometry_msgs::Vector3 msg1;
          msg1.x = 0;
          msg1.y = 0;
          control_data_pub_.publish(msg1); //发布，车不要动
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

} // namespace global_routing
