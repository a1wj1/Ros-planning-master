// #ifndef __LANELET2_MAP_VISUALIZATION_H
// #define __LANELET2_MAP_VISUALIZATION_H
// #include <ros/ros.h>

// #include <visualization_msgs/MarkerArray.h>

// #include <autoware_lanelet2_msgs/MapBin.h>
// #include <lanelet2_core/LaneletMap.h>
// #include <lanelet2_projection/UTM.h>

// #include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
// #include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
// #include <vector>
// class lanelet2_map
// {
// public:
//     lanelet2_map(void);
//     ~lanelet2_map(void);
//     void insertMarkerArray(visualization_msgs::MarkerArray *a1, const visualization_msgs::MarkerArray &a2);
//     void setColor(std_msgs::ColorRGBA *cl, double r, double g, double b, double a);
//     void binMapCallback(autoware_lanelet2_msgs::MapBin msg);
// };

// #endif