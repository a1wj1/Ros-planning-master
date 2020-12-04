#ifndef FRENET_H
#define FRENET_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
using std::string;
class  Frenet
{
public:
    Frenet();
    string hasData(string s);
    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                        const std::vector<double> &maps_y);
    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                     const std::vector<double> &maps_y);
    std::vector<double>  getFrenet(double x, double y, double theta,
                         const std::vector<double> &maps_x,
                         const std::vector<double> &maps_y);
   std::vector<double>  getXY(double s, double d, const std::vector<double> &maps_s,
                     const std::vector<double> &maps_x,
                     const std::vector<double> &maps_y);
};

#endif // FRENET_H
