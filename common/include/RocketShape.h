#pragma once

#include "structures.h"
#include <cmath>
#include <memory>

namespace RocketShape {

    struct Point {
        double x;
        double y;
    };

    class Shape {
    public:
        virtual bool get(const double& x, double& val) = 0;
    }

    class Line : public Shape {
        double dydx;
    public:
        const Point start;
        const Point end;

        Line(Point a, Point b) : start(a) , end(b) {
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            this->dydx = dy/dx;
        }

        bool get(const double& x, double& val) {
            if(x < start.x){
                return false;
            }
            if(x > end.x){
                return false;
            }
            val = start.y + (x - start.x)*this->dydx;
            return true;
        }
    };

    class Arc : public Shape {
        double radius;
    public:
        const Point start;
        const Point end;
        const Point center;

        Arc(Point a, Point b, Point c) : start(a), end(b), center(c) {
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            this->radius = sqrt(dx*dx + dy*dy);
        }

        bool get(const double& x, double& val) {
            // conditions are valid for 
            if(x < start.x){
                return false;
            }
            if(x > end.x){
                return false;
            }
            double dx = x - center.x;
            val = sqrt(radius*radius - dx*dx) + center.y;
            return true;
        }
    };

    class Parabola : public Shape {
        Point start;
        Point end;
        double coef[3];

    public:
    };


}

class RocketShape {

public:

    static constexpr int CONICAL = 0;
    static constexpr int BELL = 1;
    static constexpr int PARABOLA = 2;
    static constexpr int HIGHER_ORDER = 3;
    static constexpr int METHOD_OF_CHARACTERISTICS = 4;

    bool use_circular_throat = true;

    int nozzle_shape = 0;

    double throat_radius;
    double chamber_radius;
    double exit_radius;

    double chamber_angle;
    double nozzle_angle;

    double chamber_curvature;
    double throat_curvature;

    double chamber_length;

    std::vector< std::unique_ptr< RocketShape::Shape > > shapes;

    RocketShape(){}
    ~RocketShape(){}

    Curve generate(const double& dx){
        double x = 0;
        while(x < )
    }

};