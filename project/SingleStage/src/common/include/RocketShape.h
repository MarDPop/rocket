#pragma once

#include "structures.h"
#include <cmath>
#include <memory>
#include <fstream>
#include <string>
#include <iostream>

namespace Shapes {

    struct Point {
        double x;
        double y;
    };

    class Shape {
    public:
        virtual bool get(const double& x, double& val) = 0;
    };

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

    double throat_radius = 0.01;
    double chamber_radius = 0.02;
    double exit_radius = 0.03;

    double chamber_angle = 0.71;
    double nozzle_angle = 0.3;

    double chamber_curvature = 0.01;
    double nozzle_curvature = 0.005;

    double chamber_length = 0.03;

    std::vector< std::unique_ptr< Shapes::Shape > > shapes;

    RocketShape(){}
    RocketShape(std::string fn) {
        std::ifstream file(fn);

        std::string line;

        if(file.is_open()){
            while(std::getline(file,line)) {
                if(line[0] == '#') {
                    continue;
                }

                std::vector< std::string > arr = util::split(line);
                
                if(arr[0].compare("THROAT_RADIUS") == 0){
                    this->throat_radius = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("CHAMBER_RADIUS") == 0){
                    this->chamber_radius = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("EXIT_RADIUS") == 0){
                    this->exit_radius = std::stod(arr[1]);
                    continue;
                }
                
                if(arr[0].compare("CHAMBER_ANGLE") == 0){
                    this->chamber_angle = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("CHAMBER_LENGTH") == 0){
                    this->chamber_length = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("CHAMBER_CURVATURE_RADIUS") == 0){
                    this->chamber_curvature = std::stod(arr[1]);
                    continue;
                }

                if(arr[0].compare("NOZZLE_ANGLE") == 0){
                    this->nozzle_angle = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("NOZZLE_CURVATURE_RADIUS") == 0){
                    this->nozzle_curvature = std::stod(arr[1]);
                    continue;
                }
                if(arr[0].compare("NOZZLE_SHAPE") == 0){
                    this->nozzle_shape = std::stoi(arr[1]);
                    continue;
                }
            }

            file.close();
        }

    }
    ~RocketShape(){}

    Curve generate_constant_dx(const double& dx){
        Curve output;
        double x = 0;
        double x_next = chamber_length;
        double x_prev = 0;
        double y_next;
        double y_prev;
        while(x < x_next){
            output.x.push_back(x);
            output.y.push_back(chamber_radius);
            x += dx;
        }

        x_prev = x_next;
        double delta_x = chamber_curvature*sin(chamber_angle);
        double delta_y = chamber_curvature*(1 - cos(chamber_angle));
        x_next += delta_x;
        
        y_next = chamber_radius - delta_y;

        double y_center = chamber_radius - chamber_curvature;
        while(x < x_next){
            output.x.push_back(x);
            double DX = x - x_prev;
            double dy = sqrt(chamber_curvature*chamber_curvature - DX*DX); 
            output.y.push_back(y_center + dy);
            x += dx;
        }

        double dydx = -tan(chamber_angle);

        y_prev = y_next;
        x_prev = x_next;
        y_next = throat_radius + delta_y;
        x_next += (y_next - y_prev)/dydx;

        while(x < x_next){
            output.x.push_back(x);
            output.y.push_back(y_prev + dydx*(x - x_prev));
            x += dx;
        }

        y_prev = y_next;
        x_prev = x_next;
        y_next = throat_radius;
        x_next += delta_x;

        y_center = throat_radius + chamber_curvature;
        while(x < x_next){
            output.x.push_back(x);
            double DX = x - x_next;
            double dy = sqrt(chamber_curvature*chamber_curvature - DX*DX); 
            output.y.push_back(y_center - dy);
            x += dx;
        }

        delta_x = nozzle_curvature*sin(nozzle_angle);
        delta_y = nozzle_curvature*(1 - cos(nozzle_angle));

        y_prev = y_next;
        x_prev = x_next;
        y_next = throat_radius;
        x_next += delta_x;
        y_center = throat_radius + nozzle_curvature;
        while(x < x_next){
            output.x.push_back(x);
            double DX = x - x_prev;
            double dy = sqrt(nozzle_curvature*nozzle_curvature - DX*DX); 
            output.y.push_back( y_center - dy);
            x += dx;
        }

        y_prev = y_next;
        x_prev = x_next;
        y_next = exit_radius;

        if(this->nozzle_shape == 0) {
            dydx = tan(nozzle_angle);
            x_next += (y_next - y_prev)/dydx;

            while(x < x_next){
                output.x.push_back(x);
                output.y.push_back(y_prev + (x - x_prev)*dydx);
                x += dx;
            }
        }

        if(this->nozzle_shape == 1) {

            double r = (this->exit_radius - y_prev)/(1 - cos(nozzle_angle));
            x_next += r*sin(nozzle_angle);
            double y_center = this->exit_radius - r;

            while(x < x_next){
                output.x.push_back(x);
                double DX = x - x_next;
                double dy = sqrt(r*r - DX*DX); 
                output.y.push_back( y_center + dy);
                x += dx;
            }
        }

        if(this->nozzle_shape == 2) {
            dydx = tan(nozzle_angle);
            double b = y_prev*y_prev;
            double a = 2*y_prev*dydx;
            x_next += (this->exit_radius*this->exit_radius - b)/a;
            while(x < x_next){
                output.x.push_back(x);
                double DX = x - x_prev;
                output.y.push_back(sqrt(a*DX + b));
                x += dx;
            }
            std::cout << "parabola";
        }

        return output;

    }

};