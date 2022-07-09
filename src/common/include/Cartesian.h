#pragma once

#include <string>

namespace Cartesian {

    class Vector  {
        alignas(32) double array[3];

        inline Vector() {}

        inline Vector(const char& x) {
            std::memset(array,x,sizeof(array));
        }

        inline Vector(const Vector& b) {
            std::copy_n(b,3,this->array);
        }

        inline Vector(const double* b) {
            std::copy_n(b,3,this->array);
        }

        inline Vector(const double& x,const double& y,const double& z) {
            array[0] = x;
            array[1] = y;
            array[2] = z;
        }

        inline double& operator[](unsigned int& idx){
            return array[idx];
        }

        inline const double& operator[](unsigned int& idx) const {
            return array[idx];
        }

        inline Vector operator+(const Vector& b) const{
            return Vector(array[0] + b.array[0], array[1] + b.array[1],array[2] + b.array[2]);
        }

        inline Vector operator-(const Vector& b) const{
            return Vector(array[0] - b.array[0], array[1] - b.array[1],array[2] - b.array[2]);
        }

        inline Vector operator+(const double& b) const{
            return Vector(array[0] + b, array[1] + b,array[2] + b);
        }

        inline Vector operator-(const double& b) const{
            return Vector(array[0] - b, array[1] - b,array[2] - b);
        }

        inline Vector operator*(const double& b) const{
            return Vector(array[0]*b, array[1]*b,array[2]*b);
        }

        inline Vector operator/(const double& b) const{
            return Vector(array[0]/b, array[1]/b,array[2]/b);
        }

        inline const double& x() const {
            return array[0];
        }

        inline const double& y() const {
            return array[1];
        }

        inline const double& z() const {
            return array[2];
        }

    };

    class Axis {

    };

}