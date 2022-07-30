#pragma once

#include <cmath>
#include <algorithm>
#include "string.h"

namespace Cartesian {

    struct Vector  {

        alignas(32) double data[3];

        inline Vector() {}

        inline Vector(const char& x) {
            memset(data,x,sizeof(data));
        }

        inline Vector(const Vector& b) {
            std::copy_n(b,3,this->data);
        }

        inline Vector(const double* b) {
            std::copy_n(b,3,this->data);
        }

        inline Vector(const double& x,const double& y,const double& z) : data{x,y,z} {
        }

        inline void operator=(const double* b) {
            std::copy_n(b,3,this->data);
        }

        inline void operator=(const Vector& b) {
            std::copy_n(b.data,3,this->data);
        }

        inline void zero() {
            memset(data,0,sizeof(data));
        }

        inline double& operator[](unsigned int& idx){
            return data[idx];
        }

        inline const double& operator[](unsigned int& idx) const {
            return data[idx];
        }

        inline Vector operator+(const Vector& b) const{
            return Vector(data[0] + b.data[0], data[1] + b.data[1],data[2] + b.data[2]);
        }

        inline Vector operator-(const Vector& b) const{
            return Vector(data[0] - b.data[0], data[1] - b.data[1],data[2] - b.data[2]);
        }

        inline Vector operator+(const double& b) const {
            return Vector(data[0] + b, data[1] + b,data[2] + b);
        }

        inline Vector operator-(const double& b) const {
            return Vector(data[0] - b, data[1] - b,data[2] - b);
        }

        inline Vector operator*(const double& b) const {
            return Vector(data[0]*b, data[1]*b,data[2]*b);
        }

        inline Vector operator/(const double& b) const {
            return Vector(data[0]/b, data[1]/b,data[2]/b);
        }

        inline void operator+=(const Vector& a) {
            this->data[0] += a.data[0];
            this->data[1] += a.data[1];
            this->data[2] += a.data[2];
        }

        inline void operator-=(const Vector& a) {
            this->data[0] -= a.data[0];
            this->data[1] -= a.data[1];
            this->data[2] -= a.data[2];
        }

        inline void operator+=(const double& a) {
            this->data[0] += a;
            this->data[1] += a;
            this->data[2] += a;
        }

        inline void operator-=(const double& a) {
            this->data[0] -= a;
            this->data[1] -= a;
            this->data[2] -= a;
        }

        inline void operator*=(const double& a) {
            this->data[0] *= a;
            this->data[1] *= a;
            this->data[2] *= a;
        }

        inline double dot(const Vector& b) const {
            return this->data[0]*b.data[0] + this->data[1]*b.data[1] + this->data[2]*b.data[2];
        }

        inline Vector cross(const Vector& b) const {
            Vector out;
            out.data[0] = this->data[1]*b.data[2] - this->data[2]*b.data[1];
            out.data[1] = this->data[2]*b.data[0] - this->data[0]*b.data[2];
            out.data[2] = this->data[0]*b.data[1] - this->data[1]*b.data[0];
            return out;
        }

        inline double norm() const {
            return sqrt(this->data[0]*this->data[0] + this->data[1]*this->data[1] + this->data[2]*this->data[2]);
        }

        inline void normalize() {
            double n = 1/sqrt(this->data[0]*this->data[0] + this->data[1]*this->data[1] + this->data[2]*this->data[2]);
            this->data[0] *= n;
            this->data[1] *= n;
            this->data[2] *= n;
        }

        inline const double& x() const {
            return data[0];
        }

        inline const double& y() const {
            return data[1];
        }

        inline const double& z() const {
            return data[2];
        }

        static inline void cross(const Vector& a, const Vector& b, Vector& c) {
            c.data[0] = a.data[1]*b.data[2] - a.data[2]*b.data[1];
            c.data[1] = a.data[2]*b.data[0] - a.data[0]*b.data[2];
            c.data[2] = a.data[0]*b.data[1] - a.data[1]*b.data[0];
        }

    };

    struct Axis {

        alignas(32) double data[9];
        double* const axis[3];

        Axis() : axis{&data[0],&data[3],&data[6]} {}

        Axis(double b[9]) : axis{&data[0],&data[3],&data[6]} {
            std::copy_n(b,9,this->data);
        }

        Axis(const Axis& b) : axis{&data[0],&data[3],&data[6]} {
            std::copy_n(b.data,9,this->data);
        }

        inline Axis operator+(const Axis& b) {
            Axis a;
            for(int i = 0; i < 9; i++) {
                a.data[i] = this->data[i] + b.data[i];
            }
            return a;
        }

        inline Axis operator-(const Axis& b) {
            Axis a;
            for(int i = 0; i < 9; i++) {
                a.data[i] = this->data[i] - b.data[i];
            }
            return a;
        }

        inline void operator+=(const Axis& b) {
            for(int i = 0; i < 9; i++) {
                this->data[i] += b.data[i];
            }
        }

        inline void operator-=(const Axis& b) {
            for(int i = 0; i < 9; i++) {
                this->data[i] -= b.data[i];
            }
        }

        inline void operator*=(const double& b) {
            for(int i = 0; i < 9; i++) {
                this->data[i] *= b;
            }
        }

        inline Vector operator*(const Vector& a) {
            Vector b;
            for(int i = 0; i < 3; i++){
                b.data[i] = this->data[i]*a[0] + this->data[i + 1]*a[1] + this->data[i + 2]*a[2];
            }
            return b;
        }

        inline Axis get_transpose() {
            Axis t;
            t.data[0] = this->data[0];
            t.data[1] = this->data[3];
            t.data[2] = this->data[6];
            t.data[3] = this->data[1];
            t.data[4] = this->data[4];
            t.data[5] = this->data[7];
            t.data[6] = this->data[2];
            t.data[7] = this->data[5];
            t.data[8] = this->data[8];
            return t;
        }

        inline Axis get_inverse() {
            Axis Inv;
            Inv.data[0] = this->data[4]*this->data[8] - this->data[5]*this->data[7];
            Inv.data[1] = this->data[2]*this->data[7] - this->data[1]*this->data[8];
            Inv.data[2] = this->data[1]*this->data[5] - this->data[2]*this->data[4];

            Inv.data[3] = this->data[5]*this->data[6] - this->data[3]*this->data[8];
            Inv.data[4] = this->data[0]*this->data[8] - this->data[2]*this->data[6];
            Inv.data[5] = this->data[2]*this->data[3] - this->data[0]*this->data[5];

            Inv.data[6] = this->data[3]*this->data[7] - this->data[4]*this->data[6];
            Inv.data[7] = this->data[1]*this->data[6] - this->data[0]*this->data[7];
            Inv.data[8] = this->data[0]*this->data[4] - this->data[1]*this->data[3];

            double det = 1/(this->data[0]*Inv.data[0] + this->data[1]*Inv.data[3] + this->data[2]*Inv.data[6]);

            for(int i = 0; i < 9; i++) {
                Inv.data[i] *= det;
            }

            return Inv;
        }

    };

}
