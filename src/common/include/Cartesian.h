#pragma once

#include <cmath>
#include <algorithm>
#include "string.h"

namespace Cartesian {

    inline void add(const double* u, const double* v, double* w) {
        w[0] = u[0] + v[0];
        w[1] = u[1] + v[1];
        w[2] = u[2] + v[2];
    }

    inline void sub(const double* u, const double* v, double* w) {
        w[0] = u[0] - v[0];
        w[1] = u[1] - v[1];
        w[2] = u[2] - v[2];
    }

    inline void cross(const double* u, const double* v, double* w) {
        w[0] = u[1]*v[2] - u[2]*v[1];
        w[1] = u[2]*v[0] - u[0]*v[2];
        w[2] = u[0]*v[1] - u[1]*v[0];
    }

    inline double dot(const double* u, const double* v) {
        return u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
    }

    struct Vector {

        alignas(32) double data[3];

        inline Vector() {}

        inline ~Vector(){}

        inline Vector(const char x) {
            memset(data,x,sizeof(data));
        }

        inline Vector(const Vector& b) {
            std::copy(b.data,b.data + 3,this->data);
        }

        inline Vector(const double* b) {
            std::copy(b,b+3,this->data);
        }

        inline Vector(double x, double y ,double z) : data{x,y,z} {
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

        inline double& operator[](unsigned int idx){
            return data[idx];
        }

        inline const double operator[](unsigned int idx) const {
            return data[idx];
        }

        inline Vector operator+(const Vector& b) const{
            return Vector(data[0] + b.data[0], data[1] + b.data[1],data[2] + b.data[2]);
        }

        inline Vector operator-(const Vector& b) const{
            return Vector(data[0] - b.data[0], data[1] - b.data[1],data[2] - b.data[2]);
        }

        inline Vector operator+(double b) const {
            return Vector(data[0] + b, data[1] + b,data[2] + b);
        }

        inline Vector operator-(double b) const {
            return Vector(data[0] - b, data[1] - b,data[2] - b);
        }

        inline Vector operator*(double b) const {
            return Vector(data[0]*b, data[1]*b,data[2]*b);
        }

        inline Vector operator/(double b) const {
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

        inline void operator+=(double a) {
            this->data[0] += a;
            this->data[1] += a;
            this->data[2] += a;
        }

        inline void operator-=(double a) {
            this->data[0] -= a;
            this->data[1] -= a;
            this->data[2] -= a;
        }

        inline void operator*=(double a) {
            this->data[0] *= a;
            this->data[1] *= a;
            this->data[2] *= a;
        }

        inline double operator*(const Vector& b) const {
            return this->data[0]*b.data[0] + this->data[1]*b.data[1] + this->data[2]*b.data[2];
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

        inline double x() const {
            return data[0];
        }

        inline void x(double v) {
            data[0] = v;
        }

        inline double y() const {
            return data[1];
        }

        inline void y(double v) {
            data[1] = v;
        }

        inline double z() const {
            return data[2];
        }

        inline void z(double v) {
            data[2] = v;
        }

        static inline void cross(const Vector& a, const Vector& b, Vector& c) {
            c.data[0] = a.data[1]*b.data[2] - a.data[2]*b.data[1];
            c.data[1] = a.data[2]*b.data[0] - a.data[0]*b.data[2];
            c.data[2] = a.data[0]*b.data[1] - a.data[1]*b.data[0];
        }

    };

    struct Axis {

        union {
            alignas(32) double data[9];
            struct {
                Vector x;
                Vector y;
                Vector z;
            };
        };

        inline Axis() {}
        inline ~Axis(){}

        inline Axis(const char x) {
            memset(data,x,sizeof(data));
        }

        inline Axis(double b[9]) {
            std::copy_n(b,9,this->data);
        }

        inline Axis(const Axis& b) {
            std::copy_n(b.data,9,this->data);
        }

        inline void zero() {
            memset(this->data,0,sizeof(this->data));
        }

        inline Axis operator+(const Axis& b) const noexcept {
            Axis a;
            for(int i = 0; i < 9; i++) {
                a.data[i] = this->data[i] + b.data[i];
            }
            return a;
        }

        inline Axis operator-(const Axis& b) const noexcept {
            Axis a;
            for(int i = 0; i < 9; i++) {
                a.data[i] = this->data[i] - b.data[i];
            }
            return a;
        }

        inline void operator+=(const Axis& b) noexcept {
            for(int i = 0; i < 9; i++) {
                this->data[i] += b.data[i];
            }
        }

        inline void operator-=(const Axis& b) noexcept {
            for(int i = 0; i < 9; i++) {
                this->data[i] -= b.data[i];
            }
        }

        inline void operator*=(double b) noexcept {
            for(int i = 0; i < 9; i++) {
                this->data[i] *= b;
            }
        }

        inline Vector operator*(const Vector& a) const noexcept {
            Vector b;
            int irow = 0;
            for(int i = 0; i < 3; i++){
                b.data[i] = this->data[irow]*a.data[0] + this->data[irow + 1]*a.data[1] + this->data[irow + 2]*a.data[2];
                irow += 3;
            }
            return b;
        }

        inline Axis operator*(const Axis& a) const noexcept {
            Axis b((char)0);
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++){
                    b.data[i] += this->data[i]*a.data[j];
                }
            }
            return b;
        }

        inline static void mult(const Axis& A, const Vector& b, Vector& x) noexcept {
            int irow = 0;
            for(int i = 0; i < 3; i++){
                x.data[i] = A.data[irow]*b.data[0] + A.data[irow + 1]*b.data[1] + A.data[irow + 2]*b.data[2];
                irow += 3;
            }
            return b;
        }

        inline Axis get_transpose() const noexcept {
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

        inline Vector transpose_mult(const Vector& v) const noexcept {
            Vector b;
            b.data[0] = this->data[0]*v.data[0] + this->data[3]*v.data[1] + this->data[6]*v.data[2];
            b.data[1] = this->data[1]*v.data[0] + this->data[4]*v.data[1] + this->data[7]*v.data[2];
            b.data[2] = this->data[2]*v.data[0] + this->data[5]*v.data[1] + this->data[8]*v.data[2];
            return b;
        }

        inline Axis get_inverse() const noexcept {
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

    struct Spherical {
        alignas(32) double data[3];

        Spherical() {}

        Spherical(double r, double el, double az) : data{r,el,az} {
        }

        Spherical(double b[3]) {
            std::copy_n(b,3,this->data);
        }

        Spherical(const Spherical& b) {
            std::copy_n(b.data,3,this->data);
        }

        Spherical(const Vector& x) {
            this->data[0] = x.norm();
            this->data[1] = asin(x[2]/this->data[0]);
            this->data[2] = atan2(x[1],x[0]);
        }

        Vector to_cartesian() {
            Vector out;
            out.data[2] = sin(this->data[1]);
            double r_t = sqrt(1.0 - out.data[2]*out.data[2])*this->data[0];
            out.data[0] = cos(this->data[2])*r_t;
            out.data[1] = sin(this->data[2])*r_t;
            out.data[2] *= this->data[0];
            return out;
        }

        inline double r() const {
            return data[0];
        }

        inline void r(double v) {
            data[0] = v;
        }

        inline double el() const {
            return data[1];
        }

        inline void el(double v) {
            data[1] = v;
        }

        inline double az() const {
            return data[2];
        }

        inline void az(double v) {
            data[2] = v;
        }

    };

    struct Quaternion {

        alignas(32) double data[4];

        Quaternion() {}

    };

}
