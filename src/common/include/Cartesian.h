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

    inline void rotation_between(const double* u, const double* v, double* q) {
        //https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        q[1] = u[1]*v[2] - u[2]*v[1];
        q[2] = u[2]*v[0] - u[0]*v[2];
        q[3] = u[0]*v[1] - u[1]*v[0];
        q[0] = 1 + dot(u,v);

        double recipNorm = 1.0 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

    inline void q2rotm(const double* q, double* rotm) {
        double qi2 = q[1]*2;
        double qj2 = q[2]*2;
        double qk2 = q[3]*2;
        rotm[0] = 1 - (qj2*q[2] + qk2*q[3]);
        rotm[4] = 1 - (qi2*q[1] + qk2*q[3]);
        rotm[8] = 1 - (qi2*q[1] + qj2*q[2]);
        rotm[1] = qi2*q[2] - qk2*q[0];
        rotm[2] = qi2*q[3] + qj2*q[0];
        rotm[3] = qi2*q[2] + qk2*q[0];
        rotm[5] = qj2*q[3] - qi2*q[0];
        rotm[6] = qi2*q[3] - qj2*q[0];
        rotm[7] = qj2*q[3] + qi2*q[0];
    }


    inline void rotm2q( const double* rotm, double* q ) {
        double s = rotm[0] + rotm[4] + rotm[8];
        if( s > 0 ) {
            s = 0.5 / sqrt(s + 1.0);
            q[0] = 0.25 / s;
            q[1] = ( rotm[7] - rotm[5] ) * s;
            q[2] = ( rotm[2] - rotm[6] ) * s;
            q[3] = ( rotm[3] - rotm[1] ) * s;
        } else {
            if ( rotm[0] > rotm[4] && rotm[0] > rotm[8] ) {
              s = 0.5 / sqrt( 1.0f + rotm[0] - rotm[4] - rotm[8]);
              q[0] = (rotm[7] - rotm[5] ) * s;
              q[1] = 0.25 / s;
              q[2] = (rotm[1] + rotm[3] ) * s;
              q[3] = (rotm[2] + rotm[6] ) * s;
            } else if (rotm[4] > rotm[8]) {
              s = 0.5f / sqrt( 1.0f + rotm[4] - rotm[0] - rotm[8]);
              q[0] = (rotm[2] - rotm[6] ) * s;
              q[1] = (rotm[1] + rotm[3] ) * s;
              q[2] = 0.25 / s;
              q[3] = (rotm[5] + rotm[7] ) * s;
            } else {
              s = 0.5 / sqrt( 1.0f + rotm[8] - rotm[0] - rotm[4] );
              q[0] = (rotm[3] - rotm[1] ) * s;
              q[1] = (rotm[2] + rotm[6] ) * s;
              q[2] = (rotm[5] + rotm[7] ) * s;
              q[3] = 0.25 / s;
            }
        }
    }

    struct Vector {

        double data[3];

        inline Vector() {}

        inline ~Vector(){}

        inline Vector(const char x) {
            memset(data,x,sizeof(data));
        }

        inline Vector(const Vector& b) {
            memcpy(data,b.data,sizeof(data));
        }

        inline Vector(const double* b) {
            memcpy(data,b,sizeof(data));
        }

        inline Vector(double x, double y ,double z) : data{x,y,z} {
        }

        inline void operator=(const double* b) {
            memcpy(data,b,sizeof(data));
        }

        inline void operator=(const Vector& b) {
            memcpy(data,b.data,sizeof(data));
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

        static constexpr double IDENTITY[9] = {1,0,0,0,1,0,0,0,1};

        union {
            double data[9];
            struct {
                Vector x;
                Vector y;
                Vector z;
            } axis;
        };

        inline Axis() {}
        inline ~Axis(){}

        inline Axis(const char x) {
            memset(data,x,sizeof(data));
        }

        inline Axis(const double b[9]) {
            memcpy(this->data,b,sizeof(this->data));
        }

        inline Axis(const Axis& b) {
            memcpy(this->data,b.data,sizeof(this->data));
        }

        inline void zero() {
            memset(this->data,0,sizeof(this->data));
        }

        inline void identity() {
            memcpy(this->data,IDENTITY,sizeof(this->data));
        }

        static inline Axis eye() {
            return Axis(IDENTITY);
        }

        inline void operator=(const Axis& b) {
            memcpy(this->data,b.data,sizeof(this->data));
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

        inline Axis operator*(const Axis& B) const noexcept {
            Axis C;
            C.data[0] = data[0]*B.data[0] + data[1]*B.data[3] + data[2]*B.data[6];
            C.data[1] = data[0]*B.data[1] + data[1]*B.data[4] + data[2]*B.data[7];
            C.data[2] = data[0]*B.data[2] + data[1]*B.data[5] + data[2]*B.data[8];
            C.data[3] = data[0]*B.data[0] + data[1]*B.data[3] + data[2]*B.data[6];
            C.data[4] = data[0]*B.data[1] + data[1]*B.data[4] + data[2]*B.data[7];
            C.data[5] = data[0]*B.data[2] + data[1]*B.data[5] + data[2]*B.data[8];
            C.data[6] = data[0]*B.data[0] + data[1]*B.data[3] + data[2]*B.data[6];
            C.data[7] = data[0]*B.data[1] + data[1]*B.data[4] + data[2]*B.data[7];
            C.data[8] = data[0]*B.data[2] + data[1]*B.data[5] + data[2]*B.data[8];
            return C;
        }

        inline static void mult(const Axis& A, const Axis& B, Axis& C) noexcept {
            C.data[0] = A.data[0]*B.data[0] + A.data[1]*B.data[3] + A.data[2]*B.data[6];
            C.data[1] = A.data[0]*B.data[1] + A.data[1]*B.data[4] + A.data[2]*B.data[7];
            C.data[2] = A.data[0]*B.data[2] + A.data[1]*B.data[5] + A.data[2]*B.data[8];
            C.data[3] = A.data[0]*B.data[0] + A.data[1]*B.data[3] + A.data[2]*B.data[6];
            C.data[4] = A.data[0]*B.data[1] + A.data[1]*B.data[4] + A.data[2]*B.data[7];
            C.data[5] = A.data[0]*B.data[2] + A.data[1]*B.data[5] + A.data[2]*B.data[8];
            C.data[6] = A.data[0]*B.data[0] + A.data[1]*B.data[3] + A.data[2]*B.data[6];
            C.data[7] = A.data[0]*B.data[1] + A.data[1]*B.data[4] + A.data[2]*B.data[7];
            C.data[8] = A.data[0]*B.data[2] + A.data[1]*B.data[5] + A.data[2]*B.data[8];
        }

        inline static void mult(const Axis& A, const Vector& b, Vector& x) noexcept {
            int irow = 0;
            for(int i = 0; i < 3; i++){
                x.data[i] = A.data[irow]*b.data[0] + A.data[irow + 1]*b.data[1] + A.data[irow + 2]*b.data[2];
                irow += 3;
            }
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

        Quaternion(const double* v) {
            memcpy(data,v,sizeof(data));
        }

        Quaternion(const Quaternion& v) {
            memcpy(data,v.data,sizeof(data));
        }

        Quaternion(double a, double i, double j, double k) : data{a,i,j,k} {
        }

        Quaternion conjugate() {
            Quaternion q;
            q.data[0] = this->data[0];
            q.data[1] = -this->data[1];
            q.data[2] = -this->data[2];
            q.data[3] = -this->data[3];
            return q;
        }

        inline Vector get_vector() {
            return Vector(data + 1);
        }

        inline double get_scalar() {
            return data[0];
        }

        inline double mag() {
            return data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3];
        }

        inline void normalize() {
            double n = 1.0/sqrt(this->mag());
            data[0] *= n;
            data[1] *= n;
            data[2] *= n;
            data[3] *= n;
        }

        inline Axis to_rotation_matrix(){
            Axis out;
            double i2 = 2*data[1];
            double j2 = 2*data[2];
            double k2 = 2*data[3];

            out.data[0] = 1.0 - j2*data[2] - k2*data[3];
            out.data[4] = 1.0 - i2*data[1] - k2*data[3];
            out.data[8] = 1.0 - i2*data[1] - j2*data[2];
            double tmp = k2*data[0];
            out.data[1] = out.data[3] = i2*data[2];
            out.data[1] -= tmp;
            out.data[3] += tmp;

            tmp = j2*data[0];
            out.data[2] = out.data[6] = i2*data[3];
            out.data[2] += tmp;
            out.data[6] -= tmp;

            tmp = i2*data[0];
            out.data[5] = out.data[7] = j2*data[3];
            out.data[1] -= tmp;
            out.data[3] += tmp;
            return out;
        }

        inline double w() {
            return data[0];
        }

        inline double x() {
            return data[1];
        }

        inline double y() {
            return data[2];
        }

        inline double z() {
            return data[3];
        }

    };

    inline void rotation_matrix_angle_axis(const double angle, const Vector& axis, Axis& mat) {
        double c = cos(angle);
        double s = sin(angle);
        double c1 = 1 - c;
        double xc = axis.x()*c1;
        double yc = axis.y()*c1;
        mat.data[0] = c + axis.x()*xc;
        mat.data[4] = c + axis.y()*yc;
        mat.data[8] = c + axis.z()*axis.z()*c1;
        double uc = axis.x()*yc;
        double us = axis.z()*s;
        mat.data[1] = uc - us;
        mat.data[3] = uc + us;
        uc = axis.z()*xc;
        us = axis.y()*s;
        mat.data[1] = uc + us;
        mat.data[6] = uc - us;
        uc = axis.z()*yc;
        us = axis.x()*s;
        mat.data[5] = uc - us;
        mat.data[7] = uc + us;
    }

}
