#pragma once

#include "../../Eigen/Dense"

namespace Coordinate
{

    struct Frame
    {
        Frame* reference_frame = nullptr;

        Eigen::Matrix3d& orientation;

        Eigen::Vector3d& location;

        inline Frame(Eigen::Matrix3d& _orientation, Eigen::Vector3d& _location) : orientation(_orientation), location(_location) {}
    };

    typedef Eigen::Vector3d _3D;

    class ECEF : public virtual _3D
    {
    public:

    };

    class ECI : public virtual _3D
    {
    public:

    };

    class Geodetic : public virtual _3D
    {
    public:

    };

    class LocalTangentPlane : public virtual _3D
    {
    public:

    };

    class Spherical : public virtual _3D
    {
    public:

    };

}
