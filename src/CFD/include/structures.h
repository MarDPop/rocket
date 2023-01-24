#pragma once

#include "../../../lib/Eigen"

struct Cell_Axisymmetric;

struct Face_Axisymmetric
{
    Fluid_Axisymmetric fluid;

    Eigen::Vector2d vertex[2];
    Eigen::Vector2d normal;
    double area;
    Cell_Axisymmetric cells[2];

};
