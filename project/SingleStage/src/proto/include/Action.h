#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

/**
* struct to contain an action to a body
* ie, force and moment.
* all units are in SI
*/
struct BodyAction
{
    /**
    * Force Vector in body frame
    */
    Vector force;

    /**
    * Force Vector in body frame
    */
    Vector moment;

    /**
    * Force Vector reference point in body
    */
    Vector location;

    inline void zero()
    {
        this->force.zero();
        this->moment.zero();
        this->location.zero();
    }

    inline Vector get_torque(const Vector& location) const
    {
        Vector arm = location - this->location;
        return this->moment + arm.cross(this->force);
    }

    inline BodyAction operator+(const BodyAction& otherAction) const
    {
        BodyAction output;
        output.location.zero();

        Vector thisTorque = this->location.cross(this->force);
        Vector otherTorque = otherAction.location.cross(otherAction.force);

        for(int i = 0; i < 3; i++)
        {
            output.force.data[i] = this->force.data[i] + otherAction.force.data[i];
            output.moment.data[i] = this->moment.data[i] + otherAction.moment.data[i];
        }
        for(int i = 0; i < 3; i++)
        {
            output.moment.data[i] += thisTorque.data[i] + otherTorque.data[i];
        }
        return output;
    }

    inline void operator+=(const BodyAction& otherAction)
    {
        this->force += otherAction.force;
        this->moment += otherAction.get_torque(this->location);
    }
};

struct CoordinateFrame
{
    CoordinateFrame* reference_frame = nullptr;

    Axis& orientation;

    Vector& location;

    inline CoordinateFrame(Axis& _orientation, Vector& _location) : orientation(_orientation), location(_location) {}
};

/**
* struct to contain an action to a body
* ie, force and moment.
* all units are in SI
*/
struct GeneralAction
{
    /**
    * Force Vector in frame
    */
    Vector force;

    /**
    * Moment Vector in frame
    */
    Vector moment;

    /**
    * Location at which the force and moment are action from the origin of reference frame
    */
    Vector arm;

    /**
    * if nullptr, inertial frame
    */
    CoordinateFrame* reference_frame = nullptr;

    inline void zero()
    {
        this->force.zero();
        this->moment.zero();
        this->arm.zero();
    }

    inline Vector get_torque() const
    {
        return this->moment + this->arm.cross(this->force);
    }

    inline GeneralAction operator+(const GeneralAction& otherAction) const
    {
        GeneralAction inertialAction;
        inertialAction.arm.zero();
        if(this->reference_frame)
        {
            inertialAction.force = this->reference_frame->orientation.transpose_mult(this->force);
            inertialAction.moment = this->reference_frame->orientation.transpose_mult(this->get_torque());
        }
        else
        {
            inertialAction.force = this->force;
            inertialAction.moment = this->get_torque();
        }
        if(otherAction.reference_frame)
        {
            inertialAction.force += otherAction.reference_frame->orientation.transpose_mult(otherAction.force);
            inertialAction.moment += otherAction.reference_frame->orientation.transpose_mult(otherAction.get_torque());
        }
        else
        {
            inertialAction.force += otherAction.force;
            inertialAction.moment += otherAction.get_torque();
        }
        return inertialAction;
    }

    inline void operator+=(const GeneralAction& otherAction)
    {
        if(otherAction.reference_frame == this->reference_frame)
        {
            this->force += otherAction.force;
            this->moment += otherAction.get_torque();
        }
        else
        {
            if(!this->reference_frame)
            {
                this->force += otherAction.reference_frame->orientation.transpose_mult(otherAction.force);
                this->moment += otherAction.reference_frame->orientation.transpose_mult(otherAction.get_torque());
            }
            else
            {
                // Don't do this... lol
                // Also find a better way to do this
                Vector forceInertial = otherAction.reference_frame->orientation.transpose_mult(otherAction.force);
                Vector momentInertial = otherAction.reference_frame->orientation.transpose_mult(otherAction.get_torque());
                Vector armInertial = otherAction.reference_frame->orientation.transpose_mult(otherAction.get_torque());
            }
        }
    }
};
