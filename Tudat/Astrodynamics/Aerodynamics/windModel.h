#ifndef WINDMODELBASE_H
#define WINDMODELBASE_H

//! Standard libraries.
#include <iostream>

//! External libraries: BOOST.
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

//! External libraries: EIGEN.
#include <Eigen/Core>

//! Own libraries.
#include <Tudat/Mathematics/BasicMathematics/linearAlgebraTypes.h>
#include "Tudat/Astrodynamics/ReferenceFrames/referenceFrameTransformations.h"

namespace tudat
{

namespace aerodynamics
{

class WindModel
{
    public:
        virtual ~WindModel(){}

        virtual void updateWindVelocity( ) = 0;

        void updateWindModel(
                const double &currentAltitude ,
                const tudat::basic_mathematics::Vector6d &stateInRotatingFrame ,
                const double &currentLongitude ,
                const double &currentLatitude )
        {
            // Store current altitude to be used by the specified wind model implemented.
            currentAltitude_ = currentAltitude;

            // Update wind velocity vector by calling the specified wind model through a virtual function.
            updateWindVelocity( );

            // Relate the velocity vectors to compute the new airspeed vector.
            currentGroundspeedVectorInRotatingFrame_
                    = stateInRotatingFrame.tail< 3 >();

            transformationV2R_
                    = reference_frames::getLocalVerticalToRotatingPlanetocentricFrameTransformationMatrix( currentLongitude , currentLatitude );

            transformationR2V_
                    = reference_frames::getRotatingPlanetocentricToLocalVerticalFrameTransformationMatrix( currentLongitude , currentLatitude );

            currentWindspeedVectorInRotatingFrame_
                    = transformationV2R_ * currentWindspeedVectorInLocalVerticalFrame_;

            currentAirspeedVectorInRotatingFrame_
                    = currentGroundspeedVectorInRotatingFrame_ - currentWindspeedVectorInRotatingFrame_;

            // Transform airspeed and groundspeed to the vertical frame.
            currentAirspeedVectorInLocalVerticalFrame_
                    = transformationR2V_ * currentAirspeedVectorInRotatingFrame_;

            currentGroundspeedVectorInLocalVerticalFrame_
                    = transformationR2V_ * currentGroundspeedVectorInRotatingFrame_;

            // Compute airspeed-based flight-path and heading angles.
            currentFlightPathAngleAirspeedBased_
                    = reference_frames::calculateFlightPathAngle( currentAirspeedVectorInLocalVerticalFrame_ );

            currentHeadingAngleAirspeedBased_
                    = reference_frames::calculateHeadingAngle( currentAirspeedVectorInLocalVerticalFrame_ );
        }

        Eigen::Vector3d getAirspeedVector( )
        {
            return currentAirspeedVectorInRotatingFrame_;
        }

        Eigen::Vector3d getWindspeedVector()
        {
            return currentWindspeedVectorInRotatingFrame_;
        }

        Eigen::Vector3d getWindspeedVectorInLocalVerticalFrame( )
        {
            return currentWindspeedVectorInLocalVerticalFrame_;
        }

        Eigen::Vector3d getGroundspeedVector( )
        {
            return currentGroundspeedVectorInRotatingFrame_;
        }

        double getFlightPathAngleAirspeedBased()
        {
            return currentFlightPathAngleAirspeedBased_;
        }

        double getHeadingAngleAirspeedBased()
        {
            return currentHeadingAngleAirspeedBased_;
        }

    protected:
        double currentAltitude_;

        double currentFlightPathAngleAirspeedBased_;

        double currentHeadingAngleAirspeedBased_;


        Eigen::Vector3d currentWindspeedVectorInLocalVerticalFrame_ = Eigen::Vector3d::Zero();

        Eigen::Vector3d currentWindspeedVectorInRotatingFrame_;


        Eigen::Vector3d currentGroundspeedVectorInLocalVerticalFrame_;

        Eigen::Vector3d currentGroundspeedVectorInRotatingFrame_;


        Eigen::Vector3d currentAirspeedVectorInLocalVerticalFrame_;

        Eigen::Vector3d currentAirspeedVectorInRotatingFrame_;


        Eigen::Matrix3d transformationV2R_;

        Eigen::Matrix3d transformationR2V_;

    private:

};

} // namespace_aerodynamics
} // namespace_tudat

#endif // WINDMODELBASE_H
