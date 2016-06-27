#ifndef TOOL_HWM07_WINDMODEL_H
#define TOOL_HWM07_WINDMODEL_H

//! Standard libraries.
#include <stdio.h>

//! External libraries: BOOST.
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

//! External libraries: EIGEN.
#include <Eigen/Core>

//! Own libraries.
#include <Tudat/Mathematics/Interpolators/cubicSplineInterpolator.h>
#include <Tudat/Astrodynamics/Aerodynamics/windModel.h>

namespace tudat
{

namespace aerodynamics
{

using namespace tudat::interpolators;

class HorizontalWindModel07 : public WindModel
{
    public:

        //! Constructor.
        HorizontalWindModel07( );

        int getSelectedSample( ){ return selectedSample_; }

    protected:
        void updateWindVelocity( );

    private:
        int selectedSample_;

        CubicSplineInterpolatorDoublePointer northWindInterpolator_;
        CubicSplineInterpolatorDoublePointer eastWindInterpolator_;

        CubicSplineInterpolatorDoublePointer
            getOneDimSmoothInterpolator( Eigen::MatrixXd dataSpace , Eigen::MatrixXd rowValues );
};

} // namespace_aerodynamics
} // namespace_tudat

#endif // TOOL_HWM07_WINDMODEL_H
