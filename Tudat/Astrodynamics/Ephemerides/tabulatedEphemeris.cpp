#include "Tudat/Astrodynamics/Ephemerides/tabulatedEphemeris.h"

namespace tudat
{

namespace ephemerides
{

//! Get cartesian state from ephemeris (in double precision), for double class StateScalarType
template< >
basic_mathematics::Vector6d TabulatedCartesianEphemeris< double, double >::getCartesianStateFromEphemeris(
        const double ephemerisTime, const double julianDayAtEpoch )
{
    if( julianDayAtEpoch != julianDayAtEpoch_ )
    {
        throw std::runtime_error(
                    "Error in Tabulated Ephemeris, reference epochs are inconsistent" );
    }

    return interpolator_->interpolate( ephemerisTime );
}

//! Get cartesian state from ephemeris (in long double precision), for double class StateScalarType
template< >
Eigen::Matrix< long double, 6, 1 > TabulatedCartesianEphemeris< double, double >::getCartesianLongStateFromEphemeris(
        const double secondsSinceEpoch, const double julianDayAtEpoch )
{
    if( julianDayAtEpoch != julianDayAtEpoch_ )
    {
        throw std::runtime_error(
                    "Error in Tabulated Ephemeris, reference epochs are inconsistent" );
    }

    return interpolator_->interpolate( secondsSinceEpoch ).cast< long double >( );
}


//! Get cartesian state from ephemeris (in double precision), for long double class StateScalarType
template< >
basic_mathematics::Vector6d TabulatedCartesianEphemeris< long double, double >::getCartesianStateFromEphemeris(
        const double ephemerisTime, const double julianDayAtEpoch )
{
    if( julianDayAtEpoch != julianDayAtEpoch_ )
    {
        throw std::runtime_error(
                    "Error in Tabulated Ephemeris, reference epochs are inconsistent" );
    }

    return interpolator_->interpolate( ephemerisTime ).cast< double >( );
}

//! Get cartesian state from ephemeris (in long double precision), for long double class StateScalarType
template< >
Eigen::Matrix< long double, 6, 1 > TabulatedCartesianEphemeris< long double, double >::getCartesianLongStateFromEphemeris(
        const double secondsSinceEpoch, const double julianDayAtEpoch )
{
    if( julianDayAtEpoch != julianDayAtEpoch_ )
    {
        throw std::runtime_error(
                    "Error in Tabulated Ephemeris, reference epochs are inconsistent" );
    }

    return interpolator_->interpolate( secondsSinceEpoch );
}


}

}

