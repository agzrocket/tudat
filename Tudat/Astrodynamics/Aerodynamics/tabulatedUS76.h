/*    Copyright (c) 2010-2015, Delft University of Technology
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without modification, are
 *    permitted provided that the following conditions are met:
 *      - Redistributions of source code must retain the above copyright notice, this list of
 *        conditions and the following disclaimer.
 *      - Redistributions in binary form must reproduce the above copyright notice, this list of
 *        conditions and the following disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *      - Neither the name of the Delft University of Technology nor the names of its contributors
 *        may be used to endorse or promote products derived from this software without specific
 *        prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *    OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *    Changelog
 *      YYMMDD    Author            Comment
 *      110620    F.M. Engelen      File created.
 *      110721    J. Melman         Comments, file names, and consistency modified.
 *      130120    K. Kumar          Made function calls const-correct; added shared-pointer
 *                                  typedef.
 *
 *    References
 *
 *    Notes
 *      The provided USSA1976 table file, generated with the pascal file, has a small error which
 *      can be observed at the pressure at sea level. This in 101320 in the file but should be
 *      101325. If this error is not acceptable, another table file should be used.
 *
 */

#ifndef TUDAT_TABULATEDUS76_ATMOSPHERE_H
#define TUDAT_TABULATEDUS76_ATMOSPHERE_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>

#include "Tudat/Basics/utilityMacros.h"

#include "Tudat/Astrodynamics/Aerodynamics/standardAtmosphere.h"
#include "Tudat/Astrodynamics/Aerodynamics/aerodynamics.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/physicalConstants.h"
#include "Tudat/Mathematics/Interpolators/cubicSplineInterpolator.h"
#include "Tudat/Mathematics/Interpolators/linearInterpolator.h"

namespace tudat
{
namespace aerodynamics
{

//! Tabulated atmosphere class.
/*!
 * Tabulated atmospheres class, for example US1976. The default path from which the files are
 * obtained is: /External/AtmosphereTables
 * NOTE: for the moment it only works for tables with 4 columns: altitude, density, pressure and
 * temperature.
 */
class TabulatedUS76 : public StandardAtmosphere
{
public:

    //! Default constructor.
    /*!
     *  Default constructor.
     *  \param atmosphereTableFile File containing atmospheric properties.
     *  The file name of the atmosphere table. The file should contain four columns of data,
     *  containing altitude (first column), and the associated density, pressure and density values
     *  in the second, third and fourth columns
     *  \param specificGasConstant The constant specific gas constant of the air
     *  \param ratioOfSpecificHeats The constant ratio of specific heats of the air
     */
    TabulatedUS76( const std::string& atmosphereTableFile )
        : atmosphereTableFile_( atmosphereTableFile )
    {
        initialize( atmosphereTableFile_ );
    }

    //! Get atmosphere table file name.
    /*!
     * Returns atmosphere table file name.
     * \return The atmosphere table file.
     */
    std::string getAtmosphereTableFile( ) { return atmosphereTableFile_; }

    //! Get local density.
    /*!
     * Returns the local density parameter of the atmosphere in kg per meter^3.
     * \param altitude Altitude at which density is to be computed.
     * \param longitude Longitude at which density is to be computed (not used but included for
     * consistency with base class interface).
     * \param latitude Latitude at which density is to be computed (not used but included for
     * consistency with base class interface).
     * \param time Time at which density is to be computed (not used but included for
     * consistency with base class interface).
     * \return Atmospheric density at specified altitude.
     */
    double getDensity( const double altitude, const double longitude = 0.0,
                       const double latitude = 0.0, const double time = 0.0 )
    {
        TUDAT_UNUSED_PARAMETER( longitude );
        TUDAT_UNUSED_PARAMETER( latitude );
        TUDAT_UNUSED_PARAMETER( time );
        return cubicSplineInterpolationForDensity_->interpolate( altitude );
    }

    //! Get local pressure.
    /*!
     * Returns the local pressure of the atmosphere in Newton per meter^2.
     * \param altitude Altitude  at which pressure is to be computed.
     * \param longitude Longitude at which pressure is to be computed (not used but included for
     * consistency with base class interface).
     * \param latitude Latitude at which pressure is to be computed (not used but included for
     * consistency with base class interface).
     * \param time Time at which pressure is to be computed (not used but included for
     * consistency with base class interface).
     * \return Atmospheric pressure at specified altitude.
     */
    double getPressure( const double altitude, const double longitude = 0.0,
                        const double latitude = 0.0, const double time = 0.0 )
    {
        TUDAT_UNUSED_PARAMETER( longitude );
        TUDAT_UNUSED_PARAMETER( latitude );
        TUDAT_UNUSED_PARAMETER( time );
        return cubicSplineInterpolationForPressure_->interpolate( altitude );
    }

    //! Get local temperature.
    /*!
     * Returns the local temperature of the atmosphere in Kelvin.
     * \param altitude Altitude at which temperature is to be computed
     * \param longitude Longitude at which temperature is to be computed (not used but included for
     * consistency with base class interface).
     * \param latitude Latitude at which temperature is to be computed (not used but included for
     * consistency with base class interface).
     * \param time Time at which temperature is to be computed (not used but included for
     * consistency with base class interface).
     * \return constantTemperature Atmospheric temperature at specified altitude.
     */
    double getTemperature( const double altitude, const double longitude = 0.0,
                           const double latitude = 0.0, const double time = 0.0 )
    {
        TUDAT_UNUSED_PARAMETER( longitude );
        TUDAT_UNUSED_PARAMETER( latitude );
        TUDAT_UNUSED_PARAMETER( time );
        return cubicSplineInterpolationForTemperature_->interpolate( altitude );
    }

    //! Get local speed of sound in the atmosphere.
    /*!
     * Returns the speed of sound in the atmosphere in m/s.
     * \param altitude Altitude at which speed of sound is to be computed.
     * \param longitude Longitude at which speed of sound is to be computed (not used but included
     * for consistency with base class interface).
     * \param latitude Latitude at which speed of sound is to be computed (not used but included
     * for consistency with base class interface).
     * \param time Time at which speed of sound is to be computed (not used but included for
     * consistency with base class interface).
     * \return Atmospheric speed of sound at specified altitude.
     */
    double getSpeedOfSound( const double altitude, const double longitude = 0.0,
                            const double latitude = 0.0, const double time = 0.0 )
    {
        TUDAT_UNUSED_PARAMETER( longitude );
        TUDAT_UNUSED_PARAMETER( latitude );
        TUDAT_UNUSED_PARAMETER( time );
        return cubicSplineInterpolationForSpeedOfSound_->interpolate( altitude );

    }

protected:

private:

    //! Initialize atmosphere table reader.
    /*!
     * Initializes the atmosphere table reader.
     * \param atmosphereTableFile The name of the atmosphere table.
     */
    void initialize( const std::string& atmosphereTableFile );

    //! The file name of the atmosphere table.
    /*!
     *  The file name of the atmosphere table. The file should contain four columns of data,
     *  containing altitude (first column), and the associated density, pressure and density values
     *  in the second, third and fourth columns.
     */
    std::string atmosphereTableFile_;

    //! Vector containing the altitude.
    /*!
     *  Vector containing the altitude.
     */
    std::vector< double > altitudeData_;

    //! Vector containing the density data as a function of the altitude.
    /*!
     *  Vector containing the density data as a function of the altitude.
     */
    std::vector< double > densityData_;

    //! Vector containing the pressure data as a function of the altitude.
    /*!
     *  Vector containing the pressure data as a function of the altitude.
     */
    std::vector< double > pressureData_;

    //! Vector containing the temperature data as a function of the altitude.
    /*!
     *  Vector containing the temperature data as a function of the altitude.
     */
    std::vector< double > temperatureData_;

    //! Vector containing the speed of sound data as a function of the altitude.
    /*!
     *  Vector containing the speed of sound data as a function of the altitude.
     */
    std::vector< double > speedOfSoundData_;

    interpolators::CubicSplineInterpolatorDoublePointer cubicSplineInterpolationForDensity_;

    interpolators::CubicSplineInterpolatorDoublePointer cubicSplineInterpolationForPressure_;

    interpolators::CubicSplineInterpolatorDoublePointer cubicSplineInterpolationForTemperature_;

    interpolators::CubicSplineInterpolatorDoublePointer cubicSplineInterpolationForSpeedOfSound_;


//    interpolators::LinearInterpolatorDoublePointer cubicSplineInterpolationForDensity_;

//    interpolators::LinearInterpolatorDoublePointer cubicSplineInterpolationForPressure_;

//    interpolators::LinearInterpolatorDoublePointer cubicSplineInterpolationForTemperature_;

//    interpolators::LinearInterpolatorDoublePointer cubicSplineInterpolationForSpeedOfSound_;
};

//! Typedef for shared-pointer to TabulatedAtmosphere object.
typedef boost::shared_ptr< TabulatedUS76 > TabulatedUS76Pointer;

} // namespace aerodynamics
} // namespace tudat

#endif // TUDAT_TABULATED_ATMOSPHERE_H
