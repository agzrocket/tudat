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
 *      102511    D. Dirkx          First version of file.
 *      110501    D. Dirkx          Added more comments.
 *      110203    L. Abdulkadir     Code check.
 *      110211    K. Kumar          Corrected Doxygen errors; corrected layout errors; corrected
 *                                  function-naming.
 *
 *    References
 *      Anderson Jr., J.D. , Fundamentals of Aerodynamics, 3rd edition, McGraw Hill, 2001.
 *      Gentry, A., Smyth, D., and Oliver, W. . The Mark IV Supersonic-Hypersonic Arbitrary Body
 *          Program, Volume II - Program Formulation, Douglas Aircraft Company, 1973.
 *      Anderson Jr. , J.D, Hypersonic and High-Temperature Gas Dynamics, 2nd edition,
 *          AIAA Education Series, 2006.
 *
 *    Notes
 *
 */

#ifndef TUDAT_AERODYNAMICS_H
#define TUDAT_AERODYNAMICS_H

#include <boost/function.hpp>

#include <Eigen/Core>

#include <cmath>

#include "Tudat/Mathematics/BasicMathematics/mathematicalConstants.h"
#include "Tudat/Astrodynamics/Aerodynamics/flightConditions.h"
#include "Tudat/Astrodynamics/SystemModels/vehicleSystems.h"

namespace tudat
{
namespace aerodynamics
{

//! Maximum Prandtl-Meyer function value.
/*!
 * Maximum Prandtl-Meyer function value for ratio of specific heats = 1.4.
 */
static const double maximumPrandtlMeyerFunctionValue = 
    mathematical_constants::PI / 2.0 * ( std::sqrt( 6.0 ) - 1.0 );

//! Constant for use in inverse Prandtl-Meyer function calculation.
/*!
 * Constant for use in inverse Prandtl-Meyer function calculation for ratio of
 *  specific heats = 1.4.
 */
static const double PrandtlMeyerParameter1 = 1.3604;

//! Constant for use in inverse Prandtl-Meyer function calculation.
/*!
 * Constant for use in inverse Prandtl-Meyer function calculation for ratio of
 * specific heats = 1.4.
 */
static const double PrandtlMeyerParameter2 = 0.0962;

//! Constant for use in inverse Prandtl-Meyer function calculation.
/*!
 * Constant for use in inverse Prandtl-Meyer function calculation for ratio of
 * specific heats = 1.4.
 */
static const double PrandtlMeyerParameter3 = -0.5127;

//! Constant for use in inverse Prandtl-Meyer function calculation.
/*!
 * Constant for use in inverse Prandtl-Meyer function calculation for ratio of
 * specific heats = 1.4.
 */
static const double PrandtlMeyerParameter4 = -0.6722;

//! Constant for use in inverse Prandtl-Meyer function calculation.
/*!
 * Constant for use in inverse Prandtl-Meyer function calculation for ratio of
 * specific heats = 1.4.
 */
static const double PrandtlMeyerParameter5 = -0.3278;

//! Compute local-to-static pressure ratio.
/*!
 * Computes the local to static pressure ratio, assuming a thermally and
 * calorically perfect gas.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \return Local-to-static pressure ratio.
 */
double computeLocalToStaticPressureRatio( double machNumber,
                                          double ratioOfSpecificHeats );

//! Compute Prandtl-Meyer function.
/*!
 * Computes the value of the Prandtl-Meyer function at the given Mach number
 * and ratio of specific heat.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \return Prandtl-Meyer function value.
 */
double computePrandtlMeyerFunction( double machNumber, double ratioOfSpecificHeats );

//! Compute stagnation pressure coefficient in supersonic flow.
/*!
 * Computes the stagnation pressure coefficient, assuming a thermally and
 * calorically perfect gas.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \return Stagnation pressure coefficient.
 */
double computeStagnationPressure( double machNumber, double ratioOfSpecificHeats );

//! Compute pressure coefficient based on Newtonian theory.
/*!
 * Computes the pressure coefficient based on Newtonian theory.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \return Newtonian pressure coefficient.
 */
double computeNewtonianPressureCoefficient( double inclinationAngle );

//! Compute pressure coefficient based on modified Newtonian theory.
/*!
 * Computes the pressure coefficient based on modified Newtonian theory.
 * \param inclinationAngle Angle between the wall and the freestream
 *         velocity vector.
 * \param stagnationPressureCoefficient Stagnation pressure coefficient.
 * \return Newtonian pressure coefficient.
 */
double computeModifiedNewtonianPressureCoefficient(
        double inclinationAngle, double stagnationPressureCoefficient );

//! Compute pressure coefficient using empirical tangent wedge method.
/*!
 * Computes tangent wedge pressure coefficient based on empirical correlation
 * for ratio of specific heats = 1.4 ( terrestrial atmosphere).
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return Empirical tangent wedge pressure coefficient.
 */
double computeEmpiricalTangentWedgePressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute pressure coefficient using empirical tangent cone method.
/*!
 * Computes tangent cone pressure coefficient based on empirical correlation
 * for ratio of specific heats = 1.4 ( terrestrial atmosphere).
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return Empirical tangent wedge pressure coefficient.
 */
double computeEmpiricalTangentConePressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute pressure coefficient using modified Dahlem-Buck method.
/*!
 * Computes tangent cone pressure coefficient based on Dahlem-Buck empirical
 * method.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return Dahlem-Buck pressure coefficient.
 */
double computeModifiedDahlemBuckPressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute pressure coefficient using the Hankey flat surface method.
/*!
 * Computes tangent cone pressure coefficient based on the Hankey flat surface
 * method.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return Hankey Flat surface pressure coefficient.
 */
double computeHankeyFlatSurfacePressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute pressure coefficient using the Smyth delta wing method.
/*!
 * Computes tangent cone pressure coefficient based on the Smyth delta wing
 * surface method.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return Smyth delta wing pressure coefficient.
 */
double computeSmythDeltaWingPressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute pressure coefficient using the van Dyke unified method.
/*!
 * Computes tangent cone pressure coefficient based on the van Dyke unified
 * method.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \param type ( expansion ( 1 ) or compression( -1 ) ).
 * \return Hankey Flat surface pressure coefficient.
 */
double computeVanDykeUnifiedPressureCoefficient(
        double inclinationAngle, double machNumber,
        double ratioOfSpecificHeats, int type );

//! Compute pressure coefficient using Prandtl-Meyer expansion.
/*!
 * Computes pressure coefficient using Prandtl-Meyer expansion from
 * freestream. Currently only terrestrial atmosphere
 * ( ratio of specific heat = 1.4 ) is supported due to the use of an empirical
 * fit for the inverse Prandtl-Meyer function determination.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \param freestreamPrandtlMeyerFunction Freestream Prandtl-Meyer function.
 * \return Prandtl-Meyer pressure coefficient.
 */
double computePrandtlMeyerFreestreamPressureCoefficient(
        double inclinationAngle,  double machNumber,
        double ratioOfSpecificHeats, double freestreamPrandtlMeyerFunction );

//! Compute pressure coefficient at vacuum.
/*!
 * Computes the pressure coefficient at vacuum assuming a thermally and
 * calorically perfect gas.
 * \param machNumber Flow Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \return Vacuum pressure coefficient.
 */
double computeVacuumPressureCoefficient(
        double machNumber, double ratioOfSpecificHeats );

//! Compute high Mach base pressure coefficient.
/*!
 * This function calculates the high Mach base pressure coefficient
 * approximation.
 * \param machNumber Flow Mach number.
 * \return Vacuum pressure coefficient.
 */
double computeHighMachBasePressure( double machNumber );

//! Compute pressure coefficient using the ACM empirical method.
/*! Computes tangent cone pressure coefficient based on the ACM empirical
 * method.
 * \param inclinationAngle Angle between wall and freestream velocity vector.
 * \param machNumber Flow Mach number.
 * \return ACM empirical surface pressure coefficient.
 */
double computeAcmEmpiricalPressureCoefficient(
        double inclinationAngle, double machNumber );

//! Compute Mach number from Prandtl-Meyer function.
/*!
 * Computes the inverse of the Prandtl-Meyer function. Currently the function
 * is limited to use with ratio of specific heats of 1.4 as it uses an
 * empirical correlation.
 * \param prandtlMeyerFunctionValue Prandyl-Meyer function value.
 * \return Mach number.
 */
double computeInversePrandtlMeyerFunction( double prandtlMeyerFunctionValue );

//! Compute ratio of post- to pre-shock pressure.
/*!
 * Computes ratio of post- to pre-shock pressure, assuming thermally and calorically perfect gas.
 * \param normalMachNumber Mach number of flow velocity normal to shock.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *          specific heat at constant volume.
 */
double computeShockPressureRatio( double normalMachNumber,
                                  double ratioOfSpecificHeats );

//! Compute ratio of post- to pre-shock density.
/*!
 * Compute ratio of post- to pre-shock density, assuming thermally and calorically perfect gas.
 * \param normalMachNumber Mach number of flow velocity normal to shock.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 */
double computeShockDensityRatio( double normalMachNumber,
                                 double ratioOfSpecificHeats );

//! Compute ratio of post- to pre-shock temperature.
/*!
 * Computes ratio of post- to pre-shock temperature, assuming thermally and calorically perfect
 * gas.
 * \param normalMachNumber Mach number of flow velocity normal to shock.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 */
double computeShockTemperatureRatio( double normalMachNumber,
                                     double ratioOfSpecificHeats );

//! Compute jump in entropy across a shock wave.
/*!
 * Compute jump in entropy across a shock wave, assuming thermally and calorically perfect gas.
 * \param normalMachNumber Mach number of flow velocity normal to shock.
 * \param ratioOfSpecificHeats ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \param specificGasConstant gas constant per unit mass for flow composition.
 */
double computeShockEntropyJump( double normalMachNumber, double ratioOfSpecificHeats,
                                double specificGasConstant );

//! Compute post- to pre-shock total pressure ratio.
/*!
 * Compute post- to pre-shock total pressure ratio from the entropy jump across
 * a shock wave. Assumption of thermally and calorically perfect gas is made.
 * \param normalMachNumber Mach number of flow velocity normal to shock.
 * \param ratioOfSpecificHeats ratio of specific heat at constant pressure to
 *         specific heat at constant volume.
 * \param specificGasConstant gas constant per unit mass for flow composition.
 */
double computeShockTotalPressureRatio( double normalMachNumber,
                                       double ratioOfSpecificHeats,
                                       double specificGasConstant );

//! Compute shock deflection angle.
/*!
 * Computes the flow deflection angle across a shock wave.
 * \param shockAngle Angle of shock wave w.r.t. freestream flow.
 * \param machNumber Freestream Mach number.
 * \param ratioOfSpecificHeats Ratio of specific heat at constant pressure to
 *          specific heat at constant volume.
 */
double computeShockDeflectionAngle( double shockAngle, double machNumber,
                                    double ratioOfSpecificHeats );

//! Function to compute the speed of sound in a gas
/*!
 * Function to compute the speed of sound in a gas
 * \param temperature Gas temperature
 * \param ratioOfSpecificHeats Ratio of specific heats aat constant pressure and constant volume
 * \param specificGasConstant Specific gas constant of the gas
 * \return Speed of sound in the gas.
 */
double computeSpeedOfSound( const double temperature, const double ratioOfSpecificHeats,
                            const double specificGasConstant );

//! Compute Mach number
/*!
 * Compute Mach number
 * \param speed Airspeed of object for which Mach number is to be computed.
 * \param speedOfSound Speed of sound for atmosphere position at which Mach number is to be computed.
 * \return Mach number
 */
double computeMachNumber( const double speed, const double speedOfSound );

//! Function to compute the mean free path of a particle.
/*!
 * Function to compute the mean free path of a particle from e.g. (Chapman, S. & Cowling, T. The mathematical theory of
 * nonuniform gases Cambridge University Press, 1970)
 * \param weightedAverageCollisionDiameter Weighted (using specie number density) average collision diameter of the
 * particles in the gas.
 * \param averageNumberDensity Average number density of the gas.
 * \return Mean free path of a particle in the gas.
 */
double computeMeanFreePath( const double weightedAverageCollisionDiameter, const double averageNumberDensity );

//! Function to compute the aerodynamic load experienced by a vehicle.
/*!
 * Function that computes the aerodynamic load (a.k.a. load factor) experienced by a vehicle.
 * \param airDensity Freestream air density.
 * \param airSpeed Airspeed of the vehicle.
 * \param referenceArea Reference area of the vehicle.
 * \param vehicleMass Mass of the vehicle.
 * \param aerodynamicForceCoefficients Aerodynamic force coefficients of the vehicle.
 * \return Aerodynamic load experienced by the vehicle.
 */
double computeAerodynamicLoad( const double airDensity,
                               const double airSpeed,
                               const double referenceArea,
                               const double vehicleMass,
                               const Eigen::Vector3d& aerodynamicForceCoefficients );

//! Function to compute the aerodynamic load experienced by a vehicle.
/*!
 * Function that computes the aerodynamic load (a.k.a. load factor) experienced by a vehicle.
 * \return Aerodynamic load experienced by the vehicle.
 */
double computeAerodynamicLoadFromAcceleration( const Eigen::Vector3d& aerodynamicAccelerationVector );

double computeEquilibriumHeatflux( const boost::function< double( const double ) > heatTransferFunction,
                                   const double wallEmmisivity,
                                   const double adiabaticWallTemperature );

double computeEquilibriumFayRiddellHeatFlux( const double airDensity,
                                             const double airSpeed,
                                             const double airTemperature,
                                             const double machNumber,
                                             const double noseRadius,
                                             const double wallEmissivity = 0.80 );

//! Function to compute the heat flux experienced by a vehicle.
/*!
 * Function that computes the heat flux experienced by a vehicle. This function is an implementation of the
 * Fay-Riddell formula.
 * \param airSpeed Airspeed of the vehicle.
 * \param airTemperature Freestream air temperature.
 * \param machNumber Freestream Mach number.
 * \param noseRadius Nose radius of the vehicle.
 * \param wallEmissivity Wall emissivity of the vehicle.
 * \return Heat flux experienced by the vehicle.
 */
double computeFayRiddellHeatFlux( const double airDensity,
                                  const double airSpeed,
                                  const double airTemperature,
                                  const double noseRadius,
                                  const double wallTemperature );

//! Function to compute the adiabatic wall temperature experienced by a vehicle.
/*!
 * Function that computes the adiabatic wall temperature experienced by a vehicle.
 * \param airTemperature Freestream air temperature.
 * \param machNumber Freestream Mach number.
 * \return Adiabatic wall temperature experienced by the vehicle.
 */
double computeAdiabaticWallTemperature(
        const double airTemperature, const double machNumber, const double ratioSpecificHeats = 1.4,
        const double recoveryFactor = 0.845 );

} // namespace aerodynamics
} // namespace tudat

#endif // TUDAT_AERODYNAMICS_H
