/*    Copyright (c) 2010-2016, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#define BOOST_TEST_MAIN

#include <string>

#include <boost/make_shared.hpp>
#include <boost/test/unit_test.hpp>

#include "Tudat/Mathematics/BasicMathematics/linearAlgebra.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/physicalConstants.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/orbitalElementConversions.h"

#include "Tudat/External/SpiceInterface/spiceInterface.h"
#include "Tudat/Mathematics/NumericalIntegrators/rungeKuttaCoefficients.h"
#include "Tudat/Mathematics/Interpolators/lagrangeInterpolator.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/accelerationModel.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/keplerPropagator.h"
#include "Tudat/InputOutput/basicInputOutput.h"

#include "Tudat/Astrodynamics/BasicAstrodynamics/orbitalElementConversions.h"
#include "Tudat/SimulationSetup/body.h"
#include "Tudat/Astrodynamics/Propagators/nBodyCowellStateDerivative.h"
#include "Tudat/Astrodynamics/Propagators/dynamicsSimulator.h"
#include "Tudat/Mathematics/NumericalIntegrators/createNumericalIntegrator.h"
#include "Tudat/SimulationSetup/createBodies.h"
#include "Tudat/SimulationSetup/createAccelerationModels.h"
#include "Tudat/SimulationSetup/defaultBodies.h"


namespace tudat
{

namespace unit_tests
{


//Using declarations.
using namespace tudat::ephemerides;
using namespace tudat::interpolators;
using namespace tudat::numerical_integrators;
using namespace tudat::spice_interface;
using namespace tudat::simulation_setup;
using namespace tudat::basic_astrodynamics;
using namespace tudat::orbital_element_conversions;
using namespace tudat::propagators;

BOOST_AUTO_TEST_SUITE( test_cowell_propagator )

//! Test to check whether the frame origin transformations are handled properly.
BOOST_AUTO_TEST_CASE( testCowellPopagatorCentralBodies )
{
    //Load spice kernels.
    std::string kernelsPath = input_output::getSpiceKernelPath( );
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "de-403-masses.tpc");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "de421.bsp");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "naif0009.tls");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "pck00009.tpc");

    // Define bodies in simulation.
    unsigned int totalNumberOfBodies = 4;
    std::vector< std::string > bodyNames;
    bodyNames.resize( totalNumberOfBodies );
    bodyNames[ 0 ] = "Earth";
    bodyNames[ 1 ] = "Sun";
    bodyNames[ 2 ] = "Moon";
    bodyNames[ 3 ] = "Mars";


    // Specify initial time
    double initialEphemerisTime = 1.0E7;
    double finalEphemerisTime = 2.0E7;
    double maximumTimeStep = 3600.0;
    double buffer = 5.0 * maximumTimeStep;

    // Create bodies needed in simulation
    std::map< std::string, boost::shared_ptr< BodySettings > > bodySettings =
            getDefaultBodySettings( bodyNames, initialEphemerisTime - buffer, finalEphemerisTime + buffer );
    bodySettings[ "Mars" ]->ephemerisSettings->resetFrameOrigin( "Earth" );
    bodySettings[ "Earth" ]->ephemerisSettings->resetFrameOrigin( "Sun" );
    bodySettings[ "Moon" ]->ephemerisSettings->resetFrameOrigin( "Earth" );


    std::map< std::string, boost::shared_ptr< Body > > bodyMap = createBodies( bodySettings );

    setGlobalFrameBodyEphemerides( bodyMap, "SSB", "ECLIPJ2000" );

    // Set accelerations between bodies that are to be taken into account (mutual point mass gravity between all bodies).
    SelectedAccelerationMap accelerationMap;
    std::map< std::string, std::vector< boost::shared_ptr< AccelerationSettings > > > accelerationsOfEarth;
    accelerationsOfEarth[ "Sun" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfEarth[ "Moon" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfEarth[ "Mars" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationMap[ "Earth" ] = accelerationsOfEarth;

    std::map< std::string, std::vector< boost::shared_ptr< AccelerationSettings > > > accelerationsOfSun;
    accelerationsOfSun[ "Moon" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfSun[ "Earth" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfSun[ "Mars" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationMap[ "Sun" ] = accelerationsOfSun;

    std::map< std::string, std::vector< boost::shared_ptr< AccelerationSettings > > > accelerationsOfMoon;
    accelerationsOfMoon[ "Sun" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfMoon[ "Earth" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfMoon[ "Mars" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationMap[ "Moon" ] = accelerationsOfMoon;

    std::map< std::string, std::vector< boost::shared_ptr< AccelerationSettings > > > accelerationsOfMars;
    accelerationsOfMars[ "Sun" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfMars[ "Earth" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationsOfMars[ "Moon" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationMap[ "Mars" ] = accelerationsOfMars;

    // Define list of bodies to propagate
    std::vector< std::string > bodiesToIntegrate;
    bodiesToIntegrate.push_back( "Earth" );
    bodiesToIntegrate.push_back( "Sun" );
    bodiesToIntegrate.push_back( "Moon" );
    bodiesToIntegrate.push_back( "Mars" );
    unsigned int numberOfNumericalBodies = bodiesToIntegrate.size( );

    // Define numerical integrator settings.
    boost::shared_ptr< IntegratorSettings< > > integratorSettings =
            boost::make_shared< IntegratorSettings< > >
            ( rungeKutta4, initialEphemerisTime, finalEphemerisTime, 240.0 );

    // Define central bodies to use in propagation (all w.r.t SSB).
    std::vector< std::string > centralBodies;
    std::map< std::string, std::string > centralBodyMap;
    centralBodies.resize( numberOfNumericalBodies );
    for( unsigned int i = 0; i < numberOfNumericalBodies; i++ )
    {
        centralBodies[ i ] = "SSB";
        centralBodyMap[ bodiesToIntegrate[ i ] ] = centralBodies[ i ];
    }

    // Get initial state vector as input to integration.
    Eigen::VectorXd systemInitialState = getInitialStatesOfBodies(
                bodiesToIntegrate, centralBodies, bodyMap, initialEphemerisTime );

    // Create acceleration models and propagation settings.
    AccelerationMap accelerationModelMap = createAccelerationModelsMap(
                bodyMap, accelerationMap, centralBodyMap );
    boost::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings =
            boost::make_shared< TranslationalStatePropagatorSettings< double > >
            ( centralBodies, accelerationModelMap, bodiesToIntegrate, systemInitialState );

    // Create simulation object and propagate dynamics.
    SingleArcDynamicsSimulator< > dynamicsSimulator(
                bodyMap, integratorSettings, propagatorSettings, true, false, false );

    // Define new central bodies (hierarchical system)
    centralBodies[ 0 ] = "Sun";
    centralBodies[ 1 ] = "SSB";
    centralBodies[ 2 ] = "Earth";
    centralBodies[ 3 ] = "Sun";
    for( unsigned int i = 0; i < numberOfNumericalBodies; i++ )
    {
        centralBodyMap[ bodiesToIntegrate[ i ] ] = centralBodies[ i ];
    }

    systemInitialState = getInitialStatesOfBodies(
                bodiesToIntegrate, centralBodies, bodyMap, initialEphemerisTime );

    // Create new acceleration models and propagation settings.
    AccelerationMap accelerationModelMap2 = createAccelerationModelsMap(
                bodyMap, accelerationMap, centralBodyMap );
    boost::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings2 =
            boost::make_shared< TranslationalStatePropagatorSettings< double > >
            ( centralBodies, accelerationModelMap2, bodiesToIntegrate, systemInitialState );

    // Create new simulation object and propagate dynamics.
    SingleArcDynamicsSimulator< > dynamicsSimulator2(
                bodyMap, integratorSettings, propagatorSettings2, true, false );

    // Retrieve dynamics solution for the two different central body settings and create interpolators.
    std::map< double, Eigen::VectorXd > solutionSet1 = dynamicsSimulator.getEquationsOfMotionNumericalSolution( );
    std::map< double, Eigen::VectorXd > solutionSet2 = dynamicsSimulator2.getEquationsOfMotionNumericalSolution( );

    LagrangeInterpolator< double, Eigen::VectorXd > interpolator1( solutionSet1, 8 );
    LagrangeInterpolator< double, Eigen::VectorXd > interpolator2( solutionSet2, 8 );

    // Define step size to be out of sync with integration step size.
    double stepSize = 2001.1 + mathematical_constants::PI;
    double currentTime = initialEphemerisTime + stepSize;

    std::map< double, Eigen::VectorXd > analyticalSolutions;

    // Define maps to retrieve propagated orbits from interpolator.
    Eigen::VectorXd currentInertialSolution = Eigen::VectorXd::Zero( 6 * numberOfNumericalBodies );
    Eigen::VectorXd currentNonInertialSolution = Eigen::VectorXd::Zero( 6 * numberOfNumericalBodies );

    // Define map to put inertial orbit reconstructed from non-inertial orbits.
    Eigen::VectorXd reconstructedInertialSolution = Eigen::VectorXd::Zero( 6 * numberOfNumericalBodies );

    // Define error maps.
    Eigen::VectorXd stateDifference = Eigen::VectorXd::Zero( 6 * numberOfNumericalBodies );

    // Test numerical output against results with SSB as origin for ech body,
    boost::shared_ptr< ephemerides::Ephemeris > sunEphemeris = bodyMap[ "Sun" ]->getEphemeris( );
    while( currentTime < finalEphemerisTime - stepSize )
    {
        // Retrieve data from interpolators; transform to inertial frames and compare.
        currentInertialSolution = interpolator1.interpolate( currentTime );
        currentNonInertialSolution = interpolator2.interpolate( currentTime );
        reconstructedInertialSolution.segment( 0, 6 ) = currentNonInertialSolution.segment( 0, 6 ) +
                sunEphemeris->getCartesianStateFromEphemeris( currentTime );
        reconstructedInertialSolution.segment( 6, 6 ) = currentNonInertialSolution.segment( 6, 6 );
        reconstructedInertialSolution.segment( 12, 6 )= currentNonInertialSolution.segment( 12, 6 ) +
                reconstructedInertialSolution.segment( 0, 6 );
        reconstructedInertialSolution.segment( 18, 6 ) = currentNonInertialSolution.segment( 18, 6 ) +
                sunEphemeris->getCartesianStateFromEphemeris( currentTime );

        // Compare states.
        stateDifference = reconstructedInertialSolution - currentInertialSolution;
        for( unsigned j = 0; j < 4; j++ )
        {
            if( j != 2 )
            {
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 0 + 6 * j ) ), 1.0E-2 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 1 + 6 * j ) ), 1.0E-2 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 2 + 6 * j ) ), 1.0E-2 );

                BOOST_CHECK_SMALL( std::fabs( stateDifference( 3 + 6 * j ) ), 2.0E-9 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 4 + 6 * j ) ), 2.0E-9 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 5 + 6 * j ) ), 2.0E-9 );
            }
            else
            {
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 0 + 6 * j ) ), 1.0E-1 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 1 + 6 * j ) ), 1.0E-1 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 2 + 6 * j ) ), 1.0E-1 );

                BOOST_CHECK_SMALL( std::fabs( stateDifference( 3 + 6 * j ) ), 2.0E-7 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 4 + 6 * j ) ), 2.0E-7 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 5 + 6 * j ) ), 2.0E-7 );
            }
        }
        currentTime += stepSize;
    }

    // Test whether ephemeris objects have been properly reset, i.e. whether all states have been properly transformed to the
    // ephemeris frame.
    boost::shared_ptr< ephemerides::Ephemeris > earthEphemeris = bodyMap[ "Earth" ]->getEphemeris( );
    boost::shared_ptr< ephemerides::Ephemeris > marsEphemeris = bodyMap[ "Mars" ]->getEphemeris( );
    boost::shared_ptr< ephemerides::Ephemeris > moonEphemeris = bodyMap[ "Moon" ]->getEphemeris( );

    while( currentTime < finalEphemerisTime - stepSize )
    {
        // Retrieve data from interpolators; transform to inertial frames and compare.
        currentInertialSolution = interpolator1.interpolate( currentTime );

        reconstructedInertialSolution.segment( 0, 6 ) = earthEphemeris->getCartesianStateFromEphemeris( currentTime ) +
                sunEphemeris->getCartesianStateFromEphemeris( currentTime );
        reconstructedInertialSolution.segment( 6, 6 ) = sunEphemeris->getCartesianStateFromEphemeris( currentTime );
        reconstructedInertialSolution.segment( 12, 6 ) = moonEphemeris->getCartesianStateFromEphemeris( currentTime ) +
                earthEphemeris->getCartesianStateFromEphemeris( currentTime ) +
                sunEphemeris->getCartesianStateFromEphemeris( currentTime );
        reconstructedInertialSolution.segment( 18, 6 ) = marsEphemeris->getCartesianStateFromEphemeris( currentTime ) +
                sunEphemeris->getCartesianStateFromEphemeris( currentTime );

        // Compare states.
        stateDifference = reconstructedInertialSolution - currentInertialSolution;
        for( unsigned j = 0; j < 4; j++ )
        {
            if( j != 2 )
            {
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 0 + 6 * j ) ), 1.0E-2 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 1 + 6 * j ) ), 1.0E-2 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 2 + 6 * j ) ), 1.0E-2 );

                BOOST_CHECK_SMALL( std::fabs( stateDifference( 3 + 6 * j ) ), 2.0E-9 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 4 + 6 * j ) ), 2.0E-9 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 5 + 6 * j ) ), 2.0E-9 );
            }
            else
            {
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 0 + 6 * j ) ), 1.0E-1 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 1 + 6 * j ) ), 1.0E-1 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 2 + 6 * j ) ), 1.0E-1 );

                BOOST_CHECK_SMALL( std::fabs( stateDifference( 3 + 6 * j ) ), 2.0E-7 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 4 + 6 * j ) ), 2.0E-7 );
                BOOST_CHECK_SMALL( std::fabs( stateDifference( 5 + 6 * j ) ), 2.0E-7 );
            }
        }

        currentTime += stepSize;
    }

}

//! Test to ensure that a point-mass acceleration on a body produces a Kepler orbit (to within numerical error bounds).
BOOST_AUTO_TEST_CASE( testCowellPopagatorKeplerCompare )
{
    //Load spice kernels.
    std::string kernelsPath = input_output::getSpiceKernelPath( );

    spice_interface::loadSpiceKernelInTudat( kernelsPath + "de-403-masses.tpc");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "de421.bsp");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "naif0009.tls");
    spice_interface::loadSpiceKernelInTudat( kernelsPath + "pck00009.tpc");

    // Define bodies in simulation.
    std::vector< std::string > bodyNames;
    bodyNames.push_back( "Earth" );
    bodyNames.push_back( "Moon" );

    // Specify initial time
    double initialEphemerisTime = 1.0E7;
    double finalEphemerisTime = 2.0E7;
    double maximumTimeStep = 3600.0;
    double buffer = 5.0 * maximumTimeStep;

    // Create bodies needed in simulation
    std::map< std::string, boost::shared_ptr< BodySettings > > bodySettings =
            getDefaultBodySettings( bodyNames, initialEphemerisTime - buffer, finalEphemerisTime + buffer );

    // Change ephemeris settings of Moon and Earth to make test results analysis more transparent.
    boost::dynamic_pointer_cast< InterpolatedSpiceEphemerisSettings >( bodySettings[ "Moon" ]->ephemerisSettings )->
            resetFrameOrigin( "Earth" );
    bodySettings[ "Earth" ]->ephemerisSettings = boost::make_shared< ConstantEphemerisSettings >(
                basic_mathematics::Vector6d::Zero( ), "SSB", "ECLIPJ2000" );

    std::map< std::string, boost::shared_ptr< Body > > bodyMap = createBodies( bodySettings );
    setGlobalFrameBodyEphemerides( bodyMap, "SSB", "ECLIPJ2000" );

    // Set accelerations between bodies that are to be taken into account.
    SelectedAccelerationMap accelerationMap;
    std::map< std::string, std::vector< boost::shared_ptr< AccelerationSettings > > > accelerationsOfMoon;
    accelerationsOfMoon[ "Earth" ].push_back( boost::make_shared< AccelerationSettings >( central_gravity ) );
    accelerationMap[ "Moon" ] = accelerationsOfMoon;

    // Propagate the moon only
    std::vector< std::string > bodiesToIntegrate;
    bodiesToIntegrate.push_back( "Moon" );
    unsigned int numberOfNumericalBodies = bodiesToIntegrate.size( );

    // Define settings for numerical integrator.
    boost::shared_ptr< IntegratorSettings< > > integratorSettings =
            boost::make_shared< IntegratorSettings< > >
            ( rungeKutta4, initialEphemerisTime, finalEphemerisTime, 120.0 );

    // Run test where Moon gravity is/is not taken into account.
    for( unsigned testCase = 0; testCase < 2; testCase++ )
    {
        // Get initial kepler elements
        double effectiveGravitationalParameter;
        if( testCase == 0 )
        {
            effectiveGravitationalParameter =
                    bodyMap.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( ) +
                    bodyMap.at( "Moon" )->getGravityFieldModel( )->getGravitationalParameter( );
        }
        else
        {
            effectiveGravitationalParameter =
                    bodyMap.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( );
        }

        // Define central bodies for integration.
        std::vector< std::string > centralBodies;
        std::map< std::string, std::string > centralBodyMap;

        if( testCase == 0 )
        {
            effectiveGravitationalParameter =
                    bodyMap.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( ) +
                    bodyMap.at( "Moon" )->getGravityFieldModel( )->getGravitationalParameter( );
            centralBodies.push_back( "Earth" );

        }
        else
        {
            effectiveGravitationalParameter =
                    bodyMap.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( );
            centralBodies.push_back( "SSB" );
        }
        centralBodyMap[ bodiesToIntegrate[ 0 ] ] = centralBodies[ 0 ];


        // Create system initial state.
        Eigen::VectorXd systemInitialState = Eigen::VectorXd( bodiesToIntegrate.size( ) * 6 );
        for( unsigned int i = 0; i < numberOfNumericalBodies ; i++ )
        {
            systemInitialState.segment( i * 6 , 6 ) =
                    spice_interface::getBodyCartesianStateAtEpoch(
                        bodiesToIntegrate[ i ], "Earth", "ECLIPJ2000", "NONE", initialEphemerisTime );
        }

        // Create acceleration models and propagation settings.
        AccelerationMap accelerationModelMap = createAccelerationModelsMap(
                    bodyMap, accelerationMap, centralBodyMap );
        boost::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings =
                boost::make_shared< TranslationalStatePropagatorSettings< double > >
                ( centralBodies, accelerationModelMap, bodiesToIntegrate, systemInitialState );

        // Create dynamics simulation object.
        SingleArcDynamicsSimulator< > dynamicsSimulator(
                    bodyMap, integratorSettings, propagatorSettings, true, false );

        basic_mathematics::Vector6d initialKeplerElements = orbital_element_conversions::convertCartesianToKeplerianElements(
                    basic_mathematics::Vector6d( systemInitialState ), effectiveGravitationalParameter );

        // Compare numerical state and kepler orbit at each time step.
        boost::shared_ptr< Ephemeris > moonEphemeris = bodyMap.at( "Moon" )->getEphemeris( );
        double currentTime = initialEphemerisTime + buffer;
        while( currentTime < finalEphemerisTime - buffer )
        {
            basic_mathematics::Vector6d stateDifference = orbital_element_conversions::convertKeplerianToCartesianElements(
                        propagateKeplerOrbit( initialKeplerElements, currentTime - initialEphemerisTime,
                                              effectiveGravitationalParameter ),
                        effectiveGravitationalParameter ) - moonEphemeris->getCartesianStateFromEphemeris( currentTime );
            for( int i = 0; i < 3; i++ )
            {
                BOOST_CHECK_SMALL( stateDifference( i ), 1E-3 );
                BOOST_CHECK_SMALL( stateDifference( i  + 3 ), 1.0E-9 );

            }
            currentTime += 10000.0;
        }
    }
}

BOOST_AUTO_TEST_SUITE_END( )


}

}
