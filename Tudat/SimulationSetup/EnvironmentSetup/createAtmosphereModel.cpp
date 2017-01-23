/*    Copyright (c) 2010-2016, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

#include "Tudat/Astrodynamics/Aerodynamics/exponentialAtmosphere.h"
#include "Tudat/Astrodynamics/Aerodynamics/tabulatedAtmosphere.h"
#if USE_NRLMSISE00
#include "Tudat/Astrodynamics/Aerodynamics/nrlmsise00Atmosphere.h"
#include "Tudat/Astrodynamics/Aerodynamics/nrlmsise00InputFunctions.h"
#endif
#include "Tudat/InputOutput/basicInputOutput.h"
#include "Tudat/InputOutput/solarActivityData.h"
#include "Tudat/SimulationSetup/EnvironmentSetup/createAtmosphereModel.h"

#include "Tudat/Astrodynamics/Aerodynamics/tabulatedUS76.h"
#include "Tudat/Astrodynamics/Aerodynamics/tabulatedDispersedUS76.h"

namespace tudat
{

namespace simulation_setup
{

//! Function to create an atmosphere model.
boost::shared_ptr< aerodynamics::AtmosphereModel > createAtmosphereModel(
        const boost::shared_ptr< AtmosphereSettings > atmosphereSettings,
        const std::string& body )
{
    using namespace tudat::aerodynamics;

    // Declare return object.
    boost::shared_ptr< AtmosphereModel > atmosphereModel;

    // Check which type of atmosphere model is to be created.
    switch( atmosphereSettings->getAtmosphereType( ) )
    {
    case exponential_atmosphere:
    {
        // Check whether settings for atmosphere are consistent with its type.
        boost::shared_ptr< ExponentialAtmosphereSettings > exponentialAtmosphereSettings =
                boost::dynamic_pointer_cast< ExponentialAtmosphereSettings >( atmosphereSettings );
        if( exponentialAtmosphereSettings == NULL )
        {
            throw std::runtime_error(
                        "Error, expected exponential atmosphere settings for body " + body );
        }
        else
        {
            // Create and initialize exponential atmosphere model.
            boost::shared_ptr< ExponentialAtmosphere > exponentialAtmosphereModel =
                    boost::make_shared< ExponentialAtmosphere >(
                        exponentialAtmosphereSettings->getDensityScaleHeight( ) ,
                        exponentialAtmosphereSettings->getConstantTemperature( ),
                        exponentialAtmosphereSettings->getDensityAtZeroAltitude( ),
                        exponentialAtmosphereSettings->getSpecificGasConstant( ) );
            atmosphereModel = exponentialAtmosphereModel;
        }
        break;
    }
    case tabulated_atmosphere:
    {
        // Check whether settings for atmosphere are consistent with its type
        boost::shared_ptr< TabulatedAtmosphereSettings > tabulatedAtmosphereSettings =
                boost::dynamic_pointer_cast< TabulatedAtmosphereSettings >( atmosphereSettings );
        if( tabulatedAtmosphereSettings == NULL )
        {
            throw std::runtime_error(
                        "Error, expected tabulated atmosphere settings for body " + body );
        }
        else
        {
            // Create and initialize tabulatedl atmosphere model.
            atmosphereModel = boost::make_shared< TabulatedAtmosphere >(
                        tabulatedAtmosphereSettings->getAtmosphereFile( ) );
        }
        break;
    }
#if USE_NRLMSISE00
    case nrlmsise00:
    {
        std::string folder = input_output::getTudatRootPath( ) + "Astrodynamics/Aerodynamics/";
        std::string spaceWeatherFilePath = folder + "sw19571001.txt";

        tudat::input_output::solar_activity::SolarActivityDataMap solarActivityData =
                tudat::input_output::solar_activity::readSolarActivityData( spaceWeatherFilePath ) ;

        // Create atmosphere model using NRLMISE00 input function
        boost::function< tudat::aerodynamics::NRLMSISE00Input (double,double,double,double) > inputFunction =
                boost::bind(&tudat::aerodynamics::nrlmsiseInputFunction,_1,_2,_3,_4, solarActivityData , false , TUDAT_NAN );
        atmosphereModel = boost::make_shared< aerodynamics::NRLMSISE00Atmosphere >( inputFunction );
        break;
    }
#endif
    case us76_atmosphere:
    {
        // Check whether settings for atmosphere are consistent with its type
        boost::shared_ptr< TabulatedUS76Settings > tabulatedAtmosphereSettings =
                boost::dynamic_pointer_cast< TabulatedUS76Settings >( atmosphereSettings );
        if( tabulatedAtmosphereSettings == NULL )
        {
            throw std::runtime_error(
                        "Error, expected tabulated atmosphere settings for body " + body );
        }
        else
        {
            // Create and initialize tabulatedl atmosphere model.
            atmosphereModel = boost::make_shared< TabulatedUS76 >(
                        tabulatedAtmosphereSettings->getAtmosphereFile( ) );
        }
        break;
    }
    case us76_dispersed_atmosphere:
    {
        // Check whether settings for atmosphere are consistent with its type
        boost::shared_ptr< DispersedTabulatedUS76Settings > tabulatedAtmosphereSettings =
                boost::dynamic_pointer_cast< DispersedTabulatedUS76Settings >( atmosphereSettings );
        if( tabulatedAtmosphereSettings == NULL )
        {
            throw std::runtime_error(
                        "Error, expected tabulated atmosphere settings for body " + body );
        }
        else
        {
            // Create and initialize tabulatedl atmosphere model.
            atmosphereModel = boost::make_shared< DispersedTabulatedUS76 >(
                        tabulatedAtmosphereSettings->getAtmosphereFile( ) , tabulatedAtmosphereSettings->getDensityDispersion( ) );
        }
        break;
    }
    default:
        throw std::runtime_error(
                 "Error, did not recognize atmosphere model settings type " +
                  boost::lexical_cast< std::string >( atmosphereSettings->getAtmosphereType( ) ) );
    }
    return atmosphereModel;
}


} // namespace simulation_setup

} // namespace tudat
