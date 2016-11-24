/*    Copyright (c) 2010-2016, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include "Tudat/Astrodynamics/Aerodynamics/aerodynamics.h"
#include "Tudat/SimulationSetup/PropagationSetup/propagationOutput.h"

namespace tudat
{

namespace propagators
{

//! Get the vector representation of a quaternion.
Eigen::VectorXd getVectorRepresentationForRotation(
        const boost::function< Eigen::Quaterniond( ) > rotationFunction )
{
    Eigen::Matrix3d currentRotationMatrix = rotationFunction( ).toRotationMatrix( );

    Eigen::VectorXd vectorRepresentation = Eigen::VectorXd( 9 );
    for( unsigned int i = 0; i < 3; i++ )
    {
        for( unsigned int j = 0; j < 3; j++ )
        {
            vectorRepresentation( i * 3 + j ) = currentRotationMatrix( i, j );
        }
    }
    return vectorRepresentation;
}

//! Get the 3x3 matrix representation from a vector with 9 entries
Eigen::Matrix3d getMatrixFromVectorRotationRepresentation(
        const Eigen::VectorXd vectorRepresentation )
{
    if( vectorRepresentation.rows( ) != 9 )
    {
        throw std::runtime_error( "Error when putting vector in matrix representation, size is incompatible" );
    }
    Eigen::Matrix3d currentRotationMatrix;
    for( unsigned int i = 0; i < 3; i++ )
    {
        for( unsigned int j = 0; j < 3; j++ )
        {
            currentRotationMatrix( i, j ) = vectorRepresentation( i * 3 + j );
        }
    }
    return currentRotationMatrix;
}

//! Get the quaternion formulation of an orthonormal matrix, from input of a vector with 9 entries corresponding to matrix
//! entries.
Eigen::Quaterniond getQuaternionFromVectorRotationRepresentation(
        const Eigen::VectorXd vectorRepresentation )
{
    return Eigen::Quaterniond( getMatrixFromVectorRotationRepresentation( vectorRepresentation ) );
}

double computeEquilibriumFayRiddellHeatFluxFromProperties(
        const boost::shared_ptr< aerodynamics::FlightConditions > flightConditions,
        const boost::shared_ptr< system_models::VehicleSystems > vehicleSystems )
{
    return aerodynamics::computeEquilibriumFayRiddellHeatFlux(
                flightConditions->getCurrentDensity( ), flightConditions->getCurrentAirspeed( ),
                flightConditions->getCurrentFreestreamTemperature( ), flightConditions->getCurrentMachNumber( ),
                vehicleSystems->getNoseRadius( ), vehicleSystems->getWallEmissivity( ) );
}

//! Function to evaluate a set of double and vector-returning functions and concatenate the results.
Eigen::VectorXd evaluateListOfFunctions(
        const std::vector< boost::function< double( ) > >& doubleFunctionList,
        const std::vector< std::pair< boost::function< Eigen::VectorXd( ) >, int > > vectorFunctionList,
        const int totalSize)
{
    Eigen::VectorXd variableList = Eigen::VectorXd::Zero( totalSize );
    int currentIndex = 0;

    for( unsigned int i = 0; i < doubleFunctionList.size( ); i++ )
    {
        variableList( i ) = doubleFunctionList.at( i )( );
        currentIndex++;
    }

    for( unsigned int i = 0; i < vectorFunctionList.size( ); i++ )
    {
        variableList.segment( currentIndex, vectorFunctionList.at( i ).second ) =
                vectorFunctionList.at( i ).first( );
        currentIndex += vectorFunctionList.at( i ).second;
    }

    // Check consistency with input
    if( currentIndex != totalSize )
    {
        std::string errorMessage = "Error when evaluating lists of functions, sizes are inconsistent: " +
                boost::lexical_cast< std::string >( currentIndex ) + " and " +
                boost::lexical_cast< std::string >( totalSize );
        throw std::runtime_error( errorMessage );
    }

    return variableList;
}

//! Funtion to get the size of a dependent variable
int getDependentVariableSize(
        const PropagationDependentVariables dependentVariableSettings )
{
    int variableSize = -1;
    switch( dependentVariableSettings )
    {
    case mach_number_dependent_variable:
        variableSize = 1;
        break;
    case altitude_dependent_variable:
        variableSize = 1;
        break;
    case airspeed_dependent_variable:
        variableSize = 1;
        break;
    case local_density_dependent_variable:
        variableSize = 1;
        break;
    case relative_speed_dependent_variable:
        variableSize = 1;
        break;
    case relative_position_dependent_variable:
        variableSize = 3;
        break;
    case relative_distance_dependent_variable:
        variableSize = 1;
        break;
    case relative_velocity_dependent_variable:
        variableSize = 3;
        break;
    case radiation_pressure_dependent_variable:
        variableSize = 1;
        break;
    case total_acceleration_norm_dependent_variable:
        variableSize = 1;
        break;
    case single_acceleration_norm_dependent_variable:
        variableSize = 1;
        break;
    case total_acceleration_dependent_variable:
        variableSize = 3;
        break;
    case single_acceleration_dependent_variable:
        variableSize = 3;
        break;
    case aerodynamic_force_coefficients_dependent_variable:
        variableSize = 3;
        break;
    case aerodynamic_moment_coefficients_dependent_variable:
        variableSize = 3;
        break;
    case rotation_matrix_to_body_fixed_frame_variable:
        variableSize = 9;
        break;
    case intermediate_aerodynamic_rotation_matrix_variable:
        variableSize = 9;
        break;
    case relative_body_aerodynamic_orientation_angle_variable:
        variableSize = 1;
        break;
    case body_fixed_airspeed_based_velocity_variable:
        variableSize = 3;
        break;
    case dissipated_energy_dependent_variable:
        variableSize = 1;
        break;
    case total_aerodynamic_g_load_variable:
        variableSize = 1;
        break;
    case stagnation_point_heat_flux_dependent_variable:
        variableSize = 1;
        break;
    case local_temperature_dependent_variable:
        variableSize = 1;
        break;
    case heat_load_dependent_variable:
        variableSize = 1;
        break;
    case geodetic_latitude_dependent_variable:
        variableSize = 1;
        break;
    case elevator_deflection_dependent_variable:
        variableSize = 1;
        break;
    case groundspeed_dependent_variable:
        variableSize = 1;
        break;
    case wind_velocity_lvlh_frame_dependent_variable:
        variableSize = 3;
        break;
    case inertialspeed_dependent_variable:
        variableSize = 1;
        break;
    default:
        std::string errorMessage = "Error, did not recognize dependent variable size of type: " +
                boost::lexical_cast< std::string >( dependentVariableSettings );
        throw std::runtime_error( errorMessage );
    }
    return variableSize;
}


} // namespace propagators

} // namespace tudat
