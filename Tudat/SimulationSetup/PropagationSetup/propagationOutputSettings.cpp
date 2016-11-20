/*    Copyright (c) 2010-2016, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include "Tudat/SimulationSetup/PropagationSetup/propagationOutputSettings.h"

namespace tudat
{

namespace propagators
{

//! Function to get a string representing a 'named identification' of a dependent variable type
std::string getDependentVariableName( const PropagationDependentVariables propagationDependentVariables )
{
    std::string variableName = "";
    switch( propagationDependentVariables )
    {
    case mach_number_dependent_variable:
        variableName = "Mach number ";
        break;
    case altitude_dependent_variable:
        variableName = "Altitude ";
        break;
    case airspeed_dependent_variable:
        variableName = "Airspeed ";
        break;
    case local_density_dependent_variable:
        variableName = "Density ";
        break;
    case relative_speed_dependent_variable:
        variableName = "Relative speed ";
        break;
    case relative_position_dependent_variable:
        variableName = "Relative position ";
        break;
    case relative_distance_dependent_variable:
        variableName = "Relative distance ";
        break;
    case relative_velocity_dependent_variable:
        variableName = "Relative velocity ";
        break;
    case radiation_pressure_dependent_variable:
        variableName = "Radiation pressure ";
        break;
    case total_acceleration_norm_dependent_variable:
        variableName = "Total acceleration norm ";
        break;
    case single_acceleration_norm_dependent_variable:
        variableName = "Single acceleration norm of type ";
        break;
    case total_acceleration_dependent_variable:
        variableName = "Total acceleration ";
        break;
    case single_acceleration_dependent_variable:
        variableName = "Single acceleration of type ";
        break;
    case aerodynamic_force_coefficients_dependent_variable:
        variableName = "Aerodynamic force coefficients ";
        break;
    case aerodynamic_moment_coefficients_dependent_variable:
        variableName = "Aerodynamic moment coefficients ";
        break;
    case rotation_matrix_to_body_fixed_frame_variable:
        variableName = "Rotation matrix to body-fixed frame ";
        break;
    case intermediate_aerodynamic_rotation_matrix_variable:
        variableName = "Rotation matrix from ";
        break;
    case relative_body_aerodynamic_orientation_angle_variable:
        variableName = "Body orientation angle ";
        break;
    case body_fixed_airspeed_based_velocity_variable:
        variableName = "Airspeed-based velocity ";
        break;
    case dissipated_energy_dependent_variable:
        variableName = "Dissipated energy ";
        break;
    default:
        std::string errorMessage = "Error, dependent variable " +
                boost::lexical_cast< std::string >( propagationDependentVariables ) +
                "not found when retrieving parameter name ";
        throw std::runtime_error( errorMessage );
    }
    return variableName;
}

//! Function to get a string representing a 'named identification' of a dependent variable
std::string getDependentVariableId(
        const boost::shared_ptr< SingleDependentVariableSaveSettings > dependentVariableSettings )
{
    std::string variableId = getDependentVariableName( dependentVariableSettings->variableType_ );

    if( ( dependentVariableSettings->variableType_ == single_acceleration_dependent_variable ) ||
            ( dependentVariableSettings->variableType_ == single_acceleration_norm_dependent_variable ) )
    {
        boost::shared_ptr< SingleAccelerationDependentVariableSaveSettings > accelerationDependentVariableSettings =
                boost::dynamic_pointer_cast< SingleAccelerationDependentVariableSaveSettings >( dependentVariableSettings );
        if( accelerationDependentVariableSettings == NULL )
        {
            throw std::runtime_error( "Error when getting dependent variable ID, input is inconsistent (acceleration type )" );
        }
        else
        {
            variableId += basic_astrodynamics::getAccelerationModelName(
                        accelerationDependentVariableSettings->accelerationModeType_ );
        }
    }
    else if( dependentVariableSettings->variableType_ == intermediate_aerodynamic_rotation_matrix_variable )
    {
        boost::shared_ptr< IntermediateAerodynamicRotationVariableSaveSettings > rotationDependentVariableSettings =
                boost::dynamic_pointer_cast< IntermediateAerodynamicRotationVariableSaveSettings >( dependentVariableSettings );
        if( rotationDependentVariableSettings == NULL )
        {
            throw std::runtime_error( "Error when getting dependent variable ID, input is inconsistent (rotation matrix)" );
        }
        else
        {
            variableId +=
                    reference_frames::getAerodynamicFrameName( rotationDependentVariableSettings->baseFrame_ ) + "to " +
                    reference_frames::getAerodynamicFrameName( rotationDependentVariableSettings->targetFrame_ );
        }
    }

    else if( dependentVariableSettings->variableType_ == relative_body_aerodynamic_orientation_angle_variable )
    {
        boost::shared_ptr< BodyAerodynamicAngleVariableSaveSettings > angleDependentVariableSettings =
                boost::dynamic_pointer_cast< BodyAerodynamicAngleVariableSaveSettings >( dependentVariableSettings );
        if( angleDependentVariableSettings == NULL )
        {
            throw std::runtime_error( "Error when getting dependent variable ID, input is inconsistent (angle)" );
        }
        else
        {
            variableId +=
                    reference_frames::getAerodynamicAngleName( angleDependentVariableSettings->angle_ );
        }
    }



    if( ( dependentVariableSettings->variableType_ == single_acceleration_dependent_variable ) ||
            ( dependentVariableSettings->variableType_ == single_acceleration_norm_dependent_variable )  )
    {
        variableId += ", acting on " + dependentVariableSettings->associatedBody_;
        if( dependentVariableSettings->secondaryBody_ != dependentVariableSettings->associatedBody_ )
        {
            variableId += ", exerted by " + dependentVariableSettings->secondaryBody_;
        }
    }
    else
    {
        variableId += "of " + dependentVariableSettings->associatedBody_;
        if( dependentVariableSettings->secondaryBody_ != "" )
        {
            variableId += " w.r.t. " + dependentVariableSettings->secondaryBody_;
        }

    }
    return variableId;
}

}

}
