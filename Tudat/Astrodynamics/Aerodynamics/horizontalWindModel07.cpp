//! Standard libraries.
#include <iostream>
#include <ctime>

//! External libraries: BOOST.
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

//! External libraries: EIGEN.
#include <Eigen/Core>

//! TUDAT libraries.
#include <Tudat/InputOutput/matrixTextFileReader.h>


//! Own libraries.
#include <Tudat/Mathematics/Interpolators/cubicSplineInterpolator.h>
#include <Tudat/Astrodynamics/Aerodynamics/horizontalWindModel07.h>

namespace tudat
{

namespace aerodynamics
{

using namespace Eigen;
using namespace tudat;


//! Constructor.
HorizontalWindModel07::HorizontalWindModel07( )
{
    // Determine CSV file location.
    std::string cppPath( __FILE__ ); // save path of cpp file.
    std::string folder = cppPath.substr( 0, cppPath.find_last_of( "/\\" ) + 1 ); // strip filename from string and return path.

    // Initialize file identifiers.
    MatrixXd dataSpace;
    MatrixXd rowValues;

    // Read stored wind profiles.
    dataSpace   = input_output::readMatrixFromFile( folder + "WindModelCsvFiles/" + "hwm07WindProfiles.csv");
    rowValues   = input_output::readMatrixFromFile( folder + "WindModelCsvFiles/" + "hwm07Altitudes.csv");

    // Read number of samples.
    int numberOfSamples = dataSpace.rows()/2;

    // Retrieve a random sample.
    boost::random::mt19937 randomGenerator( std::time(NULL) );
    boost::random::uniform_int_distribution< > uniformDistribution( 1 , numberOfSamples );
    selectedSample_ = uniformDistribution( randomGenerator );

    // Retrieve velocity profiles for the selected function.
    int northWindIndex = 2*selectedSample_ - 2;
    int eastWindIndex  = 2*selectedSample_ - 1;

    MatrixXd northWindProfile = dataSpace.row( northWindIndex );
    MatrixXd eastWindProfile  = dataSpace.row( eastWindIndex );

    northWindInterpolator_ = getOneDimSmoothInterpolator( northWindProfile.transpose() , rowValues.transpose() );
    eastWindInterpolator_  = getOneDimSmoothInterpolator( eastWindProfile.transpose() , rowValues.transpose() );
}


void HorizontalWindModel07::updateWindVelocity( )
{
    currentWindspeedVectorInLocalVerticalFrame_[0] = northWindInterpolator_->interpolate( currentAltitude_ );
    currentWindspeedVectorInLocalVerticalFrame_[1] = eastWindInterpolator_->interpolate( currentAltitude_ );
    currentWindspeedVectorInLocalVerticalFrame_[2] = 0;
}

CubicSplineInterpolatorDoublePointer
    HorizontalWindModel07::getOneDimSmoothInterpolator( Eigen::MatrixXd dataSpace , Eigen::MatrixXd rowValues )
{
    // Define 1D matrix size.
    std::vector< double > matrixDimensions;
    matrixDimensions.push_back( dataSpace.rows() );

    // Create data vector.
    std::vector< double > independentValues;
    std::vector< double > dependentValues;

    // Fill data vectors using the CSV data.
    for ( int i = 0; i < matrixDimensions[ 0 ] ; i++ )
    {
        independentValues.push_back( rowValues( i , 0 ) );
    }

    for ( int i = 0 ; i < matrixDimensions[ 0 ] ; i++ )
    {
        dependentValues.push_back( dataSpace( i , 0 ) );

    }

    // Initialize interpolator.
    CubicSplineInterpolatorDoublePointer oneDimensionalSmoothInterpolator
            = boost::make_shared< CubicSplineInterpolatorDouble > ( independentValues, dependentValues );

    return oneDimensionalSmoothInterpolator;
}


} // namespace_wind_models
} // namespace_thesis_tools
