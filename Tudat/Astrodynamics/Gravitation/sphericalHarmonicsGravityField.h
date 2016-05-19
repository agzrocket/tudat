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
 *      101116    K. Kumar          File created.
 *      101117    K. Kumar          Added getPotential( ) and getGradientOfPotential functions.
 *      101214    K. Kumar          Updated getGradientOfPotential( ) and getLaplacianOfPotential( ).
 *      110106    K. Kumar          Added order and degree of expansion variables and set/get
 *                                  functions.
 *      110202    K. Kumar          Updated code to make use of the CartesianPositionElements
 *                                  class.
 *      110204    K. Kumar          Removed "vector" from naming.
 *      110310    K. Kumar          Changed naming from Laplacian to gradient tensor.
 *      110805    K. Kumar          Added predefined functionality with WGS-72 and WGS-84
 *                                  predefined Earth gravity fields.
 *      120326    D. Dirkx          Changed raw pointers to shared pointers.
 *      130121    K. Kumar          Added shared-ptr typedef.
 *      141020    D. Dirkx          Change of architecture.
 *
 *    References
 *      Vallado, D. A., Crawford, P., Hujsak, R., & Kelso, T. Revisiting Spacetrack Report #3:
 *          Rev 1, Proceedings of the AIAA/AAS Astrodynamics Specialist Conference. Keystone, CO,
 *          2006.
 *
 *    Notes
 *
 */

#ifndef TUDAT_SPHERICAL_HARMONICS_GRAVITY_FIELD_H
#define TUDAT_SPHERICAL_HARMONICS_GRAVITY_FIELD_H

#include <boost/function.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Tudat/Mathematics/BasicMathematics/mathematicalConstants.h"

#include "Tudat/Mathematics/BasicMathematics/legendrePolynomials.h"
#include "Tudat/Astrodynamics/Gravitation/gravityFieldModel.h"
#include "Tudat/Astrodynamics/Gravitation/sphericalHarmonicsGravityModel.h"

namespace tudat
{

namespace gravitation
{

//! Function to calculate the gravitational potential from a spherical harmonic field expansion.
/*!
 *  Function to calculate the gravitational potential from a spherical harmonic field expansion.
 *  \param bodyFixedPosition Position of point at which potential is to be calculated wrt the
 *  massive body, in the frame in which the expansion is defined (typically body-fixed).
 *  \param gravitationalParameter Gravitational parameter of massive body.
 *  \param referenceRadius Reference radius of spherical harmonic field expansion.
 *  \param cosineCoefficients Cosine spherical harmonic coefficients (geodesy normalized).
 *  \param sineCoefficients Sine spherical harmonic coefficients (geodesy normalized).
 *  \param minimumumDegree Maximum degree of spherical harmonic expansion.
 *  \param minimumumOrder Maximum order of spherical harmonic expansion.

 *  \return Gravitational potential at position defined by bodyFixedPosition
 */
double calculateSphericalHarmonicGravitationalPotential(
        const Eigen::Vector3d& bodyFixedPosition, const double gravitationalParameter,
        const double referenceRadius,
        const Eigen::MatrixXd& cosineCoefficients, const Eigen::MatrixXd& sineCoefficients,
        const int minimumumDegree = 0, const int minimumumOrder = 0 );

//! Class to represent a spherical harmonic gravity field expansion.
/*!
 *  Class to represent a spherical harmonic gravity field expansion of a massive body with
 *  time-independent spherical harmonic gravity field coefficients.
 */
class SphericalHarmonicsGravityField: public GravityFieldModel
{
public:

    //! Class constructor.
    /*!
     *  Class constructor.
     *  \param gravitationalParameter Gravitational parameter of massive body
     *  \param referenceRadius Reference radius of spherical harmonic field expansion
     *  \param cosineCoefficients Cosine spherical harmonic coefficients (geodesy normalized)
     *  \param sineCoefficients Sine spherical harmonic coefficients (geodesy normalized)
     *  \param fixedReferenceFrame Identifier for body-fixed reference frame to which the field is fixed (optional).
     */
    SphericalHarmonicsGravityField( const double gravitationalParameter,
                                    const double referenceRadius,
                                    const Eigen::MatrixXd& cosineCoefficients =
            Eigen::MatrixXd::Identity( 1, 1 ),
                                    const Eigen::MatrixXd& sineCoefficients =
            Eigen::MatrixXd::Zero( 1, 1 ),
                                    const std::string& fixedReferenceFrame = "" )
        : GravityFieldModel( gravitationalParameter ), referenceRadius_( referenceRadius ),
          cosineCoefficients_( cosineCoefficients ), sineCoefficients_( sineCoefficients ),
          fixedReferenceFrame_( fixedReferenceFrame )
    { }

    //! Virtual destructor.
    /*!
     *  Virtual destructor.
     */
    virtual ~SphericalHarmonicsGravityField( ) { }

    //! Function to get the reference radius.
    /*!
     *  Returns the reference radius used for the spherical harmonics expansion in meters.
     *  \return Reference radius of spherical harmonic field expansion
     */
    double getReferenceRadius( )
    {
        return referenceRadius_;
    }

    //! Function to get the cosine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Function to get the cosine spherical harmonic coefficients (geodesy normalized)
     *  \return Cosine spherical harmonic coefficients (geodesy normalized)
     */
    Eigen::MatrixXd getCosineCoefficients( )
    {
        return cosineCoefficients_;
    }

    //! Function to get the sine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Function to get the sine spherical harmonic coefficients (geodesy normalized)
     *  \return Sine spherical harmonic coefficients (geodesy normalized)
     */
    Eigen::MatrixXd getSineCoefficients( )
    {
        return sineCoefficients_;
    }

    //! Function to reset the cosine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Function to reset the cosine spherical harmonic coefficients (geodesy normalized)
     *  \param cosineCoefficients New cosine spherical harmonic coefficients (geodesy normalized)
     */
    void setCosineCoefficients( const Eigen::MatrixXd& cosineCoefficients )
    {
        cosineCoefficients_ = cosineCoefficients;
    }

    //! Function to reset the cosine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Function to reset the cosine spherical harmonic coefficients (geodesy normalized)
     *  \param sineCoefficients New sine spherical harmonic coefficients (geodesy normalized)
     */
    void setSineCoefficients( const Eigen::MatrixXd& sineCoefficients )
    {
        sineCoefficients_ = sineCoefficients;
    }

    //! Function to get a cosine spherical harmonic coefficient block (geodesy normalized)
    /*!
     *  Function to get a cosine spherical harmonic coefficient block (geodesy normalized)
    *   up to a given degree and order
     *  \param maximumDegree Maximum degree of coefficient block
     *  \param maximumOrder Maximum order of coefficient block
     *  \return Cosine spherical harmonic coefficients (geodesy normalized) up to given
     *  degree and order
     */
    Eigen::MatrixXd getCosineCoefficients( const int maximumDegree, const int maximumOrder )
    {
        return cosineCoefficients_.block( 0, 0, maximumDegree + 1, maximumOrder + 1 );
    }

    //! Function to get a sine spherical harmonic coefficient block (geodesy normalized)
    /*!
     *  Function to get a sine spherical harmonic coefficient block (geodesy normalized)
     *  up to a given degree and order
     *  \param maximumDegree Maximum degree of coefficient block
     *  \param maximumOrder Maximum order of coefficient block
     *  \return Sine spherical harmonic coefficients (geodesy normalized) up to given
     *  degree and order
     */
    Eigen::MatrixXd getSineCoefficients( const int maximumDegree, const int maximumOrder )
    {
        return sineCoefficients_.block( 0, 0, maximumDegree + 1, maximumOrder + 1 );
    }

    //! Get maximum degree of spherical harmonics gravity field expansion.
    /*!
     *  Returns the maximum degree of the spherical harmonics gravity field expansion.
     *  \return Degree of spherical harmonics expansion.
     */
    double getDegreeOfExpansion( )
    {
        return cosineCoefficients_.rows( ) + 1;
    }

    //! Get maximum order of spherical harmonics gravity field expansion.
    /*!
     *  Returns the maximum order of the spherical harmonics gravity field expansion.
     *  \return Order of spherical harmonics expansion.
     */
    double getOrderOfExpansion( )
    {
        return cosineCoefficients_.cols( ) + 1;
    }

    //! Function to calculate the gravitational potential at a given point
    /*!
     *  Function to calculate the gravitational potential due to this body at a given point.
     *  Note that this function, which has the same interface as in the base class and
     *  expands the gravity field to its maximum degree and order.
     *  \param bodyFixedPosition of point at which potential is to be calculated, in body-fixed
     *  frame.
     *  \return Gravitational potential at requested point.
     */
    double getGravitationalPotential( const Eigen::Vector3d& bodyFixedPosition )
    {
        return getGravitationalPotential( bodyFixedPosition, cosineCoefficients_.rows( ) - 1,
                                          sineCoefficients_.cols( ) - 1 );
    }

    //! Function to calculate the gravitational potential due to terms up to given degree and
    //! order at a given point
    /*!
     *  Function to calculate the gravitational potential due to terms up to given degree and
     *  order due to this body at a given point.
     *  \param bodyFixedPosition Position of point at which potential is to be calculate,
     *  in body-fixed frame.
     *  \param maximumDegree Maximum degree of spherical harmonic coefficients to include.
     *  \param maximumOrder Maximum order of spherical harmonic coefficients to include.
     *  \param minimumDegree Minimum degree of spherical harmonic coefficients to include, default 0
     *  \param minimumOrder Maximum order of spherical harmonic coefficients to include, default 0
     *  \return Gravitational potential due to terms up to given degree and order at
     *  requested point.
     */
    double getGravitationalPotential( const Eigen::Vector3d& bodyFixedPosition,
                                      const double maximumDegree,
                                      const double maximumOrder,
                                      const double minimumDegree = 0,
                                      const double minimumOrder = 0 )
    {
        return calculateSphericalHarmonicGravitationalPotential(
                    bodyFixedPosition, gravitationalParameter_, referenceRadius_,
                    cosineCoefficients_.block( 0, 0, maximumDegree + 1, maximumOrder + 1 ),
                    sineCoefficients_.block( 0, 0, maximumDegree + 1, maximumOrder + 1 ),
                    minimumDegree, minimumOrder );
    }

    //! Get the gradient of the potential.
    /*!
     * Returns the gradient of the potential for the gravity field selected.
     *  Note that this function, which has the same interface as in the base class and
     *  expands the gravity field to its maximum degree and order.
     * \param bodyFixedPosition Position at which gradient of potential is to be determined
     * \return Gradient of potential.
     */
    Eigen::Vector3d getGradientOfPotential( const Eigen::Vector3d& bodyFixedPosition )
    {
        return getGradientOfPotential( bodyFixedPosition, cosineCoefficients_.rows( ),
                                          sineCoefficients_.cols( ) );
    }

    //! Get the gradient of the potential.
    /*!
     *  Returns the gradient of the potential for the gravity field selected.
     *  \param bodyFixedPosition Position at which gradient of potential is to be determined
     *  \param maximumDegree Maximum degree of spherical harmonic coefficients to include.
     *  \param maximumOrder Maximum order of spherical harmonic coefficients to include.
     *  \return Gradient of potential.
     */
    Eigen::Vector3d getGradientOfPotential( const Eigen::Vector3d& bodyFixedPosition,
                                                    const double maximumDegree,
                                                    const double maximumOrder )
    {
        return computeGeodesyNormalizedGravitationalAccelerationSum(
                    bodyFixedPosition, gravitationalParameter_, referenceRadius_,
                    cosineCoefficients_.block( 0, 0, maximumDegree, maximumOrder ),
                    sineCoefficients_.block( 0, 0, maximumDegree, maximumOrder ) );
    }

    //! Function to retrieve the tdentifier for body-fixed reference frame
    /*!
     *  Function to retrieve the tdentifier for body-fixed reference frame
     *  \return Function to retrieve the tdentifier for body-fixed reference frame.
     */
    std::string getFixedReferenceFrame( )
    {
        return fixedReferenceFrame_;
    }

protected:

    //! Reference radius of spherical harmonic field expansion
    /*!
     *  Reference radius of spherical harmonic field expansion
     */
    double referenceRadius_;

    //! Cosine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Cosine spherical harmonic coefficients (geodesy normalized)
     */
    Eigen::MatrixXd cosineCoefficients_;

    //! Sine spherical harmonic coefficients (geodesy normalized)
    /*!
     *  Sine spherical harmonic coefficients (geodesy normalized)
     */
    Eigen::MatrixXd sineCoefficients_;

    //! Identifier for body-fixed reference frame
    /*!
     *  Identifier for body-fixed reference frame
     */
    std::string fixedReferenceFrame_;
};

} // namespace gravitation

} // namespace tudat

#endif // TUDAT_SPHERICAL_HARMONICS_GRAVITY_FIELD_H
