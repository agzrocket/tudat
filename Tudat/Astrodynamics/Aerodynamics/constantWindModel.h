#ifndef CONSTANT_WINDMODEL_H
#define CONSTANT_WINDMODEL_H

#include <Tudat/Astrodynamics/Aerodynamics/windModel.h>

namespace tudat
{

namespace aerodynamics
{

class ConstantWindModel : public WindModel
{
    public:

        //! Constructor.
        ConstantWindModel( double northWindspeed , double eastWindspeed , double downWindspeed )
        {
            northWindspeed_ = northWindspeed;
            eastWindspeed_ = eastWindspeed;
            downWindspeed_ = downWindspeed;
        }

    protected:
        void updateWindVelocity( )
        {
            currentWindspeedVectorInLocalVerticalFrame_[0] = northWindspeed_;
            currentWindspeedVectorInLocalVerticalFrame_[1] = eastWindspeed_;
            currentWindspeedVectorInLocalVerticalFrame_[2] = downWindspeed_;
        }

    private:
        double northWindspeed_;
        double eastWindspeed_;
        double downWindspeed_;

};

} // namespace_aerodynamics
} // namespace_tudat

#endif // CONSTANT_WINDMODEL_H
