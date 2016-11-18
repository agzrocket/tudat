#ifndef DEFAULT_WINDMODEL_H
#define DEFAULT_WINDMODEL_H

#include <Tudat/Astrodynamics/Aerodynamics/windModel.h>

namespace tudat
{

namespace aerodynamics
{

class DefaultWindModel : public WindModel
{
    public:

        //! Constructor.
        DefaultWindModel( ){}

    protected:
        void updateWindVelocity( ){ }

    private:

};

} // namespace_aerodynamics
} // namespace_tudat

#endif // DEFAULT_WINDMODEL_H
