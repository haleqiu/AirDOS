#ifndef G2O_VERTEX_DISTANCE_H
#define G2O_VERTEX_DISTANCE_H

#include "g2o_vertex_distance.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

// class VertexDistance: public g2o::BaseVertex<1,double>
class VertexDistance: public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexDistance(){};

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
        // _estimate=0;
    };

    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    virtual void oplusImpl(const double* update){
      Eigen::Map<const Eigen::Vector3d> v(update);
        _estimate += v;
    };
};

class VertexDistanceDouble: public g2o::BaseVertex<1,double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexDistanceDouble(){};

    virtual void setToOriginImpl() {
        _estimate=0;

    };

    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    virtual void oplusImpl(const double* update){
        _estimate += *update;
    };
};



#endif
