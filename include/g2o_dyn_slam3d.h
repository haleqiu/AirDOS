#ifndef G2O_DYN_SLAM3D_H
#define G2O_DYN_SLAM3D_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "g2o_vertex_se3.h"

typedef Eigen::Matrix<double,3,1,Eigen::ColMajor>  Vector3;


class LandmarkMotionTernaryEdge: public g2o::BaseMultiEdge<3,Vector3>
{
    public:
     LandmarkMotionTernaryEdge();

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError();
    void linearizeOplus();

    virtual void setMeasurement(const Vector3& m){
      _measurement = m;
    }

    public:
        double delta_t = 1;

private:
    Eigen::Matrix<double,3,6,Eigen::ColMajor> J;

};

LandmarkMotionTernaryEdge::LandmarkMotionTernaryEdge() : g2o::BaseMultiEdge<3,Vector3>()
{
    resize(3);
    J.fill(0);
    J.block<3,3>(0,0) = Matrix3::Identity();
}

bool LandmarkMotionTernaryEdge::read(std::istream& is)
{
    Eigen::Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);

    for (int i=0; i<3; i++)
      for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
    return true;

}

bool LandmarkMotionTernaryEdge::write(std::ostream& os) const
{
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<3; ++i)
      for (int j=i; j<3; ++j)
        os <<  information()(i, j) << " " ;
    return os.good();
}

void LandmarkMotionTernaryEdge::computeError()
{
    g2o::VertexSBAPointXYZ* point1 = dynamic_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    g2o::VertexSBAPointXYZ* point2 = dynamic_cast<g2o::VertexSBAPointXYZ*>(_vertices[1]);
    VertexSE3* H = static_cast<VertexSE3*>(_vertices[2]);
    Isometry3 motion = H->estimate();
    Eigen::Matrix4d m(motion.matrix());
    motion.translation() << motion.translation()*delta_t;
    // motion.matrix() = delta_t * m;
    Vector3 expected = point1->estimate() - motion.inverse()*point2->estimate();
    _error = expected - _measurement;
}

void LandmarkMotionTernaryEdge::linearizeOplus()
{
    //g2o::VertexSBAPointXYZ* point1 = dynamic_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    g2o::VertexSBAPointXYZ* point2 = dynamic_cast<g2o::VertexSBAPointXYZ*>(_vertices[1]);
    VertexSE3* H = static_cast<VertexSE3*>(_vertices[2]);
    Isometry3 motion = H->estimate();
    Eigen::Matrix4d m(motion.matrix());
    // Here the motion has no rotation but translation
    motion.translation() << motion.translation()*delta_t;
    // Vector3 invHl2 = H->estimate().inverse()*point2->estimate();
    Vector3 invHl2 = motion.inverse()*point2->estimate();

    // jacobian wrt H // coresponding to rotation of so3
    // update the translation of the motion
    J(0,0) = J(0,0)*delta_t;
    J(1,1) = J(1,1)*delta_t;
    J(2,2) = J(2,2)*delta_t;

    Eigen::Matrix<double,3,6,Eigen::ColMajor> Jhom = J;

    _jacobianOplus[0] = Matrix3::Identity();
    _jacobianOplus[1] = -motion.inverse().rotation(); // This is actually an identity matrix
    _jacobianOplus[2] = Jhom;
}


#endif
