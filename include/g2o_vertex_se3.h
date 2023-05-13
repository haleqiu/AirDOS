#ifndef G2O_VERTEX_SE3_H
#define G2O_VERTEX_SE3_H

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"

typedef Eigen::Matrix<double,3,1,Eigen::ColMajor>  Vector3;
typedef Eigen::Matrix<double,6,1,Eigen::ColMajor>  Vector6;
typedef Eigen::Matrix<double,7,1,Eigen::ColMajor>  Vector7;
typedef Eigen::Matrix<double, 3, 3, Eigen::ColMajor> Matrix3;
typedef Eigen::Transform<double,3,Eigen::Isometry,Eigen::ColMajor>            Isometry3;

Matrix3 fromCompactQuaternion(const Vector3& v) {
  double w = 1-v.squaredNorm();
  if (w<0)
    return Matrix3::Identity();
  else
    w=sqrt(w);
  return Eigen::Quaternion<double>(w, v[0], v[1], v[2]).toRotationMatrix();
}

inline Isometry3::ConstLinearPart extractRotation(const Isometry3& A)
{
  return A.matrix().topLeftCorner<3,3>();
}

Vector3 toCompactQuaternion(const Matrix3& R) {
  Eigen::Quaternion<double> q(R);
  // normalize(q);
  q.normalize();
  // return (x,y,z) of the quaternion
  return q.coeffs().head<3>();
}

Vector6 toVectorMQT(const Isometry3& t) {
  Vector6 v;
  v.block<3,1>(3,0) = toCompactQuaternion(extractRotation(t));
  v.block<3,1>(0,0) = t.translation();
  return v;
}

Isometry3 fromVectorMQT(const Vector6& v){
  Isometry3 t;
  t = fromCompactQuaternion(v.block<3,1>(3,0));
  t.translation() = v.block<3,1>(0,0);
  return t;
}

Vector7 toVectorQT(const Isometry3& t){
  Eigen::Quaternion<double> q(extractRotation(t));
  q.normalize();
  Vector7 v;
  v[3] = q.x(); v[4] = q.y(); v[5] = q.z(); v[6] = q.w();
  v.block<3,1>(0,0) = t.translation();
  return v;
}

Isometry3 fromVectorQT(const Vector7& v) {
  Isometry3 t;
  t=Eigen::Quaternion<double>(v[6], v[3], v[4], v[5]).toRotationMatrix();
  t.translation() = v.head<3>();
  return t;
}

class VertexSE3 : public g2o::BaseVertex<6, Isometry3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const int orthogonalizeAfter = 1000; //< orthogonalize the rotation matrix after N updates

    VertexSE3();

    virtual void setToOriginImpl() {
      _estimate = Isometry3::Identity();
    }

    virtual bool read(std::istream& is){ return true; };
    virtual bool write(std::ostream& os) const {return os.good(); };

    virtual bool setEstimateDataImpl(const double* est){//double
      Eigen::Map<const Vector7> v(est);
      _estimate=fromVectorQT(v);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Eigen::Map<Vector7> v(est);
      v=toVectorQT(_estimate);
      return true;
    }

    virtual int estimateDimension() const {
      return 7;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
      Eigen::Map<const Vector6> v(est);
      _estimate = fromVectorMQT(v);
      return true;
    }

    virtual bool getMinimalEstimateData(double* est) const{
      Eigen::Map<Vector6> v(est);
      v = toVectorMQT(_estimate);
      return true;
    }

    virtual int minimalEstimateDimension() const {
      return 6;
    }

    virtual void oplusImpl(const double* update)
    {
      Eigen::Map<const Vector6> v(update);
      Isometry3 increment = fromVectorMQT(v);
      _estimate = _estimate * increment;
      if (++_numOplusCalls > orthogonalizeAfter) {
        _numOplusCalls = 0;
        // approximateNearestOrthogonalMatrix(_estimate.matrix().topLeftCorner<3,3>());
      }
    }
  protected:
    int _numOplusCalls;
  };

  VertexSE3::VertexSE3() :
    BaseVertex<6, Isometry3>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }

#endif
