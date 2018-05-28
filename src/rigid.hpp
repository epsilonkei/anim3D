#include <boost/shared_ptr.hpp>
#include "particle.hpp"
#include <vector>
#include <Eigen/Dense>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

class rigid_body {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  double mass, length;
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d I_body, I;
  //
  Eigen::Vector3d com, prev_com;
  Eigen::Vector3d vel, rvel, acc, racc;
  //
  Eigen::Vector3d force, torque;
  //
  double last_dt, time;

  rigid_body() {}
  ~ rigid_body() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

};
