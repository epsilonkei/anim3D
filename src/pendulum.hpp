#include "particle.hpp"
#include <Eigen/Dense>

class pendulum {
public:
  particle pt;
  Eigen::Vector3d fixed_point, norm_vec;
  double wire_length;

  pendulum(double _wl, double* _fp, double* _nv)
    : wire_length(_wl),
      fixed_point(Eigen::Map<Eigen::Vector3d>(_fp,3)),
      norm_vec(Eigen::Map<Eigen::Vector3d>(_nv,3))
  {}
  ~ pendulum() {
  }

  void init(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    particle pt(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc);
    this->pt = pt;
  }

  void updateConstraint() {
    Eigen::Vector3d dx = this->pt.pos - this->pt.prev_pos;
    dx -= dx.dot(this->norm_vec) * this->norm_vec; // Project to moving plane
    this->pt.pos = this->pt.prev_pos + dx;
    // this->pt.pos -= this->pt.pos.dot(this->norm_vec) * this->norm_vec; // Project to moving plane
    this->pt.pos = this->fixed_point + (this->wire_length / (this->pt.pos - this->fixed_point).norm()) * (this->pt.pos - this->fixed_point); // Shift object to wire
  }

  void updateEuler(double dt) {
    this->pt.updateEuler(dt);
    updateConstraint();
  }

  void updateVerlet(double dt) {
    this->pt.updateVerlet(dt);
    updateConstraint();
  }

};
