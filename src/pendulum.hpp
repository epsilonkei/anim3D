#include "particle.hpp"
#include <Eigen/Dense>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

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
    this->pt.pos = this->fixed_point + (this->wire_length / (this->pt.pos - this->fixed_point).norm())
      * (this->pt.pos - this->fixed_point); // Shift object to wire
  }

  void updateEuler(double dt) {
    this->pt.updateEuler(dt);
    updateConstraint();
  }

  void updateVerlet(double dt) {
    this->pt.updateVerlet(dt);
    updateConstraint();
  }

  void calcForceWithVirtualWork() {
    this->pt.force = - this->pt.mass * grav * e3;
    Eigen::Vector3d pos_from_cent = this->pt.pos - this->fixed_point;
    double alpha = - (this->pt.force.dot(pos_from_cent) +
                      this->pt.mass * this->pt.vel.squaredNorm()) / (pos_from_cent.squaredNorm());
    this->pt.force += alpha * pos_from_cent;
  }

  void updateEulerForceBased(double dt) {
    calcForceWithVirtualWork();
    // this->pt.updateEuler(dt);
    //
    this->pt.acc = this->pt.force / this->pt.mass;
    this->pt.prev_pos = this->pt.pos;
    //
    this->pt.vel += dt * this->pt.acc;
    Eigen::Vector3d circum = this->pt.pos - this->fixed_point;
    circum.normalized();
    this->pt.vel -= this->pt.vel.dot(circum) * circum;
    //
    this->pt.pos += dt * this->pt.vel;
    circum = this->pt.pos - this->fixed_point;
    circum.normalized();
    this->pt.pos = this->fixed_point + circum * this->wire_length;
    //
    this->pt.last_dt = dt;
    this->pt.time += dt;
  }

  void updateVerletForceBased(double dt) {
    calcForceWithVirtualWork();
    this->pt.updateVerlet(dt);
    // this->pt.acc = this->pt.force / this->pt.mass;
    // //
    // Eigen::Vector3d pos_buf = this->pt.pos;
    // this->pt.pos += (this->pt.pos - this->pt.prev_pos) + dt * dt * this->pt.acc;
    // Eigen::Vector3d circum = this->pt.pos - this->fixed_point;
    // circum.normalized();
    // this->pt.pos = this->fixed_point + circum * this->wire_length;
    // //
    // this->pt.prev_pos = pos_buf;
    // this->pt.vel = (this->pt.pos - this->pt.prev_pos) / dt;
    // this->pt.vel -= this->pt.vel.dot(circum) * circum;
    // //
    // this->pt.last_dt = dt;
    // this->pt.time += dt;
  }
};
