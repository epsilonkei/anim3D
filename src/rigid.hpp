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
  Eigen::Matrix3d rotation, omegax;
  Eigen::Matrix3d I_body, I_body_inv, I_inv;
  //
  Eigen::Vector3d com, prev_com;
  Eigen::Vector3d vel, omega;
  //
  Eigen::Vector3d force, torque;
  Eigen::Vector3d lin_moment, ang_moment;
  //
  double last_dt, time;

  rigid_body() {}
  ~ rigid_body() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init() {
    // Calculate center and mass
    this->com = Eigen::Vector3d::Zero();
    this->mass = 0;
    for (int i=0; i<this->pl.size(); i++) {
      this->com += this->pl[i]->mass * this->pl[i]->pos;
      this->mass += this->pl[i]->mass;
    }
    this->com /= this->mass;
    this->I_body += Eigen::Matrix3d::Zero();
    //
    for (int i=0; i<this->pl.size(); i++) {
      // Update particles relative pos to center
      this->pl[i]->pos_to_cent = this->pl[i]->pos - this->com;
      // Calculate I_body
      this->I_body +=  this->pl[i]->mass * (this->pl[i]->pos_to_cent.squaredNorm() * Eigen::Matrix3d::Identity() - this->pl[i]->pos_to_cent * this->pl[i]->pos_to_cent.transpose());
    }
    this->I_body_inv = this->I_body.inverse();
    // Calculate initial linear momentum and angular momentum
    this->rotation = Eigen::Matrix3d::Identity();
    this->lin_moment = Eigen::Vector3d::Zero();
    for (int i=0; i<this->pl.size(); i++) {
      this->lin_moment += this->pl[i]->mass * this->pl[i]->vel;
      // this->ang_moment = this->rotation * this->I_body * this->rotation.transpose() * this->omega;
      this->ang_moment = this->I_body * this->omega;
    }
  }

  void update_rigid_movement(double dt) {
    // Calculation moment of inertia
    this->I_inv = rotation * this->I_body_inv * rotation.transpose();
    // Calculation force and torque
    this->force = Eigen::Vector3d::Zero();
    this->torque = Eigen::Vector3d::Zero();
    for (int i=0; i<this->pl.size(); i++) {
      this->force += this->pl[i]->force;
      this->torque += (this->pl[i]->pos - this->com).cross(this->pl[i]->force);
    }
    // Update linear momentum and angular momentum
    this->lin_moment += this->force * dt;
    this->ang_moment += this->torque * dt;
    // Update velocity & position and angular velocity & rotation
    this->vel = this->lin_moment / this->mass;
    this->com += this->vel * dt;
    this->I_inv = this->rotation * this -> I_body_inv * this->rotation.transpose();
    this->omega = this->I_inv * this->ang_moment;
    infinite_sigmal_matrix(this->omega, &this->omegax);
    this->rotation += this->omegax * this->rotation * dt;
  }

  void infinite_sigmal_matrix(Eigen::Vector3d &v, Eigen::Matrix3d *ret) {
    (*ret)(0,0) = 0; (*ret)(0,1) = -v(2); (*ret)(0,2) = v(1);
    (*ret)(1,0) = v(2); (*ret)(1,1) = 0; (*ret)(1,2) = -v(0);
    (*ret)(2,0) = -v(1); (*ret)(2,1) = v(0); (*ret)(2,2) = 0;
  }

  void update_particles_movement() {
    for (int i=0; i<this->pl.size(); i++) {
      // ri = R * r0i + x
      this->pl[i]->pos = this->rotation * this->pl[i]->pos_to_cent + this->com;
      this->pl[i]->vel = this->vel;
    }
  }
};
