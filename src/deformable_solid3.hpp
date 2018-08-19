#include <boost/shared_ptr.hpp>
#include "particle.hpp"
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define K_SPRING 1e2
#define K_DAMPER 1e1
#define K_TO_RIGID 1e0
#define SPRING_DAMPER_THRE 1e-3

class deformable_solid {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  double mass, length;
  Eigen::Matrix3d rotation, omegax;
  Eigen::Matrix3d I_body, I_body_inv, I_inv;
  //
  Eigen::Vector3d com, prev_com;
  Eigen::Vector3d vel, omega, angle;
  //
  Eigen::Vector3d force, torque;
  Eigen::Vector3d lin_moment, ang_moment;
  //
  double max_radius;
  //
  std::vector<Eigen::Vector3d> bounding_plane_cent;
  std::vector<Eigen::Vector3d> bounding_plane_out_normv;
  std::vector<Eigen::Vector3d> bounding_plane_cent_local;
  //
  std::vector<std::vector<int> > next_part;
  Eigen::Vector3d tmp_d;

  deformable_solid()
    : vel(Eigen::Vector3d::Zero()), omega(Eigen::Vector3d::Zero()), max_radius(0)
  {}
  ~ deformable_solid() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init() {
    // Calculate center and mass
    this->com = Eigen::Vector3d::Zero();
    this->mass = 0;
    for (uint i=0; i<this->pl.size(); i++) {
      this->com += this->pl[i]->mass * this->pl[i]->pos;
      this->mass += this->pl[i]->mass;
    }
    this->com /= this->mass;
    this->I_body = Eigen::Matrix3d::Zero();
    // this->lin_moment = Eigen::Vector3d::Zero();
    //
    for (uint i=0; i<this->pl.size(); i++) {
      // Update particles relative pos to center
      this->pl[i]->pos_to_cent = this->pl[i]->pos - this->com;
      if (this->pl[i]->pos_to_cent.norm() > this->max_radius)
        this->max_radius = this->pl[i]->pos_to_cent.norm();
      // Calculate I_body
      this->I_body +=  this->pl[i]->mass * (this->pl[i]->pos_to_cent.squaredNorm() * Eigen::Matrix3d::Identity() - this->pl[i]->pos_to_cent * this->pl[i]->pos_to_cent.transpose());
      // Calculate initial linear momentum based on particles
      // this->lin_moment += this->pl[i]->mass * this->pl[i]->vel;
    }
    this->I_body_inv = this->I_body.inverse();
    // Calculate initial linear momentum and angular momentum
    this->rotation = Eigen::Matrix3d::Identity();
    // this->ang_moment = this->rotation * this->I_body * this->rotation.transpose() * this->omega;
    this->ang_moment = this->I_body * this->omega;
    this->lin_moment = this->mass * this->vel;
    // Init bounding plane center in local coord (only worked for cube and convex polygon in advance)
    this->bounding_plane_cent_local.push_back(this->pl[0]->pos_to_cent + this->pl[2]->pos_to_cent);
    this->bounding_plane_cent_local.push_back(this->pl[2]->pos_to_cent + this->pl[5]->pos_to_cent);
    this->bounding_plane_cent_local.push_back(this->pl[5]->pos_to_cent + this->pl[7]->pos_to_cent);
    this->bounding_plane_cent_local.push_back(this->pl[7]->pos_to_cent + this->pl[0]->pos_to_cent);
    this->bounding_plane_cent_local.push_back(this->pl[2]->pos_to_cent + this->pl[7]->pos_to_cent);
    this->bounding_plane_cent_local.push_back(this->pl[0]->pos_to_cent + this->pl[5]->pos_to_cent);
    copy(this->bounding_plane_cent_local.begin(), this->bounding_plane_cent_local.end(),
         std::back_inserter(this->bounding_plane_out_normv));
    copy(this->bounding_plane_cent_local.begin(), this->bounding_plane_cent_local.end(),
         std::back_inserter(this->bounding_plane_cent));
    // for deformable solid
        this->length = (this->pl[1]->pos - this->pl[0]->pos).norm();
    // Create next_particle list, Note: assuming object as a cube
    std::vector<int> npl;
    npl.push_back(1); npl.push_back(3); npl.push_back(4);
    for (uint i=0; i<this->pl.size(); i++) {
      if (i==0) { npl[0] = 1; npl[1] = 3; npl[2] = 4; }
      else if (i==1) { npl[0] = 0; npl[1] = 2; npl[2] = 5; }
      else if (i==2) { npl[0] = 1; npl[1] = 3; npl[2] = 6; }
      else if (i==3) { npl[0] = 0; npl[1] = 2; npl[2] = 7; }
      else if (i==4) { npl[0] = 0; npl[1] = 5; npl[2] = 7; }
      else if (i==5) { npl[0] = 1; npl[1] = 4; npl[2] = 6; }
      else if (i==6) { npl[0] = 2; npl[1] = 5; npl[2] = 7; }
      else if (i==7) { npl[0] = 3; npl[1] = 4; npl[2] = 6; }
      this->next_part.push_back(npl);
    }
  }

  void update_rigid_movement(double dt) {
    // Calculation moment of inertia
    this->I_inv = rotation * this->I_body_inv * rotation.transpose();
    // Calculation force and torque
    this->force = Eigen::Vector3d::Zero();
    this->torque = Eigen::Vector3d::Zero();
    for (uint i=0; i<this->pl.size(); i++) {
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
    double rot = this->omega.norm();
    if ( dt*rot > 0.0 ) {
      Eigen::Matrix3d dR = Eigen::AngleAxisd(dt*rot, this->omega/rot).toRotationMatrix();
      this->rotation = dR * this->rotation;
    }
  }

  void update_particles_movement(double dt) {
    for (uint i=0; i<this->pl.size(); i++) {
      // r_i = R * r0_i + x
      this->pl[i]->prev_pos = this->pl[i]->pos;
      this->pl[i]->pos = this->rotation * this->pl[i]->pos_to_cent + this->com;
      // rdot_i = omega x R * r0_i + v = omega x (r_i - x) + v
      // this->pl[i]->vel = this->omegax * this->rotation * this->pl[i]->pos_to_cent + this->vel;
      // this->pl[i]->vel = this->omegax * (this->pl[i]->pos - this->com) + this->vel;
      this->pl[i]->vel = (this->pl[i]->pos - this->pl[i]->prev_pos) / dt;
      this->pl[i]->last_dt = dt;
    }
  }

  void rotate(Eigen::Vector3d rot) {
    double rotn = rot.norm();
    Eigen::Matrix3d dR = Eigen::AngleAxisd(rotn, rot/rotn).toRotationMatrix();
    this->rotation = dR * Eigen::Matrix3d::Identity();
    for (uint i=0; i<this->pl.size(); i++) {
      this->pl[i]->pos = this->rotation * this->pl[i]->pos_to_cent + this->com;
    }
  }

  void update_spring_damper() {
    // only checked distance with next particles
    for (uint i=0; i<this->pl.size(); i++) {
      // Add spring and dampers force
      for (uint j=0; j<this->next_part[i].size(); j++) {
        int nid = this->next_part[i][j];
        this->tmp_d = (this->pl[nid]->pos - this->pl[i]->pos);
        double dist = this->tmp_d.norm();
        if (dist > SPRING_DAMPER_THRE) {
          this->tmp_d /= dist;
          this->pl[i]->force += K_SPRING * (dist - this->length) * this->tmp_d;
          this->pl[i]->force += K_DAMPER * (this->pl[nid]->vel.dot(this->tmp_d)
                                            - this->pl[i]->vel.dot(this->tmp_d)) * this->tmp_d;
        }
      }
    }
    // Apply spring force for distance if moving as rigid
    for (uint i=0; i<this->pl.size(); i++) {
      this->tmp_d = this->pl[i]->pos - this->rotation * this->pl[i]->pos_to_cent + this->com;
      this->pl[i]->force -= K_TO_RIGID * this->tmp_d;
    }
  }
};
