#include <boost/shared_ptr.hpp>
#include "particle.hpp"
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define K_SPRING 5e2
#define K_DAMPER 5e1
#define SPRING_DAMPER_THRE 1e-3

class deformable_solid {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  std::vector<std::vector<double> > const_length;
  double mass;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d com, tmp_d;

  deformable_solid() {}
  ~ deformable_solid() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init() {
    std::vector<double> cl;
    for (uint i=0; i<this->pl.size(); i++) {
      cl.clear();
      for (uint j=0; j<this->pl.size(); j++) {
        if (j==i) cl.push_back(0.0);
        else cl.push_back((this->pl[j]->pos - this->pl[i]->pos).norm());
      }
      this->const_length.push_back(cl);
    }
    // Calculate center and mass
    this->com = Eigen::Vector3d::Zero();
    this->mass = 0;
    for (uint i=0; i<this->pl.size(); i++) {
      this->com += this->pl[i]->mass * this->pl[i]->pos;
      this->mass += this->pl[i]->mass;
    }
    this->com /= this->mass;
    for (uint i=0; i<this->pl.size(); i++) {
      // Update particles relative pos to center
      this->pl[i]->pos_to_cent = this->pl[i]->pos - this->com;
    }
  }

  void rotate(Eigen::Vector3d rot, double dt) {
    double rotn = rot.norm();
    Eigen::Matrix3d dR = Eigen::AngleAxisd(rotn, rot/rotn).toRotationMatrix();
    this->rotation = dR * Eigen::Matrix3d::Identity();
    for (uint i=0; i<this->pl.size(); i++) {
      this->pl[i]->pos = this->rotation * this->pl[i]->pos_to_cent + this->com;
      this->pl[i]->prev_pos = this->pl[i]->pos - this->pl[i]->vel * dt;
    }
  }

  void update_spring_damper() {
    for (uint i=0; i<this->pl.size(); i++) {
      // Add spring and dampers force
      for (uint j=0; j<this->const_length[i].size(); j++) {
        if (j == i) continue;
        this->tmp_d = (this->pl[j]->pos - this->pl[i]->pos);
        double dist = this->tmp_d.norm();
        if (dist > SPRING_DAMPER_THRE) {
          this->tmp_d /= dist;
          this->pl[i]->force += K_SPRING * (dist - this->const_length[i][j]) * this->tmp_d;
          this->pl[i]->force += K_DAMPER * (this->pl[j]->vel - this->pl[i]->vel).dot(this->tmp_d)
            * this->tmp_d;
        }
      }
    }
  }
};
