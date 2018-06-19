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
  std::vector<std::vector<int> > next_part;
  double mass, length;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d com, tmp_d;

  deformable_solid() {}
  ~ deformable_solid() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init() {
    this->length = (this->pl[1]->pos - this->pl[0]->pos).norm();
    // Create next_particle list, Note: assuming object as a cube
    std::vector<int> npl;
    npl.push_back(1); npl.push_back(3); npl.push_back(4);
    for (int i=0; i<this->pl.size(); i++) {
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
    // Calculate center and mass
    this->com = Eigen::Vector3d::Zero();
    this->mass = 0;
    for (int i=0; i<this->pl.size(); i++) {
      this->com += this->pl[i]->mass * this->pl[i]->pos;
      this->mass += this->pl[i]->mass;
    }
    this->com /= this->mass;
    for (int i=0; i<this->pl.size(); i++) {
      // Update particles relative pos to center
      this->pl[i]->pos_to_cent = this->pl[i]->pos - this->com;
    }
  }

  void rotate(Eigen::Vector3d rot, double dt) {
    double rotn = rot.norm();
    Eigen::Matrix3d dR = Eigen::AngleAxisd(rotn, rot/rotn).toRotationMatrix();
    this->rotation = dR * Eigen::Matrix3d::Identity();
    for (int i=0; i<this->pl.size(); i++) {
      this->pl[i]->pos = this->rotation * this->pl[i]->pos_to_cent + this->com;
      this->pl[i]->prev_pos = this->pl[i]->pos - this->pl[i]->vel * dt;
    }
  }

  void update_spring_damper() {
    // only checked distance with next particles
    for (int i=0; i<this->pl.size(); i++) {
      // Add spring and dampers force
      for (int j=0; j<this->next_part[i].size(); j++) {
        int nid = this->next_part[i][j];
        this->tmp_d = (this->pl[nid]->pos - this->pl[i]->pos);
        double dist = this->tmp_d.norm();
        if (dist > SPRING_DAMPER_THRE) {
          this->tmp_d /= dist;
          this->pl[i]->force += K_SPRING * (dist - this->length) * this->tmp_d;
          this->pl[i]->force += K_DAMPER * (this->pl[nid]->vel - this->pl[i]->vel).dot(this->tmp_d)
            * this->tmp_d;
        }
      }
    }
  }
};
