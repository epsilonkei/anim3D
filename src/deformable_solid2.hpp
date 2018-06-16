#include <boost/shared_ptr.hpp>
#include "particle.hpp"
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define K_SPRING 1e2
#define K_DAMPER 5e0
#define SPRING_DAMPER_THRE 1e-3

class deformable_solid {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  std::vector<std::vector<double> > const_length;
  Eigen::Vector3d tmp_d;

  deformable_solid() {}
  ~ deformable_solid() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init() {
    std::vector<double> cl;
    for (int i=0; i<this->pl.size(); i++) {
      cl.clear();
      for (int j=0; j<this->pl.size(); j++) {
        if (j==i) cl.push_back(0.0);
        else cl.push_back((this->pl[j]->pos - this->pl[i]->pos).norm());
      }
      this->const_length.push_back(cl);
    }
    std::cerr << this->const_length.size() << std::endl;
  }

  void update_spring_damper() {
    for (int i=0; i<this->pl.size(); i++) {
      // Add spring and dampers force
      for (int j=0; j<this->const_length[i].size(); j++) {
        if (j == i) continue;
        this->tmp_d = (this->pl[j]->pos - this->pl[i]->pos);
        double dist = this->tmp_d.norm();
        if (dist > SPRING_DAMPER_THRE) {
          this->tmp_d /= dist;
          this->pl[i]->force += K_SPRING * (dist - this->const_length[i][j]) * this->tmp_d;
          this->pl[i]->force += K_DAMPER * (this->pl[j]->vel.dot(this->tmp_d)
                                            - this->pl[i]->vel.dot(this->tmp_d)) * this->tmp_d;
        }
      }
    }
  }
};
