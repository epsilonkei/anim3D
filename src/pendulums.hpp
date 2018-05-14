#include <boost/shared_ptr.hpp>
#include "pendulum.hpp"
#include <vector>
#include <Eigen/Dense>

class pendulums {
public:
  std::vector<boost::shared_ptr<pendulum> > pl;

  pendulums() {}
  ~ pendulums() {}

  void add_pendulum(double _wl, double* _fp, double* _nv, double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    pendulum* pend = new pendulum(_wl, _fp, _nv);
    pend->init(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc);
    this->pl.push_back(boost::shared_ptr<pendulum>(pend));
  }

  bool is_collided (particle* Ball1, particle* Ball2) {
    return ((Ball1->pos - Ball2->pos).norm() < (Ball1->radius + Ball2->radius));
  }

  void forceBasedResponse(particle* Ball1,
                            particle* Ball2) { // without considering collision timing
    // Push away form each other
    Eigen::Vector3d collide_dir = (Ball1->pos - Ball2->pos);
    double collide_length = Ball1->radius + Ball2->radius - collide_dir.norm();
    collide_dir.normalize();
    Ball1->pos += collide_length * 0.5 * collide_dir;
    Ball2->pos -= collide_length * 0.5 * collide_dir;
    // Calculate velocity after collision
    double a = 2 * collide_dir.dot(Ball1->vel - Ball2->vel) / (1/ Ball1->mass + 1/ Ball2->mass);
    Ball1->vel -= a / Ball1->mass * collide_dir;
    Ball2->vel += a / Ball2->mass * collide_dir;
    // Update prev_pos
    Ball1->prev_pos = Ball1->pos - Ball1->vel * Ball1->last_dt;
    Ball2->prev_pos = Ball2->pos - Ball2->vel * Ball2->last_dt;
  }

  void posBasedResponse(particle* Ball1,
                          particle* Ball2) { // equal mass
    // Push away form each other
    Eigen::Vector3d collide_dir = (Ball1->pos - Ball2->pos);
    double collide_length = Ball1->radius + Ball2->radius - collide_dir.norm();
    collide_dir.normalize();
    Ball1->pos += collide_length * 0.5 * collide_dir;
    Ball2->pos -= collide_length * 0.5 * collide_dir;
    // Update velocity
    Eigen::Vector3d b1_v_on_collide = Ball1->vel.dot(collide_dir) * collide_dir;
    Eigen::Vector3d b1_v_other = Ball1->vel - b1_v_on_collide;
    Eigen::Vector3d b2_v_on_collide = Ball2->vel.dot(collide_dir) * collide_dir;
    Eigen::Vector3d b2_v_other = Ball2->vel - b2_v_on_collide;
    // Note: after collided, keep v_other component, exchange v_on_collide part
    Ball1->vel = b1_v_other + b2_v_on_collide;
    Ball2->vel = b2_v_other + b1_v_on_collide;
    // Update prev_pos
    Ball1->prev_pos = Ball1->pos - Ball1->vel * Ball1->last_dt;
    Ball2->prev_pos = Ball2->pos - Ball2->vel * Ball2->last_dt;
    // TODO: Modify the previous locations according to the conservation of energy
  }

  void updateVerletAll(double dt) {
    for ( int i=0; i<this->pl.size(); i++ ) {
      // this->pl[i]->updateVerlet(dt);
      this->pl[i]->updateVerletForceBased(dt);
    }
  }

  void updateEulerAll(double dt) {
    for ( int i=0; i<this->pl.size(); i++ ) {
      // this->pl[i]->updateEuler(dt);
      this->pl[i]->updateEulerForceBased(dt);
    }
  }

  int getPendulumAmount() { return this->pl.size(); }

  void update(double dt) {
    updateVerletAll(dt);
    // updateEulerAll(dt);
    for ( int i=0; i<this->pl.size(); i++ ) {
      for ( int j=i+1; j<this->pl.size(); j++ ) {
        if (is_collided( &(this->pl[i]->pt), &(this->pl[j]->pt) )) {
          posBasedResponse( &(this->pl[i]->pt), &(this->pl[j]->pt) );
        }
      }
    }
    // Update Contraint
    for ( int i=0; i<this->pl.size(); i++ ) {
      this->pl[i]->updateConstraint();
    }
  }

};
