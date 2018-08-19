#include <boost/shared_ptr.hpp>
// #include "particle.hpp"
#include <vector>
#include <Eigen/Dense>
#include "node.hpp"

double G_CONST = 6.67408e-3; // 6.67408e-11
double EPSILON = 1e-2;
double FAR_ENOUGH_THRE = 0.5;
double COLLIDE_THRE = 1e-6;

class particles {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  double table_length;

  particles(double _table_length)
    : table_length(_table_length)
  {}
  ~ particles() {
  }

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  bool is_collided (boost::shared_ptr<particle>& Ball1, boost::shared_ptr<particle>& Ball2) {
    return ((Ball1->pos - Ball2->pos).norm() < (Ball1->radius + Ball2->radius));
  }

  // double binSearchCollidedTime(boost::shared_ptr<particle>& Ball1,
  //                              boost::shared_ptr<particle>& Ball2, double start, double dt) {
  //   // update moving and calculate distance
  //   Ball1.updateEuler(start + dt);
  //   Ball2.updateEuler(start + dt);
  //   double collide_length = Ball1->radius + Ball2->radius - (Ball1->pos - Ball2->pos).norm();
  //   // revert moving
  //   Ball1.revertEuler(start + dt);
  //   Ball2.revertEuler(start + dt);
  //   if (collide_length < 0) { // pre-collided
  //     return binSearchCollidedTime(Ball1, Ball2, start + dt * 0.5, dt * 0.5);
  //   }
  //   else if (collide_length < COLLIDE_THRE) { // collided time
  //     return dt;
  //   } else { // after collided
  //     return binSearchCollidedTime(Ball1, Ball2, start, dt * 0.5);
  //   }
  // }

  void forceBasedResponse(boost::shared_ptr<particle>& Ball1,
                          boost::shared_ptr<particle>& Ball2) { // without considering collision timing
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

  // void forceBasedResponseWithColTime(boost::shared_ptr<particle>& Ball1,
  //                           boost::shared_ptr<particle>& Ball2) { // without considering collision timing
  //   // Push away form each other
  //   Eigen::Vector3d collide_dir = (Ball1->pos - Ball2->pos);
  //   double collide_length = Ball1->radius + Ball2->radius - collide_dir.norm();
  //   collide_dir.normalize();
  //   Ball1->pos += collide_length * 0.5 * collide_dir;
  //   Ball2->pos -= collide_length * 0.5 * collide_dir;
  //   // Calculate velocity after collision
  //   double a = 2 * collide_dir.dot(Ball1->vel - Ball2->vel) / (1/ Ball1->mass + 1/ Ball2->mass);
  //   Ball1->vel -= a / Ball1->mass * collide_dir;
  //   Ball2->vel += a / Ball2->mass * collide_dir;
  //   // Update prev_pos
  //   Ball1->prev_pos = Ball1->pos - Ball1->vel * Ball1->last_dt;
  //   Ball2->prev_pos = Ball2->pos - Ball2->vel * Ball2->last_dt;
  // }

  void posBasedResponse(boost::shared_ptr<particle>& Ball1,
                          boost::shared_ptr<particle>& Ball2) { // equal mass
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
    for ( uint i=0; i<this->pl.size(); i++ ) {
      this->pl[i]->updateVerlet(dt);
    }
  }

  void updateEulerAll(double dt) {
    for ( uint i=0; i<this->pl.size(); i++ ) {
      this->pl[i]->updateEuler(dt);
    }
  }

  int getParticleAmount() { return this->pl.size(); }

  void checkBounceCollide(boost::shared_ptr<particle>& Ball) {
    if (Ball->pos[0] > this->table_length - Ball->radius) {
      Ball->pos[0] = this->table_length - Ball->radius;
      Ball->vel[0] = -Ball->vel[0];
    } else if (Ball->pos[0] < - this->table_length + Ball->radius) {
      Ball->pos[0] = - this->table_length + Ball->radius;
      Ball->vel[0] = -Ball->vel[0];
    }
    if (Ball->pos[1] > this->table_length - Ball->radius) {
      Ball->pos[1] = this->table_length - Ball->radius;
      Ball->vel[1] = -Ball->vel[1];
    } else if (Ball->pos[1] < - this->table_length + Ball->radius) {
      Ball->pos[1] = - this->table_length + Ball->radius;
      Ball->vel[1] = - Ball->vel[1];
    }
    Ball->prev_pos = Ball->pos - Ball->vel * Ball->last_dt;
  }

  void checkBounceCollideAll() {
    for ( uint i=0; i<this->pl.size(); i++ ) {
      checkBounceCollide(this->pl[i]);
    }
  }

  void setGravitationalForceAll() {
    for ( uint i=0; i<this->pl.size(); i++ ) {
      // Reset all force/acc to zero
      this->pl[i]->force = Eigen::Vector3d::Zero();
      // this->pl[i]->acc = Eigen::Vector3d::Zero();
      for ( uint j=0; j<this->pl.size(); j++ ) {
        if ( i == j ) continue;
        Eigen::Vector3d dist_vec = (this->pl[j]->pos - this->pl[i]->pos);
        double dist = std::max(dist_vec.norm(), EPSILON); // Deal with too small distance problem
        this->pl[i]->force += G_CONST * this->pl[i]->mass * this->pl[j]->mass
         / (dist * dist * dist) * dist_vec;
        // this->pl[i]->acc += G_CONST * this->pl[j]->mass / (dist * dist * dist) * dist_vec;
      }
    }
  }

  void calcForceFromNode(int pid, boost::shared_ptr<node>& _node) {
    Eigen::Vector3d dist_vec;
    double dist;
    if (_node->childs.size() > 0) {
      Eigen::Vector3d com = _node->sum_of_pos_mass / _node->mass;
      dist_vec = (com - this->pl[pid]->pos);
      dist = std::max(dist_vec.norm(), EPSILON); // Deal with too small distance problem
      if (_node->size / dist < FAR_ENOUGH_THRE) {
        this->pl[pid]->force += G_CONST * this->pl[pid]->mass * _node->mass
          / (dist * dist * dist) * dist_vec;
        // this->pl[pid]->acc += G_CONST * _node->mass / (dist * dist * dist) * dist_vec;
      } else {
        for (uint i=0; i < _node->childs.size(); i++) {
          calcForceFromNode(pid, _node->childs[i]);
        }
      }
    } else if (_node->p_id != -1 and _node->p_id != pid) {
      dist_vec = (this->pl[_node->p_id]->pos - this->pl[pid]->pos);
      dist = std::max(dist_vec.norm(), EPSILON); // Deal with too small distance problem
      this->pl[pid]->force += G_CONST * this->pl[pid]->mass * this->pl[_node->p_id]->mass
        / (dist * dist * dist) * dist_vec;
      // this->pl[pid]->acc += G_CONST * this->pl[_node->p_id]->mass / (dist * dist * dist) * dist_vec;
    }
  }

  void setGravitationalForceAllWithBHAlg() {
    // Create quadtree
    boost::shared_ptr<node> root(new node(2 * this->table_length,
                                          -this->table_length, -this->table_length));
    for ( uint i=0; i<this->pl.size(); i++ ) {
      root->add_particle(i, pl);
    }
    // Calculate gravitational force
    for ( uint i=0; i<this->pl.size(); i++ ) {
      // Reset all force/acc to zero
      // this->pl[i]->acc = Eigen::Vector3d::Zero();
      this->pl[i]->force = Eigen::Vector3d::Zero();
      calcForceFromNode(i, root);
    }
  }

  void update(double dt) {
    updateVerletAll(dt);
    // updateEulerAll(dt);
    for ( uint i=0; i<this->pl.size(); i++ ) {
      for ( uint j=i+1; j<this->pl.size(); j++ ) {
        if (is_collided(this->pl[i], this->pl[j])) {
          posBasedResponse(this->pl[i], this->pl[j]);
        }
      }
    }
    checkBounceCollideAll();
  }

  void updateWithGravitationalForce(double dt) {
    setGravitationalForceAll();
    // setGravitationalForceAllWithBHAlg();
    updateVerletAll(dt);
    // updateEulerAll(dt);
    // for ( int i=0; i<this->pl.size(); i++ ) {
    //   for ( int j=i+1; j<this->pl.size(); j++ ) {
    //     if (is_collided(this->pl[i], this->pl[j])) {
    //       posBasedResponse(this->pl[i], this->pl[j]);
    //     }
    //   }
    // }
    checkBounceCollideAll();
  }
};
