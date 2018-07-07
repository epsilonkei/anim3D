#include "fluid.hpp"
#include "floor.hpp"

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define WALL_DGAIN 3e4
#define WALL_VGAIN 5e2

class floors_fluid {
public:
  std::vector<boost::shared_ptr<static_floor> > floors;
  std::vector<boost::shared_ptr<fluid> > fluids;

  floors_fluid() {}
  ~ floors_fluid() {}

  void clearAll(){ this->floors.clear(); this->fluids.clear(); }

  void add_floor(double* _org, double* _norm, double _elas) {
    this->floors.push_back(boost::shared_ptr<static_floor>(new static_floor(_org, _norm, _elas)));
  }

  void add_particle(double _elas, double _mass, double _radius, double* _prev_pos, double* _pos, double* _vel, double* _acc, int i=0) {
    while ( this->fluids.size() <= i )
      this->fluids.push_back(boost::shared_ptr<fluid>(new fluid));
    this->fluids[i]->add_particle(_elas, _mass, _radius, _prev_pos, _pos, _vel, _acc);
  }

  void floor_collision_penalty(boost::shared_ptr<particle>& part, bool apply_grav=true) {
    if (apply_grav) part->force = -part->mass * grav * e3;
    else part->force = Eigen::Vector3d::Zero();
    for (int k=0; k<this->floors.size(); k++) {
      double dist_to_floor = this->floors[k]->norm_vec.dot(part->pos - this->floors[k]->origin)
        - part->radius;
      if (dist_to_floor < 0) {
        part->force += part->mass * (- WALL_DGAIN*dist_to_floor - WALL_VGAIN*
                        part->vel.dot(this->floors[k]->norm_vec)) * this->floors[k]->norm_vec;
      }
    }
  }

  void floor_collision_pos_response(boost::shared_ptr<particle>& part, bool apply_grav=true) {
    Eigen::Vector3d vel_on_norm, vel_other;
    if (apply_grav) part->force = -part->mass * grav * e3;
    else part->force = Eigen::Vector3d::Zero();
    for (int k=0; k<this->floors.size(); k++) {
      double dist_to_floor = this->floors[k]->norm_vec.dot(part->pos - this->floors[k]->origin)
        - part->radius;
      if (dist_to_floor < 0) {
         part->pos -= this->floors[k]->norm_vec * dist_to_floor;
         vel_on_norm = part->vel.dot(this->floors[k]->norm_vec) * this->floors[k]->norm_vec;
         vel_other = part->vel - vel_on_norm;
         part->vel = vel_other - vel_on_norm * this->floors[k]->elasticity;
         part->prev_pos = part->pos - part->vel * part->last_dt;
      }
    }
  }

  void floor_collision_penalty_all(bool apply_grav=true) {
    for (int i=0; i<this->fluids.size(); i++) {
      for (int j=0; j<this->fluids[i]->pl.size(); j++) {
        floor_collision_penalty(this->fluids[i]->pl[j] ,apply_grav);
        // floor_collision_pos_response(this->fluids[i]->pl[j] ,apply_grav);
      }
    }
  }

  void update_movement(double dt) {
    // Notes: initial force was apply in floor collision penalty function
    floor_collision_penalty_all(true);
    // for (int i=0; i<this->rl.size(); i++ ) {
    //   for (int j=i+1; j<this->rl.size(); j++ ) {
    //     collide_penalty(this->rl[i], this->rl[j]);
    //   }
    // }
    for (int i=0; i<this->fluids.size(); i++ ) {
      this->fluids[i]->update(dt);
    }
  }
};
