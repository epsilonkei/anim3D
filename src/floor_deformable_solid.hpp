#include "deformable_solid2.hpp"
#include "floor.hpp"

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define WALL_DGAIN 3e4
#define WALL_VGAIN 5e2

class floor_rigids {
public:
  std::vector<boost::shared_ptr<deformable_solid> > rl;
  std::vector<boost::shared_ptr<static_floor> > floors;

  floor_rigids() {}
  ~ floor_rigids() {}

  void clearAll(){ this->rl.clear(); this->floors.clear(); }

  void add_floor(double* _org, double* _norm, double _elas) {
    this->floors.push_back(boost::shared_ptr<static_floor>(new static_floor(_org, _norm, _elas)));
  }

  void add_particle(double _elas, double _mass, double _radius, double* _prev_pos, double* _pos, double* _vel, double* _acc, uint i=0) {
    while ( this->rl.size() <= i ) this->rl.push_back(boost::shared_ptr<deformable_solid>(new deformable_solid));
    this->rl[i]->add_particle(_elas, _mass, _radius, _prev_pos, _pos, _vel, _acc);
  }


  void floor_collision_penalty(boost::shared_ptr<particle>& part, bool apply_grav=true) {
    if (apply_grav) part->force = -part->mass * grav * e3;
    else part->force = Eigen::Vector3d::Zero();
    for (uint k=0; k<this->floors.size(); k++) {
      double dist_to_floor = this->floors[k]->norm_vec.dot(part->pos - this->floors[k]->origin)
        - part->radius;
      if (dist_to_floor < 0) {
        part->force += (- WALL_DGAIN*dist_to_floor - WALL_VGAIN*
                        part->vel.dot(this->floors[k]->norm_vec)) * this->floors[k]->norm_vec;
      }
    }
  }

  void floor_collision_penalty_all(bool apply_grav=true) {
    for (uint i=0; i<this->rl.size(); i++) {
      for (uint j=0; j<this->rl[i]->pl.size(); j++) {
        floor_collision_penalty(this->rl[i]->pl[j] ,apply_grav);
      }
    }
  }

  void update_movement(double dt) {
    // Notes: initial force was apply in floor collision penalty function
    floor_collision_penalty_all(true);
    for (uint i=0; i<this->rl.size(); i++) {
      for (uint j=0; j<this->rl[i]->pl.size(); j++) {
        this->rl[i]->pl[j]->updateVerlet(dt);
      }
      this->rl[i]->update_spring_damper();
      for (uint j=0; j<this->rl[i]->pl.size(); j++) {
        this->rl[i]->pl[j]->updateVerlet(dt);
      }
    }
  }
};
