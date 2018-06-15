#include "rigid.hpp"
#include "floor.hpp"

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define DGAIN 5e3
#define VGAIN 1e3

class floor_rigids {
public:
  std::vector<boost::shared_ptr<rigid_body> > rl;
  std::vector<boost::shared_ptr<static_floor> > floors;

  floor_rigids() {}
  ~ floor_rigids() {}

  void clearAll(){ this->rl.clear(); this->floors.clear(); }

  void add_floor(double* _org, double* _norm, double _elas) {
    this->floors.push_back(boost::shared_ptr<static_floor>(new static_floor(_org, _norm, _elas)));
  }

  void add_particle(double _elas, double _mass, double _radius, double* _prev_pos, double* _pos, double* _vel, double* _acc, int i=0) {
    while ( this->rl.size() <= i ) this->rl.push_back(boost::shared_ptr<rigid_body>(new rigid_body));
    this->rl[i]->add_particle(_elas, _mass, _radius, _prev_pos, _pos, _vel, _acc);
  }

  void floor_collision_penalty(boost::shared_ptr<particle>& part) {
    for (int k=0; k<this->floors.size(); k++) {
      double dist_to_floor = this->floors[k]->norm_vec.dot(part->pos - this->floors[k]->origin);
      //- part.radius;
      part->force = -part->mass * grav * e3;
      if (dist_to_floor < 0) {
        part->force += (- DGAIN*dist_to_floor - VGAIN*
                        part->vel.dot(-dist_to_floor*this->floors[k]->norm_vec))
          * (-dist_to_floor*this->floors[k]->norm_vec);
      }
    }
  }

  void floor_collision_penalty_all() {
    for (int i=0; i<this->rl.size(); i++) {
      for (int j=0; j<this->rl[i]->pl.size(); j++) {
        floor_collision_penalty(this->rl[i]->pl[j]);
      }
    }
  }

  void update_movement(double dt) {
    floor_collision_penalty_all();
    for (int i=0; i<this->rl.size(); i++) {
      rl[i]->update_rigid_movement(dt);
      rl[i]->update_particles_movement();
    }
  }
};
