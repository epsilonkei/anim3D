#include "rigid.hpp"
#include "floor.hpp"

extern double grav;
extern Eigen::Vector3d e1, e2, e3;

#define WALL_DGAIN 3e3
#define WALL_VGAIN 5e1
#define RIGID_DGAIN 3e3
#define RIGID_VGAIN 5e1

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

  void add_particle(double _elas, double _mass, double _radius, double* _prev_pos, double* _pos, double* _vel, double* _acc, uint i=0) {
    while ( this->rl.size() <= i ) this->rl.push_back(boost::shared_ptr<rigid_body>(new rigid_body));
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

  bool max_radius_collided(boost::shared_ptr<rigid_body>& r1, boost::shared_ptr<rigid_body>& r2) {
    return (r1->com - r2->com).norm() < (r1->max_radius + r2->max_radius);
  }

  void update_bounding(boost::shared_ptr<rigid_body>& r) {
    for (uint i=0; i<r->bounding_plane_cent_local.size(); i++) {
      r->bounding_plane_out_normv[i] = r->rotation * r->bounding_plane_cent_local[i];
      r->bounding_plane_cent[i] = r->bounding_plane_out_normv[i] + r->com;
      r->bounding_plane_out_normv[i].normalize();
    }
  }

  // TODO: need other method when r1 is completedly inside r2
  void apply_penalty_force(boost::shared_ptr<rigid_body>& r1, boost::shared_ptr<rigid_body>& r2) {
    double dist_to_plane;
    bool collided; int opp_id = 0;
    for (uint id=0; id<r1->pl.size(); id++) {
      collided = true;
      for (uint j=0; j<r2->bounding_plane_cent.size(); j++) {
        dist_to_plane = r2->bounding_plane_out_normv[j].dot(r1->pl[id]->pos
                                                            - r2->bounding_plane_cent[j]);
        if (dist_to_plane > 0) {
          collided = false;
          break;
        }
      }
      if (collided) {
        if (id == 0) opp_id = 6;
        else if (id == 1) opp_id = 7;
        else if (id == 2) opp_id = 4;
        else if (id == 3) opp_id = 5;
        else if (id == 4) opp_id = 2;
        else if (id == 5) opp_id = 3;
        else if (id == 6) opp_id = 0;
        else if (id == 7) opp_id = 1;
        for (uint j=0; j<r2->bounding_plane_cent.size(); j++) {
          dist_to_plane = r2->bounding_plane_out_normv[j].dot(r1->pl[opp_id]->pos
                                                              - r2->bounding_plane_cent[j]);
          if (dist_to_plane > 0) {
            // Apply penalty force to this plane
            dist_to_plane = r2->bounding_plane_out_normv[j].dot(r1->pl[id]->pos
                                                                - r2->bounding_plane_cent[j]);
            r1->pl[id]->force += (- RIGID_DGAIN*dist_to_plane - RIGID_VGAIN*
                                 r1->pl[id]->vel.dot(r2->bounding_plane_out_normv[j]))
              * r2->bounding_plane_out_normv[j];
          }
        }
      }
    }
  }

  void collide_penalty(boost::shared_ptr<rigid_body>& r1, boost::shared_ptr<rigid_body>& r2) {
    if (max_radius_collided(r1, r2)) {
      // update bouding plane center global coord
      update_bounding(r1);
      update_bounding(r2);
      // Apply penalty force
      apply_penalty_force(r1, r2);
      apply_penalty_force(r2, r1);
    }
  }

  void update_movement(double dt) {
    // Notes: initial force was apply in floor collision penalty function
    floor_collision_penalty_all(true);
    for (uint i=0; i<this->rl.size(); i++ ) {
      for (uint j=i+1; j<this->rl.size(); j++ ) {
        collide_penalty(this->rl[i], this->rl[j]);
      }
    }
    for (uint i=0; i<this->rl.size(); i++) {
      rl[i]->update_rigid_movement(dt);
      rl[i]->update_particles_movement(dt);
    }
  }
};
