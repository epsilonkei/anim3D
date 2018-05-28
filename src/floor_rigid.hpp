#include "rigid.hpp"
#include "floor.hpp"

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

};
