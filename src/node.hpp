#include <boost/shared_ptr.hpp>
#include "particle.hpp"
#include <vector>
#include <Eigen/Dense>

class node {
public:
  double size, bottom_left_x, bottom_left_y, mass;
  int p_id;
  std::vector<boost::shared_ptr<node> > childs;
  Eigen::Vector3d sum_of_pos_mass;

  node(double _size, double _tlx, double _tly)
    : size(_size), bottom_left_x(_tlx), bottom_left_y(_tly), mass(0),
      p_id(-1), sum_of_pos_mass(Eigen::Vector3d::Zero())
  {}
  ~ node() {}

  void add_to_child(int id, std::vector<boost::shared_ptr<particle> > pl) {
    double hsize = this->size * 0.5;
    int ix = int((pl[id]->pos[0] - this->bottom_left_x)/hsize);
    int iy = int((pl[id]->pos[1] - this->bottom_left_y)/hsize);
    int id_child = ix + iy * 2;
    this->childs[id_child]->add_particle(id, pl);
    this->mass += pl[id]->mass;
    this->sum_of_pos_mass += pl[id]->mass * pl[id]->pos;
  }

  void add_particle(int id, std::vector<boost::shared_ptr<particle> > pl) {
    if (this->childs.size() > 0) {
      add_to_child(id, pl);
    } else if (this->p_id != -1) {
      double hsize = this->size * 0.5;
      double blx = this->bottom_left_x, bly = this->bottom_left_y;
      this->childs.push_back(boost::shared_ptr<node>(new node(hsize, blx, bly)));
      this->childs.push_back(boost::shared_ptr<node>(new node(hsize, blx + hsize, bly)));
      this->childs.push_back(boost::shared_ptr<node>(new node(hsize, blx, bly + hsize)));
      this->childs.push_back(boost::shared_ptr<node>(new node(hsize, blx + hsize, bly + hsize)));
      add_to_child(this->p_id, pl);
      add_to_child(id, pl);
    } else {
      this->p_id = id;
    }
  }

};
