#include <cmath>

class line_segment {
public:
  Eigen::Vector3d start, end; // cent, c_to_s, c_to_e, dstart, dend;
  //double half_length;

  line_segment(double* _start, double* _end)
    : start(Eigen::Map<Eigen::Vector3d>(_start,3)),
      end(Eigen::Map<Eigen::Vector3d>(_end,3))
  {}
  ~ line_segment() {}

  void copy(line_segment* _this) {
    _this->start = this->start;
    _this->end = this->end;
  }

  // void rotate(double alpha, Eigen::Vector3d axis) { // alpha: rotate angle (Unit: degree)
  //   this->half_length = (this->end - this->start).norm() * 0.5;
  //   this->cent = (this->end + this->start) * 0.5;
  //   this->c_to_s = this->start - cent;
  //   this->c_to_e = this->end - cent;
  //   this->dstart = axis.cross(this->c_to_s)/(axis.norm() * this->c_to_s.norm()) * sin(alpha);
  //   this->c_to_s += this->dstart;
  //   this->c_to_s = this->c_to_s / this->c_to_s.norm() * half_length;
  //   this->start = cent + this->c_to_s;
  //   //
  //   this->dend = axis.cross(this->c_to_e)/(axis.norm() * this->c_to_e.norm()) * sin(alpha);
  //   this->c_to_e += this->dend;
  //   this->c_to_e = this->c_to_e / this->c_to_e.norm() * half_length;
  //   this->end = cent + this->c_to_e;
  // }
};
