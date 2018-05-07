class circle_wire {
public:
  Eigen::Vector3d center, norm_vec;
  double radius;

  circle_wire(double* _cent, double* _norm, double _radius)
    : center(Eigen::Map<Eigen::Vector3d>(_cent,3)),
      norm_vec(Eigen::Map<Eigen::Vector3d>(_norm,3)),
      radius(_radius)
  {}
  ~ circle_wire() {}

  void copy(circle_wire* _this) {
    _this->center = this->center;
    _this->norm_vec = this->norm_vec;
    _this->radius = this->radius;
  }

};
