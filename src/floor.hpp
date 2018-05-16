class static_floor {
public:
  Eigen::Vector3d origin, norm_vec;
  double elasticity;

  static_floor(double* _org, double* _norm, double _elas)
    : origin(Eigen::Map<Eigen::Vector3d>(_org,3)),
      norm_vec(Eigen::Map<Eigen::Vector3d>(_norm,3)),
      elasticity(_elas)
  {}
  ~ static_floor() {}

  void copy(static_floor* _this) {
    _this->origin = this->origin;
    _this->norm_vec = this->norm_vec;
    _this->elasticity = this->elasticity;
  }

};
