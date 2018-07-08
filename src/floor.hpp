class static_floor {
public:
  Eigen::Vector3d origin, norm_vec;
  double elasticity;
  double min_x, max_x, min_y, max_y, min_z, max_z;

  static_floor(double* _org, double* _norm, double _elas)
    : origin(Eigen::Map<Eigen::Vector3d>(_org,3)),
      norm_vec(Eigen::Map<Eigen::Vector3d>(_norm,3)),
      elasticity(_elas),
      min_x (- std::numeric_limits<double>::max()),
      max_x (std::numeric_limits<double>::max()),
      min_y (- std::numeric_limits<double>::max()),
      max_y (std::numeric_limits<double>::max()),
      min_z (- std::numeric_limits<double>::max()),
      max_z (std::numeric_limits<double>::max())
  {}
  ~ static_floor() {}

  void set_xlimit(double _min_x, double _max_x) {
    min_x = _min_x;
    max_x = _max_x;
  }

  void set_ylimit(double _min_y, double _max_y) {
    min_y = _min_y;
    max_y = _max_y;
  }

  void set_zlimit(double _min_z, double _max_z) {
    min_z = _min_z;
    max_z = _max_z;
  }

  void copy(static_floor* _this) {
    _this->origin = this->origin;
    _this->norm_vec = this->norm_vec;
    _this->elasticity = this->elasticity;
  }

};
