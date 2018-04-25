class particle {
public:
  Eigen::Vector3d prev_pos, pos, vel, acc;
  double mass, radius;
  Eigen::Vector3d force;
  // buf
  double last_dt, time;

  particle(double _mass, double _radius, double* _prev_pos, double* _pos, double* _vel, double* _acc)
    : prev_pos(Eigen::Map<Eigen::Vector3d>(_prev_pos,3)),
      pos(Eigen::Map<Eigen::Vector3d>(_pos,3)),
      vel(Eigen::Map<Eigen::Vector3d>(_vel,3)),
      acc(Eigen::Map<Eigen::Vector3d>(_acc,3)),
      mass(_mass), radius(_radius), last_dt(0), time(0),
      force(Eigen::Vector3d::Zero())
  {}
  ~ particle() {}

  void copy(particle* _this) {
    _this->prev_pos = this->prev_pos;
    _this->pos = this->pos;
    _this->vel = this->vel;
    _this->acc = this->acc;
    _this->mass = this->mass;
    _this->radius = this->radius;
    _this->force = this->force;
    _this->last_dt = this->last_dt;
    _this->time = this->time;
  }

  void updateEuler(double dt) {
    // this->acc = this->force / this->mass;
    this->prev_pos = this->pos;
    this->vel += dt * this->acc;
    this->pos += dt * this->vel;
    this->last_dt = dt;
    this->time += dt;
  }

};
