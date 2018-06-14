double grav = 9.8;
Eigen::Vector3d e1(1,0,0), e2(0,1,0), e3(0,0,1);

class particle {
public:
  Eigen::Vector3d prev_pos, pos, pos_to_cent, vel, acc;
  double mass, radius;
  Eigen::Vector3d force;
  // buf
  double last_dt, time;

  particle() {}
  particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc)
    : prev_pos(Eigen::Map<Eigen::Vector3d>(_prev_pos,3)),
      pos(Eigen::Map<Eigen::Vector3d>(_pos,3)),
      vel(Eigen::Map<Eigen::Vector3d>(_vel,3)),
      acc(Eigen::Map<Eigen::Vector3d>(_acc,3)),
      pos_to_cent(Eigen::Vector3d::Zero()),
      mass(_mass), radius(_radius), last_dt(_last_dt), time(0),
      force(Eigen::Vector3d::Zero())
  {}
  ~ particle() {
    // std::cout << "destroy particle" << std::endl;
  }

  void initialize(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->prev_pos = Eigen::Map<Eigen::Vector3d>(_prev_pos,3);
    this->pos = Eigen::Map<Eigen::Vector3d>(_pos,3);
    this->vel = Eigen::Map<Eigen::Vector3d>(_vel,3);
    this->acc = Eigen::Map<Eigen::Vector3d>(_acc,3);
    this->mass = _mass;
    this->radius = _radius;
    this->last_dt = _last_dt;
    this->time = 0;
    this->force = Eigen::Vector3d::Zero();
  }

  void init() {
    this->prev_pos = this->pos - last_dt * this->vel;
  }

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
    // this->force = - this->mass * grav * e3;
    this->acc = this->force / this->mass;
    this->prev_pos = this->pos;
    this->vel += dt * this->acc;
    this->pos += dt * this->vel;
    this->last_dt = dt;
    this->time += dt;
  }

  void revertEuler(double dt) {
    // this->force = - this->mass * grav * e3;
    this->acc = this->force / this->mass;
    this->time -= dt;
    this->last_dt = dt;
    this->pos -= dt * this->vel;
    this->vel -= dt * this->acc;
    this->prev_pos = this->pos - dt * this->vel;
  }

  void updateVerlet(double dt) {
    // this->force = - this->mass * grav * e3;
    this->acc = this->force / this->mass;
    Eigen::Vector3d pos_buf = this->pos;
    this->pos += (this->pos - this->prev_pos) + dt * dt * this->acc;
    this->prev_pos = pos_buf;
    this->vel = (this->pos - this->prev_pos) / dt;
    this->last_dt = dt;
    this->time += dt;
  }

  void revertVerlet(double dt) {
    // this->force = - this->mass * grav * e3;
    this->time -= dt;
    this->last_dt = dt;
    this->acc = this->force / this->mass;
    this->pos = this->prev_pos;
    Eigen::Vector3d pos_buf = this->pos;
    this->prev_pos = 2 * this->pos - pos_buf + dt * dt * this->acc;
    this->vel = (this->pos - this->prev_pos) / dt;
  }

};
