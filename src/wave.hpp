#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <cmath>

double grav = 9.8;

class wave {
public:
  double max_x, max_y;
  double dt, dx, dy;
  Eigen::MatrixXd abs_h, gr, h, h_buf;
  Eigen::MatrixXd vx, vy, vx_buf, vy_buf;
  //
  int max_i, max_j;
  double volume;
  double ggain;

  wave() {}
  ~wave() {}

  void init(double _max_x, double _max_y, double _dt, double _dx, double _dy) {
    this->max_x = _max_x;
    this->max_y = _max_y;
    this->dt = _dt;
    this->dx = _dx;
    this->dy = _dy;
    this->max_i = (int)(this->max_x / this->dx);
    this->max_j = (int)(this->max_y / this->dy);
    this->abs_h = Eigen::MatrixXd(this->max_i, this->max_j);
    this->gr = Eigen::MatrixXd::Zero(this->max_i, this->max_j); // set all ground as 0
    this->h = Eigen::MatrixXd(this->max_i, this->max_j);
    this->vx = Eigen::MatrixXd(this->max_i, this->max_j);
    this->vy = Eigen::MatrixXd(this->max_i, this->max_j);
    this->h_buf = Eigen::MatrixXd(this->max_i, this->max_j);
    this->vx_buf = Eigen::MatrixXd(this->max_i, this->max_j);
    this->vy_buf = Eigen::MatrixXd(this->max_i, this->max_j);
  }

  void calc_volume() {
    this->volume = 0;
    for ( int i=0; i<this->max_i; i++ ) {
      for ( int j=0; j<this->max_j; j++ ) {
        this->volume += this->h(i,j);
      }
    }
  }

  void fix_volume_conservation() {
    double vol = this->volume;
    calc_volume();
    double add = (volume - this->volume)/(this->max_i*this->max_j);
    for ( int i=0; i<this->max_i; i++ ) {
      for ( int j=0; j<this->max_j; j++ ) {
        this->h(i,j) += add;
      }
    }
    this->volume = volume;
  }

  void set_const_height(double h, int i0, int j0, int di, int dj) {
    for ( int i=i0; i<di+i0; i++ ) {
      if ( i>=this->max_i ) break;
      for ( int j=j0; j<dj+j0; j++ ) {
        if ( j>=this->max_j ) break;
        this->h(i,j) = h;
      }
    }
    this->abs_h = this->h + this->gr;
    calc_volume();
  }

  void set_random_height(double _h, bool seed=false) {
    if ( seed ) std::srand(std::time(0));
    for ( int i=1; i<this->max_i-1; i++ ) {
      for ( int j=1; j<this->max_j-1; j++ ) {
        this->h(i,j) += (_h * ((2.0 * std::rand())/RAND_MAX - 1));
      }
    }
    this->abs_h = this->h + this->gr;
    calc_volume();
  }

  void set_exp_height(double _max_h, int _max_i, int _max_j) {
    double cent_x = this->max_x * 0.5, cent_y = this->max_y * 0.5;
    int cent_i = this->max_i / 2, cent_j = this->max_j / 2;
    for ( int i=-max_i/2 ; i<_max_i/2; i++ ) {
      if ( i+cent_i>=this->max_i || i+cent_i < 0) break;
      for ( int j=0; j<_max_j; j++ ) {
        if ( j+cent_j>=this->max_j || j+cent_j <0 ) break;
        this->h(i + cent_i,j + cent_j) += _max_h * exp(- pow(i*this->dx, 2)
                                                       - pow(j*this->dy, 2));
      }
    }
    this->abs_h = this->h + this->gr;
    calc_volume();
  }

  double get_value_with_bound_check(int i, int j, Eigen::MatrixXd &map) {
    if ( i < 0 ) i=0; else if(i>=map.rows()) i = map.rows()-1;
    if ( j < 0 ) j=0; else if(j>=map.cols()) j = map.cols()-1;
    return map(i,j);
  }

  double bilinear_interpolate(double x, double y, Eigen::MatrixXd *s) {
    double i = x / this->dx;
    double j = y / this->dy;
    int i0 = (int)i;
    int j0 = (int)j;
    int i1 = i0+1;
    int j1 = j0+1;
    double w00 = (1 - (i-i0)) * (1 - (j-j0));
    double w01 = (1 - (i-i0)) * (1 - (j1-j));
    double w11 = (1 - (i1-i)) * (1 - (j1-j));
    double w10 = (1 - (i1-i)) * (1 - (j-j0));
    double ret = get_value_with_bound_check(i0,j0,*s) * w00
      + get_value_with_bound_check(i0,j1,*s) * w01
      + get_value_with_bound_check(i1,j1,*s) * w11
      + get_value_with_bound_check(i1,j0,*s) * w10;
    return ret;
  }

  void advect(Eigen::MatrixXd *ret, Eigen::MatrixXd *s, Eigen::MatrixXd *v1, Eigen::MatrixXd *v2) {
    for ( int i=0; i<this->max_i-1; i++ ) {
      for ( int j=0; j<this->max_j-1; j++ ) {
        double x_vxij = i*this->dx - this->dt * (*v1)(i,j);
        double y_vyij = j*this->dy - this->dt * (*v2)(i,j);
        (*ret)(i,j) = bilinear_interpolate(x_vxij, y_vyij, s);
      }
    }
  }

  void update_height() {
    for ( int i=0; i<this->max_i-1; i++ ) {
      for ( int j=0; j<this->max_j-1; j++ ) {
        this->h(i,j) -= this->h(i,j) * ((this->vx(i+1, j) - this->vx(i,j))/this->dx +
                                        (this->vy(i, j+1) - this->vy(i,j))/this->dy) * this->dt;
      }
    }
  }

  void update_velocity() {
    for ( int i=1; i<this->max_i; i++ ) {
      for ( int j=0; j<this->max_j; j++ ) {
        this->vx(i,j) += grav * (this->abs_h(i-1,j) - this->abs_h(i,j)) / this->dx * this->dt;
      }
    }
    for ( int i=0; i<this->max_i; i++ ) {
      for ( int j=1; j<this->max_j; j++ ) {
        this->vy(i,j) += grav * (this->abs_h(i,j-1) - this->abs_h(i,j)) / this->dy * this->dt;
      }
    }
  }

  void constrained_with_reflecting_boundary() {
    for ( int j=0; j<this->max_j; j++ ) {
      this->abs_h(0,j) = this->abs_h(1,j);
      this->vx(1,j) = this->vx(0,j) = this->vy(0,j) = 0;
      //
      this->abs_h(this->max_i-1,j) = this->abs_h(this->max_i-2,j);
      this->vx(this->max_i-2,j) = this->vx(this->max_i-1,j) = this->vy(this->max_i-1,j) = 0;
    }
    //
    for ( int i=0; i<this->max_i; i++ ) {
      this->abs_h(i,0) = this->abs_h(i,1);
      this->vx(i,0) = this->vy(i,0) = this->vy(i,1) = 0;
      //
      this->abs_h(i,this->max_j-1) = this->abs_h(i,this->max_j-2);
      this->vx(i,this->max_j-1) = this->vy(i,this->max_j-1) = this->vy(i,this->max_j-2) = 0;
    }
    //
    this->abs_h(0,0) = (this->abs_h(0,1) + this->abs_h(1,0))/2;
    this->abs_h(0,this->max_j-1) = (this->abs_h(0,this->max_j-2)
                                    + this->abs_h(1,this->max_j-1))/2;
    this->abs_h(this->max_i-1,0) = (this->abs_h(this->max_i-1,1)
                                    + this->abs_h(this->max_i-2,0))/2;
    this->abs_h(this->max_i-1,this->max_j-1) = (this->abs_h(this->max_i-1,this->max_j-2)
                                                + this->abs_h(this->max_i-2,this->max_j-1))/2;
  }

  void update() {
    this->h_buf = this->h; this->vx_buf = this->vx; this->vy_buf = this->vy;
    // h = advect(h,v)
    advect(&this->h, &this->h_buf, &this->vx_buf, &this->vy_buf);
    // vx = advect(vx,v), vy = advect(vy,v)
    advect(&this->vx, &this->vx_buf, &this->vx_buf, &this->vy_buf);
    advect(&this->vy, &this->vy_buf, &this->vx_buf, &this->vy_buf);
    // Update-height
    update_height();
    // abs_h = h + gr
    this->abs_h = this->h + this->gr;
    // Update-velocity
    update_velocity();
    // boundaries condition
    constrained_with_reflecting_boundary();
    fix_volume_conservation();
  }
};
