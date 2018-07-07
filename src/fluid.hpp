#include <boost/shared_ptr.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <cmath>
#include <vector>
#include "particle.hpp"

extern double grav;
extern Eigen::Vector3d e1, e2, e3;
double PI = 3.14159265;
double K_PRESS = 20;
double VISC_COEFF = 0.018;
double MAX_ACC = 75.0;
double MAX_VEL = 75.0;
double MOTION_DP_COEFF = 0.0;
double COURANT_SAFETY_FACTOR = 1.0;
double SPECIFIC_HEATS_RATIO = 1.0;
double EPSILON = 1e-9;

class fluid {
public:
  std::vector<boost::shared_ptr<particle> > pl;
  // std::vector<std::vector<int> > nb;
  double init_dens;
  double kern_size, poly6_coeff, spikey_coeff, visc_laplacian_coeff;

  fluid() {}
  ~fluid() {}

  void add_particle(double _mass, double _radius, double _last_dt, double* _prev_pos, double* _pos, double* _vel, double* _acc) {
    this->pl.push_back(boost::shared_ptr<particle>(new particle(_mass, _radius, _last_dt, _prev_pos, _pos, _vel, _acc)));
  }

  void init(double _kern_size, double _init_dens) {
    this->kern_size = _kern_size;
    this->poly6_coeff = 315.0/(64 *PI * pow(this->kern_size, 9));
    this->spikey_coeff = -45.0/(PI * pow(this->kern_size, 6));
    this->visc_laplacian_coeff = 45.0/(PI * pow(this->kern_size, 6)); // ???
    this->init_dens = _init_dens;
  }

  void find_neighbor() {

  }

  void update_density_and_pressure() {
    for (uint i = 0; i < this->pl.size(); i++) {
      uint j_nb;
      double dist_sq, diff, _dens = 0.0;
      // Use neighboring particles
      // for (uint j=0; j < nb[i].size(); j++) {
      //   j_nb = nb[i][j];
      //   dist_sq = (pl[i]->pos - pl[j_nb]->pos).squaredNorm();
      //   diff = this->kern_size * this->kern_size - dist_sq;
      //   pl[i]->dens += pl[j_nb]->mass * this->poly6_coeff * diff * diff * diff;
      // }
      // Use all particles
      for (uint j=0; j < this->pl.size(); j++) {
        if (i == j) continue;
        dist_sq = (this->pl[i]->pos - this->pl[j]->pos).squaredNorm();
        diff = this->kern_size * this->kern_size - dist_sq;
        if (diff > 0)
          _dens += this->pl[j]->mass * this->poly6_coeff * diff * diff * diff;
      }
      this->pl[i]->dens = std::max(_dens, this->init_dens);
      this->pl[i]->pres = K_PRESS*(this->pl[i]->dens - this->init_dens);
    }
  }

  void update_fluid_acc() {
    uint j_nb;
    double dist, diff, spikey, massRatio, pterm, lap, mag;
    Eigen::Vector3d r, acc, vdiff, damp;
    for (uint i=0; i<this->pl.size(); i++) {
      acc = Eigen::Vector3d::Zero();
      // Use neighboring particles
      // for (uint j=0; j < nb[pid].size(); j++) {
      //   j_nb = nb[pid][j];
      //   r = (pl[i]->pos - pl[j_nb]->pos);
      //   dist = r.norm();
      //   if (dist == 0.0) { continue; }
      //   r = r/dist;
      //   // acceleration due to pressure term
      //   diff = this->kern_size - dist;
      //   spikey = this->spikey_coeff*diff*diff;
      //   massRatio = pl[j_nb]->mass/pl[i]->mass;
      //   pterm = (pl[i]->pres + pl[j_nb]->pres) / (2*pl[i]->dens * pl[j_nb]->dens);
      //   acc -= (massRatio*pterm*spikey)*r;
      //   // acceleration due to viscosity term
      //   if (!pl[j_nb]->isObstacle) {
      //     lap = this->visc_laplacian_coeff * diff;
      //     vdiff = pj->velocity - pi->velocity;
      //     acc += (VISC_COEFF * massRatio * (1/pj->density) * lap) *vdiff;
      //   }
      // }
      // Use all particles
      for (uint j=0; j < this->pl.size(); j++) {
        if (i == j) continue;
        r = (this->pl[i]->pos - this->pl[j]->pos);
        dist = r.norm();
        if (dist == 0.0) continue;
        r = r/dist;
        // acceleration due to pressure term
        diff = this->kern_size - dist;
        if (diff > 0) {
          spikey = this->spikey_coeff*diff*diff;
          massRatio = this->pl[j]->mass / this->pl[i]->mass;
          pterm = (this->pl[i]->pres + this->pl[j]->pres) /
            (2*this->pl[i]->dens * this->pl[j]->dens);
          acc -= (massRatio*pterm*spikey)*r;
          // acceleration due to viscosity term
          // if (!pl[j]->isObstacle) {
          //   lap = this->visc_laplacian_coeff * diff;
          //   vdiff = pl[j]->vel - pl[i]->vel;
          //   acc += (VISC_COEFF * massRatio * (1/pl[j]->dens) * lap) *vdiff;
          // }
        }
      }
      // motion damping;
      mag = acc.norm();
      // if (motion_damping_enabled) {
      //   damp = pl[i]->vel * MOTION_DP_COEFF;
      //   if (damp.norm() > mag) {
      //     acc = Eigen::Vector3d::Zero();
      //   } else {
      //     acc -= damp;
      //   }
      // }
      if (mag > MAX_ACC) {
        acc = (acc / mag) * MAX_ACC;
      }
      this->pl[i]->force += this->pl[i]->mass * acc;
    }
  }

  double calc_sound_speed(int pid) {
    if (this->pl[pid]->dens < 1e-5) {
        return 0.0;
    }
    return sqrt(SPECIFIC_HEATS_RATIO * (this->pl[pid]->pres)/this->pl[pid]->dens);
  }

  double calc_time_step() {
    double maxv = 0.0;         // max velocity
    double maxc = 0.0;         // max speed of sound
    double maxa = 0.0;         // max accelleration
    double _v, _a, _c;
    for (uint i=0; i<pl.size(); i++) {
      _v = pl[i]->vel.norm();
      _a = pl[i]->acc.norm();
      _c = calc_sound_speed(i);
      if (_v > maxv) { maxv = _v; }
      if (_c > maxc) { maxc = _c; }
      if (_a > maxa) { maxa = _a; }
    }
    double vStep = COURANT_SAFETY_FACTOR * this->kern_size / std::max(1.0, maxv);
    double cStep = COURANT_SAFETY_FACTOR * this->kern_size / std::max(EPSILON, maxc);
    double aStep = sqrt(this->kern_size / std::max(EPSILON, maxa));
    return std::max(vStep, std::max(cStep, aStep));
  }

  void update(double dt) {
    double time_left = dt, time_step;
    while (time_left > 0.0) {
      // Search neighboring particles
      find_neighbor();
      // Update density and pressure
      update_density_and_pressure();
      // Update acceleration
      update_fluid_acc();
      // calculate next time step
      time_step = calc_time_step();
      time_left -= time_step;
      if (time_left < 0.0) {
        time_step = time_step + time_left;
        time_left = 0.0;
      }
      // Move particles
      for(uint i=0; i<pl.size(); i++) {
        pl[i]->updateVerlet(time_step);
      }
      // updateFluidPosition(time_step);
      // updateObstacleVelocity(time_step);
    }
  }
};
