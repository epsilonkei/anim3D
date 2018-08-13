#ifndef STOP_WATCH_HPP
#define STOP_WATCH_HPP

#include <ctime>

// used to computed simulation times
class stop_watch {
public:
  bool isStarted = false;
  timespec ts_begin, ts_end;
  double timeRunning = 0.0;

  stop_watch() {}
  ~ stop_watch() {}

  void start() {
    clock_gettime(CLOCK_REALTIME, &ts_begin);
    isStarted = true;
  }

  void stop() {
    if (!isStarted) {
      return;
    }
    clock_gettime(CLOCK_REALTIME, &ts_end);
    double time = (ts_end.tv_sec - ts_begin.tv_sec) + (ts_end.tv_nsec - ts_begin.tv_nsec) / 1e9;
    timeRunning += time;
  }

  void reset() {
    isStarted = false;
    timeRunning = 0.0;
  }

  double getTime() { // in seconds
    return timeRunning;
  }
};

#endif // STOP_WATCH_HPP
