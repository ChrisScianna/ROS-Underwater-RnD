
class podPID {
 public:
  podPID();
  podPID(double Kp, double Ki, double Kd);
  ~podPID();

 private:
  double kp;  // proportional gain
  double ki;  // integral gain
  double kd;  // derivative gain
};
