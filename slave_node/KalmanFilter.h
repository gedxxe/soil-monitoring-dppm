#pragma once
class KalmanFilter {
public:
  KalmanFilter(float q = 0.02f, float r = 8.0f, float p = 1.0f, float x = 0.0f)
  : Q(q), R(r), P(p), X(x), inited(false) {}
  float update(float z){
    if(!inited){ X=z; inited=true; }
    P += Q;
    float K = P/(P+R);
    X = X + K*(z - X);
    P = (1 - K)*P;
    return X;
  }
private:
  float Q,R,P,X; bool inited;
};
