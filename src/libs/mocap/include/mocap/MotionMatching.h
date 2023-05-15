#pragma once

#include "mocap/MocapClip.h"

namespace crl::mocap {
class Inertialization {
public:
    Inertialization(MocapSkeleton *model, double blendTime = 0.3, double dt = 0.3);

    void blend(const BVHClip::StateType &lastlastState, 
               const BVHClip::StateType &lastState, 
               const BVHClip::StateType &nextState,
               const BVHClip::StateType &nextnextState);

    BVHClip::StateType* evaluate(double t);

    void setBlendTime(double t);

    double getBlendTime();

private:
    double t1, dt;
    int Nj;

    Eigen::MatrixXd coeffs;

    BVHClip::StateType eval_state;

    P3D x1_root;
    V3D x0_root_unit;

    Quaternion q1_root;
    V3D axis_root;

    std::vector<Quaternion> q1_joint;
    std::vector<V3D> axis_joint;

    void calc_coeff(int idx, double x0, double v0, double x1, double v1);
};

}  // namespace crl::mocap
