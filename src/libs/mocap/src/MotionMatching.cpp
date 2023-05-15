#include "mocap/MotionMatching.h"

namespace crl::mocap {

static inline void rectify_quat(Quaternion &q);
static inline double angle_from_quat(Quaternion q);
static inline double twist_from_quats(Quaternion q, Quaternion q0);
static inline V3D axis_from_quat(Quaternion q);

Inertialization::Inertialization(MocapSkeleton *model, double blendTime, double dt) : t1(blendTime), dt(dt), eval_state(model, true) {
    Nj = model->getMarkerCount();
    coeffs.resize(Nj+2, 6); /*!< prepend root position and orientation */
    q1_joint.reserve(Nj);
    axis_joint.reserve(Nj);
    for(int i = 0; i < Nj; i++) {
        q1_joint.push_back(Quaternion(1, 0, 0, 0));
        axis_joint.push_back(V3D(0,0,0));
    }
}


void Inertialization::blend(const BVHClip::StateType &lastlastState, 
                            const BVHClip::StateType &lastState, 
                            const BVHClip::StateType &nextState,
                            const BVHClip::StateType &nextnextState) {
    /* root position interpolation */
    x1_root = nextState.getRootPosition();
    V3D x0_root(x1_root, lastState.getRootPosition());
    x0_root_unit = x0_root.unit();
    V3D x00_root(x1_root, lastlastState.getRootPosition());
    double v0 = (x0_root.norm()- x00_root.dot(x0_root)/x0_root.norm())/dt;
    V3D x01_root(nextnextState.getRootPosition(), lastState.getRootPosition());
    double v1 = (x0_root.norm()- x01_root.dot(x0_root)/x0_root.norm())/dt;
    calc_coeff(0, x0_root.norm(), v0, 0., v1);

    /* root orientation interpolation */
    q1_root = nextState.getRootOrientation();
    Quaternion q0_root = lastState.getRootOrientation()*q1_root.inverse();
    axis_root = axis_from_quat(q0_root);
    Quaternion q00_root = lastlastState.getRootOrientation()*q1_root.inverse();
    v0 = (angle_from_quat(q0_root) - twist_from_quats(q00_root, q0_root))/dt;
    Quaternion q01_root = lastState.getRootOrientation()*nextnextState.getRootOrientation().inverse();
    v1 = (angle_from_quat(q0_root) - twist_from_quats(q01_root, q0_root))/dt;
    calc_coeff(1, angle_from_quat(q0_root), v0, 0., v1);

    /* joint rotation interpolation */
    for(int i = 0; i < Nj; i++) {
        q1_joint[i] = nextState.getJointRelativeOrientation(i);

        Quaternion q0_joint = lastState.getJointRelativeOrientation(i)*q1_joint[i].inverse();
        axis_joint[i] = axis_from_quat(q0_joint);

        Quaternion q00_joint = lastlastState.getJointRelativeOrientation(i)*q1_joint[i].inverse();
        v0 = (angle_from_quat(q0_joint) - twist_from_quats(q00_joint, q0_joint))/dt;

        Quaternion q01_joint = lastState.getJointRelativeOrientation(i)*nextnextState.getJointRelativeOrientation(i).inverse();
        v1 = (angle_from_quat(q0_joint) - twist_from_quats(q01_joint, q0_joint))/dt;
        calc_coeff(i+2, angle_from_quat(q0_joint), v0, 0., v1);
    }

}

BVHClip::StateType* Inertialization::evaluate(double t) {
    if(t > t1) return &eval_state;

    Eigen::VectorXd tt(6);
    tt(0) = std::pow(t,5), tt(1) = std::pow(t,4), tt(2) = std::pow(t,3), tt(3) = std::pow(t,2), tt(4) = t,  tt(5) = 1.0;

    /* root position interpolation */
    eval_state.setRootPosition((tt.dot(coeffs.row(0))*x0_root_unit + (V3D)x1_root).data());

    // std::cout << "Time " << t << ": beta = " << tt.dot(coeffs.row(0)) << std::endl;

    /* root orientation interpolation */
    eval_state.setRootOrientation(Quaternion(Eigen::AngleAxisd(tt.dot(coeffs.row(1)), axis_root)*q1_root));

    // /* joint rotation interpolation */
    for(int i = 0; i < Nj; i++) {
        eval_state.setJointRelativeOrientation(Quaternion(Eigen::AngleAxisd(tt.dot(coeffs.row(i+2)), axis_joint[i])*q1_joint[i]), i);
    }

    return &eval_state;

}


void Inertialization::calc_coeff(int idx, double x0, double v0, double x1, double v1) {
    coeffs(idx,0) = (-1./std::pow(t1,5)*x0 - 1./(2*std::pow(t1,4))*v0 + 1./std::pow(t1,5)*x1 -1./(2*std::pow(t1,4))*v1);
    coeffs(idx,1) = (5./(2*std::pow(t1,4))*x0 + 5./(4*std::pow(t1,3))*v0 -5./(2*std::pow(t1,4))*x1 + 5./(4*std::pow(t1,3))*v1);
    coeffs(idx,2) = 0.0;
    coeffs(idx,3) = (-5./(2*std::pow(t1,2))*x0 - 7./(4*t1)*v0 + 5./(2*std::pow(t1,2))*x1 - 3./(4*t1)*v1);
    coeffs(idx,4) = v0;
    coeffs(idx,5) = x0;
}

void Inertialization::setBlendTime(double blendTime) {t1 = blendTime;}
double Inertialization::getBlendTime(){return t1;}

/*!< some utility functions  */
static inline void rectify_quat(Quaternion &q) {q.normalized(); if(q.w()<0) q=Quaternion(-q.coeffs());}
static inline double angle_from_quat(Quaternion q) {rectify_quat(q); return 2*std::acos(q.w()); }
static inline V3D axis_from_quat(Quaternion q) {rectify_quat(q); return V3D(q.vec()/q.vec().norm());}
static inline double twist_from_quats(Quaternion q, Quaternion q0) {rectify_quat(q); rectify_quat(q0); double prj=q.vec().dot(axis_from_quat(q0)); double t0=2*std::atan2(prj, q.w()); return t0;}

}  // namespace crl::mocap