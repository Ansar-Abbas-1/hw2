#include "kdl_control.h"
#include <cmath>
#include <stdexcept>


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp,
                                      double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;

    return robot_->getJsim() *
               (ddqd + _Kd * de + _Kp * e)
           + robot_->getCoriolis()
           + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kpo,
                                      double _Kdp,
                                      double _Kdo)
{
}

Eigen::VectorXd KDLController::velocity_ctrl_null(
    const Eigen::Vector3d &_ep,
    double _Kp,
    double _lambda)
{
    // Current joint positions
    Eigen::VectorXd q = robot_->getJntValues();


    // Joint limits
    Eigen::MatrixXd limits = robot_->getJntLimits();

    // EE Jacobian
    Eigen::MatrixXd J =
        robot_->getEEJacobian().data;

    // Position Jacobian only
    Eigen::MatrixXd Jp =
        J.topRows(3);

    // Pseudoinverse
    Eigen::MatrixXd Jp_pinv =
        pseudoinverse(Jp);

    // Primary task
    Eigen::VectorXd qdot_task =
        Jp_pinv * (_Kp * _ep);

    // Joint-limit avoidance velocity
    Eigen::VectorXd qdot0(q.size());

    for (int i = 0; i < q.size(); ++i)
    {
        double q_lower = limits(i, 0);
        double q_upper = limits(i, 1);



        qdot0(i) =
            -(1.0 / _lambda) *
            std::pow(q_upper - q_lower, 2) *
            (2.0*q(i) - q_upper - q_lower) /
            (
                std::pow(q_upper - q(i), 2) *
                std::pow(q(i) - q_lower, 2)
            );
    }

    // Null-space projector
    Eigen::MatrixXd I =
        Eigen::MatrixXd::Identity(q.size(), q.size());

    Eigen::MatrixXd N =
        I - Jp_pinv * Jp;

    // Null-space velocity
    Eigen::VectorXd qdot_null =
        N * qdot0;


    // Total velocity
    return qdot_task + qdot_null;
}

Eigen::VectorXd KDLController::vision_ctrl(
    const Eigen::Vector3d &_s,
    const Eigen::Vector3d &_s_desired,
    double _marker_distance,
    const Eigen::Matrix3d &_Rc,
    const Eigen::MatrixXd &_Jc,
    double _Kp,
    double _lambda)
{
    // ---------------------------------------------------------
    // Safety checks
    // ---------------------------------------------------------

    if (_marker_distance < 1e-6)
    {
        return Eigen::VectorXd::Zero(robot_->getNrJnts());
    }

    if (_Jc.rows() != 6)
    {
        return Eigen::VectorXd::Zero(robot_->getNrJnts());
    }


    // ---------------------------------------------------------
    // Current robot configuration
    // ---------------------------------------------------------

    Eigen::VectorXd q = robot_->getJntValues();

    const int n = q.size();


    // ---------------------------------------------------------
    // Interaction matrix L(s)
    //
    // Homework equation:
    //
    // L(s) =
    // [ -(1/||cPo||)(I - s s^T)    S(s) ] R
    //
    // with
    //
    // R = [ Rc^T     0
    //        0       Rc^T ]
    // ---------------------------------------------------------

    Eigen::Matrix3d I3 =
        Eigen::Matrix3d::Identity();

    Eigen::Matrix3d L_translation =
        -(1.0 / _marker_distance) *
        (I3 - _s * _s.transpose());

    Eigen::Matrix3d L_rotation =
        skew(_s);


    // First part of L(s), before multiplication by R
    Eigen::Matrix<double, 3, 6> L0;

    L0.block<3,3>(0,0) = L_translation;
    L0.block<3,3>(0,3) = L_rotation;


    // ---------------------------------------------------------
    // Build:
    //
    // R = diag(Rc^T, Rc^T)
    // ---------------------------------------------------------

    Eigen::Matrix<double, 6, 6> R =
        Eigen::Matrix<double, 6, 6>::Zero();

    R.block<3,3>(0,0) = _Rc.transpose();
    R.block<3,3>(3,3) = _Rc.transpose();


    // Complete interaction matrix
    Eigen::Matrix<double, 3, 6> L =
        L0 * R;


    // ---------------------------------------------------------
    // Combined visual Jacobian
    //
    // A = L(s) Jc
    //
    // Dimensions:
    //
    // L   : 3 x 6
    // Jc  : 6 x 7
    // A   : 3 x 7
    // ---------------------------------------------------------

    Eigen::MatrixXd A =
        L * _Jc;

    Eigen::MatrixXd A_pinv =
        pseudoinverse(A);


    // ---------------------------------------------------------
    // Gain matrix K
    //
    // K is diagonal.
    //
    // Here we use the same gain for all joints:
    //
    // K = Kp * I
    // ---------------------------------------------------------

    Eigen::MatrixXd K =
        _Kp *
        Eigen::MatrixXd::Identity(n, n);


    // ---------------------------------------------------------
    // Primary visual task
    //
    //
    // qdot_task =
    // K * (L(s) Jc)^dagger * sd
    //
    // where:
    //
    // sd = [0, 0, 1]^T
    // ---------------------------------------------------------

    Eigen::VectorXd qdot_task =
        K * A_pinv * _s_desired;


    // ---------------------------------------------------------
    // Secondary task:
    // joint-limit avoidance
    //
    // qdot0 is the same joint-limit avoidance velocity used
    // in velocity_ctrl_null().
    // ---------------------------------------------------------

    Eigen::MatrixXd limits =
        robot_->getJntLimits();

    Eigen::VectorXd qdot0(n);

    for (int i = 0; i < n; ++i)
    {
        double q_lower = limits(i, 0);
        double q_upper = limits(i, 1);

        qdot0(i) =
            -(1.0 / _lambda) *
            std::pow(q_upper - q_lower, 2) *
            (2.0 * q(i) - q_upper - q_lower) /
            (
                std::pow(q_upper - q(i), 2) *
                std::pow(q(i) - q_lower, 2)
            );
    }


    // ---------------------------------------------------------
    // Null-space projector
    //
    //
    // N =
    // I - (L(s)Jc)^dagger L(s)Jc
    //
    // Since:
    //
    // A = L(s)Jc
    //
    // then:
    //
    // N = I - A^dagger A
    // ---------------------------------------------------------

    Eigen::MatrixXd I =
        Eigen::MatrixXd::Identity(n, n);

    Eigen::MatrixXd N =
        I - A_pinv * A;


    // ---------------------------------------------------------
    // Null-space contribution
    // ---------------------------------------------------------

    Eigen::VectorXd qdot_null =
        N * qdot0;



    // ---------------------------------------------------------
    // Final control law
    //
    // qdot =
    // K (L(s)Jc)^dagger sd + N qdot0
    // ---------------------------------------------------------

    Eigen::VectorXd qdot =
        qdot_task + qdot_null;
        

    return qdot;


}
