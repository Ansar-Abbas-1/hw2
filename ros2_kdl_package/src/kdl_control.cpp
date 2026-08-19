#include "kdl_control.h"
#include <cmath>
#include <stdexcept>
#include <iostream>   


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

    // DEBUG
    // std::cout << "q: "
    //         << q.transpose()
    //         << std::endl;

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


        // DEBUG
        // double dist_lower = q(i) - q_lower;
        // double dist_upper = q_upper - q(i);

        // std::cout << "Joint " << i + 1
        //         << " q=" << q(i)
        //         << " lower_dist=" << dist_lower
        //         << " upper_dist=" << dist_upper
        //         << std::endl;


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

    // TEMPORARY DEBUG
    // std::cout << "qdot_task: "
    //         << qdot_task.transpose()
    //         << std::endl;

    // std::cout << "qdot0: "
    //         << qdot0.transpose()
    //         << std::endl;

    // std::cout << "qdot_null: "
    //         << qdot_null.transpose()
    //         << std::endl;

    // std::cout << "Jp*qdot_null norm: "
    //         << (Jp * qdot_null).norm()
    //         << std::endl;    

    // Total velocity
    return qdot_task + qdot_null;
}
