#pragma once

#ifndef MY_SLAM_G2O_TYPES_H
#define MY_SLAM_G2O_TYPES_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace myslam {

// typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> BlockSolverType;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> BlockSolverType;
typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType_Cur;

// typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType_Cur;

class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update_) override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3d::exp(update) * estimate());
    }
};

// class VertexPoint A;
class VertexPoint : public g2o::BaseVertex<3, Sophus::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    ~VertexPoint() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::Vector3d::Zero();
    }


    virtual void oplusImpl(const double *update_) override{ 
        _estimate[0] += update_[0];
        _estimate[1] += update_[1];
        _estimate[2] += update_[2];
    }

};


// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class OnlyForPoseProjectionEdge : public g2o::BaseUnaryEdge<2, Vector2d, VertexSophus> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OnlyForPoseProjectionEdge(Matrix<double,3,3> K, Vector3d &Pr) {
        _K = K;
        _Pr = Pr;
    }
  // 计算曲线模型误差
  virtual void computeError() override {
    VertexSophus *pose = (VertexSophus *)_vertices[0];
    Vector3d Pc = pose->estimate()*_Pr;
    Vector3d pixel = _K*Pc/Pc[2];

    // _error = _measurement - pixel.block<2,1>(0,0);
    _error = pixel.block<2,1>(0,0) - _measurement;
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    // const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    // const Eigen::Vector3d abc = v->estimate();
    // double y = v->estimate();
    auto v0 = (VertexSophus *)_vertices[0];
    Eigen::Vector3d Pc = v0->estimate()*_Pr;
    double Z_2 = Pc(2,0)*Pc(2,0);
    Matrix<double, 2, 3> Jacob_Pc;
    Jacob_Pc << _K(0,0)/Pc(2,0), 0, -_K(0,0)*Pc(0,0)/Z_2, 0, _K(1,1)/Pc(2,0), -_K(1,1)*Pc(1,0)/Z_2;
    Matrix<double, 3, 6> Jacob_Pc_Tj;
    Jacob_Pc_Tj.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    Jacob_Pc_Tj.block<3,3>(0,3) = -Sophus::SO3d::hat(Pc);

    _jacobianOplusXi = Jacob_Pc * Jacob_Pc_Tj;
    // cout << "cur pc is " << Pc.transpose() << endl;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  Vector3d _Pr;  // x 值， y 值为 _measurement
  Eigen::Matrix<double,3,3> _K;
};

class EdgeProjection : public g2o::BaseBinaryEdge<2, Vector2d, VertexSophus, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(Matrix<double,3,3> K, Sophus::SE3d cam_ext) {
      _K = K;
      _cam_ext = cam_ext;}
    virtual void computeError() override {
        VertexSophus *pose = (VertexSophus *)_vertices[0];
        VertexPoint *Pw = (VertexPoint *)_vertices[1];
        Eigen::Vector3d Pc = _cam_ext * pose->estimate() * Pw->estimate();
        Eigen::Vector3d pixel = _K*Pc/Pc[2];

        _error = pixel.block<2,1>(0,0) - _measurement;
    }

    virtual void linearizeOplus() override {
        auto v0 = (VertexSophus *)_vertices[0];
        auto v1 = (VertexPoint *)_vertices[1];

        Eigen::Vector3d Pc = _cam_ext * v0->estimate() * v1->estimate();
        double Z_2 = Pc(2)*Pc(2);
        Matrix<double,2,3> Jacob_Pc;
        Jacob_Pc << _K(0,0)/Pc(2,0), 0, -_K(0,0)*Pc(0,0)/Z_2, 0, _K(1,1)/Pc(2,0), -_K(1,1)*Pc(1,0)/Z_2;
        Matrix<double, 3, 6> Jacob_Pc_Tj;
        Jacob_Pc_Tj.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        Jacob_Pc_Tj.block<3,3>(0,3) = -Sophus::SO3d::hat(Pc);

        _jacobianOplusXi =  Jacob_Pc * Jacob_Pc_Tj;
        _jacobianOplusXj = Jacob_Pc * v0->estimate().rotationMatrix();
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
  Matrix<double,3,3> _K;
  Sophus::SE3d _cam_ext;
};


}


#endif
