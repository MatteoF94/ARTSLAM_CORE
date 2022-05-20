#include "artslam_types/types.hpp"

#include "artslam_g2o/edge_se3_priorxyz.hpp"
#include "artslam_g2o/edge_se3_plane.hpp"

#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>

G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // be aware of that csparse brings LGPL unless it is dynamically linked

using namespace artslam::core::types;

g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose, g2o::SparseOptimizer* graph_) {
    auto* vertex(new g2o::VertexSE3());
    vertex->setId(static_cast<int>(graph_->vertices().size()));
    vertex->setEstimate(pose);
    graph_->addVertex(vertex);

    return vertex;
}

g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs, g2o::SparseOptimizer* graph_) {
    auto* vertex(new g2o::VertexPlane());
    vertex->setId(static_cast<int>(graph_->vertices().size()));
    vertex->setEstimate(plane_coeffs);
    graph_->addVertex(vertex);

    return vertex;
}

g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix, g2o::SparseOptimizer* graph_) {
    auto* edge(new g2o::EdgeSE3Plane());
    edge->setMeasurement(plane_coeffs);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_plane;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix, g2o::SparseOptimizer* graph_) {
    auto* edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_->addEdge(edge);

    return edge;
}

int main(int argc, char** argv) {
    std::unique_ptr<g2o::HyperGraph> graph_ = std::make_unique<g2o::SparseOptimizer>();
    auto* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct("lm_var", solver_property);
    graph->setAlgorithm(solver);

    EigIsometry3d init_pose = EigIsometry3d::Identity();
    g2o::VertexSE3* v0 = add_se3_node(init_pose, graph);

    EigVector4d ref_plane = {0.0, 0.0, 1.0, 0.0};
    g2o::VertexPlane* vp = add_plane_node(ref_plane, graph);
    vp->setFixed(true);

    EigVector3d pos = {1.0, -5.5, 1.5};
    EigVector4d ep = {0.0, 0.0, 1.0, 2.0};

    g2o::EdgeSE3Plane* edge_v0_vp = add_se3_plane_edge(v0, vp, ep, EigMatrix3d::Identity(), graph);
    g2o::EdgeSE3PriorXYZ* edge_v0_prior = add_se3_prior_xyz_edge(v0, pos, EigMatrix3d::Identity(), graph);

    // TODO add more nodes and edges of different type

    graph->initializeOptimization();
    graph->optimize(128);
    std::cout << v0->estimate().matrix() << std::endl;

    return 0;
}


