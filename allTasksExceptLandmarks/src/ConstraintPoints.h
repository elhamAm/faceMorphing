#ifndef __proj__ConstraintPoints__
#define __proj__ConstraintPoints__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
using Viewer = igl::opengl::glfw::Viewer;
#include <unordered_set>
#include <igl/boundary_loop.h>
#include <igl/slice_into.h>
#include <igl/slice.h>
#include <igl/cat.h>

#include "SpatialIndex.h"
#include "Enums.h"
#include "LandmarkLoader.h"


using namespace std;

class ConstraintPoints
{
public:
public:
	ConstraintPoints(string path_landmark_template, string path_landmark_scan, Eigen::MatrixXd& V_template_, Eigen::MatrixXd& V_scan_, Eigen::MatrixXi& F_template_, Eigen::MatrixXi& F_scan_, double thresh_global);
	ConstraintPoints();
    ~ConstraintPoints();

private:

    // Meshes
    Eigen::MatrixXd V_scan, V_template;
    Eigen::MatrixXi F_scan, F_template;

    string path_scan_landmarks; //= "../data/lankmarks_example/person0__23landmarks";

    // this is fixed forever and ever
    string path_template_landmarks; // = "../data/lankmarks_example/headtemplate_23landmarks";

    BoundaryMode boundary_mode = FIX_TEMPLATE_BOUNDARY;

    double closepoint_thresh_max = 0.1;
    double closepoint_thresh = 0.05; // distance threshhold for closepoint_constraints

    SpatialIndex spatial_index_global;

    unordered_set<int> template_constraints_indices_set; 
    unordered_set<int> scan_constraints_indices_set; 

public:
    // Vertex indices positions of boundary vertices
    Eigen::VectorXi boundary_constrained_indices;
    Eigen::MatrixXd boundary_constrained_positions;

    // Vertex indices positions of landmark vertices
    Eigen::VectorXi landmark_constrained_indices;
    Eigen::MatrixXd landmark_constrained_positions;

    // Vertex indices positions of closepoint vertices
    Eigen::VectorXi closepoint_constrained_indices;
    Eigen::MatrixXd closepoint_constrained_positions;

    
    void recomputeSpatialIndex(double closepoint_thresh);
    void getStaticConstraints(Eigen::VectorXi& constrained_indices, Eigen::MatrixXd& constrained_positions);
    void getDynamicConstraints(Eigen::MatrixXd& V_current, Eigen::VectorXi& constrained_indices, Eigen::MatrixXd& constrained_positions, double closepoint_thresh_);
    void getLandmarkConstraints();
    void getBoundaryConstraints();
    void getClosepointConstraints(Eigen::MatrixXd& V_current);
};

#endif