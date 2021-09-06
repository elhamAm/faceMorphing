#ifndef __proj__non_rigid__
#define __proj__non_rigid__

#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <igl/cotmatrix.h>
#include <igl/cat.h>

#include "LandmarkLoader.h"
#include "ConstraintPoints.h"
#include "Enums.h"
#include "Paths.h"

using namespace std;
using namespace Eigen;

class NonRigid
{
public:
	NonRigid();
	~NonRigid();

private:
    int num_landmarks = 23;

    BoundaryMode boundary_mode = FIX_TEMPLATE_BOUNDARY;

    bool scanOverlay = true;
    int pointsize_overlay = 2;
    int pointsize_landmarks = 10;

    string path_scan_rigidaligned;
    string path_template_rigidaligned;

    double closepoint_thresh; // distance threshhold for closepoint_constraints

    // Input meshes: rigidly aligned scan and template face
    Eigen::MatrixXd V_scan, V_template, V_warped;
    Eigen::MatrixXi F_scan, F_template, F_warped;

    // Constrained indices of template mesh
    Eigen::VectorXi constrained_indices; // can be used to pick the vertices to be constrained
    // Desired positions of constrained vertices of template mesh
    Eigen::MatrixXd constrained_positions; // Corresponds to "c" in the error formulation

    ConstraintPoints cp;

    double lambda = 0.01;



public:
    void loadData();
    void nonRigidAlignmentForAll(int staticIterations, int dynmaicIterations, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_);
    void nonRigidAlignmentForOne(string rigidPath, string landmarkPath, string nonRigidPath,string commonName, int staticIterations, int dynmaicIterations, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_);
	void nonRigidAlignment(string resultName, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_);
    void getConstraintPoints(ConstraintsMode constraints_mode);
    void iterate();
    void visualizeResult(Viewer &viewer, VisualizationMode visualization_mode);
    void drawLandmarks(Viewer& viewer);

};

#endif