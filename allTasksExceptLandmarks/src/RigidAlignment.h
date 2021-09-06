#ifndef __proj__RigidAlignment__
#define __proj__RigidAlignment__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
using Viewer = igl::opengl::glfw::Viewer;

#include "Paths.h"

using namespace std;
using namespace Eigen;

class RigidAlignment
{
public:
public:
    RigidAlignment();
    ~RigidAlignment();

private:
    //string path_scan = "../../../all_data/lankmarks_example/person0_.obj";
    string path_scan; // = "../data/scanned_faces_cleaned_consistent/alain_normal.obj";
	string path_template_ra; // = "../data/template_rigid_aligned.obj";  

    string path_scan_landmarks; //= "../data/landmarks/alain_normal_landmarks";
	string path_template_landmarks; // = "../data/template_landmarks";   

    string saving_suffix = "rigid_aligned.obj";

    // vertex array, #V x3
    Eigen::MatrixXd V_template, V_scan;

    //face array, #F x3
    Eigen::MatrixXi F_template, F_scan;

    // Vertex indices of landmark vertices
    Eigen::VectorXi landmark_vertices_template, landmark_vertices_scan;

    // Vertex positions of landmark vertices
    Eigen::MatrixXd landmark_vertex_positions_template, landmark_vertex_positions_scan;

    bool use_landmarks = true;
    int pointsize = 20;

public:
    void rigidAlignment(bool alignScanToTemplate);
    void rigidAlignmentAll(bool alignScanToTemplate);
    void loadData();
    void getLandmarks();
    void drawLandmarks(Viewer& viewer);
    void fillLandmarkVertexPositions(bool alignScanToTemplate);
    void showMesh(Viewer& viewer, int mesh_type);
    void saveMesh(string face_name, bool demo);
};

#endif
