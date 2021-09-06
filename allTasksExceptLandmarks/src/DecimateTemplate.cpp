#include "DecimateTemplate.h"

#include <iostream>
#include <fstream>
#include <igl/decimate.h>
#include <igl/read_triangle_mesh.h>

using namespace igl;
using namespace std;
using namespace Eigen;


DecimateTemplate::DecimateTemplate(Eigen::MatrixXd& V_initial_, Eigen::MatrixXi& F_initial_):
V_initial(V_initial_),
F_initial(F_initial_)
{       
}

DecimateTemplate::~DecimateTemplate()
{
}

void DecimateTemplate::createDecimations(){
    // Create low resolution mesh
    decimateMesh(num_faces_low, path_to_low_res);
    // Create low resolution mesh
    decimateMesh(num_faces_mid, path_to_mid_res);
    // Store high resolution mesh (initial mesh)
    writeOBJ(path_to_high_res, V_initial, F_initial);
}

void DecimateTemplate::decimateMesh(int num_faces, string path_to_output){

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    Eigen::VectorXi J;

    igl::decimate(V_initial,F_initial,num_faces,V_out,F_out, J);

    writeOBJ(path_to_output, V_out, F_out);
}

void DecimateTemplate::visualize(Viewer& viewer){
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    igl::read_triangle_mesh(path_to_mid_res, V, F);
    viewer.data().set_mesh(V, F);
}