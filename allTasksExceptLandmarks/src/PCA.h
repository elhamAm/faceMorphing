#ifndef __proj__PCA__
#define __proj__PCA__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>
#include "Paths.h"
using Viewer = igl::opengl::glfw::Viewer;

using namespace std;

class PCA
{
public:
public:
    PCA();
    ~PCA();

private:
    //const char *face_dir = "../../../all_data/aligned_faces_example/example1";
    //string filename = std::string("../../../all_data/aligned_faces_example/example1/fabian-neutral.objaligned.obj");
    const char *face_dir = "../data/nonRigidFaces";
    string filename = std::string("../data/nonRigidFaces/alain_normal_non_rigid_aligned.obj");
    // all faces
    Eigen::MatrixXd Faces;

    // PCA
    std::vector<float> eigenfaceStrength;
    bool visualizeEigenfaces = false;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    Eigen::MatrixXd mean_face;
    Eigen::MatrixXd reconstruction;
    Eigen::MatrixXd original;

    // Reconstruction
    bool reconstructionAnalysis = false;
    //int maxEigenfaces = 10;

    // Morphing
    //bool visualizeMorphing = false;
    float morphing = 0.0;
    int id1 = 0;
    int id2 = 1;

    // Vertex array, #Vx3
    //Eigen::MatrixXd V;
    Eigen::MatrixXd S;
    // Face array, #Fx3
    Eigen::MatrixXi F;
    // Per-vertex normal array, #Vx3
    Eigen::MatrixXd N;

public:
    Eigen::MatrixXd V;
    void prepare(Viewer &viewer);
    void show_face(Viewer &viewer);
    int reconstruction_analysis(Viewer &viewer, int maxEigenfaces);
    Eigen::MatrixXd encode_face(Eigen::MatrixXd face);
    Eigen::MatrixXd decode_face(Eigen::MatrixXd face);
    void load_faces();
    void facePCA();
    void show_mean_face(Viewer &viewer);
    void show_eigen_face(Viewer &viewer, std::vector<float> coeff);
    double mesh_difference();
    std::vector<float> project_face(Eigen::MatrixXd face);
    void morph_faces(Viewer &viewer, float morphing, int id1, int id2);
    bool load_mesh(Viewer &viewer, string filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F);
};

#endif