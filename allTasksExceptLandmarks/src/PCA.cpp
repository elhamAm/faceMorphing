#include "PCA.h"

#include <iostream>
#include <fstream>
#include <string>
//#include <dirent.h>
#include <experimental/filesystem>
//namespace fs = std::experimental::filesystem;

using namespace igl;
using namespace std;
using namespace Eigen;

PCA::PCA()
{
}

PCA::~PCA()
{
}

//LEA: new function
void PCA::prepare(Viewer &viewer)
{
    load_mesh(viewer, filename, V, F);

    // needed for normal orientations
    igl::per_vertex_normals(V, F, N);

    eigenfaceStrength.resize(10);

    //callback_key_down(viewer, '1', 0);
    show_face(viewer);

    load_faces();

    original = V;
    reconstruction = V;
}

//LEA: new function
void PCA::show_face(Viewer &viewer)
{
    //callback_key_down(viewer, '1', 0);
    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V, F);
}

//LEA: new function
int PCA::reconstruction_analysis(Viewer &viewer, int maxEigenfaces)
{
    maxEigenfaces = (maxEigenfaces > svd.nonzeroSingularValues()) ? svd.nonzeroSingularValues() : maxEigenfaces;
    std::vector<float> coeff = project_face(V);
    coeff.resize(maxEigenfaces);
    show_eigen_face(viewer, coeff);
    return maxEigenfaces;
}

Eigen::MatrixXd PCA::encode_face(Eigen::MatrixXd face)
{
    /* #V x 3 to 1 x 3#V */
    Eigen::MatrixXd facevector;
    facevector.conservativeResize(1, face.rows() * 3);
    for (int v = 0; v < face.rows(); ++v)
    {
        // flatten
        facevector(0, 3 * v) = face.row(v)[0];
        facevector(0, 3 * v + 1) = face.row(v)[1];
        facevector(0, 3 * v + 2) = face.row(v)[2];
    }
    return facevector;
}

Eigen::MatrixXd PCA::decode_face(Eigen::MatrixXd face)
{
    /* 1x3#V to  #Vx3 */
    Eigen::MatrixXd Vnew;
    Vnew.resizeLike(V);
    for (int v = 0; v < Vnew.rows(); ++v)
    {
        Vnew.row(v) = Eigen::RowVector3d(face(0, 3 * v), face(0, 3 * v + 1), face(0, 3 * v + 2));
    }
    return Vnew;
}

double PCA::mesh_difference()
{
    Eigen::MatrixXd diff = (reconstruction - original);
    return diff.rowwise().squaredNorm().mean();
}

void PCA::load_faces()
{
    ifstream faces_list_pre;
    faces_list_pre.open(path_faces_list);
    string face_name_pre;
    string face_name_warped_pre;
    int num_meshes = 0;
    int num_vertices = 0;
    while (faces_list_pre >> face_name_pre)
    {
        Eigen::MatrixXd Vtmp;

        //std::cout << file_name << std::endl;
        // save mesh into matrix
        face_name_warped_pre = std::string(face_dir) + "/" + face_name_pre + "_non_rigid_aligned.obj";
        if (num_meshes == 0)
        {
            igl::read_triangle_mesh(face_name_warped_pre, Vtmp, F);
            num_vertices = Vtmp.rows() * 3;
        }
        num_meshes++;
    }
    Faces.conservativeResize(num_meshes, num_vertices);
    //const char *face_dir = "../../../all_data/aligned_faces_example/example1";
    //std::cout << face_dir << std::endl;
    int subject = 0;

    /*
    for (const auto &entry : fs::directory_iterator(face_dir))
    {
        // print all the files and directories within directory
        Eigen::MatrixXd Vtmp;
        string file_name = entry.path().string();
        if (file_name.find("neutral") != std::string::npos || file_name.find("smile") != std::string::npos) //|| file_name.find("smile") != std::string::npos
        {
            //std::cout << file_name << std::endl;
            // save mesh into matrix
            igl::read_triangle_mesh(file_name, Vtmp, F);
            Faces.row(subject++) = encode_face(Vtmp).transpose();
        }
    }
    */
    // LEA: this works on all compilers
    ifstream faces_list;
    faces_list.open(path_faces_list);
    string face_name;
    string face_name_warped;
    while (faces_list >> face_name)
    {
        Eigen::MatrixXd Vtmp;

        //std::cout << file_name << std::endl;
        // save mesh into matrix
        face_name_warped = std::string(face_dir) + "/" + face_name + "_non_rigid_aligned.obj";
        igl::read_triangle_mesh(face_name_warped, Vtmp, F);
        Faces.row(subject++) = encode_face(Vtmp).transpose();
    }
}

void PCA::facePCA()
{
    mean_face = Faces.colwise().mean();
    Eigen::MatrixXd centered = Faces.rowwise() - Faces.colwise().mean();
    Eigen::MatrixXd cov = centered.adjoint() * centered;

    // we can directly take SVD
    svd = Eigen::JacobiSVD<Eigen::MatrixXd>(centered, Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::ComputeEigenvectors);

    std::cout << "PCA done" << std::endl;
}

void PCA::show_mean_face(Viewer &viewer)
{
    // show mean face
    Eigen::MatrixXd V_mean = decode_face(mean_face);

    viewer.data().clear();
    viewer.data().set_mesh(V_mean, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V_mean, F);
}

void PCA::show_eigen_face(Viewer &viewer, std::vector<float> coeff)
{
    // apply eigenfaces
    Eigen::MatrixXd eig_face = mean_face;
    for (int e = 0; e < coeff.size(); ++e)
    {
        eig_face += coeff[e] * (svd.singularValues()[e] * svd.singularValues()[e] / (V.rows() - 1)) * svd.matrixV().col(e).transpose();
    }
    Eigen::MatrixXd V_eig = decode_face(eig_face);
    reconstruction = V_eig;

    viewer.data().clear();
    viewer.data().set_mesh(V_eig, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V_eig, F);
}

std::vector<float> PCA::project_face(Eigen::MatrixXd face)
{
    std::vector<float> coeff;
    Eigen::MatrixXd restFace = (encode_face(face) - mean_face);

    for (int i = 0; i < svd.nonzeroSingularValues(); ++i)
    {
        float strength = restFace.row(0).dot(svd.matrixV().col(i));
        //std::cout << strength << std::endl;
        restFace = restFace - strength * svd.matrixV().col(i).transpose();
        coeff.push_back(strength / (svd.singularValues()[i] * svd.singularValues()[i] / (V.rows() - 1)));
    }

    //LEA: added this line just to be sure
    eigenfaceStrength = coeff;
    return coeff;
}

void PCA::morph_faces(Viewer &viewer, float morphing, int id1, int id2)
{
    std::vector<float> coeff1 = project_face(decode_face(Faces.row(id1)));
    std::vector<float> coeff2 = project_face(decode_face(Faces.row(id2)));

    std::vector<float> morphedCoeff;
    for (int i = 0; i < coeff1.size(); ++i)
    {
        morphedCoeff.push_back(coeff1.at(i) - morphing * (coeff1.at(i) - coeff2.at(i)));
    }
    // apply eigenfaces
    Eigen::MatrixXd morph_face = mean_face;
    for (int e = 0; e < morphedCoeff.size(); ++e)
    {
        morph_face += morphedCoeff[e] * (svd.singularValues()[e] * svd.singularValues()[e] / (V.rows() - 1)) * svd.matrixV().col(e).transpose();
    }
    Eigen::MatrixXd V_morph = decode_face(morph_face);

    viewer.data().clear();
    viewer.data().set_mesh(V_morph, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V_morph, F);
}

bool PCA::load_mesh(Viewer &viewer, string filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    igl::read_triangle_mesh(filename, V, F);
    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().compute_normals();
    viewer.core.align_camera_center(V, F);
    return true;
}