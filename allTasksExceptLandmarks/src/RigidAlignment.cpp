#include "RigidAlignment.h"

#include <iostream>
#include <fstream>

#include "LandmarkLoader.h"
#include <igl/read_triangle_mesh.h>

using namespace igl;
using namespace std;
using namespace Eigen;


RigidAlignment::RigidAlignment()
{
    //cout << "path_cleanfaces_demofile: "<< path_cleanfaces_demofile << endl;
    path_scan = path_cleanfaces_demofile;
    path_template_ra = path_template;

    path_scan_landmarks = path_landmarks_demofile;
    path_template_landmarks = path_landmarks_template;
}

RigidAlignment::~RigidAlignment()
{
}


void RigidAlignment::rigidAlignmentAll(bool alignScanToTemplate){
    ifstream faces_list;
    faces_list.open(path_faces_list);
    string commonName;
    while (faces_list >> commonName) {
        path_scan = path_cleanfaces_folder + commonName + ".obj";
        path_scan_landmarks = path_landmarks_folder + commonName + "_landmarks";
        loadData();
		rigidAlignment(alignScanToTemplate);
        fillLandmarkVertexPositions(true);
        saveMesh(commonName, false);
    }

    /*
    // Reset to demofile
    path_scan = path_cleanfaces_demofile;
    path_scan_landmarks = path_landmarks_demofile;
    loadData();
    */
}

// if alignScanToTemplate = true, then this method rigidly aligns the scan to the template
// otherwise, it rigidly aligns the template to the scan
void RigidAlignment::rigidAlignment(bool alignScanToTemplate)
{
    // in the following code, source denotes the mesh that is being rigidly aligned to the target mesh
    Eigen::MatrixXd V_source, landmark_vertex_positions_source, landmark_vertex_positions_target;

    if(alignScanToTemplate)
    {
        V_source = V_scan;
        landmark_vertex_positions_source = landmark_vertex_positions_scan;
        landmark_vertex_positions_target = landmark_vertex_positions_template;
    }
    else
    {
        V_source = V_template;
        landmark_vertex_positions_source = landmark_vertex_positions_template;
        landmark_vertex_positions_target = landmark_vertex_positions_scan;
    }

    int num_landmarks = landmark_vertex_positions_source.rows();

    RowVector3d mean_landmark_vertex_positions_target = landmark_vertex_positions_target.colwise().mean();
    RowVector3d mean_landmark_vertex_positions_source = landmark_vertex_positions_source.colwise().mean();

    // translate the source vertices such that mean of the landmark_vertex_positions_source is at (0,0,0)
    for (int row_idx = 0; row_idx < V_source.rows(); ++row_idx)
    {
        V_source.row(row_idx) -= mean_landmark_vertex_positions_source.transpose();
    }

    // compute the mean distance to the mean of all landmarks for both the target and the source landmarks
    double mean_distance_to_landmark_target = 0.0;
    double mean_distance_to_landmark_source = 0.0;

    for (unsigned int i = 0; i < num_landmarks; ++i) {
        mean_distance_to_landmark_target += (landmark_vertex_positions_target.row(i) - mean_landmark_vertex_positions_target).norm();
        mean_distance_to_landmark_source += (landmark_vertex_positions_source.row(i) - mean_landmark_vertex_positions_source).norm();
    }

    mean_distance_to_landmark_target /= num_landmarks;
    mean_distance_to_landmark_source /= num_landmarks;

    // determine the scaling factor such that the mean distance to the mean of all landmarks is equal for both the target and the source landmarks
    double scaling_factor = mean_distance_to_landmark_target / mean_distance_to_landmark_source;

    V_source *= scaling_factor;

    // also scale landmark vertex positions and compute new mean based on the scaled landmark vertices:
    landmark_vertex_positions_source *= scaling_factor;
    mean_landmark_vertex_positions_source = landmark_vertex_positions_source.colwise().mean();

    // compute the best rotation and translation using https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    MatrixXd A, U, U_tilde, R, D, V, V_dash;

    // center the landmark vertices
    V = MatrixXd(landmark_vertex_positions_source);
    for (int row_idx = 0; row_idx < V.rows(); ++row_idx)
    {
        V.row(row_idx) -= mean_landmark_vertex_positions_source;
    }

    V_dash = MatrixXd(landmark_vertex_positions_target);
    for (int row_idx = 0; row_idx < V_dash.rows(); ++row_idx)
    {
        V_dash.row(row_idx) -= mean_landmark_vertex_positions_target;
    }

    A = V.transpose() * V_dash;

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    U = svd.matrixU();
    U_tilde = svd.matrixV();

    D = Eigen::MatrixXd::Identity(3, 3);
    D(2, 2) = (U_tilde * U.transpose()).determinant();

    // compute the optimal rotation matrix R
    R = U_tilde * D * U.transpose();

    // rotate the source vertices
    for (unsigned int i = 0; i < V_source.rows(); ++i) {
        V_source.row(i) = R * V_source.row(i).transpose();
    }
    // rotate the positions stored in landmark_vertex_positions_source
    for (unsigned int i = 0; i < landmark_vertex_positions_source.rows(); ++i) {
        landmark_vertex_positions_source.row(i) = R * landmark_vertex_positions_source.row(i).transpose();
    }

    // compute means again
    mean_landmark_vertex_positions_target = landmark_vertex_positions_target.colwise().mean();
    mean_landmark_vertex_positions_source = landmark_vertex_positions_source.colwise().mean();

    // the vertices in V_source are alread centered in the beginning such that the mean landmark is at the origin,
    // thus we now only want to add the mean template landmark to every vertex in V_source.
    RowVector3d t = mean_landmark_vertex_positions_target;
    // translate the source vertices
    for (int row_idx = 0; row_idx < V_source.rows(); ++row_idx)
    {
        V_source.row(row_idx) += t;
        // translate landmark vertices as well:
        landmark_vertex_positions_source += t;
    }

    if(alignScanToTemplate)
    {
        V_scan = V_source;
        landmark_vertex_positions_scan = landmark_vertex_positions_source;
        landmark_vertex_positions_template = landmark_vertex_positions_target;
    }
    else
    {
        V_template = V_source;
        landmark_vertex_positions_scan = landmark_vertex_positions_target;
        landmark_vertex_positions_template = landmark_vertex_positions_source;
    }
}

void RigidAlignment::loadData(){
    //cout << "path_template_ra: " << endl;
    //cout << "path_scan: " << path_scan << endl;
    cout << "load data" << endl;
    if(useBonusLandmarks){
        igl::read_triangle_mesh(path_bonus_template, V_template, F_template);
    }else{
        igl::read_triangle_mesh(path_template_ra, V_template, F_template);
    }
	igl::read_triangle_mesh(path_scan, V_scan, F_scan);

    getLandmarks();
}

void RigidAlignment::getLandmarks()
{
    cout << "get landmarks" << endl;
    LandmarkLoader ll = LandmarkLoader();
    //cout << "landmarks: " << path_scan_landmarks << endl;
    if(useBonusLandmarks){
        ll.landmarksFromPositionsBonus(landmark_vertices_scan, landmark_vertex_positions_scan);
        ll.parseLandmarkFileBonus(path_landmarks_bonus_templatefile, path_bonus_template,landmark_vertices_template, landmark_vertex_positions_template);

        //ll.landmarksFromPositionsBonus(landmark_vertices_scan, landmark_vertex_positions_scan);
        //ll.landmarksFromPositionsBonus(path_landmarks_bonus_templatefile, path_bonus_template,landmark_vertices_template, landmark_vertex_positions_template);
    }else{
        ll.parseLandmarkFile(path_scan_landmarks, V_scan, landmark_vertices_scan, landmark_vertex_positions_scan);
        ll.parseLandmarkFile(path_template_landmarks, V_template, landmark_vertices_template, landmark_vertex_positions_template);
    }
    cout << "landmarks " << landmark_vertex_positions_template.rows() << "  " << landmark_vertices_template.size() << endl;

    //cout << "landmark_vertices_template" << landmark_vertices_template << endl;
}

void RigidAlignment::drawLandmarks(Viewer& viewer)
{
    viewer.data().point_size = pointsize;
    viewer.data().set_points(Eigen::MatrixXd::Zero(0, 3), Eigen::MatrixXd::Zero(0, 3));

    for (int i = 0; i < landmark_vertex_positions_template.rows(); ++i)
    {
        viewer.data().add_points(landmark_vertex_positions_template.row(i), RowVector3d(128.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0));
        viewer.data().add_label(landmark_vertex_positions_template.row(i), std::to_string(i + 1));
    }

    //viewer.data().add_points(landmark_vertex_positions_template.colwise().mean(), RowVector3d(230.0 / 255.0, 25.0 / 255.0, 75.0 / 255.0));
    //viewer.data().add_label(landmark_vertex_positions_template.colwise().mean(), "mean landmark template");

    for (int i = 0; i < landmark_vertex_positions_scan.rows(); ++i)
    {
        viewer.data().add_points(landmark_vertex_positions_scan.row(i), RowVector3d(70.0 / 255.0, 240.0 / 255.0, 240.0 / 255.0));
        viewer.data().add_label(landmark_vertex_positions_scan.row(i), std::to_string(i + 1));
    }

    //viewer.data().add_points(landmark_vertex_positions_scan.colwise().mean(), RowVector3d(60.0 / 255.0, 180.0 / 255.0, 75.0 / 255.0));
    //viewer.data().add_label(landmark_vertex_positions_scan.colwise().mean(), "mean landmark scan");
}

void RigidAlignment::fillLandmarkVertexPositions(bool alignScanToTemplate)
{
    if(alignScanToTemplate)
    {
        landmark_vertex_positions_scan = MatrixXd(landmark_vertices_scan.size(), 3);

        for (int idx = 0; idx < landmark_vertices_scan.size(); ++idx)
        {
            landmark_vertex_positions_scan.row(idx) = V_scan.row(landmark_vertices_scan(idx));
        }
    }
    else
    {
        landmark_vertex_positions_template = MatrixXd(landmark_vertices_template.size(), 3);

        for (int idx = 0; idx < landmark_vertices_template.size(); ++idx)
        {
            landmark_vertex_positions_template.row(idx) = V_template.row(landmark_vertices_template(idx));
        }
    }
}

void RigidAlignment::showMesh(Viewer& viewer, int mesh_type)
{   
    viewer.data().clear();
    if(mesh_type == 0)
    {
        // Scan
        MatrixXd colors = MatrixXd(F_scan.rows(), 3);
       
        colors << Eigen::RowVector3d(0.0 / 255.0, 0.0 / 255.0, 128.0 / 255.0).replicate(F_scan.rows(), 1);

        viewer.data().set_mesh(V_scan, F_scan);
        viewer.data().set_colors(colors);
    }
    else if (mesh_type == 1)
    {
        // Template
        MatrixXd colors = MatrixXd(F_template.rows(), 3);
        colors << Eigen::RowVector3d(255.0 / 255.0, 225.0 / 255.0, 25.0 / 255.0).replicate(F_template.rows(), 1);

        viewer.data().set_mesh(V_template, F_template);
        viewer.data().set_colors(colors);
    }
    else if (mesh_type == 2)
    {
        // Scan & Template Overlaid
        MatrixXd all_V(V_template.rows() + V_scan.rows(), 3);
        MatrixXi all_F(F_template.rows() + F_scan.rows(), 3);

        MatrixXi F_scan_translated = MatrixXi(F_scan) + MatrixXi::Constant(F_scan.rows(), F_scan.cols(), V_template.rows());

        all_V << V_template, V_scan;
        all_F << F_template, F_scan_translated;

        MatrixXd colors = MatrixXd(all_F.rows(), 3);
        colors << Eigen::RowVector3d(255.0 / 255.0, 225.0 / 255.0, 25.0 / 255.0).replicate(F_template.rows(), 1), Eigen::RowVector3d(0.0 / 255.0, 0.0 / 255.0, 128.0 / 255.0).replicate(F_scan_translated.rows(), 1);

        viewer.data().set_mesh(all_V, all_F);
        viewer.data().set_colors(colors);
    }

    viewer.core.align_camera_center(V_template);
}

void RigidAlignment::saveMesh(string face_name, bool demo){
    string fileNameObj;

    if(demo)
    {
        fileNameObj = path_rigidaligned_demofile;
    }
    else
    {
        fileNameObj = path_rigidaligned_folder + face_name + "_rigid_aligned.obj";
    }
    cout << "save mesh into: " << fileNameObj << endl;
    igl::writeOBJ(fileNameObj, V_scan, F_scan);
}
