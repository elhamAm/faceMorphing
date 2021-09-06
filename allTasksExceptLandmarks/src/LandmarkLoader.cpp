#include "LandmarkLoader.h"

#include <iostream>
#include <fstream>

using namespace igl;
using namespace std;
using namespace Eigen;


LandmarkLoader::LandmarkLoader()
{
    createMapping();
}

LandmarkLoader::~LandmarkLoader()
{
}

void LandmarkLoader::createMapping(){
    for(int i=0; i<num_landmarks; i++){
        map_name_to_index.insert(make_pair( landmark_names[i], i ));
    }
}

void LandmarkLoader::parseLandmarkFile(string path, Eigen::MatrixXd& V, Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions){
    ifstream landmark_file;
    landmark_file.open(path);

    // Get Indices
    string line;
    int vertex;
    string name;
    Indices.setZero(num_landmarks);

    while (landmark_file >> vertex) {
        landmark_file >> name;
        Indices[map_name_to_index[name]] = vertex;
    }

    // Get Positions
    Positions.setZero(num_landmarks,3);

    for(int i=0; i<num_landmarks; i++){
        Positions.row(i) = V.row(Indices[i]);
    }
}

// for the bonus task
void LandmarkLoader::landmarksFromPositionsBonus(Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions){
    ifstream landmark_file;
    landmark_file.open(path_landmarks_bonus_demofile);

    // Load corresponding face scan
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(path_cleanfaces_bonus_demofile, V, F);
    
    // create spatial index for later searching the closest vertex for each landmark
    Eigen::RowVector3d bb_min = V.colwise().minCoeff().eval();
    Eigen::RowVector3d bb_max = V.colwise().maxCoeff().eval();
    Eigen::RowVector3d dim_mesh = bb_max - bb_min;
    double m = dim_mesh.maxCoeff();
    
    SpatialIndex spatial_index = SpatialIndex(V, 0.1*m);
    
    // Get Indices
    string position;
    double x,y,z;
    Indices.setZero(num_landmarks_bonus);
    int index = 0;
    while (landmark_file >> x) {
        landmark_file >> y;
        landmark_file >> z;

        //find closest point
        int closest_point_index = -1;
        Eigen::MatrixXd query(1,3);
        query << x, y, z;
        spatial_index.closest_point(query, closest_point_index);

        Indices[index] = closest_point_index;
        index++;
    }

    // Get Positions
    Positions.setZero(num_landmarks_bonus,3);

    for(int i=0; i<num_landmarks_bonus; i++){
        Positions.row(i) = V.row(Indices[i]);
    }

}

void LandmarkLoader::landmarksFromPositionsBonusRigidaligned(Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions){
    ifstream landmark_file;
    landmark_file.open(path_landmarks_bonus_demofile);

    // Load corresponding face scan
    Eigen::MatrixXd V, V_align;
    Eigen::MatrixXi F, F_align;
    igl::read_triangle_mesh(path_cleanfaces_bonus_demofile, V, F);
    igl::read_triangle_mesh(path_bonus_demofile_rigidaligned, V_align, F_align);
    
    // create spatial index for later searching the closest vertex for each landmark
    Eigen::RowVector3d bb_min = V.colwise().minCoeff().eval();
    Eigen::RowVector3d bb_max = V.colwise().maxCoeff().eval();
    Eigen::RowVector3d dim_mesh = bb_max - bb_min;
    double m = dim_mesh.maxCoeff();
    
    SpatialIndex spatial_index = SpatialIndex(V, 0.1*m);
    
    // Get Indices
    string position;
    double x,y,z;
    Indices.setZero(num_landmarks_bonus);
    int index = 0;
    while (landmark_file >> x) {
        landmark_file >> y;
        landmark_file >> z;

        //find closest point
        int closest_point_index = -1;
        Eigen::MatrixXd query(1,3);
        query << x, y, z;
        spatial_index.closest_point(query, closest_point_index);

        Indices[index] = closest_point_index;
        index++;
    }

    // Get Positions
    Positions.setZero(num_landmarks_bonus,3);

    for(int i=0; i<num_landmarks_bonus; i++){
        Positions.row(i) = V_align.row(Indices[i]);
    }

}

void LandmarkLoader::parseLandmarkFileBonus(string path_landmarks, string path_obj, Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions){
    
    cout << "parseLandmarkFileBonus" << endl;
    ifstream landmark_file;
    landmark_file.open(path_landmarks);

    // Load corresponding face scan
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(path_obj, V, F);

    // Get Indices
    string line;
    int vertex;
    string name;
    Indices.setZero(num_landmarks_bonus);
    int index = 0;
    while (landmark_file >> vertex) {
        landmark_file >> name;
        Indices[index] = vertex;
        index++;
    }

    // Get Positions
    Positions.setZero(num_landmarks_bonus,3);

    for(int i=0; i<num_landmarks_bonus; i++){
        Positions.row(i) = V.row(Indices[i]);
    }
}
