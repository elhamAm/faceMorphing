#ifndef __proj__LandmarkLoader__
#define __proj__LandmarkLoader__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>

#include "Paths.h"
#include "SpatialIndex.h"

using Viewer = igl::opengl::glfw::Viewer;


using namespace std;

class LandmarkLoader
{
public:
public:
	LandmarkLoader();
	~LandmarkLoader();

private:

    char landmark_names[23][4] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23" };
    std::map<string,int> map_name_to_index;

    
public:
    int num_landmarks = 23;
    int num_landmarks_bonus = 73;
    
    void createMapping();
    void parseLandmarkFile(string path, Eigen::MatrixXd& V, Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions);
    void landmarksFromPositionsBonus(Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions);
    void landmarksFromPositionsBonusRigidaligned(Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions);
    void parseLandmarkFileBonus(string path_landmarks, string path_obj, Eigen::VectorXi& Indices, Eigen::MatrixXd& Positions);
};

#endif