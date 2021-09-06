#ifndef __proj__DecimateTemplate__
#define __proj__DecimateTemplate__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
using Viewer = igl::opengl::glfw::Viewer;

using namespace std;
using namespace igl;

class DecimateTemplate
{
public:
public:
	DecimateTemplate(Eigen::MatrixXd& V_initial, Eigen::MatrixXi& F_initial);
	~DecimateTemplate();

private:
	Eigen::MatrixXd& V_initial;
	Eigen::MatrixXi& F_initial;

	string path_to_low_res = "../data/template_low.obj";
	string path_to_mid_res = "../data/template_mid.obj";
	string path_to_high_res = "../data/template_high.obj";
	int num_faces_low = 5000;
	int num_faces_mid = 10000;

	void decimateMesh(int num_faces, string path_to_output);

public:
	void createDecimations();
	void visualize(Viewer& viewer);
};

#endif