#include "NonRigid.h"
#include <experimental/filesystem>
#include <string>
#include <iostream>
#include <fstream>


using namespace igl;
using namespace std;
using namespace Eigen;

typedef Triplet<double> T;

NonRigid::NonRigid()
{
}

NonRigid::~NonRigid()
{
}

void NonRigid::loadData(){
	
	string path_scan_landmarks;
	string path_template_landmarks;

	if(useBonusLandmarks){
		path_scan_rigidaligned = path_bonus_demofile_rigidaligned;
		path_template_rigidaligned = path_bonus_template;
		
		// load obj files of scan and template face
		igl::read_triangle_mesh(path_scan_rigidaligned, V_scan, F_scan);
		igl::read_triangle_mesh(path_template_rigidaligned, V_template, F_template);

		V_warped = V_template.replicate(1,1);
		F_warped = F_template.replicate(1,1);	

		path_scan_landmarks = path_landmarks_bonus_demofile;
		path_template_landmarks = path_landmarks_bonus_templatefile;

	}else{
		path_scan_rigidaligned = path_rigidaligned_demofile;
		path_template_rigidaligned = path_template;

		// load obj files of scan and template face
		igl::read_triangle_mesh(path_scan_rigidaligned, V_scan, F_scan);
		igl::read_triangle_mesh(path_template_rigidaligned, V_template, F_template);

		V_warped = V_template.replicate(1,1);
		F_warped = F_template.replicate(1,1);	

		path_scan_landmarks = path_landmarks_demofile;
		path_template_landmarks = path_landmarks_template;

	}
	
	cout << "variables loaded" << endl;
	cp = ConstraintPoints(path_template_landmarks, path_scan_landmarks, V_template, V_scan, F_template, F_scan, closepoint_thresh);

}

void NonRigid::nonRigidAlignmentForAll(int staticIterations, int dynmaicIterations, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_){
	cout << "Non-Rigid Alignment for all" << endl;
	closepoint_thresh = closepoint_thresh_;
	lambda = lambda_;
	string rigidPath = path_rigidaligned_folder;
	string landmarkPath = path_landmarks_folder; 
	string nonRigidPath = path_nonrigidaligned_folder; 

	const char *landmarkPathChar = path_landmarks_folder.c_str();
	
	// Parse the file names and perform the rigid alignment on all of them
	ifstream faces_list;
    faces_list.open(path_faces_list);
    string commonName;
	string commonName_extend;
    while (faces_list >> commonName) {
		commonName_extend = commonName + "_";
		cout << "  "<<commonName << endl;
		nonRigidAlignmentForOne(rigidPath, landmarkPath, nonRigidPath, commonName_extend, staticIterations, dynmaicIterations, lambda, iterations, constraints_mode, closepoint_thresh);
    }


}

void NonRigid::nonRigidAlignmentForOne(string rigidPath, string landmarkPath, string nonRigidPath, string commonName, int staticIterations, int dynmaicIterations, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_){
	closepoint_thresh = closepoint_thresh_;

	string add = "rigid_aligned.obj";
	string path_scan_rigidaligned_One = rigidPath + commonName + add;

	igl::read_triangle_mesh(path_scan_rigidaligned_One, V_scan, F_scan);
    igl::read_triangle_mesh(path_template, V_template, F_template);

	V_warped = V_template.replicate(1,1);
	F_warped = F_template.replicate(1,1);

	add = "landmarks";
	string path_scan_landmarks_One = landmarkPath + commonName + add;
	
	string path_template_landmarks_One = path_landmarks_template;

	cp = ConstraintPoints(path_template_landmarks_One, path_scan_landmarks_One, V_template, V_scan, F_template, F_scan, closepoint_thresh);
	cp.getStaticConstraints(constrained_indices, constrained_positions);

	cout << "\tstatic iterations ";
	for(int i=0; i<staticIterations; i++){
		cout << ".";
		iterate();	
	}
	cout << " done" << endl;

	cout << "\tdynamic iterations ";
	for(int i=0; i<dynmaicIterations; i++){
		cout << ".";
		cp.getDynamicConstraints(V_warped, constrained_indices, constrained_positions, closepoint_thresh);
		iterate();
	}
	cout << " done" << endl;

	add = "non_rigid_aligned.obj";
	string resultName = nonRigidPath + commonName + add;
	writeOBJ(resultName, V_warped, F_warped);
	cout << "\tsaved" << endl;
	
}

void NonRigid::nonRigidAlignment(string resultName, double lambda_, int iterations, ConstraintsMode constraints_mode, double closepoint_thresh_){
	closepoint_thresh = closepoint_thresh_;
	lambda = lambda_;

    if(constraints_mode==ALL){
		cp.getStaticConstraints(constrained_indices, constrained_positions);
		cp.getDynamicConstraints(V_warped, constrained_indices, constrained_positions, closepoint_thresh);
		for(int i=0; i<iterations; i++){
			iterate();
		}
	}
	else if(constraints_mode==LANDMARK_BOUNDARY){
		cp.getStaticConstraints(constrained_indices, constrained_positions);
		for(int i=0; i<iterations; i++){
			cout << "\tLandmark & Boundary iteration " << i << " ..." << endl;
			iterate();
		}
	}else if(constraints_mode==UPDATE_CLOSEPOINTS){
		for(int i=0; i<iterations; i++){
			cout << "\tClosepoint iteration " << i << " ..." << endl;
			cp.getDynamicConstraints(V_warped, constrained_indices, constrained_positions, closepoint_thresh);
			iterate();
		}
	}
	writeOBJ(resultName, V_warped, F_warped);
	cout << "\tsaved" << endl;
}

void NonRigid::iterate(){

	Eigen::SimplicialCholesky<SparseMatrix<double>, Eigen::RowMajor > solver;

	int iterations = 1;

	for(int k=0; k<iterations; k++){

		Eigen::MatrixXd result_points, Lx, b, RHS, C_w;
		Eigen::SparseMatrix<double> C, A, LHS, L;
		std::vector<T> tripletList_C;

		igl::cotmatrix(V_warped, F_warped, L);

		int num_cons = constrained_positions.rows();
		C.resize(num_cons, V_warped.rows());
		C.setZero();
		C_w.resize(num_cons, 3);
		C_w.setZero();

		for(int i=0; i<num_cons; i++){
			tripletList_C.push_back(T(i, constrained_indices(i), lambda ));
			C_w.row(i) << constrained_positions.row(i);
		}

		C.setFromTriplets(tripletList_C.begin(), tripletList_C.end());
		C_w = C_w*lambda;

		Lx = L * V_warped;
		
		igl::cat(1, Lx, C_w, b);
		igl::cat(1, L, C, A);
		RHS = A.transpose()*b;
		LHS = A.transpose()*A;
		
		LHS.makeCompressed();
		solver.compute(LHS);

		if (solver.info() != Eigen::Success) {
			cout << "\tSolver failed" << endl;
		} 
		result_points = solver.solve(RHS);

		V_warped = result_points;
	}

}

void NonRigid::visualizeResult(Viewer &viewer, VisualizationMode visualization_mode){
	
	viewer.data().clear();
	Eigen::MatrixXd red(1,3); red << 1.f,0.f,0.f;     
	Eigen::MatrixXd green(1,3); green << 0.f,1.f,0.f;   
	Eigen::MatrixXd blue(1,3); blue << 0.f,0.f,1.f; 

	if(visualization_mode == WARP){
		viewer.data().set_mesh(V_warped, F_warped);
	}
	else if(visualization_mode == WARP_WITH_CONSTRAINTS){
		viewer.data().set_mesh(V_warped, F_warped);   
		viewer.data().point_size = pointsize_landmarks;
		viewer.data().add_points(cp.landmark_constrained_positions, red.replicate(cp.landmark_constrained_positions.rows(),1)); 
		viewer.data().add_points(cp.boundary_constrained_positions, blue.replicate(cp.boundary_constrained_positions.rows(),1)); 
		viewer.data().add_points(cp.closepoint_constrained_positions, green.replicate(cp.closepoint_constrained_positions.rows(),1)); 
	}
	else if(visualization_mode == TEMPLATE){
		viewer.data().set_mesh(V_template, F_template);
	}
	else if(visualization_mode == SCAN){
		viewer.data().set_mesh(V_scan, F_scan);
	}
	else if(visualization_mode == TEMPLATE_WITH_OVERLAY){
		viewer.data().set_mesh(V_template, F_template);
		viewer.data().point_size = pointsize_overlay;
    	viewer.data().add_points(V_scan, blue.replicate(V_scan.rows(),1)); 
	}
	else if(visualization_mode == TEMPLATE_WITH_WARPED){
		viewer.data().set_mesh(V_template, F_template);
		viewer.data().point_size = pointsize_overlay;
    	viewer.data().add_points(V_warped, blue.replicate(V_warped.rows(),1)); 
	}
	else if(visualization_mode == TEMPLATE_WITH_CONSTRAINTS){
		viewer.data().set_mesh(V_template, F_template); 
		viewer.data().point_size = pointsize_landmarks;
		viewer.data().add_points(cp.landmark_constrained_positions, red.replicate(cp.landmark_constrained_positions.rows(),1)); 
		viewer.data().add_points(cp.boundary_constrained_positions, blue.replicate(cp.boundary_constrained_positions.rows(),1)); 
		viewer.data().add_points(cp.closepoint_constrained_positions, green.replicate(cp.closepoint_constrained_positions.rows(),1)); 
	}
}

void NonRigid::drawLandmarks(Viewer& viewer)
{
    viewer.data().point_size = pointsize_landmarks;
    viewer.data().set_points(Eigen::MatrixXd::Zero(0, 3), Eigen::MatrixXd::Zero(0, 3));

    for (int i = 0; i < cp.landmark_constrained_positions.rows(); ++i)
    {
        viewer.data().add_points(cp.landmark_constrained_positions.row(i), RowVector3d(0.0, 1.0, 0.0));
        viewer.data().add_label(cp.landmark_constrained_positions.row(i), std::to_string(i + 1));
    }
}


	
    
