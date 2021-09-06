#include "ConstraintPoints.h"

#include <iostream>
#include <fstream>

using namespace igl;
using namespace std;
using namespace Eigen;


ConstraintPoints::ConstraintPoints(string path_landmark_template, string path_landmark_scan, Eigen::MatrixXd& V_template_, Eigen::MatrixXd& V_scan_, Eigen::MatrixXi& F_template_, Eigen::MatrixXi& F_scan_, double thresh_global):
V_template(V_template_),
V_scan(V_scan_),
F_template(F_template_),
F_scan(F_scan_),
closepoint_thresh(thresh_global)
{
    spatial_index_global = SpatialIndex(V_scan, closepoint_thresh_max);
	//cout << "here" << endl;
	path_scan_landmarks = path_landmark_scan;
	//cout << "path_scan_landmarks: " << path_scan_landmarks << endl;

	path_template_landmarks = path_landmark_template;
	//cout << "path_template_landmarks: " << path_template_landmarks << endl;

}


ConstraintPoints::ConstraintPoints()
{
}

ConstraintPoints::~ConstraintPoints()
{
}

void ConstraintPoints::recomputeSpatialIndex(double closepoint_thresh){
	spatial_index_global = SpatialIndex(V_scan, closepoint_thresh);
	getLandmarkConstraints();
	getBoundaryConstraints();
}

void ConstraintPoints::getStaticConstraints(Eigen::VectorXi& constrained_indices, Eigen::MatrixXd& constrained_positions){
	getLandmarkConstraints();
	getBoundaryConstraints();
	igl::cat(1, landmark_constrained_indices, boundary_constrained_indices, constrained_indices);
	igl::cat(1, landmark_constrained_positions, boundary_constrained_positions, constrained_positions);
}

void ConstraintPoints::getDynamicConstraints(Eigen::MatrixXd& V_current, Eigen::VectorXi& constrained_indices, Eigen::MatrixXd& constrained_positions, double closepoint_thresh_){
	closepoint_thresh = closepoint_thresh_;
	getClosepointConstraints(V_current);
	Eigen::VectorXi temp_indices;
	Eigen::MatrixXd temp_positions;

	igl::cat(1, landmark_constrained_indices, boundary_constrained_indices, temp_indices);
	igl::cat(1, landmark_constrained_positions, boundary_constrained_positions, temp_positions);

	igl::cat(1, temp_indices, closepoint_constrained_indices, constrained_indices);
	igl::cat(1, temp_positions, closepoint_constrained_positions, constrained_positions);
}

/*------------------------------------------------------
            	Landmark Constrained Points
-------------------------------------------------------*/
void ConstraintPoints::getLandmarkConstraints(){

    // Vertex indices of landmark vertices
    Eigen::VectorXi Indices_scan(0,1), Indices_template(0,1); 

    // Vertex positions of landmark vertices
    Eigen::MatrixXd Positions_scan(0,3), Positions_template(0,3);

	LandmarkLoader ll = LandmarkLoader();

	if(useBonusLandmarks){
		//ll.parseLandmarkFileBonus(path_landmarks_bonus_demofile, path_bonus_demofile_rigidaligned, Indices_scan, Positions_scan);
		ll.landmarksFromPositionsBonusRigidaligned(Indices_scan, Positions_scan);
		ll.parseLandmarkFileBonus(path_landmarks_bonus_templatefile, path_bonus_template, Indices_template, Positions_template);
	}else{
		ll.parseLandmarkFile(path_scan_landmarks, V_scan, Indices_scan, Positions_scan);
		ll.parseLandmarkFile(path_template_landmarks, V_template, Indices_template, Positions_template);
	}

    for(int i=0; i<ll.num_landmarks; i++){
		/*Elham: put in the indices of the landmarks in the sets*/
		template_constraints_indices_set.insert(Indices_template[i]);
		scan_constraints_indices_set.insert(Indices_scan[i]);
    }

	landmark_constrained_indices = Indices_template;
	landmark_constrained_positions = Positions_scan;
	//cout << "landmark_constrained_indices: " << landmark_constrained_indices.rows() << endl;

}

/*------------------------------------------------------
            	Boundary Constrained Points
-------------------------------------------------------*/
void ConstraintPoints::getBoundaryConstraints(){
    Eigen::VectorXi boundary_template, boundary_scan;
    igl::boundary_loop(F_template, boundary_template);
    igl::boundary_loop(F_scan, boundary_scan);

	if(boundary_mode == CLOSEST_BOUNDARY_VERTEX){
		boundary_constrained_positions.setZero(boundary_template.count(),3);
		boundary_constrained_indices.setZero(boundary_template.count());

		// NAIVE: For each template boundary vertex, get closest scan boundary vertex
		int boundary_scan_count = boundary_scan.count();
		int boundary_template_count = boundary_template.count();

		// Get positions of scan boundary points:
		Eigen::MatrixXd boundary_scan_positions(boundary_scan_count,3);
		/*Elham: I think a slice here would be faster*/
		igl::slice(V_scan, boundary_scan, 1, boundary_scan_positions);

		for(int i=0; i<boundary_template_count; i++){
			int vertex_id = boundary_template[i];
			Eigen::MatrixXd current_vertex = V_template.row(vertex_id).replicate(boundary_scan_count, 1); // with replicated rows

			// Retrieve index of closest boundary point (stored in row_index)
			int col_index, row_index;
			double min = (boundary_scan_positions-current_vertex).rowwise().squaredNorm().minCoeff(&row_index, &col_index);

			// Set position:
			boundary_constrained_positions.row(i) = V_scan.row(boundary_scan[row_index]);
		}
		
		boundary_constrained_indices = boundary_template;
	}
	/*Elham: I think this one is good enough with a dfs to get some other points as well*/
	else if(boundary_mode == NONE || boundary_mode == FIX_TEMPLATE_BOUNDARY){
		boundary_constrained_positions.setZero(boundary_template.count(),3);
		boundary_constrained_indices = boundary_template;
		igl::slice(V_template, boundary_constrained_indices, 1, boundary_constrained_positions);
		//cout << "boundary_constrained_indices: " << boundary_constrained_indices.rows() << endl;

		/*Elham: put in the indices of the boundaries of template in the set*/
		for(int i = 0; i < boundary_template.rows(); i++){
			//cout << boundary_template.row(i) << endl;
			template_constraints_indices_set.insert(boundary_template[i]);
		}
		/*Elham: put in the indices of the boundaries of scan in the set*/
		for(int i = 0; i < boundary_scan.rows(); i++){
			//cout << boundary_scan.row(i) << endl;
			scan_constraints_indices_set.insert(boundary_scan[i]);
		}

	}   
}

/*------------------------------------------------------
            	Closepoint Constrained Points
-------------------------------------------------------*/
void ConstraintPoints::getClosepointConstraints(Eigen::MatrixXd &V_current){
	std::vector<int> closepoint_scan_indices, closepoint_template_indices;
	closepoint_scan_indices.clear();
	closepoint_template_indices.clear();

	for(int i=0; i<V_current.rows(); i++){
		/*Elham: check is this vertex is not already in the set*/
		if(template_constraints_indices_set.find(i) != template_constraints_indices_set.end()){
			continue;
		} 	
		Eigen::MatrixXd query = V_current.row(i);
		int closest_point_index = -1;
		if(spatial_index_global.closest_point_searchradius(query, closest_point_index, closepoint_thresh) > 0){
			/*Elham: check is this closest vertex is not already in the set of constraints of the scan*/
			if(scan_constraints_indices_set.find(closest_point_index) != template_constraints_indices_set.end()){
				continue;
			} 
			// We have a closest point index;
			closepoint_scan_indices.push_back(closest_point_index);
			closepoint_template_indices.push_back(i);
		}
	}

	int num_points = closepoint_scan_indices.size();
	closepoint_constrained_indices.setZero(num_points);
	closepoint_constrained_positions.setZero(num_points, 3);
    
	for(int i=0; i<num_points; i++){
		closepoint_constrained_indices[i] = closepoint_template_indices[i];
		closepoint_constrained_positions.row(i) = V_scan.row(closepoint_scan_indices[i]);
	}
}


