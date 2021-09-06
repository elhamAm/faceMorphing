#ifndef __proj__SpatialIndex__
#define __proj__SpatialIndex__

#include <Eigen/Dense>
#include <igl/opengl/glfw/Viewer.h>
using Viewer = igl::opengl::glfw::Viewer;

class SpatialIndex
{
public:
public:
	SpatialIndex(Eigen::MatrixXd& P_, double radius);
	SpatialIndex();
	~SpatialIndex();

private:
	int nx, ny, nz; // number of cells in each direction
	int size;		// Number of entries
	//int resolution; // Resolution: Maximum (or Minimum) number of cells in each direction
	double cellsize;				   // Cell size of the Spatial Index grid
	std::vector<std::vector<int> > SI; // Spatial Index
	Eigen::MatrixXd P;				   // Points stored in the index
	Eigen::VectorXi P_cell;			   // Index of every point in P
	Eigen::RowVector3d grid_origin;	   // Origin of spatial index grid
	Eigen::RowVector3d dim_mesh;
	Eigen::RowVector3d bb_min, bb_max;

	double enlarge_amount = 0.1;

public:
	void create();
	void compute_indices(Eigen::MatrixXd& points, Eigen::VectorXi& indices);
	void get_neighbor_cells(int index, std::vector<int>& neighbors);
	void query_index(Eigen::MatrixXd& query, std::vector<int>& neighbor_points);
	void query_index_searchradius(Eigen::MatrixXd& query, std::vector<int>& neighbor_points, double radius);
	int closest_point(Eigen::MatrixXd& query, int& closest_point);
	int closest_point_searchradius(Eigen::MatrixXd& query, int& closest_point, double radius);
	void closest_point_index(Eigen::MatrixXd& query, Eigen::MatrixXd& points, int& index);
};

#endif