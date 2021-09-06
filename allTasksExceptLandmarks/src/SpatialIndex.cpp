#include "SpatialIndex.h"

#include <iostream>
#include <fstream>

using namespace igl;
using namespace std;
using namespace Eigen;


SpatialIndex::SpatialIndex(Eigen::MatrixXd& P_, double radius):
P(P_),
cellsize(radius*1.05)
{
    create();
}
SpatialIndex::SpatialIndex()
{
}

SpatialIndex::~SpatialIndex()
{
}

void SpatialIndex::create()
{
    SI.clear();
    /*------------------------------------------------------
                     Enlarge Bounding Box
    -------------------------------------------------------*/
    bb_min = P.colwise().minCoeff().eval();
    bb_max = P.colwise().maxCoeff().eval();
    dim_mesh = bb_max - bb_min;
    Eigen::RowVector3d dim_mesh_enlarged = dim_mesh + (enlarge_amount*dim_mesh);

    /*------------------------------------------------------
                        Number of Cells
    -------------------------------------------------------*/
    nx = ceil(dim_mesh_enlarged[0]/cellsize + 1);
    ny = ceil(dim_mesh_enlarged[1]/cellsize + 1);
    nz = ceil(dim_mesh_enlarged[2]/cellsize + 1);

    /*------------------------------------------------------
                      Compute Grid Origin
                    (and translate points)
    -------------------------------------------------------*/
    Eigen::RowVector3d offset;
    offset << enlarge_amount*dim_mesh_enlarged/2.0;

    grid_origin = bb_min - offset;

    //Get grid cells of x,y, and z-coordinates of every point in P relative to grid origin
    Eigen::MatrixXd P_translated(P.rows(),P.cols());
    P_translated = P.array() - grid_origin.replicate(P.rows(),1).array();
    /*------------------------------------------------------
                Point Locations in Grid Space
    -------------------------------------------------------*/
    Eigen::VectorXi P_cell_i;
    compute_indices(P, P_cell_i);
    P_cell = P_cell_i;

    /*------------------------------------------------------
                  Fill Spatial Index Data Structure
    -------------------------------------------------------*/
    int num_grid_points = (nx+1)*(ny+1)*(nz+1);
    int num_grid_cells = nx*ny*nz;
    size = num_grid_cells;

    for(int i=0; i<size; i++){
        SI.push_back(std::vector<int>());
    }
    for(int i=0; i<P_cell.rows(); i++){
        int cell = P_cell[i];
        SI[cell].push_back(i);
    }

}

// Returns the index of the points in the spatial index grid
void SpatialIndex::compute_indices(Eigen::MatrixXd& points, Eigen::VectorXi& indices)
{
    Eigen::MatrixXd points_transl(points.rows(), points.cols());
    points_transl = points - grid_origin.replicate(points.rows(),1);
    Eigen::MatrixXi cell_coords(points.rows(), points.cols());
    
    cell_coords.col(0) << (points_transl.col(0)/cellsize).array().floor().cast<int>(); // (Point/Dimension)
    cell_coords.col(1) << (points_transl.col(1)/cellsize).array().floor().cast<int>();
    cell_coords.col(2) << (points_transl.col(2)/cellsize).array().floor().cast<int>();

    indices = (cell_coords.col(0) + (nx * (cell_coords.col(1) + (ny * cell_coords.col(2))))).cast<int>(); 
}

void SpatialIndex::get_neighbor_cells(int index, std::vector<int>& neighbors){
    neighbors.clear();
    Eigen::VectorXi neighbor_cells(27);
    Eigen::VectorXi slice1(9);
    slice1 << 0, 1, -1, nx, -nx, 1+nx, 1-nx, -1+nx, -1-nx;
    slice1 += Eigen::VectorXi::Constant(9,index);
    Eigen::VectorXi slice2(9);
    Eigen::VectorXi slice3(9);
    slice2 = slice1 + Eigen::VectorXi::Constant(9,nx*ny);
    slice3 = slice1 + Eigen::VectorXi::Constant(9,-nx*ny);
    neighbor_cells << slice1, slice2, slice3;

    // remove out of bound cells
    for(int i=0; i<27; i++){
        int cell = neighbor_cells[i];
        if(cell < size && cell >= 0 ){
            neighbors.push_back(cell);
        }
    }
}


// Query the Spatial index with one or more query points
void SpatialIndex::query_index(Eigen::MatrixXd& query, std::vector<int>& neighbor_points){
    Eigen::VectorXi index;
    compute_indices(query, index); //Returns only a single index

    std::vector<int> neighbor_cells;
    get_neighbor_cells(index[0], neighbor_cells);

    for(int j=0; j<neighbor_cells.size(); j++){
        std::vector<int> entry;
        entry = SI[neighbor_cells[j]];
        if(!entry.empty()){
            //there are points in that cell. We add them to the neighbor_points
            neighbor_points.insert(neighbor_points.end(), entry.begin(), entry.end());
        }
    }
}
// Query the Spatial index with one or more query points
void SpatialIndex::query_index_searchradius(Eigen::MatrixXd& query, std::vector<int>& neighbor_points, double radius){
    
    Eigen::VectorXi index;
    compute_indices(query, index); //Returns only a single index

    std::vector<int> neighbor_cells;
    get_neighbor_cells(index[0], neighbor_cells);

    for(int j=0; j<neighbor_cells.size(); j++){
        std::vector<int> entry;
        entry = SI[neighbor_cells[j]];
        if(!entry.empty()){
            //there are points in that cell. We add them to the neighbor_points
            for(int n=0; n<entry.size(); n++){
                if((P.row(entry[n])-query.row(0)).norm() < radius){
                    neighbor_points.push_back(entry[n]);
                }
            }
        }
    }
}

// Returns the index of the closest point
int SpatialIndex::closest_point(Eigen::MatrixXd& query, int& closest_point){
    std::vector<int> neighbor_points;
    query_index(query, neighbor_points);

    // Transform std:vector to Eigen::Matrix
    Eigen::MatrixXd neighbor_points_Mat(neighbor_points.size(), 3);
    for(int j=0; j<neighbor_points.size(); j++){
        neighbor_points_Mat.row(j) = P.row(neighbor_points[j]);
    }
    if(neighbor_points.size()==0){
        return 0;
    }
    int closest_point_ind = 0;
    closest_point_index(query, neighbor_points_Mat, closest_point_ind);
    closest_point = neighbor_points[closest_point_ind];
    return neighbor_points.size();
}

// Returns the index of the closest point
int SpatialIndex::closest_point_searchradius(Eigen::MatrixXd& query, int& closest_point, double radius){
    std::vector<int> neighbor_points;
    query_index_searchradius(query, neighbor_points, radius);

    // Transform std:vector to Eigen::Matrix
    Eigen::MatrixXd neighbor_points_Mat(neighbor_points.size(), 3);
    for(int j=0; j<neighbor_points.size(); j++){
        neighbor_points_Mat.row(j) = P.row(neighbor_points[j]);
    }
    if(neighbor_points.size()==0){
        return 0;
    }
    int closest_point_ind = 0;
    closest_point_index(query, neighbor_points_Mat, closest_point_ind);
    closest_point = neighbor_points[closest_point_ind];
    return neighbor_points.size();
}

void SpatialIndex::closest_point_index(Eigen::MatrixXd& query, Eigen::MatrixXd& points, int& index){
    /*------------------------------------------------------
                    Compute Squared Distances
                + Retrieve Index of Closest Point
    -------------------------------------------------------*/
    int col_index;
    double min = (points-query.replicate(points.rows(),1)).rowwise().squaredNorm().minCoeff(&index, &col_index);
}
