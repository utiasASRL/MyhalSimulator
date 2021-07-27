
#include <teb_local_planner/predicted_costmap.h>
#include <ros/console.h>
#include <ros/assert.h>


namespace teb_local_planner
{

PredictedCostmap::PredictedCostmap() : resolution_(0.0), width_(0), height_(0), grid_(0), initialized_(false)
{
}

PredictedCostmap::~PredictedCostmap()
{
}

void PredictedCostmap::initialize(const nav_msgs::OccupancyGrid occupancy_grid)
{
  // get information from ros message
  width_ = occupancy_grid.info.width;
  height_ = occupancy_grid.info.height; 
  resolution_ = occupancy_grid.info.resolution;
  grid_origin_x = occupancy_grid.info.origin.position.x;
  grid_origin_y = occupancy_grid.info.origin.position.y;
  grid_.resize(height_, std::vector<int8_t>(width_, 0));
  int count = 0;
  for(int iy=0; iy < grid_.size(); iy++){
    for(int ix=0; ix < grid_[0].size(); ix++){
      grid_[iy][ix] = occupancy_grid.data[count];
      count++;
    }
  } 

  sobel_x << 1, 0, -1,
             2, 0, -2,
             1, 0, -1;
  sobel_y << 1, 2, 1,
             0, 0, 0,
            -1, -2, -1;
  initialized_ = true;
}

void PredictedCostmap::posToIndices(PoseSE2 pos, int *row, int *col)
{
  // *row = (int) floor((this->origin_[1] - pos.y()) / this->resolution_);
  // *col = (int) floor((this->origin_[0] - pos.x()) / this->resolution_);
  *row = (int) floor((pos.y() - this->grid_origin_y) / this->resolution_);
  *col = (int) floor((pos.x() - this->grid_origin_x) / this->resolution_);
}

void PredictedCostmap::posToIndices(VertexPose pos, int *row, int *col)
{
  PoseSE2 pos_SE2(pos.x(), pos.y(), pos.theta());
  this->posToIndices(pos_SE2, row, col);
}

void PredictedCostmap::getCostmapValue(int row, int col, int *value)
{
  *value = this->grid_[row][col];
}

void PredictedCostmap::interpolateCostmapValue(PoseSE2 pos, double *interpolation)
{
  int r2 = 0, c2 = 0;
  int v00, v10, v01, v11;
  double tx, ty, a, b;
  PoseSE2 pos2(pos.x() + 0.5, pos.y() + 0.5, pos.theta());
  this->posToIndices(pos, &r2, &c2);

  v00 = this->grid_[r2-1][c2-1];
  v10 = this->grid_[r2-1][c2];
  v01 = this->grid_[r2][c2-1];
  v11 = this->grid_[r2][c2];

  tx = (pos.x() - this->grid_origin_x) / this->resolution_ + 0.5 - c2;
  ty = (pos.y() - this->grid_origin_y) / this->resolution_ + 0.5 - r2;
  a = v00 + tx * (v10 - v00);
  b = v01 + tx * (v11 - v01);
  
  *interpolation = a + ty * (b - a);
}

void PredictedCostmap::interpolateCostmapValue(VertexPose pos, double *interpolation)
{
  PoseSE2 pos_SE2(pos.x(), pos.y(), pos.theta());
  interpolateCostmapValue(pos_SE2, interpolation);
}

void PredictedCostmap::computeDerivativePos(PoseSE2 pos, int *dx, int *dy)
{
  int row_pos = 0, col_pos = 0, row_subgrid = 0, col_subgrid = 0; 
  this->posToIndices(pos, &row_pos, &col_pos);

  // convolve on 3x3 neighbouring grid poses with the sobel operators
  int sum_x = 0, sum_y = 0; 
  for(int irow = row_pos - 1; irow <= row_pos + 1; irow++)
  {
    for(int icol = col_pos - 1; icol <= col_pos + 1; icol++)
    {
      sum_x += sobel_x(row_subgrid, col_subgrid) * grid_[irow][icol];
      sum_y += sobel_y(row_subgrid, col_subgrid) * grid_[irow][icol];
      col_subgrid++;
    }
    row_subgrid++;
    col_subgrid = 0; 
  }
  *dx = sum_x;
  *dy = sum_y;
}

////// PredictedCostmap3D //////

// PredictedCostmap3D::PredictedCostmap3D() : dl_(0.0), width_(0), height_(0), depth_(0), grid_(new Grid3D()), initialized_(false)
PredictedCostmap3D::PredictedCostmap3D() : dl_(0.0), width_(0), height_(0), depth_(0), data_(0), initialized_(false)
{
}

PredictedCostmap3D::~PredictedCostmap3D()
{
}

void PredictedCostmap3D::initialize(const teb_local_planner::VoxGrid voxel_grid)
{
  // get information from ros message
  stamped_time_ = voxel_grid.header.stamp.toSec();
  initial_time_ = voxel_grid.origin.z; // origin is given (x, y, t)
  width_ = (int) voxel_grid.width;
  height_ = (int) voxel_grid.height;
  depth_ = (int) voxel_grid.depth;
  dl_ = voxel_grid.dl;
  dt_ = voxel_grid.dt;
  grid_origin_x = voxel_grid.origin.x;
  grid_origin_y = voxel_grid.origin.y;
  theta_ = voxel_grid.theta;
  
  // data_ is a flattened array 3D in row-major order
  data_.reserve(width_ * height_ * depth_);
  for (std::vector<uint8_t>::const_iterator i = voxel_grid.data.begin(); i != voxel_grid.data.end(); ++i)
      data_.push_back(((float)unsigned(*i)) / 255);

  // grid3D_ = boost::make_shared<Grid3D>(Grid3D(voxel_grid.data));

  /*

  std::cout << "------------------------------------" << std::endl;
  std::cout << "stamped_time_" << stamped_time_ << std::endl;
  std::cout << "initial_time_" << initial_time_ << std::endl;
  std::cout << "width_" << width_ << std::endl;
  std::cout << "height_" << height_ << std::endl;
  std::cout << "depth_" << depth_ << std::endl;
  std::cout << "dl_" << dl_ << std::endl;
  std::cout << "dt_" << dt_ << std::endl;
  std::cout << "grid_origin_x" << grid_origin_x << std::endl;
  std::cout << "grid_origin_y" << grid_origin_y << std::endl;
  std::cout << "theta_" << theta_ << std::endl;
  std::cout << "------------------------------------" << std::endl;
  
  // data_ is a flattened array 3D in row-major order
  int dbgi = 1;
  dbgi = 0;
  for (std::vector<uint8_t>::const_iterator i = voxel_grid.data.begin(); i != voxel_grid.data.end(); ++i)
  {
      data_.push_back(unsigned(*i));

      if (dbgi == 0 || dbgi == 1 || dbgi == 2 || dbgi == 10 || dbgi == 100 || dbgi == 1000) 
        std::cout << unsigned(*i) << std::endl;
      dbgi++;

  }
  std::cout << "------------------------------------" << std::endl;
  int row, col, dt;
  dt = 0;
  row = 0;
  col = 0;
  std::cout << data_[(row * width_ + col)*depth_ + dt] << std::endl;
  dt = 1;
  row = 0;
  col = 0;
  std::cout << data_[col + width_ * (row +  height_ * dt)] << std::endl;
  dt = 0;
  row = 1;
  col = 0;
  std::cout << data_[col + width_ * (row +  height_ * dt)] << std::endl;
  dt = 0;
  row = 0;
  col = 1;
  std::cout << data_[col + width_ * (row +  height_ * dt)] << std::endl;
  dt = 1;
  row = 2;
  col = 3;
  std::cout << data_[col + width_ * (row +  height_ * dt)] << std::endl;
  dt = 3;
  row = 1;
  col = 2;
  std::cout << data_[col + width_ * (row +  height_ * dt)]<< std::endl;
  dt = 2;
  row = 3;
  col = 1;
  std::cout << data_[col + width_ * (row +  height_ * dt)]<< std::endl;
  std::cout << "------------------------------------" << std::endl;
  */

   
  //TODO: share sobels with other PredictedCostmap
  sobel_x << 1, 0, -1,
             2, 0, -2,
             1, 0, -1;
  sobel_y << 1, 2, 1,
             0, 0, 0,
            -1, -2, -1;

  initialized_ = true;
}

void PredictedCostmap3D::interpolateCostmapValue(VertexPose pos, double *interpolation, double layer)
{

  // Get pose of the point and offset by 0.5 * grid_dl
  double offset = dl_ * 0.5;
  PoseSE2 pos2(pos.x() + offset, pos.y() + offset, pos.theta());
  
  // Get corresponding row and column of the bottom right pixel of the interpolation
  int r2 = 0, c2 = 0;
  this->posToIndices(pos2, &r2, &c2);

  if (r2 < 1 || r2 > height_ - 1 || c2 < 1 || c2 > width_ - 1)
  {
    // std::cout << "      L=" << layer << "      pos=(" << pos.x() << ", " << pos.y() << ") -  c2="  << c2 << ", r2=" << r2 << ", v=" << v11  << std::endl;
    *interpolation = 0;
    return;
  }

  // // Get the inds of the two layer between which we interpolate
  // int layer1 = (int)std::floor(layer);
  // int layer0 = layer1 - 1;

  // For now we use closest interpolation
  int closest_layer = (int)std::round(layer) - 1;

  if (closest_layer < 1 || closest_layer > depth_ - 1)
  {
    *interpolation = 0;
    return;
  }

  // Get the 4 value to interpolate from
  double v00, v10, v01, v11;
  v00 = (double)(this->get(r2-1, c2-1, closest_layer));
  v01 = (double)(this->get(r2, c2-1, closest_layer));
  v10 = (double)(this->get(r2-1, c2, closest_layer));
  v11 = (double)(this->get(r2, c2, closest_layer));

  // Interpolate
  double tx, ty, a, b;
  tx = (pos.x() - grid_origin_x) / dl_ + 0.5 - c2;
  ty = (pos.y() - grid_origin_y) / dl_ + 0.5 - r2;
  a = v00 + tx * (v10 - v00);
  b = v01 + tx * (v11 - v01);

  *interpolation = a + ty * (b - a); 
  
  return;
}

void PredictedCostmap3D::posToIndices(PoseSE2 pos, int *row, int *col)
{
  *row = (int) floor((pos.y() - grid_origin_y) / dl_);
  *col = (int) floor((pos.x() - grid_origin_x) / dl_);
}

void PredictedCostmap3D::computeDerivativePos(PoseSE2 pos, int layer, double *dx, double *dy)
{
  int row_pos = 0, col_pos = 0, row_subgrid = 0, col_subgrid = 0; 
  this->posToIndices(pos, &row_pos, &col_pos);

  // convolve on 3x3 neighbouring grid poses with the sobel operators
  double sum_x = 0, sum_y = 0; 
  for(int irow = row_pos - 1; irow <= row_pos + 1; irow++)
  {
    for(int icol = col_pos - 1; icol <= col_pos + 1; icol++)
    {
      sum_x += (double)(sobel_x(row_subgrid, col_subgrid)) * (double)(this->get(irow, icol, layer));
      sum_y += (double)(sobel_y(row_subgrid, col_subgrid)) * (double)(this->get(irow, icol, layer));
      col_subgrid++;
    }
    row_subgrid++;
    col_subgrid = 0; 
  }
  *dx = sum_x;
  *dy = sum_y; 
}


} // end namespace