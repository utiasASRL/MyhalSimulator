#ifndef PREDICTED_COSTMAP_H_
#define PREDICTED_COSTMAP_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/make_shared.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <teb_local_planner/VoxGrid.h> // For the Predicted Costmap 3D
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

// #include <tf/tf.h>

namespace teb_local_planner
{

// @TODO: would have been smarter to create only 1 class PredictedCostmap with a flag if 3D or not. Check the flag, and create the error function accordingly. 
// Having 2 classes makes the initialization of all other instances of the planner more complex by adding a parameter everywhere

/**
 * @class PredictedCostmap
 * @brief Class that defines the interface to interact with the predicted costmap of the collision checker Neural Network
 */    
class PredictedCostmap
{
public:
  /**
    * @brief Default constructor of the PredictedCostmap class
    */
  PredictedCostmap();

  /**
   * @brief Virtual destructor.
   */
  ~PredictedCostmap();

  /**
    * @brief Initializes the PredictedCostmap class with the values in the ROS message
    * @param occupancy_grid OccupancyGrid ros message
    */
  void initialize(const nav_msgs::OccupancyGrid occupancy_grid);


  /**
   * @brief  Returns the grid coordinates (row, col) corresponding to the input PoseSE2 
   * @param pos PoseSE2 (x, y, theta) 
   * @return (row, col) on costmap
   */
  void posToIndices(PoseSE2 pos, int *row, int *col);

  /**
   * @brief  Returns the grid coordinates (row, col) corresponding to the input PoseSE2 
   * @param pos VertexPose (x, y, theta) for compatibility with g2o
   * @return (row, col) on costmap
   */
  void posToIndices(VertexPose pos, int *row, int *col);

  /**
   * @brief  Pass in value the costmap value at grid coordinates (row, col)
   */
  void getCostmapValue(int row, int col, int *value);

  /**
   * @brief Interpolate costmap value at pose
   * @param pos (x, y) position where to interpolate from the costmap
   * @param interpolation Pointer to double
   */
  void interpolateCostmapValue(PoseSE2 pos, double *interpolation);

  void interpolateCostmapValue(VertexPose pos, double *interpolation);

  /**
  * @brief Use Sobel operators on a subgrid around pos to return the components of the gradient derivative = {gx, gy} at pos coordinates
  */
  void computeDerivativePos(PoseSE2 pos, int *dx, int *dy);

  void computeJacobian();

  /**
   * @brief  Returns the costmap width
   * @return The global frame of the costmap
   */
  uint32_t getWidth()
    {
      return width_;
    }

  double getResolution()
    {
      return resolution_;
    }

  bool isInitialized()
    {
      return initialized_;
    }

  void setResolution(double resolution)
    {
      resolution_ = resolution;
    }

  void setWidthHeight(uint32_t width, uint32_t height)
    {
      width_ = width;
      height_ = height;
    }
  


private:
  double grid_origin_x; //!< Grid origin of the testing environment
  double grid_origin_y; //!< Grid origin of the testing environment
  Eigen::Matrix3f sobel_x; //!< Sobel operator for x derivative
  Eigen::Matrix3f sobel_y; //!< Sobel operator for y derivative
  // std::vector<double> farid_5_k = {0.030320, 0.249724, 0.439911, 0.249724, 0.030320}; //!< See Farid and Simocelly derivation filters
  // std::vector<double> farid_5_d = {0.104550, 0.292315, 0.000000, -0.292315, -0.104550}; //!< See Farid and Simocelly derivation filters
  // std::vector<double> farid_5_dd = {-0.104550, -0.292315, 0.000000, 0.292315, 0.104550}; //!< See Farid and Simocelly derivation filters
  double resolution_; 
  int32_t width_;
  int32_t height_; 
  std::vector<std::vector<int8_t>> grid_;

  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class
};

//! Abbrev. for shared PredictedCostmap pointers
typedef boost::shared_ptr<PredictedCostmap> PredictedCostmapPtr;
//! Abbrev. for shared PredictedCostmap const pointers
typedef boost::shared_ptr<const PredictedCostmap> PredictedCostmapConstPtr;
//! Abbrev. for containers storing multiple PredictedCostmaps
typedef std::vector<PredictedCostmapPtr> PredictedCostmapContainer;


// Class declaration and type declaration for the polymorphism in PredictedCostmap3D
// class Grid3D;
// typedef boost::shared_ptr<Grid3D> Grid3DPtr;

class PredictedCostmap3D
{
public:
  /**
    * @brief Default constructor of the PredictedCostmap3D class
    */
  PredictedCostmap3D();

  /**
   * @brief Virtual destructor.
   */
  ~PredictedCostmap3D();

  /**
    * @brief Initializes the PredictedCostmap3D class with the values in the ROS message
    * @param voxel_grid OccupancyGrid ros message
    */
  void initialize(const teb_local_planner::VoxGrid voxel_grid);

  /**
    * @brief Interpolate the costmap values at \c pos
    * @param pos 2D position
    * @param interpolation pointer to the double storing interpolated value
    * @param layer indicates which layer (depth) in the costmap we are interpolating on
    */
  void interpolateCostmapValue(VertexPose pos, double *interpolation, double layer);

  /**
   * @brief Find grid coordinates (row, col) for position pos (x, y)
   */
  void posToIndices(PoseSE2 pos, int *row, int *col);

  /**
   * @brief Compute 2D Sobel derivatives on a 3x3 subgrid around position pos (x, y) on the 3D costmap at depth (layer)
   */
  void computeDerivativePos(PoseSE2 pos, int layer, double *dx, double *dy);

  /**
    * @brief Get the initial time of the predictions. This is the time of the first layer of the 3D costmap. 
    */
  double getInitialTime()
  {
    return initial_time_;
  }

  /**
    * @brief Get the stamepd time of the ros message publication.  
    */
  double getStampedTime()
  {
    return stamped_time_;
  }

  double getTemporalResolution()
  { 
    return dt_;
  }

  double getSpatialResolution()
  { 
    return dl_;
  }

  int getDepth()
  { 
    return depth_;
  }

  bool isInitialized()
  {
    return initialized_;
  }

  /**
   * @brief Return data_ value at position [row, col, dt] from the flattened array in row-major order.
   */
  float get(int row, int col, int dt)
  {
    //return data_[(row * width_ + col)*depth_ + dt];
    return data_[(dt * height_ + row) * width_ + col];
  }

  /**
   * @brief Get 3D grid size
   */
  int size()
  {
    return width_*height_*depth_;
  }

protected:
  double initial_time_; //<! Initial time of the costmap corresponding to the ros::Time of the first layer of the costmap. 
  double stamped_time_; //<! Stamped time of the ros message at publication.
  double dl_; //!< Spatial resolution (m)
  double dt_; //!< Temporal resolution (s)
  int width_; //!< Width of the costmap
  int height_; //!< Height of the costmap
  int depth_; //!< Depth of the costmap. Each layer represented predictions at a time. For example, layer 4 of depth 4 correspond to the collision prediction for time (initial_time + 4*dl_)
  double grid_origin_x; //!< Grid origin of the testing environment
  double grid_origin_y; //!< Grid origin of the testing environment
  double theta_; //!< Orientation of the map in the 2D plane [rad]
  std::vector<float> data_; //<! Dump data from VoxGrid ros message into this vector of int
  Eigen::Matrix3f sobel_x; //!< Sobel operator for x derivative
  Eigen::Matrix3f sobel_y; //!< Sobel operator for y derivative
  // Grid3DPtr grid3D_;

  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class
};

//! Abbrev. for shared PredictedCostmap3D pointers
typedef boost::shared_ptr<PredictedCostmap3D> PredictedCostmap3DPtr;
//! Abbrev. for shared PredictedCostmap3D const pointers
typedef boost::shared_ptr<const PredictedCostmap3D> PredictedCostmapConst3DPtr;
//! Abbrev. for containers storing multiple PredictedCostmap3D
typedef std::vector<PredictedCostmap3DPtr> PredictedCostmap3DContainer;


// /**
//  * @class Grid3D
//  * @brief Class that defines a 3D grid (height, width, depth) to contain the 3D collision predictions. 
//  */   
// class Grid3D: public PredictedCostmap3D
// {
// public:
//   /**
//     * @brief Default constructor of the Grid3D subclass
//     */
//   Grid3D();

//   /**
//     * @brief Constructor of the Grid3D subclass with the flattened 3D data from the ROS message. 
//     * Data must be flattened in row-major order. Transforms uint8_t into int.
//     */
//   Grid3D(const std::vector<uint8_t> grid_flattened);

//   /**
//    * @brief Virtual destructor.
//    */
//   ~Grid3D();

//   /**
//     * @brief Access coordinate (row, col, dt) from the row-order flattened data list. 
//     * TODO: would be nice to overload operator () or [] to access the elements, could not do it when I tried. 
//     */
//   int get(int row, int col, int dt);

//   inline unsigned size() const
//   {
// 		return width_*height_*depth_;
//   }

// protected: 
//   std::vector<int> data; //<! Dump data from VoxGrid ros message into this vector of int
// };


} // namespace teb_local_planner
#endif /* PREDICTED_COSTMAP_H_ */