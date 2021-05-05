
#ifndef EDGE_PREDICTED_COSTMAP3D_H_
#define EDGE_PREDICTED_COSTMAP3D_H_

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/predicted_costmap.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>


#include <iostream>
#include <fstream>



namespace teb_local_planner
{

class EdgePredictedCostmap3D : public BaseTebUnaryEdge<1, PredictedCostmap3D*, VertexPose>
{
public:
  /**
   * @brief Construct edge.
   */    
  EdgePredictedCostmap3D()
  {
    _measurement = new PredictedCostmap3D; 
  }
  
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {

    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setPredictedCostmap3D() and setRobotModel() on EdgePredictedCostmap3D()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    double interpolation = 0.0;
    std::cout << "Interpolating " << std::endl; 
    _measurement->interpolateCostmapValue(bandpt->pose(), &interpolation, layer_); 
    std::cout << "Interpolation value at layer: " << layer_ << " " << interpolation << std::endl; 
    _error[0] = interpolation * cfg_->optim.weight_predicted_costmap;
    
  }


// #ifdef USE_ANALYTIC_JACOBI
#if 1
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    int dx = 0, dy = 0;  
    _measurement->computeDerivativePos(bandpt->pose(), layer_,  &dx, &dy);

//TODO: evaluate relevance of optim weight predicted costmap weight here
    _jacobianOplusXi(0, 0) = - dx * cfg_->optim.weight_predicted_costmap; 
    _jacobianOplusXi(0, 1) = - dy * cfg_->optim.weight_predicted_costmap;
  }
#endif

  

  /**
   * @brief Set pointer to associated predicted costmap for the underlying cost function 
   * @param predictions3D 3D costmap containing information about collision probabilities
   */ 
  void setPredictedCostmap(PredictedCostmap3D* predictions3D)
  {
    _measurement = predictions3D;
  }

  /**
   * @brief Set pointer to the robot model 
   * @param robot_model Robot model required for distance calculation
   */ 
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param predictions3D 3D costmap containing information a1bout collision probabilities
   * @param layer Pointer to the layer (depth) of the 3D costmap
   */ 
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, PredictedCostmap3D* predictions3D, int layer)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = predictions3D;
    layer_ = layer;
  }


protected:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  int layer_; //!< Store to the 3D costmap layer
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace
#endif
