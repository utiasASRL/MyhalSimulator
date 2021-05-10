
#ifndef EDGE_PREDICTED_COSTMAP_H_
#define EDGE_PREDICTED_COSTMAP_H_

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

class EdgePredictedCostmap : public BaseTebUnaryEdge<1, PredictedCostmap*, VertexPose>
{
public:
  /**
   * @brief Construct edge.
   */    
  EdgePredictedCostmap()
  {
    _measurement = new PredictedCostmap; 
  }
  
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setPredictedCostmap() and setRobotModel() on EdgePredictedCostmap()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    double interpolation = 0.0; 
    
    _measurement->interpolateCostmapValue(bandpt->pose(), &interpolation);
    _error[0] = interpolation * cfg_->optim.weight_predicted_costmap; 
    
    // UNCOMMENT TO SEND ERROR TO TXT FOR PLOTTING
    // std::ofstream myfile;
    // myfile.open("/home/mgsa/devfile/edgePredictedCostmap_error.txt", std::ios_base::app);
    // myfile << bandpt->pose().x() << " " << bandpt->pose().y() << " " << _error[0] << " " << "\n";
    // myfile.close();

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgePredictedCostmap::computeError() _error[0]=%f\n",_error[0]);
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
    _measurement->computeDerivativePos(bandpt->pose(), &dx, &dy);

    _jacobianOplusXi(0, 0) = - dx * cfg_->optim.weight_predicted_costmap; 
    _jacobianOplusXi(0, 1) = - dy * cfg_->optim.weight_predicted_costmap;

    // std::ofstream myfile;
    // myfile.open("/home/mgsa/devfile/edgePredictedCostmap_jacobian.txt", std::ios_base::app);
    // myfile << bandpt->pose().x() << " " << bandpt->pose().y() << " " << _jacobianOplusXi(0, 0) << " " << _jacobianOplusXi(0, 1) << "\n";
    // myfile.close();
  }
#endif

  

  /**
   * @brief Set pointer to associated predicted costmap for the underlying cost function 
   * @param predictions 2D costmap containing information about collision probabilities
   */ 
  void setPredictedCostmap(PredictedCostmap* predictions)
  {
    _measurement = predictions;
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
   * @param predictions 2D costmap containing information about collision probabilities
   */ 
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, PredictedCostmap* predictions)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = predictions;
  }


protected:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace
#endif
