
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
    //_measurement = new PredictedCostmap3D; 
  }
  
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    // Init verifications
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setPredictedCostmap3D() and setRobotModel() on EdgePredictedCostmap3D()");

    // Get the pose to the band point
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    // Interpolate the value in the costmap
    double interpolation = 0.0;
    _measurement->interpolateCostmapValue(bandpt->pose(), &interpolation, layer_); 


    // // *************************************** Debug Numeric Gradient ***************************************
    // std::cout << "Interp 0: L=" << layer_ << ", v=" << interpolation << std::endl; 
    // for (int iii=6; iii<10; iii++)
    // {
    //   double delta = std::pow(10, -iii);
    //   double interpolation1 = 0.0;
    //   teb_local_planner::PoseSE2 offset1(0, -delta, 0);
    //   _measurement->interpolateCostmapValue(bandpt->pose() + offset1, &interpolation1, layer_); 
      
    //   double interpolation2 = 0.0;
    //   teb_local_planner::PoseSE2 offset2(0, delta, 0);
    //   _measurement->interpolateCostmapValue(bandpt->pose() + offset2, &interpolation2, layer_); 

    //   std::cout << "Jacobi_y: L=" << layer_ << ", v=" << (interpolation2 - interpolation1) / (2 * delta)  << std::endl;  
    // } 
    // double dx = 0, dy = 0;  
    // _measurement->computeDerivativePos(bandpt->pose(), layer_,  &dx, &dy);
    // std::cout << "Sobel_y : L=" << layer_ << ", v=" << dx  << std::endl;
    // std::cout << "--------------------------------------------------------" << std::endl; 
    // // *************************************** Debug Numeric Gradient ***************************************

    // minus (-) here?????? Why?
    _error[0] = interpolation * cfg_->optim.weight_predicted_costmap;

    
    // if (layer_ > 10 && layer_ < 13)
    // { 
    //   std::cout << "      L=" << layer_ << ", w=" << cfg_->optim.weight_predicted_costmap << ", v=" << _error[0]  << std::endl;
    // }


    // // *************************************** Debug interpolation  ***************************************
    // // Instead of showing every time
    // std::cout << "xyLe = " << bandpt->pose().x() << " " << bandpt->pose().y() << " " << layer_ << " " << _error[0] << std::endl;

    // // save in an image that we save afterwards (once every 5 seconds for example)
    // // Don't save a pixel taht already exists
    // // Create a local variable in the edge for it

    // double dl = 0.03;
    // double dt = 0.99;
    // size_t deb_Nx = 200;
    // size_t deb_Ny = 200;
    // size_t deb_Nt = 25;

    // double deb_x0 = bandpt->pose().x() - dl * (double)deb_Nx / 2;
    // double deb_y0 = bandpt->pose().y() - dl * (double)deb_Ny / 2;
    // double deb_t0 = 1.9;

    // // Save point cloud as text file
    // std::ofstream outfile;
    // std::string path = "/home/hth/Myhal_Simulation/simulated_runs/";
		// char buffer[200];
		// sprintf(buffer, "my_test_%.3f_%.3f_%.3f.txt", bandpt->pose().x(), bandpt->pose().y(), layer_);
    // std::string filepath = path + std::string(buffer);
    // outfile.open(filepath, std::ios_base::app);//std::ios_base::app

    // for (size_t iX = 0; iX < deb_Nx; iX++)
    // {
    //   for (size_t iY = 0; iY < deb_Ny; iY++)
    //   {
    //     for (size_t iT = 0; iT < deb_Nt; iT++)
    //     {
    //       interpolation = 0.0;
    //       VertexPose deb_pos(deb_x0 + (double)iX * dl, deb_y0 + (double)iY * dl, 0);
    //       double deb_layer = deb_t0 + dt * (double)iT;
    //       _measurement->interpolateCostmapValue(deb_pos, &interpolation, deb_layer); 

    //       outfile << deb_pos.x() << " " << deb_pos.y() << " " << deb_layer << " " << interpolation << "\n";


    //     }
    //   }
    // }

    // outfile.close();

    // throw std::invalid_argument("OLALA");
    // // *************************************** Debug interpolation  ***************************************


    // double dx, dy, dz;
    // self->getGradient(dx, dy, dz);
    // std::cout << "cost obst = " << cur_cost << " / Grad = " << dx << " , " << dy << " , " << dz << std::endl;

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n",_error[0]);
  }


#ifdef USE_ANALYTIC_JACOBI
#if 0


  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    double dx = 0, dy = 0;  
    _measurement->computeDerivativePos(bandpt->pose(), layer_,  &dx, &dy);

    //TODO: evaluate relevance of optim weight predicted costmap weight here
    _jacobianOplusXi(0, 0) = - dy * cfg_->optim.weight_predicted_costmap; 
    _jacobianOplusXi(0, 1) = - dx * cfg_->optim.weight_predicted_costmap;

  }

#endif
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
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, PredictedCostmap3D* predictions3D, double layer)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = predictions3D;
    layer_ = layer;
  }

  void getGradient(double& dx, double& dy, double& dz)
  {

    // auto test = _jacobianOplusXi;
    linearizeOplus();

    dx = _jacobianOplusXi(0, 0);
    dy = _jacobianOplusXi(0, 1);
    dz = _jacobianOplusXi(0, 2);

    return;
  }


protected:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  
public: 	
  double layer_; //!< Store to the 3D costmap layer, continuous value for interpolation
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace
#endif
