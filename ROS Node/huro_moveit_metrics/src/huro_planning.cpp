#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>


#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <map>
#include <vector>
#include <string>
#include <boost/function.hpp>
#include <string.h>
#include <cmath>
#include <boost/bind.hpp>


typedef std::map<std::string, std::string> PlannerRunData;
typedef std::vector<PlannerRunData> PlannerBenchmarkData;

moveit_msgs::MotionPlanResponse response;
static const std::string PLANNING_GROUP = "manipulator";

int i=0;
int iteration=0;
double lastCaptureTime=0;

std::string to_string(double x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}
void logDuration(int iteration,char *msg){
    try {
      double secs =ros::Time::now().toSec();
      double duration=0;
      if(lastCaptureTime!=0){
          duration=secs-lastCaptureTime;
      }

      ROS_DEBUG_NAMED("huro_pcl", "{iteration:  { id: %d, time_seg: %f, duration_seg: %f, message: '%s'}}", 
                      iteration,secs,duration, msg);
      lastCaptureTime=secs;
    } catch (const std::exception &e) {
        ROS_ERROR("Error in logDuration: %s", e.what());
    }

}


void log(int iteration,char *metric,  std::string value){
    double secs =ros::Time::now().toSec();

    
    double duration=0;
    if(lastCaptureTime!=0){
        duration=secs-lastCaptureTime;
    }

    ROS_DEBUG_NAMED("huro_planning", "{iteration:  { id: %d, time_seg: %f, duration_seg: %f, metric: '%s', value: '%s'}}", 
                    iteration,secs,duration, metric, value);
    lastCaptureTime=secs;
}



void collectMetrics(PlannerRunData& metrics, 
  const planning_interface::MotionPlanDetailedResponse& mp_res, 
  bool solved, 
  double total_time,
  const planning_scene::PlanningSceneConstPtr& planning_scene
  )
{
  metrics["time REAL"] = std::to_string(total_time);//  moveit::core::toString(total_time);
  metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

  if (solved)
  {
    iteration++;
    logDuration(iteration,"Init Solved");
    // Analyzing the trajectory(ies) geometrically
    double traj_len = 0.0;    // trajectory length
    long double clearance = 0.0;   // trajectory clearance (average)
    double smoothness = 0.0;  // trajectory smoothness (average)
    bool correct = true;      // entire trajectory collision free and in bounds

    double process_time = total_time;
    for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
    {
      correct = true;
      traj_len = 0.0;
      clearance = 0.0;
      smoothness = 0.0;
      const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];
      // compute path length
      for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
        traj_len += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

      // compute correctness and clearance
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
      {
        collision_detection::CollisionResult res;
        planning_scene->checkCollisionUnpadded(req, res, p.getWayPoint(k));

        if (res.collision)
          correct = false;
        if (!p.getWayPoint(k).satisfiesBounds())
          correct = false;
        double d =planning_scene->distanceToCollisionUnpadded(p.getWayPoint(k));
        

        if (d > 0.0)  // in case of collision, distance is negative
          clearance += d;
        
      }

      
      clearance /= (double)p.getWayPointCount();

      // compute smoothness
      if (p.getWayPointCount() > 2)
      {
        double a = p.getWayPoint(0).distance(p.getWayPoint(1));
        for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
        {
          // view the path as a sequence of segments, and look at the triangles it forms:
          //          s1
          //          /\          s4
          //      a  /  \ b       |
          //        /    \        |
          //       /......\_______|
          //     s0    c   s2     s3
          //

          // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
          double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
          double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
          double acos_value = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
          if (acos_value > -1.0 && acos_value < 1.0)
          {
            // the smoothness is actually the outside angle of the one we compute
            double angle = (M_PI- acos(acos_value));

            // and we normalize by the length of the segments
            double u = 2.0 * angle;  /// (a + b);
            smoothness += u * u;
          }
          a = b;
          //std::cout<<"k:" <<k<<" smoothness "<<smoothness<<std::endl ;  
        }
        smoothness /= (double)p.getWayPointCount();
        
      }
      metrics["path_" + mp_res.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
      metrics["path_" + mp_res.description_[j] + "_length REAL"] =std::to_string(traj_len);
      metrics["path_" + mp_res.description_[j] + "_clearance REAL"] = std::to_string(clearance);
      metrics["path_" + mp_res.description_[j] + "_smoothness REAL"] = std::to_string(smoothness);
      metrics["path_" + mp_res.description_[j] + "_time REAL"] = std::to_string(mp_res.processing_time_[j]);

      if (j == mp_res.trajectory_.size() - 1)
      {
        metrics["final_path_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
        metrics["final_path_length REAL"] = std::to_string(traj_len);
        metrics["final_path_clearance REAL"] = std::to_string(clearance);
        metrics["final_path_smoothness REAL"] = std::to_string(smoothness);
        metrics["final_path_time REAL"] = std::to_string(mp_res.processing_time_[j]);
      }
      process_time -= mp_res.processing_time_[j];
 
      std::string str = to_string(smoothness);

      
    }
    if (process_time <= 0.0)
      process_time = 0.0;
    metrics["process_time REAL"] = std::to_string(process_time);
    logDuration(iteration,"Fin Metric Calc");
  }

  
}


void huroMetricCallback(ros::NodeHandle &node_handle,  
                      moveit::planning_interface::MoveGroupInterface &move_group, 
                      robot_model::RobotModelPtr &robot_model,
                      const geometry_msgs::PoseStamped::ConstPtr&  msg)
{
    if (!robot_model) {
        ROS_ERROR("Null pointer detected, cannot proceed with planning or execution.");
        return;
    }
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_pipeline::PlanningPipelinePtr planning_pipeline( new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    // A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.1);  
    std::vector<double> tolerance_angle(3, 0.1); 
    const robot_state::JointModelGroup* joint_model_group =  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState start_state(*move_group.getCurrentState());
    if (i==1)
    {
        planning_scene->setCurrentState(response.trajectory_start);
        robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    }
    move_group.setStartState(start_state);

    req.group_name = "manipulator";
    req.planner_id = "RRT"; //SPARS,RRT
    ros::WallTime start = ros::WallTime::now();

    moveit_msgs::Constraints pose_goal =  kinematic_constraints::constructGoalConstraints("tool0", *msg, tolerance_pose, tolerance_angle);
    
    req.goal_constraints.clear();
    robot_state = planning_scene->getCurrentStateNonConst();
    
    req.goal_constraints.push_back(pose_goal);
    std::vector<planning_interface::MotionPlanDetailedResponse> responses(1);
    std::vector<bool> solved(1);


    solved[0]=planning_pipeline->generatePlan(planning_scene, req, res);
    responses[0].error_code_ = res.error_code_;
    if (res.trajectory_)
    {
        responses[0].description_.push_back("plan");
        responses[0].trajectory_.push_back(res.trajectory_);
        responses[0].processing_time_.push_back(res.planning_time_);
    }
    double total_time = (ros::WallTime::now() - start).toSec();

    PlannerBenchmarkData planner_data(1);
    collectMetrics(planner_data[0], responses[0], solved[0], total_time, planning_scene);
    for(auto it = planner_data[0].cbegin(); it != planner_data[0].cend(); ++it)
    {
        std::cout << it->first << " " << it->second << "\n";
    }
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
    }
    
    res.getMessage(response);

  
    if (res.error_code_.val == res.error_code_.SUCCESS)
    {
        i=1;
        try {
            move_group.execute(response.trajectory);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to execute trajectory: %s", e.what());
        }
    }
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "huro_calculate_metrics");
  ros::NodeHandle n("");

   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
     
    move_group.setGoalPositionTolerance(0.5);
    move_group.setGoalOrientationTolerance(0.5);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
     

  ros::AsyncSpinner spinner(4);

  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("huro_move_robot_metric", 1000, 
            boost::bind(&huroMetricCallback, 
              boost::ref(n),
              boost::ref(move_group),
              boost::ref(robot_model),
             _1) ); 

  spinner.start();  
  ros::waitForShutdown(); 
  return 0;
}
