#include <memory>
#include <iostream>
#include <typeinfo>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <vector>

int main(int argc, char *argv[])
{

  // ROS 초기화 및 노드 생성
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_coordinate", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto& LOGGER = node->get_logger();

  // //로봇의 상태 정보 획득을 위한 SingleThreadedExecutor 가동
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string END_EFFECTER = "panda_hand";
  static const std::string END_EFFECTER_LINK = "panda_link8";

  // ROS logger 생성 (ROS관련 에러 메시지 출력하는데 쓰임)
  auto const logger = rclcpp::get_logger("move_coordinate");

  // MoveIt MoveGroup Interface 생성
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
  
  // MoveIt MotionPlanningDisplay 생성
  //using moveit_rviz_plugin::MotionPlanningDisplay;
  //auto motion_planning_display = MotionPlanningDisplay();

  // 로봇의 현재 상태 참조하는 포인터 생성 (각,속도,돌림힘의 현재 상태 출력하는데 필요)
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 시뮬레이션 속도 조절
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.4);
  while (true)
  {
    char c;
    
    
    
    std::cout << "\n위치 초기화: R / 좌표 입력: I / Joint 입력: A / 현재 Pose 확인: P / 현재 RPY 확인: Y / 현재 End Effector link 확인: L / 프로그램 종료: Q" << std::endl;
    
    std::cin.clear();
    std::cin >> c;

    if (c == 'r' || c == 'R')
    {

      joint_group_positions[0] = 0.0;
      joint_group_positions[1] = -0.8;
      joint_group_positions[2] = 0.0;
      joint_group_positions[3] = -2.4;
      joint_group_positions[4] = 0.0;
      joint_group_positions[5] = 1.6;
      joint_group_positions[6] = 0.8;
      move_group_interface.setJointValueTarget(joint_group_positions);
      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        move_group_interface.execute(plan);
        current_state = move_group_interface.getCurrentState(10);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        for (int i = 0; i < 7; i++)
        {
          //해당 좌표에 대한 관절 각도 출력
          printf("joint[%d] : %0.1lf\n", i, joint_group_positions[i]);
        }
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }

    else if (c == 'i' || c == 'I')
    {
      move_group_interface.stop();
      // 목표 좌표 세팅
      auto const target_pose = []
      {
        geometry_msgs::msg::Pose msg;       
        msg.orientation.w = 0;
        msg.orientation.x = 0.9;
        msg.orientation.y = -0.4;
        msg.orientation.z = 0.0;
        std::cout << "좌표 입력 :";
        std::cin.clear();
        std::cin >> msg.position.x >> msg.position.y >> msg.position.z;
        return msg;
      }();
      // 목표 좌표에 대한 경로 생성
      move_group_interface.setPoseTarget(target_pose); 
      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        ////execute전에 plan에 있는 orientation 출력
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << plan.start_state_.multi_dof_joint_state.transforms[0].rotation.x << "\n");
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << plan.start_state_.multi_dof_joint_state.transforms[0].rotation.y << "\n");
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << plan.start_state_.multi_dof_joint_state.transforms[0].rotation.z << "\n");
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << plan.start_state_.multi_dof_joint_state.transforms[0].rotation.w << "\n");
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << sizeof(plan.start_state_.multi_dof_joint_state.transforms) << "\n");
        // RCLCPP_INFO_STREAM(LOGGER, "plan: " << sizeof(plan.start_state_.multi_dof_joint_state.transforms[0]) << "\n");
        
        move_group_interface.execute(plan);
        current_state = move_group_interface.getCurrentState(10);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        for (int i = 0; i < 7; i++)
        {  
          //해당 좌표에 대한 관절 각도 출력
          printf("joint[%d] : %0.1lf\n", i, joint_group_positions[i]);
        }
        //현재 Pose의 Position, Orientation을 
        geometry_msgs::msg::PoseStamped p = move_group_interface.getPoseTarget(END_EFFECTER_LINK);
        printf("%0.1lf\n",p.pose.position.x);
        printf("%0.1lf\n",p.pose.position.y);
        printf("%0.1lf\n",p.pose.position.z);
        printf("%0.1lf\n",p.pose.orientation.x);
        printf("%0.1lf\n",p.pose.orientation.y);
        printf("%0.1lf\n",p.pose.orientation.z);
        printf("%0.1lf\n",p.pose.orientation.w);


        const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("panda_link8");
        RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
        RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");
        
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }

    else if (c == 'a' || c == 'A')
    {
      move_group_interface.stop();
      // 목표 좌표 세팅
        std::cout << "joint값 입력(7개) :";
        std::cin.clear();
        std::cin >> joint_group_positions[0] >> joint_group_positions[1] >> joint_group_positions[2] >> joint_group_positions[3] >> joint_group_positions[4] 
                 >> joint_group_positions[5] >> joint_group_positions[6];
      // 목표 좌표에 대한 경로 생성
      move_group_interface.setJointValueTarget(joint_group_positions);
      auto const [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      // 경로 실행
      if (success)
      {
        
        move_group_interface.execute(plan);

          geometry_msgs::msg::PoseStamped p = move_group_interface.getCurrentPose(END_EFFECTER_LINK);
          printf("%0.1lf\n",p.pose.position.x);
          printf("%0.1lf\n",p.pose.position.y);
          printf("%0.1lf\n",p.pose.position.z);
      }
      else
      {
        RCLCPP_ERROR(logger, "Planning failed!");
      }
    }

    else if (c == 'p' || c == 'P') // 현재 Pose 확인
    {
      geometry_msgs::msg::PoseStamped p = move_group_interface.getCurrentPose(END_EFFECTER_LINK);
      printf("position x: %0.1lf\n",p.pose.position.x);
      printf("position y: %0.1lf\n",p.pose.position.y);
      printf("position z: %0.1lf\n",p.pose.position.z);
      printf("orientation x: %0.1lf\n",p.pose.orientation.x);
      printf("orientation y: %0.1lf\n",p.pose.orientation.y);
      printf("orientation z: %0.1lf\n",p.pose.orientation.z);
      printf("orientation w: %0.1lf\n",p.pose.orientation.w);
    }

    else if (c == 'y' || c == 'Y') // 현재 Roll,Pitch,Yaw 값을 확인
    {
      std::vector<double> c_RPY = move_group_interface.getCurrentRPY(END_EFFECTER_LINK);
      for(int i = 0; i<(int)(c_RPY.size());i++)
      {
        std::cout<< c_RPY[i] << std::endl;
      }
      
    }

    else if (c == 'l' || c == 'L') // 현재 End Effector link가 어디 파트인지 확인
    {
      const std::string end = move_group_interface.getEndEffectorLink();
      std::cout << end << std::endl;
    }

    // else if (c == 't' || c == 'T')
    // {
    //   const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("panda_link8");
    //   RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
    //   RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");
    // }

    else if (c == 'q' || c == 'Q')
    {
      break; // q
    }
    
    
    
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
