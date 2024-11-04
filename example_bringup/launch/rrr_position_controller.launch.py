from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro

def generate_launch_description():
  package_path = os.path.join( get_package_share_directory('example_description') )
  rviz_config_path = os.path.join( package_path , 'rviz', 'urdf.rviz' )
  # Ruta al archivo URDF y a la ruta del archivo de mundo
  model_path = os.path.join( package_path , 'urdf', 'rrr_full_robot_stl_position_controller.urdf')
  world_path = os.path.join( package_path , 'worlds', 'new_world.world')
  #Gazebo
  gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
    output='screen'
  )

  #Modelo con controladores
  xacro_file =  model_path
  # Compilar el archivo xacro
  doc = xacro.parse(open(xacro_file))
  xacro.process_doc(doc)    
  params = {'robot_description': doc.toxml()}   
  node_robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[params]
  )

  spawn_rrr = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'rrr',
        '-topic', 'robot_description'
    ],
    output='screen'
  )

  load_joint_state_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
      'joint_state_broadcaster'],
    output='screen'
  )

  rrr_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
      'rrr_position_controller'],
    output='screen'
  )
  spawn_event_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=spawn_rrr,
      on_exit=[load_joint_state_controller],
    )
  )
  controller_event_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=load_joint_state_controller,
      on_exit=[rrr_controller],
    )
  )
  #Ejecutar Rviz
  config_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = rviz_config_path)
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')]
  )

  return LaunchDescription([
    spawn_event_handler,
    controller_event_handler,
    gazebo,
    node_robot_state_publisher,
    spawn_rrr,
    config_arg,
    rviz_node    
  ])