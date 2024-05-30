#!/usr/bin/env python3
#Autor: Pedro Palomeque Ortega

import rclpy
import numpy as np
import time
import math
import csv
from action_msgs import msg
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose,PoseStamped, Point, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParameters,GetParameters, SetParametersAtomically, DescribeParameters, ListParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter, Log
from std_msgs.msg import Empty
from tf_transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped

from tabulate import tabulate
from colorama import Fore, Style        
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class GoalPoseNode(Node):
    def __init__(self, z, i, j, n, obs, pose):
        super().__init__(f'goal_pose_{n+1}_{j+1}_{i+1}_{z}_{obs}')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the navigate_to_pose action server...')
        
        self.i = i
        self.start_time = 0
        self.end_time = 0
        self.initial_pose = None
        self.goal_msg = pose_to_goal_msg(pose)

        # Variables para la distancia real recorrida
        self.real_distance_traveled = 0.0
        self.previous_position = None

        self.get_path()
        self.start_time = time.time()
        self.send_goal_future = self.nav_client.send_goal_async(self.goal_msg)
        rclpy.spin_until_future_complete(self, self.send_goal_future)

        if self.send_goal_future.result() is not None:
            self.result = self.send_goal_future.result()
            if self.result.accepted:
                self.get_logger().info(f'{Fore.BLUE}{i+1}º GOAL ACCEPTED{Style.RESET_ALL}')
                self.get_logger().info(
                    "Position goal: %s Orientation goal: %s" %
                    (str(self.goal_msg.pose.pose.position), str(self.goal_msg.pose.pose.orientation))
                )
                self.start_time = time.time()
                self.wait_for_action()
            else:
                print(f'{msg.GoalStatus = }')
                self.get_logger().error('Goal not accepted by server')
                self.get_logger().error(
                    "Position goal: %s Orientation goal: %s" %
                    (str(self.goal_msg.pose.pose.position), str(self.goal_msg.pose.pose.orientation))
                )
        else:
            self.get_logger().error('Action server not available!')

    def wait_for_action(self):
        self.get_logger().info('Waiting for the action to complete...')
        self.result_future = self.result.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        if self.goal_path is not None:
            self.initial_pose = self.goal_path.poses[0].pose
        
        if self.result_future.result().status == msg.GoalStatus.STATUS_SUCCEEDED:
            self.end_time = time.time()
            self.elapsed_time = self.end_time - self.start_time
            self.get_logger().info(f'{Fore.GREEN}{self.i+1}º GOAL SUCCEEDED!{Style.RESET_ALL}')
            self.get_logger().info(f'Time taken: {self.elapsed_time} seconds')
            self.exito = True
            self.get_logger().info(f'Initial pose = {self.initial_pose.position} || Final pose = {self.goal_msg.pose.pose.position}')
            self.teorical_distance_traveled = 0
            for a in range(len(self.goal_path.poses) - 1):
                self.teorical_distance_traveled += distance_2p(self.goal_path.poses[a].pose.position.x, self.goal_path.poses[a].pose.position.y, 
                                                      self.goal_path.poses[a+1].pose.position.x, self.goal_path.poses[a+1].pose.position.y)
            self.get_logger().info(f'Distance traveled: {self.teorical_distance_traveled} m')
            self.get_logger().info(f'Real distance traveled: {self.real_distance_traveled} m')
        else:
            self.get_logger().error('Goal failed')
            self.exito = False
            self.end_time = time.time()
            self.elapsed_time = None
            self.teorical_distance_traveled = None
            self.real_distance_traveled = None
            if self.goal_path is not None:
                self.get_logger().error('The path to the target pose was found, but navigation failed.')
                self.get_logger().error(f'Initial pose = {self.initial_pose.position} || Final pose = {self.goal_msg.pose.pose.position}')
            else:
                self.get_logger().error('NO path to the target pose was found')


            if self.result_future.result().status == msg.GoalStatus.STATUS_CANCELED:
                self.get_logger().error('The action server successfully canceled the goal.')
            elif self.result_future.result().status == msg.GoalStatus.STATUS_ABORTED:
                self.get_logger().error('The action server failed reached the goal.')
    
        self.destroy_subscription(self.odom_sub)

    def get_path(self):
        self.goal_path = None
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def path_callback(self, msg):
        self.goal_path = msg
        self.destroy_subscription(self.path_sub)

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        if self.previous_position is not None:
            self.real_distance_traveled += distance_2p(self.previous_position.x, self.previous_position.y, 
                                                       current_position.x, current_position.y)
        self.previous_position = current_position

class MapBoundariesNode(Node):
    def __init__(self):
        super().__init__('map_boundaries_node')
        self.last_map = None
        
        self.origin_x = None
        self.origin_y = None

        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile,
        )
        self.get_logger().info("Subscribed to /map topic, waiting for map...")

    def map_callback(self, msg):
        self.last_map = msg
        self.print_map_boundaries()

    def print_map_boundaries(self):
        if self.last_map is not None:
            # Calcula los límites del mapa
            width = self.last_map.info.width
            height = self.last_map.info.height
            resolution = self.last_map.info.resolution

            # Coordenadas del origen del mapa (esquina inferior izquierda)
            self.origin_x = self.last_map.info.origin.position.x
            self.origin_y = self.last_map.info.origin.position.y

            # Calcula las coordenadas de las esquinas del mapa
            upper_left = (self.origin_x, self.origin_y + height * resolution)
            upper_right = (self.origin_x + width * resolution, self.origin_y + height * resolution)
            lower_left = (self.origin_x, self.origin_y)
            lower_right = (self.origin_x + width * resolution, self.origin_y)

            self.get_logger().info(
                f'Map Boundaries:\n'
                f'Upper Left: {upper_left}\n'
                f'Upper Right: {upper_right}\n'
                f'Lower Left: {lower_left}\n'
                f'Lower Right: {lower_right}'
            )
            self.destroy_subscription(self.map_subscription)

        else:
            self.get_logger().info("No map data received yet.")
        
class ReactiveNavParamsNode(Node):
    def __init__(self, v, w, j):
        super().__init__(f'reactive_nav_params_node_{j+1}')

        #--------------------------------------
        # Crear un cliente de servicio para "get_parameters".
        self.robot_radius=None
        self.get_params_local_costmap_client = self.create_client(GetParameters, '/local_costmap/local_costmap/get_parameters')

        # Esperar hasta que el cliente de servicio esté conectado al servidor de servicio.
        while not self.get_params_local_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the get_parameters service...')

        # Crear un mensaje de solicitud para obtener el valor de robot_radius.
        self.request = GetParameters.Request()
        self.request.names = ['robot_radius']

        # Llamar al servicio para obtener el valor de robot_radius.
        future = self.get_params_local_costmap_client.call_async(self.request)

        # Esperar a que se complete la llamada al servicio.
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            params = future.result().values
            for param in params:
                robot_radius = param.double_value
                self.get_logger().info(f'Robot radius: {robot_radius}')
                break
            else:
                self.get_logger().warn("robot_radius parameter not found in response.")
        else:
            self.get_logger().error("Failed to get parameters.")

        #--------------------------------------

        # Crear un cliente de servicio para "set_parameters".
        self.set_params_local_costmap_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.set_params_global_costmap_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')

        # Esperar hasta que el cliente de servicio esté conectado al servidor de servicio.
        while not self.set_params_local_costmap_client.wait_for_service(timeout_sec=1.0) and not self.set_params_global_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_parameters service...')

        # Crear un mensaje de solicitud con radio del robot.
        self.request = SetParameters.Request()
        self.request.parameters = [
            Parameter(name='inflation_layer.inflation_radius', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=robot_radius+0.05))
        ]

        # Enviar la solicitud de manera asincrónica.
        self.future = self.set_params_local_costmap_client.call_async(self.request)
        self.future.add_done_callback(self.callback)
        self.future = self.set_params_global_costmap_client.call_async(self.request)
        self.future.add_done_callback(self.callback)
        #--------------------------------------     
        #VELOCITY SMOOTHER: https://navigation.ros.org/configuration/packages/configuring-velocity-smoother.html

        # Crear un cliente de servicio para "set_parameters".
        self.set_params_vel_smoother_client = self.create_client(SetParameters, '/velocity_smoother/set_parameters')

        # Esperar hasta que el cliente de servicio esté conectado al servidor de servicio.
        while not self.set_params_vel_smoother_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_parameters service...')

        # Crear un mensaje de solicitud con los parámetros de velocidad lineal y angular.
        self.request = SetParameters.Request()
        self.request.parameters = [
            #Velocidades máximas y minimas(m/s) en cada eje [x, y, theta]. 
            Parameter(name='max_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[v, 0.0, w])),            
            Parameter(name='min_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[-v, 0.0, -w])),

            #Aceleración máxima y minima a aplicar a cada eje [x, y, theta]
            Parameter(name='max_accel', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[2.5, 0.0, 3.2])),
            Parameter(name='max_decel', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[-2.5, 0.0, -3.2]))
        ]

        # Enviar la solicitud de manera asincrónica.
        self.future = self.set_params_vel_smoother_client.call_async(self.request)
        self.future.add_done_callback(self.callback)
        
        #CONTROLLER SERVER: https://navigation.ros.org/configuration/packages/dwb-params/kinematic.html
        
        # Crear un cliente de servicio para "controller_server/set_parameters".
        self.set_params_controller_client = self.create_client(SetParameters, '/controller_server/set_parameters')

        # Esperar hasta que el cliente de servicio esté conectado al servidor de servicio.
        while not self.set_params_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_parameters service...')

        # Crear un mensaje de solicitud con los parámetros de velocidad lineal y angular.
        self.request = SetParameters.Request()
        self.request.parameters = [
            Parameter(name='FollowPath.max_vel_x', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)),
            Parameter(name='FollowPath.max_speed_xy', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)),
            Parameter(name='FollowPath.max_vel_theta', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=w)),
            Parameter(name='FollowPath.acc_lim_x', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=5.0))
        ]

        # Enviar la solicitud de manera asincrónica.
        self.future = self.set_params_controller_client.call_async(self.request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        self.get_logger().info(f'{response = }')        
        #Comprobar parametros con $ ros2 param get /controller_server FollowPath.max_vel_theta o en rqt
    
            
    def speed_limit_callback (self,msg):
        self.last_speed_limit = msg
        print(f'{self.last_speed_limit = }')

class ChangingGlobalNavigationTechniques(Node):
    def __init__(self, n,nav_technique):
        super().__init__(f'Change_Nav_Technique{n+1}')
        
        #------------------PLANNER SERVER--------------------
        #https://navigation.ros.org/configuration/packages/configuring-navfn.html
        #https://navigation.ros.org/configuration/packages/configuring-planner-server.html   
        
        # Crear un cliente de servicio para "planner_server/set_parameters".
        self.set_params_planner_client = self.create_client(SetParameters, '/planner_server/set_parameters')
        
        # Esperar hasta que el cliente de servicio esté conectado al servidor de servicio.
        while not self.set_params_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_parameters service...')

        # Crear un mensaje de solicitud con la técnica de navegación.
        self.request = SetParameters.Request()
        self.request.parameters = [
                Parameter(name='GridBased.use_astar', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=nav_technique)),
            ]

        # Enviar la solicitud de manera asincrónica.
        self.future = self.set_params_planner_client.call_async(self.request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        self.get_logger().info(f'{response = }')        
        #Comprobar parametros con $ ros2 param get /controller_server FollowPath.max_vel_theta o en rqt

class DataHolder:
    def __init__(self, len_nav_technique, len_config, len_poses, robot_name):
        #Variables para medias
        self.exito_medio_array = np.array([])
        self.tiempo_medio_array = np.array([])
        self.distancia_teorica_media_array = np.array([])
        self.distancia_real_media_array = np.array([])

        self.v_array = np.array([])
        self.w_array = np.array([])
        self.nav_technique_array=np.array([])

        # Variables para datos individuales por pose
        self.exitos={num_global_technique: {num_config: {num_action: [] for num_action in range(len_poses)} for num_config in range(len_config)} for num_global_technique in range(len_nav_technique)}
        self.tiempos = {num_global_technique: {num_config: {num_action: [] for num_action in range(len_poses)} for num_config in range(len_config)} for num_global_technique in range(len_nav_technique)}
        self.distancias_teorica = {num_global_technique: {num_config: {num_action: [] for num_action in range(len_poses)} for num_config in range(len_config)} for num_global_technique in range(len_nav_technique)}
        self.distancias_real = {num_global_technique: {num_config: {num_action: [] for num_action in range(len_poses)} for num_config in range(len_config)} for num_global_technique in range(len_nav_technique)}
        self.len_nav_technique = len_nav_technique
        self.len_config = len_config
        self.len_poses = len_poses
        self.entity_name = robot_name

    def CalculateAveragesValues(self, exito, tiempo, distancia_teorica, distancia_real, v, w, nav_technique, i, j, n):
        if i==0 and j==0:
            self.nav_technique_array=np.append(self.nav_technique_array, nav_technique)
            
        if i==0:
            #Variables para medias
            self.exito_array = np.array([])
            self.tiempo_array = np.array([])
            self.distancia_teorica_array = np.array([])
            self.distancia_real_array = np.array([])
            self.w_array=np.append(self.w_array, w)
            self.v_array=np.append(self.v_array, v)

        self.tiempos[n][j][i] = tiempo
        self.distancias_teorica[n][j][i] = distancia_teorica
        self.distancias_real[n][j][i] = distancia_real
        self.exitos[n][j][i] = exito
        self.exito_array=np.append(self.exito_array,exito)

        if exito:    
            self.tiempo_array=np.append(self.tiempo_array,tiempo)
            self.distancia_teorica_array=np.append(self.distancia_teorica_array,distancia_teorica)
            self.distancia_real_array=np.append(self.distancia_real_array,distancia_real)        

        if i==(self.len_poses-1):
            self.exito_medio_array=np.append(self.exito_medio_array,float(np.mean(self.exito_array)))
            self.tiempo_medio_array=np.append(self.tiempo_medio_array,float(np.mean(self.tiempo_array)))
            self.distancia_teorica_media_array=np.append(self.distancia_teorica_media_array,float(np.mean(self.distancia_teorica_array)))
            self.distancia_real_media_array=np.append(self.distancia_real_media_array,float(np.mean(self.distancia_real_array)))

        if (i==(self.len_poses-1) and j==(self.len_config-1)) and n==(self.len_nav_technique-1):
            
            self.exito_medio_array*=100
            self.total_navigations=self.len_poses*self.len_config*self.len_nav_technique
            print('FINISH SIMULATION \n RESULTS:\n')
            self.PrintTable()
            self.SaveToCSV("/home/pedro/TFG/results"+self.entity_name+".csv")



    def PrintTable (self):
            
        headers = [f'{self.total_navigations} navegations',
                f'use_astar= {bool(self.nav_technique_array[0])} || v = {self.v_array[0]} || w = {self.w_array[0]}',
                f'use_astar= {bool(self.nav_technique_array[0])} || v = {self.v_array[1]} || w = {self.w_array[1]}',
                f'use_astar= {bool(self.nav_technique_array[1])} || v = {self.v_array[0]} || w = {self.w_array[0]}',
                f'use_astar= {bool(self.nav_technique_array[1])} || v = {self.v_array[1]} || w = {self.w_array[1]}',
                
        ]
        
        data = [
            ["Éxito [%]", self.exito_medio_array[0], self.exito_medio_array[1], self.exito_medio_array[2], self.exito_medio_array[3]],
            ["Tiempo [s]", self.tiempo_medio_array[0], self.tiempo_medio_array[1], self.tiempo_medio_array[2], self.tiempo_medio_array[3]],
            ["Distancia teorica [m]", self.distancia_teorica_media_array[0], self.distancia_teorica_media_array[1], self.distancia_teorica_media_array[2], self.distancia_teorica_media_array[3]],
            ["Distancia real [m]", self.distancia_real_media_array[0], self.distancia_real_media_array[1], self.distancia_real_media_array[2], self.distancia_real_media_array[3]]

        ]     

        # Imprimir la tabla en formato Markdown
        print(tabulate(data, headers=headers, tablefmt="github", numalign="center"))

    def SaveToCSV(self, filename):
        # Collect data into rows
        rows = []
        for n in self.exitos:
            for j in self.exitos[n]:
                for i in self.exitos[n][j]:
                    exito = self.exitos[n][j][i]
                    tiempo = self.tiempos[n][j].get(i, "")
                    distancia_teorica = self.distancias_teorica[n][j].get(i, "")
                    distancia_real = self.distancias_real[n][j].get(i, "")

                    rows.append([n, j, i, exito, tiempo, distancia_teorica, distancia_real])

        # Write data to a CSV file
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file, delimiter=";")
            # Write the header
            writer.writerow(["n", "j", "i", "exito", "tiempo empleado","distancia recorrida teorica", "distancia recorrida real"])
            # Write the data rows
            writer.writerows(rows)

class RobotTeleporterNode(Node):
    def __init__(self, i, obs, entity_name, initial_pose, v, w, x_max, y_max):
        super().__init__(f'robot_teleporter_node{i+1}_{obs}')
        self.i=i
        self.entity_name = entity_name
        self.v=v
        self.w=w
        self.client = self.create_client(SetEntityState, '/plug/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /set_entity_state no disponible, esperando...')

        self.x_max=x_max
        self.y_max=y_max
        self.behavior_messages_received = False
        self.new_pose=None
        self.attemps=0
        self.teleport_robot(initial_pose)

    def teleport_robot(self,initial_pose):
        self.initial_pose=initial_pose
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.entity_name
        request.state.pose.position.x = self.initial_pose[0]
        request.state.pose.position.y = self.initial_pose[1]
        request.state.pose.position.z = self.initial_pose[2]
        request.state.pose.orientation.x = self.initial_pose[3]
        request.state.pose.orientation.y = self.initial_pose[4]
        request.state.pose.orientation.z = self.initial_pose[5]
        request.state.pose.orientation.w = self.initial_pose[6]

        # Velocidades lineales y angulares
        """request.state.twist.linear.x=self.v
        request.state.twist.angular.z=self.w"""

        request.state.reference_frame = 'world'  # Marco de referencia mundial
        self.get_logger().info(f'New pose: {request}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,future)

        if future.done():
            try:
                if future.result().success:
                    self.get_logger().info(f'Estado del robot {self.entity_name} actualizado con éxito')
                else:
                    self.get_logger().info(f'Error al actualizar el estado del robot {self.entity_name}.')
            except Exception as e:
                self.get_logger().info(f'La llamada al servicio falló: {e}')

        # Crea un publicador para el tema '/initialpose' que publica mensajes de tipo 'PoseWithCovarianceStamped'.
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        self.init_msg = pose_to_initial_pose(self.initial_pose)
        self.get_logger().info('Finish teleport')
        self.publish_initial_pose(self.init_msg)
        self.get_logger().info('ACTUAL INITIAL POSE IS IN THE MAP')
        self.initial_pose_in_map=self.initial_pose

    # Define una función para publicar la pose inicial.
    def publish_initial_pose(self, init_msg):
        self.get_logger().info('Setting initial pose')
        self.time_to_sleep=2.0
        self.publisher_.publish(init_msg)
        time.sleep(self.time_to_sleep)
        self.publisher_.publish(init_msg)   
        time.sleep(self.time_to_sleep)
        self.publisher_.publish(init_msg)    
        time.sleep(1)
        self.get_logger().info('Initial pose is set')
        self.check_pose_within_map_bounds()
    
    def check_pose_within_map_bounds(self):
        self.odom_pose=np.array([])
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.start_time=time.time()

        while len(self.odom_pose) == 0:
            rclpy.spin_once(self,timeout_sec=0.001)

        self.destroy_subscription(self.odom_sub)

        #if self.odom_pose[0].pose.pose.position.z < 0:
        if self.odom_pose[0].pose.pose.position.z < -0.5 or self.odom_pose[0].pose.pose.position.z > 0.3 :
            self.get_logger().warn(f'POSE IS OUT OF MAP, POSITION Z: {self.odom_pose[0].pose.pose.position.z}m')
            self.get_logger().warn(f'TELEPORTING ROBOT TO NEW RANDOM INITIAL POSE ...')
            self.new_pose=generate_random_poses(1,self.x_max,self.y_max)[0]
            self.attemps+=1
            print(self.attemps)
            self.teleport_robot(self.new_pose)

    def odom_callback(self, msg):
        self.odom_pose = np.append(self.odom_pose, msg) 
        
class ClearCostmapNode(Node):
    def __init__(self):
        super().__init__('clear_costmap_client')
        
        self.client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for the clear_costmap service to be available...')
        self.req = ClearEntireCostmap.Request()
        self.req.request=Empty()
        self.send_request()
        
    def send_request(self):

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result().response
            if response == Empty():
                self.get_logger().info('Global costmap successfully cleared.')
                time.sleep(0.5)
            else:
                self.get_logger().info(f'Error al actualizar el Global Costmap')
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

class RobotPoseChecker(Node):
    def __init__(self):
        super().__init__('robot_pose_checker')
        self.current_pose = None        
        self.pose_received = False
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_profile,
        )
        self.get_logger().info('RobotPoseChecker node has been started.')

    def pose_callback(self, msg):
        self.pose_received = True
        self.current_pose = msg.pose.pose
        self.get_logger().info('Current robot pose received.')

    def get_current_pose(self):
        if self.current_pose:
            return self.current_pose
        else:
            self.get_logger().error('No current pose available.')
            return None
        
class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1, self.publish_initial_pose)

    def publish_initial_pose(self):
        # Crea un mensaje de tipo PoseWithCovarianceStamped
        initial_pose = PoseWithCovarianceStamped()

        # Configura el encabezado del mensaje
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'  # o el marco de referencia apropiado

        # Establece la posición (x, y, z) y la orientación (x, y, z, w) como un cuaternión
        initial_pose.pose.pose.position.x = 0.0  # Ejemplo de posición en x
        initial_pose.pose.pose.position.y = 0.0  # Ejemplo de posición en y
        initial_pose.pose.pose.position.z = 0.0  # La mayoría de las veces en 0 para mapas 2D
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0  # Orientación neutra

        # Publica la pose inicial
        self.publisher.publish(initial_pose)
        self.get_logger().info('Publicando la pose inicial del robot.')

def check_and_publish_initial_pose(node, initial_pose_publisher):
    # Espera a recibir al menos una pose del robot
    end_time = node.get_clock().now() + rclpy.duration.Duration(seconds=3.0)
    while rclpy.ok() and node.get_clock().now() < end_time and not node.pose_received:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.pose_received:
        # Si no se ha recibido ninguna pose, publica una pose inicial
        initial_pose_publisher.publish_initial_pose()
        node.get_logger().info('Pose inicial publicada debido a la falta de pose actual del robot.')
    else:
        node.get_logger().info('Pose actual del robot detectada; omitiendo publicación de pose inicial.')

def pose_to_goal_msg(pose):
    pose_msg = NavigateToPose.Goal()
    pose_msg.pose.header.frame_id = 'map'    
    pose_msg.pose.pose.position.x = pose[0]
    pose_msg.pose.pose.position.y = pose[1]
    pose_msg.pose.pose.position.z = pose[2]
    pose_msg.pose.pose.orientation.x = pose[3]
    pose_msg.pose.pose.orientation.y = pose[4]
    pose_msg.pose.pose.orientation.z = pose[5]
    pose_msg.pose.pose.orientation.w = pose[6] 
    return pose_msg

def pose_to_initial_pose(pose):
    #Construir el mensaje de la pose inicial
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.frame_id = 'map'
    init_msg.pose.pose.position.x = pose[0]
    init_msg.pose.pose.position.y = pose[1]
    init_msg.pose.pose.position.z = pose[2]
    init_msg.pose.pose.orientation.x = pose[3]
    init_msg.pose.pose.orientation.y = pose[4]
    init_msg.pose.pose.orientation.z = pose[5]
    init_msg.pose.pose.orientation.w = pose[6]
    return init_msg

def distance_2p(x1, y1, x2, y2):
    distance = math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
    return distance

def generate_random_poses(num_poses,x_max,y_max):
    poses = np.empty((num_poses, 7))  # Crear un array vacío para almacenar las poses
    
    for i in range(num_poses):
        # Genera una pose aleatoria con los primeros 2 valores aleatorios, y luego (0,0,0,0,1)
        poses[i] = np.concatenate((np.random.uniform(low=-x_max, high=x_max, size=1),np.random.uniform(low=-y_max, high=y_max, size=1), [0.0,0.0,0.0,0.0,1.0]))
    
    return poses

def main(args=None):
    rclpy.init(args=args)

    # Configuraciones de parámetros de navegación para técnica reactiva.
    configs = [
        {'v': 0.7, 'w': 1.0},
        {'v': 0.35, 'w': 0.75},
    ]

    # Configuraciones para técnicas globales de navegación
    nav_techniques=[False,True]

    len_config=len(configs)
    len_nav_technique=len(nav_techniques)
    
    robot_name=str(input("Introduce el nombre que el robot tiene en gazebo (Ej: Turtlebot3 = waffle, ClearPath: dingo_o): "))
    num_poses = int(input("Cantidad de poses aleatorias a generar: "))

    map_boundaries_node=MapBoundariesNode()
    rclpy.spin_once(map_boundaries_node)
    
    x_max=abs(map_boundaries_node.origin_x)-1
    y_max=abs(map_boundaries_node.origin_y)-1

    map_boundaries_node.destroy_node()

    np.random.seed(19)  # Configurar la semilla antes de generar poses aleatorias
    goal_poses = generate_random_poses(num_poses,x_max,y_max)
    len_poses = len(goal_poses)

    initial_poses =generate_random_poses(num_poses,x_max,y_max)

    data_holder=DataHolder(len_nav_technique, len_config, len_poses, robot_name)
    robot_pose_checker = RobotPoseChecker()
    initial_pose_publisher = InitialPosePublisher()

    # Comprueba y publica la pose inicial si es necesario
    check_and_publish_initial_pose(robot_pose_checker, initial_pose_publisher)

    robot_pose_checker.destroy_node()
    initial_pose_publisher.destroy_node()

    for n, nav_technique in enumerate(nav_techniques):
        # Cambiar parametros de navegacion global.
        print(f"{Fore.MAGENTA}CHANGING GLOBAL NAVIGATION TECHNIQUES ...")
        Change_Global_Nav_Technique_node = ChangingGlobalNavigationTechniques(n,nav_technique)
        rclpy.spin_until_future_complete(Change_Global_Nav_Technique_node, Change_Global_Nav_Technique_node.future)
        Change_Global_Nav_Technique_node.destroy_node()

        for j, config in enumerate(configs):
            print(f"{Fore.MAGENTA}CHANGING REACTIVE NAVIGATION PARAMETERS ...")
            # Cambiar los parámetros de navegación reactiva.
            reactive_nav_params_node = ReactiveNavParamsNode(config['v'], config['w'], j)
            rclpy.spin_until_future_complete(reactive_nav_params_node, reactive_nav_params_node.future)
            reactive_nav_params_node.destroy_node()

            # Lanzar las poses objetivo.

            for i,goal_pose in enumerate(goal_poses):
                print(f"{Fore.MAGENTA}NEW ITERATION: Nº{i+1}/{len_poses}")
                if n == 0 and j == 0:
                    robot_pose_is_inside_obstacle = True
                    obs = 0
                    z = 0
                    while robot_pose_is_inside_obstacle == True:
                        if obs == 0:
                            initial_pose = initial_poses[i,:]
                        else:
                            initial_pose = generate_random_poses(1,x_max,y_max)[0]
                            
                        robot_teleporter_node = RobotTeleporterNode(i, obs, robot_name, initial_pose, config['v'], config['w'], x_max, y_max)
                        initial_pose = robot_teleporter_node.initial_pose_in_map
                        robot_teleporter_node.destroy_node()
                        clear_costmap_node=ClearCostmapNode()
                        clear_costmap_node.destroy_node()
                        goal_pose_in_map=False
                        z=0

                        while goal_pose_in_map == False and z<2:
                            goal_pose_node = GoalPoseNode(z, i, j, n, obs, goal_pose)

                            if goal_pose_node.goal_path is not None:
                                goal_pose_in_map=True
                                robot_pose_is_inside_obstacle = False
                            
                            else:
                                if z == 0:
                                    goal_pose_node.get_logger().warn(f'PATH TO GOAL POSE NOT FOUND, GETTING NEW RANDOM GOAL POSE')
                                    goal_pose=generate_random_poses(1,x_max,y_max)[0]

                                elif z == 1:
                                    goal_pose = goal_poses[i,:]
                                    goal_pose_node.get_logger().warn(f'INITIAL POSE IS INSIDE AN OBSTACLE, GETTING NEW RANDOM INITIAL POSE')
                                
                            z+=1
                            goal_pose_node.destroy_node()           
                        obs+=1

                    initial_poses[i,:] = initial_pose
                    goal_poses[i,:] = goal_pose
                
                else:
                    initial_pose = initial_poses[i,:]
                    robot_teleporter_node = RobotTeleporterNode(i, obs, robot_name, initial_pose, config['v'], config['w'], x_max, y_max)
                    robot_teleporter_node.destroy_node()
                    clear_costmap_node=ClearCostmapNode()
                    clear_costmap_node.destroy_node()
                    goal_pose_node = GoalPoseNode(z, i, j, n, obs, goal_pose)
                    goal_pose_node.destroy_node()
                
                data_holder.CalculateAveragesValues(goal_pose_node.exito,goal_pose_node.elapsed_time,goal_pose_node.teorical_distance_traveled, goal_pose_node.real_distance_traveled, config['v'], config['w'], nav_technique, i, j, n)

    rclpy.shutdown()
if __name__ == '__main__':
    main()