# import rclpy
# from rclpy.node import Node
# from rclpy.task import Future
# from gazebo_msgs.srv import SpawnEntity
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import Pose
# import random
# import numpy as np
# import time

# class ChaosSpawner(Node):
#     def __init__(self):
#         super().__init__('chaos_spawner')
        
#         # 1. Client to spawn objects in Gazebo
#         self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
#         # 2. Subscriber to read the map and find empty spaces
#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             '/map',
#             self.map_callback,
#             10
#         )
        
#         self.map_data = None
#         self.get_logger().info('Waiting for /map data...')

#     def map_callback(self, msg):
#         """Called once we get the map. We only need it once."""
#         if self.map_data is None:
#             self.get_logger().info('Map received! finding empty spot...')
#             self.map_data = msg
#             self.spawn_random_cone()

#     def get_random_free_pose(self):
#         """Finds a random free pixel in the map and converts it to World Coordinates."""
        
#         # Access map metadata
#         width = self.map_data.info.width
#         height = self.map_data.info.height
#         resolution = self.map_data.info.resolution
#         origin_x = self.map_data.info.origin.position.x
#         origin_y = self.map_data.info.origin.position.y
        
#         # Convert 1D array to numpy for easier searching
#         # Data: -1 (Unknown), 0 (Free), 100 (Occupied)
#         grid = np.array(self.map_data.data)
        
#         # Find all indices where value is 0 (Free Space)
#         free_indices = np.where(grid == 0)[0]
        
#         if len(free_indices) == 0:
#             self.get_logger().error("Map has no free space!")
#             return None

#         # --- SAFETY CHECK LOOP ---
#         # Pick a random spot, but make sure it isn't too close to the robot (assumed at 0,0)
#         # We try 100 times to find a valid spot.
#         for _ in range(100):
#             random_idx = random.choice(free_indices)
            
#             # Convert Index -> Grid X,Y
#             grid_y = random_idx // width
#             grid_x = random_idx % width
            
#             # Convert Grid X,Y -> World Meters
#             real_x = (grid_x * resolution) + origin_x
#             real_y = (grid_y * resolution) + origin_y
            
#             # Calculate distance from (0,0)
#             dist_sq = real_x**2 + real_y**2
            
#             # If it is more than 1.5 meters away from start, it's a good spot
#             if dist_sq > 2.25: # 1.5^2 = 2.25
#                 pose = Pose()
#                 pose.position.x = real_x
#                 pose.position.y = real_y
#                 pose.position.z = 0.0
#                 return pose
        
#         return None

#     def spawn_random_cone(self):
#         target_pose = self.get_random_free_pose()
        
#         if target_pose is None:
#             self.get_logger().error("Could not find a safe spawn spot.")
#             return

#         self.get_logger().info(f"Spawning Cone at X: {target_pose.position.x:.2f}, Y: {target_pose.position.y:.2f}")

#         # Wait for service
#         while not self.spawn_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for /spawn_entity service...')

#         request = SpawnEntity.Request()
#         request.name = f'construction_cone_{int(time.time())}'
#         request.initial_pose = target_pose
#         request.reference_frame = 'map'
        
#         # Define a simple Red Cylinder (No need for external mesh files)
#         request.xml = """
#         <?xml version='1.0'?>
#         <sdf version='1.7'>
#           <model name='simple_cone'>
#             <static>true</static>
#             <link name='link'>
#               <visual name='visual'>
#                 <geometry>
#                   <cylinder>
#                     <radius>0.15</radius>
#                     <length>0.6</length>
#                   </cylinder>
#                 </geometry>
#                 <material>
#                   <ambient>1 0 0 1</ambient>
#                   <diffuse>1 0 0 1</diffuse>
#                 </material>
#               </visual>
#               <collision name='collision'>
#                 <geometry>
#                   <cylinder>
#                     <radius>0.15</radius>
#                     <length>0.6</length>
#                   </cylinder>
#                 </geometry>
#               </collision>
#             </link>
#           </model>
#         </sdf>
#         """

#         future = self.spawn_client.call_async(request)
#         future.add_done_callback(self.spawn_done_callback)

#     def spawn_done_callback(self, future):
#         try:
#             result = future.result()
#             self.get_logger().info(f"Spawn Result: {result.status_message}")
#             # We are done, exit the node
#             raise SystemExit
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     spawner = ChaosSpawner()
    
#     try:
#         rclpy.spin(spawner)
#     except SystemExit:
#         pass
    
#     spawner.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, QoSPresetProfiles
import random
import math
import time

class SmartChaosSpawner(Node):
    def __init__(self):
        super().__init__('smart_chaos_spawner')
        
        # 1. QoS for Map (Transient Local)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. QoS for Odom (Best Effort / Sensor Data)
        # This is CRITICAL. Many robots publish odom as "Best Effort".
        odom_qos = QoSPresetProfiles.SENSOR_DATA.value

        # 3. Client to spawn objects
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # 4. Subscribe to Map
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            map_qos)
        
        # 5. Subscribe to Odom (Changed Topic and QoS)
        # We try '/diff_cont/odom' first as it's often the raw source
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/diff_cont/odom', 
            self.odom_callback, 
            odom_qos)
        
        self.map_data = None
        self.current_pose = None
        self.has_spawned = False
        
        self.get_logger().info('Waiting for Map... (Odom listener active on /diff_cont/odom)')

    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info(f'Map Received! {msg.info.width}x{msg.info.height}')
        self.map_data = msg
        self.attempt_spawn()

    def odom_callback(self, msg):
        # Log ONLY the first time we get odom
        if self.current_pose is None:
            self.get_logger().info('Odom Connection SUCCESS! Position received.')
            
        self.current_pose = msg.pose.pose
        self.attempt_spawn()

    def attempt_spawn(self):
        # We need BOTH to proceed
        if self.map_data is None:
            return
            
        if self.current_pose is None:
            # We have map but no odom yet.
            return 
            
        if self.has_spawned:
            return

        self.get_logger().info('Map & Odom Ready. Finding valid spawn point...')
        self.spawn_near_robot()

    def get_nearby_free_pose(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        data = self.map_data.data

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Try 200 times
        for _ in range(200):
            # DONUT LOGIC: 1.5m to 3.0m radius
            dist = random.uniform(1.5, 3.0)
            angle = random.uniform(0, 2 * math.pi)

            cand_x = robot_x + dist * math.cos(angle)
            cand_y = robot_y + dist * math.sin(angle)

            grid_x = int((cand_x - origin_x) / resolution)
            grid_y = int((cand_y - origin_y) / resolution)

            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                # Check collision (0 = Free space)
                if data[index] == 0:
                    pose = Pose()
                    pose.position.x = cand_x
                    pose.position.y = cand_y
                    pose.position.z = 0.0
                    return pose
        return None

    def spawn_near_robot(self):
        self.has_spawned = True
        target_pose = self.get_nearby_free_pose()
        
        if target_pose is None:
            self.get_logger().error("Could not find a valid spot nearby! (Maybe robot is in unknown space?)")
            return

        self.get_logger().info(f"Spawning Cone at X:{target_pose.position.x:.2f}, Y:{target_pose.position.y:.2f}")

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        request = SpawnEntity.Request()
        request.name = f'cone_{int(time.time())}'
        request.initial_pose = target_pose
        request.reference_frame = 'map'
        
        request.xml = """
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='simple_cone'>
            <static>true</static>
            <link name='link'>
              <visual name='visual'>
                <geometry>
                  <cylinder>
                    <radius>0.15</radius>
                    <length>0.6</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
              <collision name='collision'>
                <geometry>
                  <cylinder>
                    <radius>0.15</radius>
                    <length>0.6</length>
                  </cylinder>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_done_callback)

    def spawn_done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Spawn Result: {result.status_message}")
            raise SystemExit 
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    spawner = SmartChaosSpawner()
    try:
        rclpy.spin(spawner)
    except SystemExit:
        pass
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()