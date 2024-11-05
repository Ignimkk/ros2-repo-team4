import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
# from minibot_interfaces.msg import RackList, GoalStatus
from lrobot.a_star import AStarPlanner  # AStar 클래스 임포트
from std_msgs.msg import String, Bool

pose_dict = {
    "R_A": [2.52, 1.31, 0.0, 0.0], "R_B": [2.44, 0.28, 0.0, 0.0],
    "R_C": [1.50, 1.29, 0.0, 0.0], "R_D": [1.39, 0.30, 0.0, 0.0],
    "StopOver": [2.14, 1.89, 0.0, 0.0]
}

class PathServer(Node):
    def __init__(self):
        super().__init__('path_server1')

        self.initial_position = (0.0, 0.0)
        self.current_pose = None
        self.goal_pose = None
        self.path_index = 0
        self.goal_list = []
        self.light_off_2 = False
        self.go_home_2 = False
        self.arrived = False
        
        self.lock = threading.Lock()
        
        # Subscriber
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.rack_list_subscription = self.create_subscription(String, 'rack_list_1', self.goal_callback, 10)
        self.task_done_subscription = self.create_subscription(Bool, 'light_off_1', self.light_on_callback2, 10)
        self.arrive_subscription = self.create_subscription(Bool, 'Arrive', self.arrive_callback, 10)
        
        # Publisher
        self.path_publisher_1 = self.create_publisher(Path, 'planned_path_1', 10)
        self.arrive_send_publisher_1 = self.create_publisher(String, 'goal_status', 10)

        self.path_thread = threading.Thread(target=self.path_processing_thread)
        self.path_thread.start()
        
    def pose_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)
        self.get_logger().info(f'Current pose: {self.current_pose}')
    
    def light_on_callback2(self, msg: Bool):
        if msg.data:
            with self.lock:
                self.light_off_2 = True
            self.get_logger().info("Rack List 내 직전 goal location LED 켜졌다. 다음 goal location으로 이동해야함")
            self.process_next_goal()
    
    def arrive_callback(self, msg: Bool):
        if msg.data:
            goal_status_msg = GoalStatus()
            with self.lock:
                goal_status_msg.current_rack = self.goal_list[self.path_index - 1]
            goal_status_msg.status = 'completed'
            self.arrive__send_publisher_2.publish(goal_status_msg)
            self.get_logger().info(f"Published Goal Status: {goal_status_msg.current_rack}, {goal_status_msg.status}")
    
    def goal_callback(self, msg):
        with self.lock:
            self.goal_list = [msg.data]
            
            if not self.goal_list:
                self.get_logger().info("Received empty goal list.")
                return

            self.path_index = 0
            self.get_logger().info(f'Received new goal: {self.goal_list}')
            self.process_next_goal()

    def path_processing_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def process_next_goal(self):
        if not self.goal_list:
            self.get_logger().info("No goals to process.")
            return
        if self.current_pose and self.path_index < len(self.goal_list):
            key = self.goal_list[self.path_index]
            self.goal_pose = pose_dict.get(key, None)
            if self.goal_pose:
                self.light_off_2 = False
                self.move_to_target()
            else:
                self.get_logger().error(f"Invalid goal key: {key}")
        elif self.path_index >= len(self.goal_list):
            self.get_logger().info("All goals processed.")
            self.light_off_2 = False
            if len(self.goal_list) == 0:
                self.go_home_2 = True
                self.light_off_2 = True

    def move_to_target(self):
        if not self.goal_pose:
            self.get_logger().error("Goal pose is not set.")
            return

        sx_real, sy_real = self.current_pose if self.current_pose else self.initial_position
        gx_real, gy_real, gz_real, gw_real = self.goal_pose

        sx_real = round(sx_real, 2)
        sy_real = round(sy_real, 2)
        gx_real = round(gx_real, 2)
        gy_real = round(gy_real, 2)

        # 현재 위치와 목표 위치의 x 좌표 비교
        if gx_real > sx_real:
            # 목표 위치의 x 좌표가 더 클 경우, "StopOver" 경유
            stopover_pose = pose_dict.get("StopOver", None)
            if stopover_pose:
                stopover_x, stopover_y, stopover_z, stopover_w = stopover_pose
                # StopOver 경유
                self.get_logger().info(f"Moving to StopOver at ({stopover_x}, {stopover_y}) before target.")
                self.plan_and_move(sx_real, sy_real, stopover_x, stopover_y, stopover_z, stopover_w)

                # StopOver에서 목표 위치로 이동
                sx_real, sy_real = stopover_x, stopover_y  # StopOver에서 다시 시작
                self.get_logger().info(f"Proceeding to target at ({gx_real}, {gy_real}) from StopOver.")
        
        # 목표로 이동 (StopOver 경유 여부와 상관없이 마지막 목표로 이동)
        self.plan_and_move(sx_real, sy_real, gx_real, gy_real, gz_real, gw_real)
    
    def plan_and_move(self, sx_real, sy_real, gx_real, gy_real, gz_real, gw_real):
        # A* 경로 계획 수행
        a_star = AStarPlanner(resolution=1, rr=0.4, padding=1)
        
        rx, ry, tpx, tpy, tvec_x, tvec_y = a_star.planning(sx_real, sy_real, gx_real, gy_real)

        if not tpx or not tpy:
            self.get_logger().error("No path found")
            return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()  

        for x, y in zip(tpx, tpy):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = Quaternion(z=gz_real, w=gw_real)
            path.poses.append(pose)

        self.path_publisher_2.publish(path)
        self.get_logger().info(f"Published path with {len(tpx)} waypoints.")
        self.path_index += 1

def main(args=None):
    rclpy.init(args=args)
    path_server = PathServer()
    try:
        rclpy.spin(path_server)
    except KeyboardInterrupt:
        pass
    finally:
        path_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
