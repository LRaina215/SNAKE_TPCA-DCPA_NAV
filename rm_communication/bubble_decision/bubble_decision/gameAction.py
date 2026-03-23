import math
import time
from rclpy.node import Node
from game_msgs.msg import RobotHP
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

try:
    from auto_aim_interfaces.msg import Target
except ImportError:
    try:
        from rm_interfaces.msg import Target
    except ImportError:
        pass 

# ==========================================
# 新增：场地战术与坐标转换规划器
# ==========================================
class FieldTacticsPlanner:
    def __init__(self, team_color):
        self.team_color = team_color.lower()
        
        # 1. 记录场地绝对坐标 (人类易读的坐标，以场地左下角为 0,0)
        # X 轴向右 (0~12m), Y 轴向上 (0~8m)
        self.abs_points = {
            'center': (6.0, 4.0),
            'blue_choke': (3.0, 4.0),
            'red_choke': (9.0, 4.0),
            'blue_base': (2.0, 6.0),
            'red_base': (10.0, 2.0),
            'blue_flank_top': (3.0, 6.5),
            'blue_flank_bottom': (3.0, 1.5),
            'red_flank_top': (9.0, 6.5),
            'red_flank_bottom': (9.0, 1.5),
            'blue_heal': (0.5, 7.5),
            'red_heal': (11.5, 0.5)
        }

    def abs_to_nav(self, abs_x, abs_y):
        """将绝对坐标转换为 Nav2 底层需要的相对坐标"""
        if self.team_color == 'blue':
            # 蓝方出生点即为 Nav2 的 (0,0) -> 对应绝对坐标 (0.75, 7.0)
            return {'x': abs_x - 0.75, 'y': abs_y - 7.0}
        elif self.team_color == 'red':
            # 红方出生点即为 Nav2 的 (0,0) -> 对应绝对坐标 (11.25, 1.0)
            return {'x': abs_x - 11.25, 'y': abs_y - 1.0}
        else:
            return {'x': abs_x, 'y': abs_y} # 回退默认

    def get_patrol_route(self):
        """生成 8字形/回字形 动态巡航路线"""
        route = []
        if self.team_color == 'blue':
            # 蓝方：基地防御 -> 上侧翼 -> 咽喉要道 -> 下侧翼 -> 咽喉要道(循环)
            route_abs = [
                self.abs_points['blue_base'],
                self.abs_points['blue_flank_top'],
                self.abs_points['blue_choke'],
                self.abs_points['blue_flank_bottom'],
                self.abs_points['blue_choke'] 
            ]
        else:
            # 红方：基地防御 -> 下侧翼 -> 咽喉要道 -> 上侧翼 -> 咽喉要道(循环)
            route_abs = [
                self.abs_points['red_base'],
                self.abs_points['red_flank_bottom'],
                self.abs_points['red_choke'],
                self.abs_points['red_flank_top'],
                self.abs_points['red_choke']
            ]

        # 批量转换坐标为字典格式 {'x':.., 'y':..}
        for x, y in route_abs:
            route.append(self.abs_to_nav(x, y))
        return route
        
    def get_heal_zone(self):
        """获取回血点坐标"""
        if self.team_color == 'blue':
            return self.abs_to_nav(*self.abs_points['blue_heal'])
        else:
            return self.abs_to_nav(*self.abs_points['red_heal'])


# ==========================================
# 哨兵游戏动作核心逻辑
# ==========================================
class SentryGameAction():
    def __init__(self, node: Node, team_color: str) -> None:
        self.node = node    
        
        # 容错处理：如果不小心传错颜色，默认给 red
        if team_color not in ['red', 'blue']:
            self.node.get_logger().error(f"未知阵营: {team_color}，回退至 red！")
            self.team_color = 'red'
        else:
            self.team_color = team_color
            
        self.blackboard = {
            'hp': 400,
            'hp_low_threshold': 200,    
            'last_target_time': 0.0,    
            'target_distance': 999.0,   
            'current_pose': {'x': 0.0, 'y': 0.0}, 
            'current_patrol_index': 0   
        }
        
        self.current_goal_name = None
        
        # === 动态加载战术地图 ===
        try:
            planner = FieldTacticsPlanner(self.team_color)
            self.waypoints = {
                'patrol': planner.get_patrol_route(),
                'heal_zone': planner.get_heal_zone()
            }
            self.node.get_logger().info(f"成功加载 {self.team_color} 方战术巡航坐标！第一站坐标: X={self.waypoints['patrol'][0]['x']:.2f}, Y={self.waypoints['patrol'][0]['y']:.2f}")
        except Exception as e:
            self.node.get_logger().error(f"坐标加载失败: {e}")
            self.waypoints = {'patrol': [{'x':0.0, 'y':0.0}], 'heal_zone': {'x':0.0, 'y':0.0}}

        # === ROS 发布与订阅 ===
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.hp_sub = self.node.create_subscription(RobotHP, '/status/robotHP', self.hp_callback, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.target_sub = self.node.create_subscription(Target, '/tracker/target', self.target_callback, 10)

    # ================= 回调函数 =================
    def hp_callback(self, msg: RobotHP):
        if self.team_color == 'red':
            self.blackboard['hp'] = msg.red_7_robot_hp
        else:
            self.blackboard['hp'] = msg.blue_7_robot_hp

    def odom_callback(self, msg: Odometry):
        self.blackboard['current_pose']['x'] = msg.pose.pose.position.x
        self.blackboard['current_pose']['y'] = msg.pose.pose.position.y

    def target_callback(self, msg: Target):
        if msg.tracking:
            self.blackboard['last_target_time'] = time.time()
            target_x = msg.position.x
            target_y = msg.position.y
            dx = target_x - self.blackboard['current_pose']['x']
            dy = target_y - self.blackboard['current_pose']['y']
            self.blackboard['target_distance'] = math.sqrt(dx**2 + dy**2)
            self.blackboard['pursuit_pose'] = {'x': target_x, 'y': target_y}

    def has_target(self):
        return (time.time() - self.blackboard['last_target_time']) < 0.5

    # ================= 动作执行 (Actions) =================
    def publish_nav_goal(self, point_dict, name):
        if self.current_goal_name == name:
            return
            
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = 'map' 
        goal.pose.position.x = float(point_dict['x'])
        goal.pose.position.y = float(point_dict['y'])
        
        self.goal_pub.publish(goal)
        self.current_goal_name = name
        self.node.get_logger().info(f"Nav2 新目标: {name} ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")

    def execute_retreat_to_heal(self):
        self.publish_nav_goal(self.waypoints['heal_zone'], "heal_zone")
        twist = Twist()
        twist.angular.z = 3.14  # 回血时保命小陀螺
        self.cmd_vel_pub.publish(twist)

    def execute_combat(self):
        distance = self.blackboard['target_distance']
        if distance < 1.5:
            self.publish_nav_goal(self.blackboard['current_pose'], "stop_for_shoot")
            twist = Twist()
            twist.angular.z = 4.0 
            self.cmd_vel_pub.publish(twist)
            self.node.get_logger().info("距离极近，启动原地小陀螺迎战！")
        else:
            pursuit_target = self.blackboard.get('pursuit_pose')
            if pursuit_target:
                self.publish_nav_goal(pursuit_target, "pursuit_enemy")
                self.node.get_logger().info(f"追击敌人，目标: x={pursuit_target['x']:.2f}, y={pursuit_target['y']:.2f}")
        
    def execute_patrol(self):
        idx = self.blackboard['current_patrol_index']
        target_pt = self.waypoints['patrol'][idx]
        
        dx = self.blackboard['current_pose']['x'] - target_pt['x']
        dy = self.blackboard['current_pose']['y'] - target_pt['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.3: 
            self.node.get_logger().info(f"到达巡逻点 {idx}，切换下一个点")
            self.blackboard['current_patrol_index'] = (idx + 1) % len(self.waypoints['patrol'])
            return 
            
        self.publish_nav_goal(target_pt, f"patrol_point_{idx}")
