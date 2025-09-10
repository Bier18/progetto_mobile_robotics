#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import Marker
import math,time

class LetterCoding(Node):
    def __init__(self):
        super().__init__('letter_coding_node')

        # Parametro: parola da codificare
        self.declare_parameter('word', 'HELLO')
        self.word = self.get_parameter('word').get_parameter_value().string_value.upper()
        self.current_letter_idx = 0


        # Lista dei robot
        self.robots = ['t0','t1','t2','t3','t4','t5','t6','t7','t8','t9','t10','t11','t12']

        # Mappa delle lettere
        self.letter_map = {
            "A": ['t0','t1','t2','t3','t4','t5','t7','t8','t9','t10','t11','t12'],
            "B": ['t0','t3','t5','t8','t10'],
            "C": ['t0','t1','t2','t4','t5','t6','t7','t8','t10','t11','t12'],
            "D": ['t0','t1','t2','t4','t5','t6','t7','t9','t10','t11','t12'],
            "E": ['t0','t2','t3','t4','t5','t6','t7','t9','t12'],
            "F": ['t0','t1','t2','t3','t5','t6','t7','t9','t10','t11','t12'],
            "G": ['t0','t1','t2','t3','t5','t6','t7','t8','t9','t10','t11','t12'],
            "H": ['t0','t1','t2','t4','t7','t9','t12'],
            "I": ['t0','t1','t2','t3','t4','t5','t6','t7','t8','t9','t10','t11','t12'],
            "J": ['t0','t1','t2','t3','t4','t5','t6','t7','t9','t10','t11','t12'],
            "K": ['t0','t1','t2','t3','t4','t5','t7','t8','t9','t10','t11','t12'],
            "L": ['t0','t3','t5','t8','t10'],
            "M": ['t0','t1','t2','t4','t5','t6','t7','t8','t10','t11','t12'],
            "N": ['t0','t1','t2','t4','t5','t6','t7','t9','t10','t11','t12'],
            "O": ['t0','t2','t3','t4','t5','t6','t7','t9','t12'],
            "P": ['t0','t1','t2','t3','t5','t6','t7','t9','t10','t11','t12'],
            "Q": ['t0','t1','t2','t3','t5','t6','t7','t8','t9','t10','t11','t12'],
            "R": ['t0','t1','t2','t4','t7','t9','t12'],
            "S": ['t0','t1','t2','t3','t4','t5','t6','t7','t8','t9','t10','t11','t12'],
            "T": ['t0','t1','t2','t3','t4','t5','t6','t7','t9','t10','t11','t12'],
            "U": ['t0','t1','t2','t3','t4','t5','t7','t8','t9','t10','t11','t12'],
            "V": ['t0','t3','t5','t8','t10'],
            "W": ['t0','t1','t2','t4','t5','t6','t7','t8','t10','t11','t12'],
            "X": ['t0','t1','t2','t4','t5','t6','t7','t9','t10','t11','t12'],
            "Y": ['t0','t2','t3','t4','t5','t6','t7','t9','t12'],
            "Z": ['t0','t1','t2','t3','t5','t6','t7','t9','t10','t11','t12']
        }
        # posizioni target (griglia)
        self.target_pose = {
            't0': (-5.0, 1.0),
            't1': (-5.0, 2.0),
            't2': (-5.0, 3.0),
            't3': (-4.0, 1.0),
            't4': (-4.0, 3.0),
            't5': (-3.0, 1.0),
            't6': (-3.0, 2.0),
            't7': (-3.0, 3.0),
            't8': (-2.0, 1.0),
            't9': (-2.0, 3.0),
            't10': (-1.0, 1.0),
            't11': (-1.0, 2.0),
            't12': (-1.0, 3.0)
        }

        #numero di robot arrivati a destinazione
        self.arrived = 0
        
        #ritorno alla posizione di partenza
        self.reset = False

        # Saturazione, deadzone e collision avoidance
        self.vmax = 0.3
        self.wmax = 1.0
        self.deadzone_linear = 0.05
        self.d_safe = 0.6

        # Controllori proporzionali
        self.Kp_linear = 0.3
        self.Kp_angular = 0.5

        # Posizioni correnti globali
        self.robot_global_pose = {robot: (0.0, 0.0, 0.0) for robot in self.robots}

        # Publisher cmd_vel
        self.publisher_list = [self.create_publisher(TwistStamped, f'{r}/cmd_vel', 10) for r in self.robots]
        
        # Publisher Marker
        self.led_pub = self.create_publisher(Marker, 'led_marker', 10)

        # colore del led
        self.color = (0.0,0.0,0.0)

        # Subscriber a global_pose
        for robot in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/{robot}/global_pose',
                lambda msg, r=robot: self.global_pose_callback(msg, r),
                10
            )

        self.timer = self.create_timer(0.1, self.generate_coding)

    def global_pose_callback(self, msg: PoseStamped, robot: str):
        # Estrai yaw dalla quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_global_pose[robot] = (msg.pose.position.x, msg.pose.position.y, yaw)

    def generate_coding(self):
        if self.current_letter_idx >= len(self.word):
            return  

        letter = self.word[self.current_letter_idx]

        if 'A' <= letter <= 'J':
            self.color = (1.0,0.0,0.0)
        elif 'K' <= letter <= 'T':
            self.color = (0.0,1.0,0.0)
        else:
            self.color = (0.0,0.0,1.0)
        active_robots = self.letter_map.get(letter, [])
        robot = active_robots[self.arrived]

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        target_x, target_y = self.target_pose[robot]
        current_x, current_y, current_yaw = self.robot_global_pose[robot]

        # Errore
        err_x = target_x - current_x
        err_y = target_y - current_y
        
        # Collision Avoidance
        rep_x = rep_y = 0.0
        k_rep = 0.5
        for other, (ox, oy, _) in self.robot_global_pose.items():
            if other == robot:
                continue
            dx = current_x - ox
            dy = current_y - oy
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < self.d_safe and dist > 1e-3:
                # Blocca il robot se è troppo vicino
                rep_x += k_rep * (1/dist - 1/self.d_safe) * (dx / (dist**3))
                rep_y += k_rep * (1/dist - 1/self.d_safe) * (dy / (dist**3))
            
        total_x = self.Kp_linear * err_x + rep_x
        total_y = self.Kp_linear * err_y + rep_y
        distance = math.sqrt(total_x**2 + total_y**2)
        angle_to_target = math.atan2(total_y, total_x)
        error_yaw = math.atan2(math.sin(angle_to_target - current_yaw),
                            math.cos(angle_to_target - current_yaw))

        linear_speed = min(self.vmax, distance)
        angular_speed = max(-self.wmax, min(self.wmax, self.Kp_angular * error_yaw))

        # Deadzone
        if distance < self.deadzone_linear:
            linear_speed = 0.0
            angular_speed = 0.0
            self.arrived += 1
        
        
        if self.arrived == len(active_robots): # se tutti sono arrivati
            # allora si cambia il target
            self.arrived = 0
            self.switch_target(active_robots=active_robots)

        # Saturazione
        msg.twist.linear.x = max(-self.vmax, min(self.vmax, linear_speed))
        msg.twist.angular.z = max(-self.wmax, min(self.wmax, angular_speed))
        
        robot_idx = int(robot[1:])
        self.publisher_list[robot_idx].publish(msg)
    
    def switch_target(self, active_robots):
        if self.reset:
            self.current_letter_idx +=1
            self.letter_completed = False
            self.target_pose = {
                't0': (-5.0, 1.0),
                't1': (-4.0, 2.0),
                't2': (-5.0, 3.0),
                't3': (-4.0, 1.0),
                't4': (-4.0, 3.0),
                't5': (-3.0, 1.0),
                't6': (-3.0, 2.0),
                't7': (-3.0, 3.0),
                't8': (-2.0, 1.0),
                't9': (-2.0, 3.0),
                't10': (-1.0, 1.0),
                't11': (-1.0, 2.0),
                't12': (-1.0, 3.0)
            }
        else:
            self.blink(robot_list=active_robots)
            self.target_pose = {
                't0': (-5.0, 0.0),
                't1': (-4.0, 0.0),
                't2': (-3.0, 0.0),
                't3': (-2.0, 0.0),
                't4': (-1.0, 0.0),
                't5': (0.0, 0.0),
                't6': (1.0, 0.0),
                't7': (2.0, 0.0),
                't8': (3.0, 0.0),
                't9': (4.0, 0.0),
                't10': (5.0, 0.0),
                't11': (6.0, 0.0),
                't12': (7.0, 0.0)
            }
            self.letter_completed = True
        self.reset = not self.reset
    
    def turn_on_led(self, robot_name, idx):
        marker = Marker()
        marker.header.frame_id = "map" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "led"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posizione: sopra il robot o fissa
        x, y, _ = self.robot_global_pose[robot_name]
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.4  # altezza sopra il robot
        marker.pose.orientation.w = 1.0

        # Dimensione e colore
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = 1.0 

        self.led_pub.publish(marker)

    def turn_off_led(self, idx):
        marker = Marker()
        marker.header.frame_id = "map"  # lo stesso frame del marker originale
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "led"                # stesso namespace
        marker.id = idx                  # stesso ID
        marker.action = Marker.DELETE    # azione DELETE cancella il marker
        self.led_pub.publish(marker)

    def blink(self, robot_list):
        for r in robot_list:
            self.turn_on_led(robot_name=r, idx=int(r[1:]))
        time.sleep(0.5)
        for r in robot_list:
            self.turn_off_led(idx=int(r[1:]))

def main(args=None):
    rclpy.init(args=args)
    node = LetterCoding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
