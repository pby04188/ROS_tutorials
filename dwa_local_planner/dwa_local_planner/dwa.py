import math
import numpy as np
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from rclpy.parameter import Parameter

class Config(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # robot parameter
        self.max_speed = 0.8  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 80.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_width = 1.0  # [m] for collision check
        self.robot_length = 1.0  # [m] for collision check

        self.x = np.array([0,0,0,0,0]) # [x(m), y(m), theta(rad), linear_velocity(m/s), angular_velocity(rad/s)]
        self.u = np.array([0,0]) # [linear_velocity(m/s), angular_velocity(rad/s)]

        # cmd_vel message
        self.twist = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[0.0, 0.0]])

        # waypoints [[x(m), y(m)],[x(m), y(m)], ...]
        self.declare_parameter('num_waypoints', 4)
        self.num_waypoints = self.get_parameter('num_waypoints').value
        print(self.num_waypoints)
        self.declare_parameter('waypoint_1', None)
        self.declare_parameter('waypoint_2', None)
        self.declare_parameter('waypoint_3', None)
        self.declare_parameter('waypoint_4', None)
        waypoint1 = self.get_parameter('waypoint_1').value
        waypoint2 = self.get_parameter('waypoint_2').value
        waypoint3 = self.get_parameter('waypoint_3').value
        waypoint4 = self.get_parameter('waypoint_4').value
        self.waypoints = np.array([waypoint1, waypoint2, waypoint3, waypoint4])
        self.i = 0

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        self.declare_parameter('waypoints')

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        # turtle1's pose subscriber
        self.turtle1_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscribe_turtle1_pose,
            QOS_RKL10V)

        # turtle2's pose subscriber
        self.turtle2_sub = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.subscribe_turtle2_pose,
            QOS_RKL10V)
        
        # turtle2's cmd_vel publisher
        self.turtle2_pub = self.create_publisher(
            Twist,
            '/turtle2/cmd_vel',
            QOS_RKL10V)

        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.count = 0

    def subscribe_turtle1_pose(self, msg):
        self.ob = np.array([[msg.x, msg.y]])

    def subscribe_turtle2_pose(self, msg):
        self.x = np.array([msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])


    def publish_cmd(self):
        self.linear.x = float(self.u[0])
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = float(self.u[1])
        self.twist.linear = self.linear
        self.twist.angular = self.angular

        trajectory = np.array(self.x)
        self.u, predicted_trajectory = self.dwa_control(self.x, self.waypoints[self.i], self.ob)
        self.x = self.motion(self.x, self.u, self.dt)  # simulate robot
        trajectory = np.vstack((trajectory, self.x))  # store state history
        dist_to_goal = math.hypot(self.x[0] - self.waypoints[self.i][0], self.x[1] - self.waypoints[self.i][1])
        self.turtle2_pub.publish(self.twist)
        if dist_to_goal <= self.robot_width/2:
            self.i = self.i + 1
        if self.i == 4:
            self.i = 0


    def dwa_control(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)

        return u, trajectory


    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory


    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                        np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost


def main(gx=2.5, gy=8.58, args=None):
    rclpy.init(args=args)
    try:
        node = Config()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrypt (SIGINT)')
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
