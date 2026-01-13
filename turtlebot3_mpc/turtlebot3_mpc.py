import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult 
from nav_msgs.msg import Odometry, Path 
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from std_msgs.msg import Float32 
from math import atan2, sqrt, sin, cos
import casadi as ca
import numpy as np


class MPC(Node):
    def __init__(self):
        super().__init__('MPC')

        self.declare_parameter('q_x', 1.0)
        self.declare_parameter('q_y', 1.0)
        self.declare_parameter('q_t', 0.4)

        self.declare_parameter('qt_x', 5.0)
        self.declare_parameter('qt_y', 5.0)
        self.declare_parameter('qt_t', 2.0)

        self.declare_parameter('r_v', 0.1)
        self.declare_parameter('r_w', 0.5)

        self.declare_parameter('forward', 0)

        self.add_on_set_parameters_callback(self.parameter_callback)
                                            
        self.dt = 0.05
        self.N = 50
        self.goal_state = None
        self.current_state = None
        self.prev_sol_X = None
        self.prev_sol_U = None
        self.goal_reached = False

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback, 
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/Path',
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom',
            self.odom_callback, 
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.cost_pub = self.create_publisher(
            Float32, 
            '/mpc/cost', 
            10
        )

        self.cost_q_pub = self.create_publisher(
            Float32,
            '/mpc/q_cost',
            10
        )   

        self.cost_u_pub = self.create_publisher(
            Float32,
            '/mpc/v_cost',
            10
        )   
            
        self.f_model = self.robot_model()
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Node Initialized, Waiting for goal_pose...')

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")
        return SetParametersResult(successful=True)


    def robot_model(self):

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        v = ca.SX.sym('v')
        w = ca.SX.sym('w')
        controls = ca.vertcat(v,w)

        rhs = ca.vertcat(
            x + (v*ca.cos(theta))*self.dt,
            y + (v*ca.sin(theta))*self.dt,
            theta + w*self.dt
        )

        return ca.Function('f', [states, controls], [rhs])
    
    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation

        z1 = 2 * (q.w * q.z + q.x * q.y)
        z2 = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(z1, z2)
        
        self.get_logger().info('Got the goal_state and Moving towards it.....')
        self.goal_state = np.array([x, y, yaw])
        self.goal_reached = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        z1 = 2 * (q.w * q.z + q.x * q.y)
        z2 = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(z1, z2)
        
        self.current_state = np.array([x, y, yaw])
        
    def solve_mpc(self):
        opti = ca.Opti()

        X = opti.variable(3, self.N+1)
        U = opti.variable(2, self.N)

        x = X[0, :]
        y = X[1, :]
        theta = X[2, :]

        v = U[0, :]
        w = U[1, :]
        
        if self.prev_sol_X is not None:
            opti.set_initial(X, self.prev_sol_X)
            opti.set_initial(U, self.prev_sol_U)

        q_x = self.get_parameter('q_x').value
        q_y = self.get_parameter('q_y').value
        q_t = self.get_parameter('q_t').value

        qt_x = self.get_parameter('qt_x').value
        qt_y = self.get_parameter('qt_y').value
        qt_t = self.get_parameter('qt_t').value

        r_v = self.get_parameter('r_v').value
        r_w = self.get_parameter('r_w').value

        f = self.get_parameter('forward').value

        Q = ca.diag([q_x, q_y, q_t])
        Q_termainal = ca.diag([qt_x, qt_y, qt_t])
        R = ca.diag([r_v, r_w])
        cost = 0
        cost_q = 0
        cost_u = 0

        for k in range(self.N):
            err = X[0:2, k] - self.goal_state[0:2]
            cost_q += ca.mtimes(err.T, ca.mtimes(Q[0:2, 0:2], err))
            cost_q += Q[2, 2] * (1 -  ca.cos(X[2, k] - self.goal_state[2]))

            cost_u += ca.mtimes(U[:, k].T, ca.mtimes(R, U[:, k]))
        err_term = X[0:2, self.N] - self.goal_state[0:2]
        cost += ca.mtimes(err_term.T, ca.mtimes(Q_termainal[0:2, 0:2], err_term))
        cost += Q_termainal[2, 2] * (1 - ca.cos(X[2, self.N] - self.goal_state[2]))
        #cost += cost_q + cost_u
        opti.minimize(cost_q + cost_u + cost)


        for k in range(self.N):

            next_state_prediction = self.f_model(X[:, k], U[:, k])
            opti.subject_to(X[:, k+1] == next_state_prediction)

        opti.subject_to(X[:, 0] == self.current_state)

        opti.subject_to(opti.bounded(f, v, 0.22))
        opti.subject_to(opti.bounded(-2.0, w, 2.0))

        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb':'yes'}
        opti.solver('ipopt', opts)


        try:
            sol = opti.solve()
            self.prev_sol_X = sol.value(X)
            self.prev_sol_U = sol.value(U)

            cost_value = float(sol.value(cost_q + cost_u + cost))
            cost_msg = Float32()
            cost_msg.data = cost_value
            self.cost_pub.publish(cost_msg)

            cost_q_value = float(sol.value(cost_q))
            cost_q_msg = Float32()
            cost_q_msg.data = cost_q_value
            self.cost_q_pub.publish(cost_q_msg)

            cost_u_value = float(sol.value(cost_u))
            cost_u_msg = Float32()
            cost_u_msg.data = cost_u_value
            self.cost_u_pub.publish(cost_u_msg)
            
            return sol.value(U)[:, 0]
        except Exception as e:
            print("Solver failed, Stopping...")
            return [0.0, 0.0]
        
    def control_loop(self):
        if self.current_state is None:
            return
        if self.goal_state is None:
            return

        start_time = self.get_clock().now()
        optimal_u = np.array([0.0, 0.0])
        if(np.any(abs(self.goal_state - self.current_state) >= np.array([0.1, 0.1, 0.]))):
            optimal_u = self.solve_mpc()
            self.goal_reached = False
        else:
            self.prev_sol_X = None
            self.prev_sol_U = None
            if not self.goal_reached:
                self.get_logger().info("Goal Reached..!!!")
                self.goal_reached = True


        msg = TwistStamped()
        msg.header.stamp = start_time.to_msg()
        msg.header.frame_id = 'base_link' 
        
        msg.twist.linear.x = float(optimal_u[0])   
        msg.twist.angular.z = float(optimal_u[1])
        self.cmd_vel_pub.publish(msg)

        #duration = (self.get_clock().now() - start_time).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()