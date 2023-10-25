import rclpy
import casadi as ca
import math
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

import csv

from std_msgs.msg import Float64MultiArray
from casadi import *
import tf_transformations as tf

class NMPC_Controller(Node):

    def __init__(self):
        super().__init__('nmpc_controller')
        self.odom_ = Odometry()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)


    def setup(self):
        # NMPC Parameters
        self.dt = 0.1 # 10Hz
        self.N = 10 # Horizon
        self.nx = 3 # States
        self.nu = 2 # Controls
        self.opti = ca.Opti()

        # Constraints
        MAX_LINEAR_VELOCITY = 0.2 # m/s
        MAX_ANGULAR_VELOCITY = 2.0 # rad/s
        MAX_DELTA_VELOCITY = 1.0 # m/s
        MAX_DELTA_OMEGA = pi/2.0 # rad\/s

        # Symbolic Variables
        self.x = self.opti.variable(self.nx, self.N+1)
        self.u = self.opti.variable(self.nu, self.N)
        self.x = self.opti.variable(self.nx, 1)

        self.x_ref = self.opti.variable(3, self.N+1)
        self.u_ref = self.opti.variable(2, self.N+1)

        # Objective function as a quadratic for state and control
        Q = self.opti.parameter(self.nx, self.nx)
        R = self.opti.parameter(self.nu, self.nu)

        J = ca.dot(self.x - self.x_ref, ca.mtimes(Q, self.x - self.x_ref)) 
        + ca.dot(self.u, ca.mtimes(R, self.u)) #Remeber u_ref is 0

         # Define the control horizons
        for i in range(self.T):
            self.opti.subject_to(self.x[:, i + 1] == self.x[:, i] + self.dynamics(self.x[:, i], self.u[:, i], self.dt))

        for i in range(self.T - 1):
            dvel = self.u[:, i + 1] - self.u[:, i] / self.dt
            self.opti.subject_to(ca.bounded(-MAX_DELTA_VELOCITY, dvel[0], MAX_DELTA_VELOCITY))
            self.opti.subject_to(ca.bounded(-MAX_DELTA_OMEGA, dvel[1], MAX_DELTA_OMEGA))

        self.opti.subject_to(ca.bounded(-MAX_LINEAR_VELOCITY, self.u[0], MAX_LINEAR_VELOCITY))
        self.opti.subject_to(ca.bounded(-MAX_ANGULAR_VELOCITY, self.u[1], MAX_ANGULAR_VELOCITY))

        self.opti.subject_to(ca.bounded(-2.07, self.x[0], 2.07))
        self.opti.subject_to(ca.bounded(-2.9, self.x[1], 2.9))
        self.opti.subject_to(ca.bounded(-ca.pi, self.x[2], ca.pi))
        self.opti.subject_to(self.x[:, 0] == self.p)

        solver_options = {
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": 2000,
            "ipopt.tol": 1e-8,
            "print_time": 0,
            "ipopt.acceptable_obj_change_tol": 1e-6,
        }
        self.opti.solver("ipopt", solver_options)

        # Set initial values
        Q_value = ca.DM.diag(ca.vertcat([1.0, 1.0, 1.0]))
        R_value = ca.DM.diag(ca.vertcat([1.0, 1.0]))
        self.opti.set_value(Q, Q_value)
        self.opti.set_value(R, R_value)
        self.opti.set_value(self.x_ref, ca.DM.zeros(3, self.T + 1))
        self.opti.set_value(self.u_ref, ca.DM.zeros(2, self.T))
        self.u_init = ca.DM.repmat([0, 0], 1, self.T)
        self.x_init = ca.DM.repmat([0.0, 0.0, 0.0], 1, self.T + 1)

    def dynamics(self, x, u, dt):
        xdot = ca.vertcat([u[0] * ca.cos(x[2]), u[0] * ca.sin(x[2]), u[1]])
        return xdot * dt

    def solve(self, x0):
        # Set the initial guess
        self.opti.set_initial(self.x, self.x_init)
        self.opti.set_initial(self.u, self.u_init)

        # Set the initial state
        self.opti.set_value(self.p, x0)

        # Solve the optimization problem
        solution = self.opti.solve()

        # Get the optimal control u and push it into the control vector
        u0 = solution.value(self.u)[:, 0].full()
        control = u0.tolist()

        # Warm start
        self.x_init = ca.DM.repmat(solution.value(self.x)[:, 0], 1, self.T + 1)
        self.u_init = solution.value(self.u)[:, :].full()

        return control

    def set_reference(self, x_ref, u_ref):
        # Adding each column of the reference trajectory to the decision variable
        self.opti.set_value(self.x_ref, ca.DM.repmat(x_ref, 1, self.T + 1))
        self.opti.set_value(self.u_ref, ca.DM.repmat(u_ref, 1, self.T))

    def set_dt(self, dt):
        self.dt = dt

#Helper functions
    def csv_to_path(self):
        index = 0
        targets = []
        with open('/home/riser14/Test/odom_true_path.csv','r') as file:
            reader = csv.reader(file)
            next(reader) #Skips header
            for row in reader:
                x,y,z,o_x,o_y,o_z,o_w = map(float,row)
                yaw = math.atan2(2.0*(o_z*o_w + o_x*o_y), 1.0 - 2.0*(o_y*o_y + o_z*o_z))
                targets.append((x,y,yaw))
                index +=1
            traj = np.array(targets)
            x_ref = traj[:,0]
            y_ref = traj[:,1]
            q_ref = traj[:,2]
        return targets, index

    def true_path(self):
        # Process the path
        targets = []
        with open('/home/riser14/Test/odom_true_path.csv','r') as file:
            reader = csv.reader(file)
            next(reader) #Skips header
            for row in reader:
                x,y,z,o_x,o_y,o_z,o_w = map(float,row)
                yaw = math.atan2(2.0*(o_z*o_w + o_x*o_y), 1.0 - 2.0*(o_y*o_y + o_z*o_z))
                targets.append((x,y,yaw))
            traj = np.array(targets)
        return traj

    def control_commands(self, control_inputs):
        # Publish control inputs as Twist messages
        twist_msg = Twist()
        twist_msg.linear.x = control_inputs[0]
        twist_msg.angular.z = control_inputs[1]
        self.publisher_.publish(twist_msg)

    def odom_callback(self,msg: Odometry):
        self.odom_ = msg


def main(args=None):
    rclpy.init(args=args)
    nmpc_controller = NMPC_Controller()
    rclpy.spin(nmpc_controller)
    nmpc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
