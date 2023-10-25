import rclpy
import casadi as ca
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import csv
from casadi import *
import tf_transformations as tf
import math
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.odom_ = Odometry()
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("I have started")

        index = 27
        self.count = 0

        pos = np.array([0.0, 0.0, 0.0])
        self.targets = self.list_targets()
        #self.get_logger().info(f'target: {targets[0]}')
        controller = self.controller(pos)


    def controller(self,pos):
        # NMPC Parameters
        self.T = 0.1
        self.N = 10
        self.Q = np.diag([1.0, 1.0, 1.0]) # Weights for states
        self.R = np.diag([1.0, 1.0]) #Weights for controls

        # Constraints
        self.min_vx = 0.0
        self.max_vx = 0.2

        self.min_omega = -2.0
        self.max_omega = 2.0

        self.max_dvx = 0.2
        self.max_domega = math.pi*1.2

        # Previous states and Controls
        self.next_states = np.ones((self.N+1, 3))*pos
        self.u0 = np.zeros((self.N, 2))
        
        # Setup the controller with parameters
        controller = self.setup_control()

    def setup_control(self):
        self.opti = Opti()

        # State Variables
        self.opt_states = self.opti.variable(self.N+1, 3)
        x = self.opt_states[:,0]
        y = self.opt_states[:,1]
        theta = self.opt_states[:,2]

        # the velocity
        self.opt_controls = self.opti.variable(self.N, 2)
        vx = self.opt_controls[0]
        omega = self.opt_controls[1]

        # create model
        f = lambda x_, u_: ca.vertcat(*[
            ca.cos(x_[1]) * u_[0],               # dx
            ca.sin(x_[1]) * u_[0],               # dy
            u_[1],                               # dtheta (angular velocity)
        ])

        # parameters - reference trajectories of the pose and inputs
        self.opt_u_ref = self.opti.parameter(self.N, 2)
        self.opt_x_ref = self.opti.parameter(self.N+1, 3)

        # initial condition
        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
        for i in range(self.N):
            x_next = self.opt_states[i, :] + f(self.opt_states[i, :], self.opt_controls[i, :]).T*self.T
            self.opti.subject_to(self.opt_states[i+1, :] == x_next)
        
        # cost function
        obj = 0
        for i in range(self.N):
            state_error_ = self.opt_states[i, :] - self.opt_x_ref[i+1, :]
            obj = obj + ca.mtimes([state_error_, self.Q, state_error_.T]) + \
            ca.mtimes([self.opt_controls[i, :], self.R, self.opt_controls[i, :].T])
        self.opti.minimize(obj)


        # constraint about change of velocity
        for i in range(self.N-1):
            dvel = (self.opt_controls[i+1,:] - self.opt_controls[i,:])/self.T
            self.opti.subject_to(self.opti.bounded(-self.max_dvx, dvel[0], self.max_dvx))
            self.opti.subject_to(self.opti.bounded(-self.max_domega, dvel[1], self.max_domega))

        # boundary and control conditions
        self.opti.subject_to(self.opti.bounded(self.min_vx, vx, self.max_vx))
        self.opti.subject_to(self.opti.bounded(self.min_omega, omega, self.max_omega))

        opts_setting = {'ipopt.max_iter':2000,
                        'ipopt.print_level':0,
                        'print_time':0,
                        'ipopt.acceptable_tol':1e-8,
                        'ipopt.acceptable_obj_change_tol':1e-6}

        self.opti.solver('ipopt', opts_setting)

    def solve(self, next_trajectories, next_controls):
        # set parameter, here only update initial state of x (x0)
        self.opti.set_value(self.opt_x_ref, next_trajectories)
        self.opti.set_value(self.opt_u_ref, next_controls)
        
        # provide the initial guess of the optimization targets
        self.opti.set_initial(self.opt_states, self.next_states)
        self.opti.set_initial(self.opt_controls, self.u0)
        
        # solve the problem
        sol = self.opti.solve()
        
        # obtain the control input
        self.u0 = sol.value(self.opt_controls)
        self.next_states = sol.value(self.opt_states)
        return self.u0[0,:]

#Helper functions
    def calculate_distance(self, x1,x2,y1,y2):
        return math.hypot(x2 - x1, y2 - y1)

    def getYaw(self):
        ori_list = [self.odom_.pose.pose.orientation.x,
                    self.odom_.pose.pose.orientation.y,
                    self.odom_.pose.pose.orientation.z,
                    self.odom_.pose.pose.orientation.w]
        roll,pitch,yaw = tf.euler_from_quaternion(ori_list)
        return yaw
    
    def list_targets(self):
        # Process the path
        targets = []
        with open('/home/riser14/Test/reference_path.csv','r') as file:
            reader = csv.reader(file)
            next(reader) #Skips header
            for row in reader:
                x,y,z,o_x,o_y,o_z,o_w = map(float,row)
                yaw = self.getYaw()
                #yaw = math.atan2(2.0*(o_w*o_z), (o_w*o_w - o_z*o_z))
                #yaw = math.atan2(2.0*(o_z*o_w + o_x*o_y), 1.0 - 2.0*(o_y*o_y + o_z*o_z))
                self.get_logger().info(f'Yaw: {yaw}')
                #self.get_logger().info(f'X: {x}, Y: {y}, Yaw: {yaw}')
                targets.append((x,y,yaw))
        return targets

    def true_path(self, N:int, pos):
        # initial state
        x_ = np.zeros((N+1, 3))
        
        targets = self.list_targets()
        traj = np.array(targets)

        x_ref = traj[:,0]
        y_ref = traj[:,1]
        q_ref = traj[:,2]

        x_ref_ = x_ref[:N]
        y_ref_ = y_ref[:N]
        q_ref_ = q_ref[:N]
        length = len(x_ref_)

        if length < N:
            x_ex = np.ones(N - length)*x_ref_[-1]
            x_ref_ = np.concatenate((x_ref_, x_ex), axis=None)

            y_ex = np.ones(N - length)*y_ref_[-1]
            y_ref_ = np.concatenate((y_ref_, y_ex), axis=None)

            q_ex = np.ones(N - length)*q_ref_[-1]
            q_ref_ = np.concatenate((q_ref_, q_ex), axis=None)

        vx_ref_ = np.zeros(N)
        omega_ref_ = np.zeros(N)
        x_ = np.array([x_ref_, y_ref_, q_ref_]).T

        x_ = np.concatenate((np.array([pos]), x_), axis=0)
        #self.get_logger().info(f"X_refs:  {x_}")
        return x_, np.array([vx_ref_, omega_ref_]).T

    def control_commands(self, control_inputs):
        # Publish control inputs as Twist messages
        twist_msg = Twist()
        twist_msg.linear.x = control_inputs[0]
        twist_msg.angular.z = control_inputs[1]
        self.publisher_.publish(twist_msg)

    def correct_state(self,states, tracjectories):
        error = tracjectories - states
        tracjectories = states + error
        return tracjectories

    def odom_callback(self,msg: Odometry):
        #self.get_logger().info(f"Odom Message: \n {msg}")
        self.odom_ = msg
        px = self.odom_.pose.pose.position.x
        py = self.odom_.pose.pose.position.y
        o_x = self.odom_.pose.pose.orientation.x
        o_y = self.odom_.pose.pose.orientation.y
        o_z = self.odom_.pose.pose.orientation.z
        o_w = self.odom_.pose.pose.orientation.w
        yaw = self.getYaw()
        #yaw = math.atan2(2.0*(o_z*o_w + o_x*o_y), 1.0 - 2.0*(o_y*o_y + o_z*o_z))
        pos = np.array([px, py, yaw])
        self.get_logger().info(f'Current position: {pos}')

        #Current target coords
        x_t, y_t, yaw_t = self.targets[self.count] 
        self.get_logger().info(f'Target: X: {x_t}, Y: {y_t}')
        self.get_logger().info(f'Current distance to waypoint: {self.calculate_distance(px,x_t,py,y_t)}')
        if self.calculate_distance(px,x_t,py,y_t) > 0.1:
            #Obtain the waypoint, control at that waypoint(zero) and the array of targets
            next_point, next_control = self.true_path(self.N,pos)
            next_point = self.correct_state(self.next_states,next_point)
            # Solve the NMPC problem
            control_inputs = self.solve(next_point,next_control)
            # Publish the control commands
            self.control_commands(control_inputs)
        else:
            self.count +=1

def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



