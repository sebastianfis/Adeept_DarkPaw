import numpy as np
import matplotlib.pyplot as plt


class FourBarLinkage:
    """
    Class to implement a four bar linkage of such a type:

                  l_ab
             A __________ B
             /            \
     l_sa   /              \ l_gb
           /  phi     theta \
         S ----------------- G
                l_sg
    S denotes the servo axis position. G the position of the joint axis. The angles phi and theta are defined positive
    from the connecting line between servo axis S and joint axis G.
    """

    def __init__(self, l_sg, l_sa, l_ab, l_gb, phi_0: float = 90):
        self.l_sg = l_sg
        self.l_sa = l_sa
        self.l_ab = l_ab
        self.l_gb = l_gb
        self.phi_max = np.pi
        self.phi_min = -np.pi
        self.theta_max = np.pi
        self.theta_min = -np.pi
        self.cur_phi = np.deg2rad(phi_0)
        self.calc_limits(self.cur_phi)
        self.cur_theta = self.calc_theta(self.cur_phi)
        self.phi_0 = self.cur_phi
        self.pwm_0 = 330
        self.actuator_direction = 1

    def set_pwm_init(self, pwm_value, actuator_direction):
        """
        method to set the initial pwm value, at which the actuator angle is phi_0 and the correct direction!
        """
        assert actuator_direction == 1 or actuator_direction == -1, "actuator_direction must be 1 or -1!"
        self.pwm_0 = pwm_value
        self.actuator_direction = actuator_direction

    def calc_PWM(self, phi: float) -> int:
        """method to calculate pwm value from set phi value (in rad)"""
        pwm_value = self.pwm_0 + self.actuator_direction * (phi - self.phi_0) * 460 / np.pi
        if pwm_value < 100:
            pwm_value = 100
        elif pwm_value > 560:
            pwm_value = 560
        return int(pwm_value)

    def calc_theta(self, phi):
        """
        method to calculate theta (in rad) from phi (in rad)
        """
        l_ag = np.sqrt(self.l_sa ** 2 + self.l_sg ** 2 - 2 * self.l_sa * self.l_sg * np.cos(phi))
        assert self.phi_min <= phi <= self.phi_max, \
            'Movement not possible, because l_ag > l_ab + l_gb: {0} > {1} + {2}'.format(l_ag, self.l_ab, self.l_gb)
        # avoid errors due to rounding issues (e.g. arccos(1.00000000000000004) by clipping to -1, 1 range)
        theta_1 = np.arccos(np.clip((self.l_sg ** 2 + l_ag ** 2 - self.l_sa ** 2) / (2 * self.l_sg * l_ag), -1, 1))
        theta_2 = np.arccos(np.clip((self.l_gb ** 2 + l_ag ** 2 - self.l_ab ** 2) / (2 * self.l_gb * l_ag), -1, 1))
        theta = theta_1 + theta_2
        return theta

    def calc_phi(self, theta):
        """
        method to calculate phi (in rad) from theta (in rad)
        """
        l_bs = np.sqrt(self.l_gb ** 2 + self.l_sg ** 2 - 2 * self.l_gb * self.l_sg * np.cos(theta))
        assert self.theta_min <= theta <= self.theta_max, \
            'Movement not possible, because l_bs > l_ab + l_sa: {0} > {1} + {2}'.format(l_bs, self.l_ab, self.l_sa)
        # avoid errors due to rounding issues (e.g. arccos(1.00000000000000004) by clipping to -1, 1 range)
        phi_1 = np.arccos(np.clip((self.l_sg ** 2 + l_bs ** 2 - self.l_gb ** 2) / (2 * self.l_sg * l_bs), -1, 1))
        phi_2 = np.arccos(np.clip((self.l_sa ** 2 + l_bs ** 2 - self.l_ab ** 2) / (2 * self.l_sa * l_bs), -1, 1))
        phi = phi_1 + phi_2
        return phi


    def calc_limits(self, phi_0):
        """Sets the movement limits to meaningful values defined by a) boundaries of the actuator of +/- 90 deg from the
        initial position. b) by calculating the angle at which the connected bars are in a straight line. Further
        movement would then revert the direction of a change in the other angle!"""
        l_ag_max = self.l_ab + self.l_gb
        phi_max = np.arccos(np.clip((self.l_sa ** 2 + self.l_sg ** 2 - l_ag_max ** 2) / (2 * self.l_sa * self.l_sg),
                                    -1, 1))
        if phi_max > phi_0 + np.pi / 2:  # Correct value, if it is out of servo movement range!
            self.phi_max = phi_0 + np.pi / 2
        else:
            self.phi_max = phi_max

        self.theta_min = self.calc_theta(self.phi_max)

        l_bs_max = self.l_ab + self.l_sa
        theta_max = np.arccos(np.clip((self.l_gb ** 2 + self.l_sg ** 2 - l_bs_max ** 2) / (2 * self.l_gb * self.l_sg),
                                      -1, 1))
        # avoid errors due to rounding issues (e.g. arccos(1.00000000000000004)) by clipping to -1, 1 range)
        self.theta_max = theta_max
        phi_min = self.calc_phi(self.theta_max)
        if phi_min < phi_0 - np.pi / 2:
            self.phi_min = phi_0 - np.pi / 2
        else:
            self.phi_min = phi_min

    def plot_current_state(self, ax=None, linestyle='-', color='blue', cs_rot_angle: float = 30,
                           g_offset=None, inv_x=1, inv_y=1, inv_z=1, act1_theta=0):
        if g_offset is None:
            g_offset = [0, 0, 0]
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        r_list = np.array([0, -self.l_sg, -self.l_sg + self.l_sa * np.cos(self.cur_phi),
                           - self.l_gb * np.cos(self.cur_theta), 0])
        z_list = np.array([0, 0, self.l_sa * np.sin(self.cur_phi),
                           self.l_gb * np.sin(self.cur_theta), 0])
        rot = np.array([[np.cos(np.deg2rad(-cs_rot_angle)), -np.sin(np.deg2rad(-cs_rot_angle))],
                        [np.sin(np.deg2rad(-cs_rot_angle)), np.cos(np.deg2rad(-cs_rot_angle))]])
        [r_list, z_list] = np.dot(rot, [r_list, z_list])
        x_list = inv_x * r_list * np.cos(act1_theta) + g_offset[0]
        y_list = inv_y * r_list * np.sin(act1_theta) + g_offset[1]
        z_list = inv_z * z_list + g_offset[2]
        ax.plot(x_list, y_list, z_list, linestyle=linestyle, color=color)
        return ax


class SpiderLeg:
    """Class that implements one single leg of the robot. All values are measured at the parts and should not require
    to be changed on object construction. Exception is the joint position (x_j,y_j) and the direction the leg is facing
    (dir_x, dir_y). Makes use of the FourBarLinkage class to implement the actuator mechanics:
    self.actuator1: Mechanism to rotate the leg around its joint
    self.actuator2: Mechanism to lift the leg (is coupled to actuator 3!!!)
    self.actuator3: Mechanism to extend the leg (is coupled to actuator 2!!!)"""

    def __init__(self, x_j=43.5, y_j=42, dir_x=1, dir_y=1, name=None,
                 r_g=66.5, z_g=-23, l_gp=30.5, l_pf=78.2,
                 psi_0=131.5, xi_0=147.5, theta_0=90 - 9.5, theta_leg=33.7):
        self.name = name
        self.actuator1 = FourBarLinkage(l_sg=42.5, l_sa=14.5, l_ab=38, l_gb=27.8, phi_0=90)
        self.actuator2 = FourBarLinkage(l_sg=35.6, l_sa=14.5, l_ab=35.6, l_gb=25.6, phi_0=99.5)
        self.actuator3 = FourBarLinkage(l_sg=35.6, l_sa=14.5, l_ab=26, l_gb=38.5, phi_0=99.5)
        self.x_j = x_j
        self.y_j = y_j
        self.dir_x = dir_x
        self.dir_y = dir_y
        self.r_g = r_g
        self.z_g = z_g
        self.l_gp = l_gp
        self.l_pf = l_pf
        self.psi_0 = np.deg2rad(psi_0)
        self.xi_0 = np.deg2rad(xi_0)
        self.theta_0 = np.deg2rad(theta_0)
        self.theta_leg = np.deg2rad(theta_leg)
        self.init_x_f, self.init_y_f, self.init_z_f = self.forward_transform(self.actuator1.cur_phi,
                                                                             self.actuator2.cur_phi,
                                                                             self.actuator3.cur_phi)
        self.cur_x_f = None
        self.cur_y_f = None
        self.cur_z_f = None
        self.init_phi = np.arctan2(self.init_x_f, self.init_y_f)
        self.update_cur_phi(self.init_x_f, self.init_y_f, self.init_z_f)

    def forward_transform(self, phi_1, phi_2, phi_3):
        """
        method to calculate the absolute position of the leg foot (with reference to the robots COG)
        from the actuator angles phi_1, phi_2, phi_3 (in rad)
        """
        theta_1 = self.actuator1.calc_theta(phi_1)
        theta_2 = self.actuator2.calc_theta(phi_2)
        theta_3 = self.actuator3.calc_theta(phi_3)
        r_f = self.r_g + self.l_gp * np.cos(theta_2 - self.theta_0) + \
            self.l_pf * np.cos(self.xi_0 - self.psi_0 + theta_3)
        z_f = self.z_g - self.l_gp * np.sin(theta_2 - self.theta_0) + \
            self.l_pf * np.sin(self.xi_0 - self.psi_0 + theta_3)
        x_f = self.x_j - self.dir_x * r_f * np.cos(theta_1 + self.theta_leg)
        y_f = self.y_j + self.dir_y * r_f * np.sin(theta_1 + self.theta_leg)
        return x_f, y_f, z_f

    def backward_transform(self, x_f, y_f, z_f):
        """
        method to calculate the required actuator angles phi_1, phi_2, phi_3 (in rad) to set the absolute position of
        the leg foot (with reference to the robots COG).
        """
        r_f = np.sqrt((x_f - self.x_j) ** 2 + (y_f - self.y_j) ** 2)
        theta_1 = np.arccos((self.x_j - x_f) / self.dir_x / r_f) - self.theta_leg
        phi_1 = self.actuator1.calc_phi(theta_1)
        vec_length = (r_f - self.r_g) ** 2 + (z_f - self.z_g) ** 2
        theta_2 = np.arccos((self.l_gp ** 2 - self.l_pf ** 2 + vec_length) / (2 * self.l_gp * np.sqrt(vec_length))) - \
            np.arctan2(z_f - self.z_g, r_f - self.r_g) + self.theta_0
        phi_2 = self.actuator2.calc_phi(theta_2)
        theta_3 = np.arccos((r_f - self.r_g - self.l_gp * np.cos(theta_2 - self.theta_0)) / self.l_pf) - self.xi_0 + \
            self.psi_0
        phi_3 = self.actuator3.calc_phi(theta_3)
        return phi_1, phi_2, phi_3

    def calc_trajectory(self, vector_list: list):
        """
        method to calculate the required actuator angles phi_1, phi_2, phi_3 (in rad) to follow a given trajectory of
        the leg foot (with reference to the robots COG), passed to the method as a list of vectors
        """
        angle_setting_list = []
        for vector in vector_list:
            angle_setting_list.append(self.backward_transform(vector[0], vector[1], vector[2]))
        return angle_setting_list

    def calc_PWM(self, angle_setting_list: list):
        """
        method to calculate the required actuator PWM to set the actuator angles phi_1, phi_2, phi_3 (in rad),
        passed to the method as a list of vectors
        """
        pwm_setting_list = []
        for vector in angle_setting_list:
            pwm_setting_list.append([self.actuator1.calc_PWM(vector[0]),
                                     self.actuator2.calc_PWM(vector[1]),
                                     self.actuator3.calc_PWM(vector[2])])
        return pwm_setting_list

    def update_actuator_angles(self, phi_1, phi_2, phi_3):
        """
        method to update the current actuator angles
        """
        self.actuator1.cur_phi, self.actuator2.cur_phi, self.actuator3.cur_phi = phi_1, phi_2, phi_3
        self.actuator1.cur_theta = self.actuator1.calc_theta(phi_1)
        self.actuator2.cur_theta = self.actuator2.calc_theta(phi_2)
        self.actuator3.cur_theta = self.actuator3.calc_theta(phi_3)

    def update_cur_pos(self, phi_1, phi_2, phi_3):
        """
        method to update the currently stored actuator angles from the current foot coordinates
        """
        self.cur_x_f, self.cur_y_f, self.cur_z_f = self.forward_transform(phi_1, phi_2, phi_3)
        self.update_actuator_angles(phi_1, phi_2, phi_3)

    def update_cur_phi(self, x_f, y_f, z_f):
        """
        method to update the currently stored foot coordinates from the current actuator angles
        """
        self.cur_x_f, self.cur_y_f, self.cur_z_f = x_f, y_f, z_f
        phi_1, phi_2, phi_3 = self.backward_transform(self.cur_x_f, self.cur_y_f, self.cur_z_f)
        self.update_actuator_angles(phi_1, phi_2, phi_3)

    def visualize_state(self, phi_1, phi_2, phi_3, ax=None, linestyle='-', color='blue'):
        """
        method to display the leg state with the set coordinates phi_1, phi_2, phi_3 in a matplotlib 3d plot
        """
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        self.update_actuator_angles(phi_1, phi_2, phi_3)
        x_g = self.x_j - self.dir_x * self.r_g*np.cos(self.actuator1.cur_theta + self.theta_leg)
        y_g = self.y_j + self.dir_y * self.r_g*np.sin(self.actuator1.cur_theta + self.theta_leg)
        ax = self.actuator2.plot_current_state(ax=ax, color=color, linestyle=linestyle,
                                               cs_rot_angle=9.5, g_offset=[x_g, y_g, self.z_g],
                                               inv_x=-self.dir_x, inv_y=-self.dir_y,
                                               act1_theta=-self.actuator1.cur_theta - self.theta_leg)
        ax = self.actuator3.plot_current_state(ax=ax, color=color, linestyle=linestyle, cs_rot_angle=9.5,
                                               g_offset=[x_g, y_g, self.z_g],
                                               inv_x=-self.dir_x, inv_y=-self.dir_y, inv_z=-1,
                                               act1_theta=-self.actuator1.cur_theta - self.theta_leg)
        parallelogram = FourBarLinkage(self.l_gp, self.actuator3.l_gb, self.l_gp,
                                       self.actuator3.l_gb, phi_0=float(np.degrees(self.psi_0 -
                                                                                   self.actuator3.cur_theta -
                                                                                   self.actuator2.cur_theta +
                                                                                   self.theta_0)))
        z_p = self.z_g - self.l_gp * np.sin(self.actuator2.cur_theta - self.theta_0)
        r_p = self.l_gp * np.cos(self.actuator2.cur_theta - self.theta_0)
        x_p = x_g - self.dir_x*r_p*np.cos(self.actuator1.cur_theta + self.theta_leg)
        y_p = y_g + self.dir_y*r_p*np.sin(self.actuator1.cur_theta + self.theta_leg)
        ax = parallelogram.plot_current_state(ax=ax, color=color, linestyle=linestyle,
                                              cs_rot_angle=-np.degrees(self.actuator2.cur_theta - self.theta_0),
                                              g_offset=[x_p, y_p, z_p],
                                              inv_x=-self.dir_x, inv_y=-self.dir_y, inv_z=-1,
                                              act1_theta=-self.actuator1.cur_theta - self.theta_leg)
        z_2b = self.z_g + self.actuator2.l_gb * np.sin(self.actuator2.cur_theta + np.deg2rad(9.5))
        r_2b = -self.actuator2.l_gb * np.cos(self.actuator2.cur_theta + np.deg2rad(9.5))
        x_2b = x_g - self.dir_x * r_2b * np.cos(self.actuator1.cur_theta + self.theta_leg)
        y_2b = y_g + self.dir_y * r_2b * np.sin(self.actuator1.cur_theta + self.theta_leg)

        r_e = self.actuator3.l_gb * np.cos(self.psi_0 - self.actuator3.cur_theta)
        x_e = x_p - self.dir_x * r_e * np.cos(self.actuator1.cur_theta + self.theta_leg)
        y_e = y_p + self.dir_y * r_e * np.sin(self.actuator1.cur_theta + self.theta_leg)
        z_e = z_p - self.actuator3.l_gb * np.sin(self.psi_0 - self.actuator3.cur_theta)

        x_f, y_f, z_f = self.forward_transform(phi_1, phi_2, phi_3)
        # r_f = np.sqrt((x_f - self.x_j) ** 2 + (y_f - self.y_j) ** 2)

        ax.plot([self.x_j, self.x_j, x_g, self.x_j],
                [self.y_j, self.y_j, y_g, self.y_j],
                [0, -46, self.z_g, 0],
                linestyle=linestyle, color=color)
        ax.plot([x_2b, x_p, x_f, x_e],
                [y_2b, y_p, y_f, y_e],
                [z_2b, z_p, z_f, z_e],
                linestyle=linestyle, color=color)
        return ax


class RobotModel:
    def __init__(self, init_pwm,
                 LF2_direction=1, LF3_direction=-1, LF1_direction=-1, RF2_direction=-1, RF3_direction=1,
                 RF1_direction=1, LB2_direction=-1, LB3_direction=1, LB1_direction=-1, RB2_direction=1,
                 RB3_direction=-1, RB1_direction=1):

        # Initiate the legs
        self.left_forward_leg = SpiderLeg(x_j=43.5, y_j=-42, dir_x=1, dir_y=-1, name='LFL')
        self.left_backward_leg = SpiderLeg(x_j=-43.5, y_j=-42, dir_x=-1, dir_y=-1, name='LBL')
        self.right_forward_leg = SpiderLeg(x_j=43.5, y_j=42, dir_x=1, dir_y=1, name='RFL')
        self.right_backward_leg = SpiderLeg(x_j=-43.5, y_j=42, dir_x=-1, dir_y=1, name='RBL')
        self.legs = [self.left_forward_leg,
                     self.left_backward_leg,
                     self.right_forward_leg,
                     self.right_backward_leg]
        self.left_forward_leg.actuator1.set_pwm_init(init_pwm['LF1_init_pwm'], LF1_direction)
        self.left_forward_leg.actuator2.set_pwm_init(init_pwm['LF2_init_pwm'], LF2_direction)
        self.left_forward_leg.actuator3.set_pwm_init(init_pwm['LF3_init_pwm'], LF3_direction)
        self.left_backward_leg.actuator1.set_pwm_init(init_pwm['LB1_init_pwm'], LB1_direction)
        self.left_backward_leg.actuator2.set_pwm_init(init_pwm['LB2_init_pwm'], LB2_direction)
        self.left_backward_leg.actuator3.set_pwm_init(init_pwm['LB3_init_pwm'], LB3_direction)
        self.right_forward_leg.actuator1.set_pwm_init(init_pwm['RF1_init_pwm'], RF1_direction)
        self.right_forward_leg.actuator2.set_pwm_init(init_pwm['RF2_init_pwm'], RF2_direction)
        self.right_forward_leg.actuator3.set_pwm_init(init_pwm['RF3_init_pwm'], RF3_direction)
        self.right_backward_leg.actuator1.set_pwm_init(init_pwm['RB1_init_pwm'], RB1_direction)
        self.right_backward_leg.actuator2.set_pwm_init(init_pwm['RB2_init_pwm'], RB2_direction)
        self.right_backward_leg.actuator3.set_pwm_init(init_pwm['RB3_init_pwm'], RB3_direction)

        # set the basic movement constants (from kinematic limits
        self.step_length_x = 80
        self.step_length_y = 20
        self.step_length_turn = 70
        self.step_height_x = 14
        self.step_height_y = 14
        self.step_height_turn = 14
        self.update_freq = 50  # how often to update the PWM data
        n_min_x = 8
        n_min_y = 4

        # calculate max velocities per direction
        self.v_x_max = self.step_length_x * self.update_freq / 4 / n_min_x
        self.v_y_max = self.step_length_y * self.update_freq / 4 / n_min_y
        self.v_t_max = self.step_length_turn * self.update_freq / 4 / n_min_x

        # initiate gait list
        self.gaits = [Gait(self, step_length=self.step_length_x, velocity=self.v_x_max, freq=self.update_freq,
                           turn_movement=False, direction='+x', name='move_forward'),
                      Gait(self, step_length=self.step_length_x, velocity=self.v_x_max, freq=self.update_freq,
                           turn_movement=False, direction='-x', name='move_backward'),
                      Gait(self, step_length=self.step_length_y, velocity=self.v_y_max, freq=self.update_freq,
                           turn_movement=False, direction='+y', name='move_right'),
                      Gait(self, step_length=self.step_length_y, velocity=self.v_y_max, freq=self.update_freq,
                           turn_movement=False, direction='-y', name='move_left'),
                      Gait(self, step_length=self.step_length_turn, velocity=self.v_t_max, freq=self.update_freq,
                           turn_movement=True, direction='-', name='turn_right'),
                      Gait(self, step_length=self.step_length_turn, velocity=self.v_t_max, freq=self.update_freq,
                           turn_movement=True, direction='+', name='turn_left')]

        self.poses = [Pose(self, self.calc_leg_pos_from_body_angles(0, 0), n=2, name='neutral'),
                      Pose(self, self.calc_leg_pos_from_body_angles(10, 0), n=2, name='look_up'),
                      Pose(self, self.calc_leg_pos_from_body_angles(-10, 0), n=2, name='look_down'),
                      Pose(self, self.calc_leg_pos_from_body_angles(0, 5.5), n=2, name='lean_right'),
                      Pose(self, self.calc_leg_pos_from_body_angles(0, -5.5), n=2, name='lean_left'),
                      Pose(self, self.calc_leg_pos_from_body_angles(0, 0, z_0=69), n=2, name='high'),
                      Pose(self, self.calc_leg_pos_from_body_angles(0, 0, z_0=37), n=2, name='low')]
        for leg in self.legs:
            self.poses.append(Pose(self, self.calc_lifted_leg_pos(leg.name, z_0=37), n=2, name='lift_'+leg.name))

    @staticmethod
    def generate_step(starting_point: tuple, end_point: tuple, step_height: float, n: int):
        """
        method to calculate a foot coordinate trajectory follwing a half-ellipse representing a step
        """
        l_step = np.sqrt((end_point[0] - starting_point[0]) ** 2 + (end_point[1] - starting_point[1]) ** 2)
        x = np.linspace(starting_point[0], end_point[0], n)
        y = np.linspace(starting_point[1], end_point[1], n)
        s = np.linspace(0, l_step, n)
        z = max([starting_point[2], end_point[2]]) - np.sqrt(1 - 4 * (s - l_step / 2) ** 2 / l_step ** 2) * step_height
        return x, y, z

    @staticmethod
    def generate_straight_line(starting_point: tuple, end_point: tuple, n: int):
        """
        method to calculate a foot coordinate trajectory following a straigth line
        """
        x = np.linspace(starting_point[0], end_point[0], n)
        y = np.linspace(starting_point[1], end_point[1], n)
        z = np.linspace(starting_point[2], end_point[2], n)
        return x, y, z

    @staticmethod
    def generate_partial_circle(starting_point: tuple, end_point: tuple, n: int):
        """
        method to calculate a foot coordinate trajectory following a circle segment around the robot COG
        """
        r = np.sqrt(starting_point[0] ** 2 + starting_point[1] ** 2)
        r_end = np.sqrt(end_point[0] ** 2 + end_point[1] ** 2)
        assert np.isclose(r, r_end), 'end_point and start_point have different radius with respect to COG!'
        phi_0 = np.arctan2(starting_point[1], starting_point[0])
        phi_end = np.arctan2(end_point[1], end_point[0])
        phi = np.linspace(phi_0, phi_end, n)
        x = r * np.cos(phi)
        y = r * np.sin(phi)
        z = np.linspace(starting_point[2], end_point[2], n)
        return x, y, z

    def calc_reset_move(self):
        """
        method to calculate a step sequence to reset all robot legs to their init values
        """
        n = int(self.update_freq/4)
        gait_list = []
        for moving_leg in self.legs:
            if not (np.isclose(moving_leg.cur_x_f, moving_leg.init_x_f) and
                    np.isclose(moving_leg.cur_y_f, moving_leg.init_y_f) and
                    np.isclose(moving_leg.cur_z_f, moving_leg.init_z_f)):
                movement_dict = {}
                for leg in self.legs:
                    if moving_leg.name == leg.name:
                        x, y, z = self.generate_step((moving_leg.cur_x_f,
                                                      moving_leg.cur_y_f,
                                                      moving_leg.cur_z_f),
                                                     (moving_leg.init_x_f,
                                                      moving_leg.init_y_f,
                                                      moving_leg.init_z_f),
                                                     12, n=n)
                        movement_dict[leg.name] = list(zip(x, y, z))
                        moving_leg.update_cur_phi(moving_leg.init_x_f, moving_leg.init_y_f, moving_leg.init_z_f)
                    else:
                        movement_dict[leg.name] = list(zip(np.ones(n) * leg.cur_x_f,
                                                           np.ones(n) * leg.cur_y_f,
                                                           np.ones(n) * leg.cur_z_f))
                    movement_dict[leg.name + '_PWM'] = leg.calc_PWM(leg.calc_trajectory(movement_dict[leg.name]))

                gait_list.append(movement_dict)
        return gait_list

    def set_velocity(self, perc):
        """
        method to change the robot velocity setting between 10 and 100 % of vmax
        """
        assert 10 <= perc <= 100, 'set value must be between 10 and 100 %!'
        self.gaits = [Gait(self, step_length=self.step_length_x, velocity=self.v_x_max * perc / 100,
                           freq=self.update_freq, turn_movement=False, direction='+x', name='move_forward'),
                      Gait(self, step_length=self.step_length_x, velocity=self.v_x_max * perc / 100,
                           freq=self.update_freq, turn_movement=False, direction='-x', name='move_backward'),
                      Gait(self, step_length=self.step_length_y, velocity=self.v_y_max * perc / 100,
                           freq=self.update_freq, turn_movement=False, direction='+y', name='move_right'),
                      Gait(self, step_length=self.step_length_y, velocity=self.v_y_max * perc / 100,
                           freq=self.update_freq, turn_movement=False, direction='-y', name='move_left'),
                      Gait(self, step_length=self.step_length_turn, velocity=self.v_t_max * perc / 100,
                           freq=self.update_freq, turn_movement=True, direction='-', name='turn_right'),
                      Gait(self, step_length=self.step_length_turn, velocity=self.v_t_max * perc / 100,
                           freq=self.update_freq, turn_movement=True, direction='+', name='turn_left')]
        return True

    def get_body_angles(self):
        """
        method to calculate the current body angles from the leg positions
        """
        # it could happen, that this method is called, when one leg is airborne, so it should be checked, that all legs
        # are approx. in one plane -> solution: calculate for left/right and fron/back and choose smaller value!
        theta_x_l = np.arctan((self.left_forward_leg.cur_z_f - self.left_backward_leg.cur_z_f) /
                              (self.left_forward_leg.cur_x_f - self.left_backward_leg.cur_x_f))
        theta_x_r = np.arctan((self.right_forward_leg.cur_z_f - self.right_backward_leg.cur_z_f) /
                              (self.right_forward_leg.cur_x_f - self.right_backward_leg.cur_x_f))
        theta_y_f = np.arctan((self.right_forward_leg.cur_z_f - self.left_forward_leg.cur_z_f) /
                              (self.right_forward_leg.cur_y_f - self.left_forward_leg.cur_y_f))
        theta_y_b = np.arctan((self.right_backward_leg.cur_z_f - self.left_backward_leg.cur_z_f) /
                              (self.right_backward_leg.cur_y_f - self.left_backward_leg.cur_y_f))
        theta_x = np.rad2deg(min(theta_x_r, theta_x_l))
        theta_y = np.rad2deg(max(theta_y_f, theta_y_b))
        return theta_x, theta_y

    def calc_leg_pos_from_body_angles(self, theta_x, theta_y, z_0=None):
        """
        method to calculalate the leg position dict for a given set of body angles in deg around a z_0 position. Can be
        used to raise/lower the robot body and set all possible angles
        """
        if z_0 is None:
            z_0 = np.mean([self.left_forward_leg.cur_z_f,
                          self.right_forward_leg.cur_z_f,
                          self.left_backward_leg.cur_z_f,
                          self.right_backward_leg.cur_z_f])
        leg_pos_dict = {}
        for leg in self.legs:
            leg_pos_dict[leg.name] = tuple([leg.cur_x_f*np.cos(np.deg2rad(theta_x)),
                                            leg.cur_y_f*np.cos(np.deg2rad(theta_y)),
                                            z_0 + np.tan(np.deg2rad(theta_x))*leg.cur_x_f +
                                            np.tan(np.deg2rad(theta_y))*leg.cur_y_f])
        return leg_pos_dict

    def calc_lifted_leg_pos(self, leg_name, z_0: float):
        """
        method to calculalate the leg position dict for to lift a single leg to z_0 position.
        """
        assert z_0 < np.mean([self.left_forward_leg.cur_z_f, self.right_forward_leg.cur_z_f,
                              self.left_backward_leg.cur_z_f, self.right_backward_leg.cur_z_f]), \
            'z_0 must be smaller than average leg z position'
        leg_pos_dict = {}
        for leg in self.legs:
            if leg.name == leg_name:
                leg_pos_dict[leg.name] = tuple([leg.cur_x_f, leg.cur_y_f, z_0])
            else:
                leg_pos_dict[leg.name] = tuple([leg.cur_x_f, leg.cur_y_f, leg.cur_z_f])
        return leg_pos_dict


class Gait:
    def __init__(self, robot_model: RobotModel, step_length: float, velocity: float, freq=50,
                 turn_movement=False, direction='+x', name='None'):
        self.robot_model = robot_model
        self.velocity = velocity  # target velocity in mm/s
        self.T = step_length / 4 / velocity  # Dauer kompletter Schrittzyklus
        self.freq = freq  # Update Frequenz für die Servos. Annahme: ~ PWM Frequenz
        self.n = int(self.T * freq)  # number of samples per step
        self.init_gait_list = []
        self.gait_list = []
        self.name = name

        if '+' in direction:
            multiplier = 1
        elif '-' in direction:
            multiplier = -1
        else:
            raise ValueError('direction must be "+x", "-x", "+y" or "-y", when turn_movement is False or "+","-",' +
                             ' when turn_movement is True')
        if not turn_movement:
            if 'x' in direction:
                self.step_length_x = multiplier * step_length
                self.step_length_y = 0
                self.step_height = self.robot_model.step_height_x
            elif 'y' in direction:
                self.step_length_x = 0
                self.step_length_y = multiplier * step_length
                self.step_height = self.robot_model.step_height_y
            else:
                raise ValueError('direction must be "+x", "-x", "+y" or "-y", when turn_movement is False or "+","-",' +
                                 ' when turn_movement is True')
        else:
            r = []
            phi = []
            for leg in self.robot_model.legs:
                r.append(np.sqrt(leg.init_x_f ** 2 + leg.init_y_f ** 2))
                phi.append(2 * np.arcsin(step_length / 2 / r[-1]))
            self.r = np.mean(r)
            self.step_length_phi = multiplier * np.mean(phi)
            self.step_height = self.robot_model.step_height_turn

        self._generate_init_gait_sequence(turn_movement)
        self._generate_gait_sequence(turn_movement)

    def _generate_init_gait_sequence(self, turn_movement):
        if not turn_movement:
            push_func = self.robot_model.generate_straight_line
        else:
            push_func = self.robot_model.generate_partial_circle

        movement_dict = {}
        x, y, z = self.robot_model.generate_step(self._generate_coord_offset(self.robot_model.left_forward_leg,
                                                                             0, turn_movement),
                                                 self._generate_coord_offset(self.robot_model.left_forward_leg,
                                                                             1 / 6, turn_movement), self.step_height,
                                                 self.n)
        movement_dict[self.robot_model.left_forward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.left_forward_leg.name + '_PWM'] = self.robot_model.left_forward_leg.calc_PWM(
            self.robot_model.left_forward_leg.calc_trajectory(movement_dict[self.robot_model.left_forward_leg.name]))
        leg_list = [self.robot_model.right_backward_leg,
                    self.robot_model.left_backward_leg,
                    self.robot_model.right_forward_leg]
        for leg in leg_list:
            x, y, z = push_func(self._generate_coord_offset(leg, 0, turn_movement),
                                self._generate_coord_offset(leg, -1 / 6, turn_movement),
                                self.n)
            movement_dict[leg.name] = list(zip(x, y, z))
            movement_dict[leg.name + '_PWM'] = leg.calc_PWM(leg.calc_trajectory(movement_dict[leg.name]))
        self.init_gait_list.append(movement_dict)

        movement_dict = {}
        x, y, z = push_func(self._generate_coord_offset(self.robot_model.left_forward_leg, 1 / 6, turn_movement),
                            self._generate_coord_offset(self.robot_model.left_forward_leg, 0, turn_movement),
                            self.n)
        movement_dict[self.robot_model.left_forward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.left_forward_leg.name + '_PWM'] = self.robot_model.left_forward_leg.calc_PWM(
            self.robot_model.left_forward_leg.calc_trajectory(movement_dict[self.robot_model.left_forward_leg.name]))
        x, y, z = self.robot_model.generate_step(self._generate_coord_offset(self.robot_model.right_backward_leg,
                                                                             -1 / 6, turn_movement),
                                                 self._generate_coord_offset(self.robot_model.right_backward_leg,
                                                                             1 / 3, turn_movement), self.step_height,
                                                 self.n)
        movement_dict[self.robot_model.right_backward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.right_backward_leg.name + '_PWM'] = self.robot_model.right_backward_leg.calc_PWM(
            self.robot_model.right_backward_leg.calc_trajectory(movement_dict[self.robot_model.right_backward_leg.name]))
        leg_list = [self.robot_model.left_backward_leg,
                    self.robot_model.right_forward_leg]
        for leg in leg_list:
            x, y, z = push_func(self._generate_coord_offset(leg, -1 / 6, turn_movement),
                                self._generate_coord_offset(leg, -1 / 3, turn_movement),
                                self.n)
            movement_dict[leg.name] = list(zip(x, y, z))
            movement_dict[leg.name + '_PWM'] = leg.calc_PWM((leg.calc_trajectory(movement_dict[leg.name])))
        self.init_gait_list.append(movement_dict)

        movement_dict = {}
        x, y, z = push_func(self._generate_coord_offset(self.robot_model.left_forward_leg, 0, turn_movement),
                            self._generate_coord_offset(self.robot_model.left_forward_leg, -1 / 6, turn_movement),
                            self.n)
        movement_dict[self.robot_model.left_forward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.left_forward_leg.name + '_PWM'] = self.robot_model.left_forward_leg.calc_PWM(
            self.robot_model.left_forward_leg.calc_trajectory(movement_dict[self.robot_model.left_forward_leg.name]))
        x, y, z = push_func(self._generate_coord_offset(self.robot_model.right_backward_leg, 1 / 3, turn_movement),
                            self._generate_coord_offset(self.robot_model.right_backward_leg, 1 / 6, turn_movement),
                            self.n)
        movement_dict[self.robot_model.right_backward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.right_backward_leg.name + '_PWM'] = self.robot_model.right_backward_leg.calc_PWM(
            self.robot_model.right_backward_leg.calc_trajectory(movement_dict[self.robot_model.right_backward_leg.name]))

        x, y, z = self.robot_model.generate_step(self._generate_coord_offset(self.robot_model.right_forward_leg,
                                                                             -1 / 3, turn_movement),
                                                 self._generate_coord_offset(self.robot_model.right_forward_leg,
                                                                             1 / 2, turn_movement), self.step_height,
                                                 self.n)
        movement_dict[self.robot_model.right_forward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.right_forward_leg.name + '_PWM'] = self.robot_model.right_forward_leg.calc_PWM(
            self.robot_model.right_forward_leg.calc_trajectory(movement_dict[self.robot_model.right_forward_leg.name]))

        x, y, z = push_func(self._generate_coord_offset(self.robot_model.left_backward_leg, -1 / 3, turn_movement),
                            self._generate_coord_offset(self.robot_model.left_backward_leg, -1 / 2, turn_movement),
                            self.n)
        movement_dict[self.robot_model.left_backward_leg.name] = list(zip(x, y, z))
        movement_dict[self.robot_model.left_backward_leg.name + '_PWM'] = self.robot_model.left_backward_leg.calc_PWM(
            self.robot_model.left_backward_leg.calc_trajectory(
                movement_dict[self.robot_model.left_backward_leg.name]))
        self.init_gait_list.append(movement_dict)

    def _generate_gait_sequence(self, turn_movement):
        leg_list = [self.robot_model.left_backward_leg,
                    self.robot_model.left_forward_leg,
                    self.robot_model.right_backward_leg,
                    self.robot_model.right_forward_leg,
                    self.robot_model.left_backward_leg,
                    self.robot_model.left_forward_leg,
                    self.robot_model.right_backward_leg]
        if not turn_movement:
            push_func = self.robot_model.generate_straight_line
        else:
            push_func = self.robot_model.generate_partial_circle

        for ii in range(4):
            movement_dict = {}
            x, y, z = self.robot_model.generate_step(self._generate_coord_offset(leg_list[ii], -1 / 2, turn_movement),
                                                     self._generate_coord_offset(leg_list[ii], 1 / 2, turn_movement),
                                                     self.step_height, self.n)
            movement_dict[leg_list[ii].name] = list(zip(x, y, z))
            movement_dict[leg_list[ii].name + '_PWM'] = leg_list[ii].calc_PWM(leg_list[ii].calc_trajectory(
                movement_dict[leg_list[ii].name]))
            for jj in range(1, 4):
                x, y, z = push_func(self._generate_coord_offset(leg_list[ii + jj], -1 / 2 + jj / 3, turn_movement),
                                    self._generate_coord_offset(leg_list[ii + jj], -1 / 2 + (jj - 1) / 3,
                                                                turn_movement),
                                    self.n)
                movement_dict[leg_list[ii + jj].name] = list(zip(x, y, z))
                movement_dict[leg_list[ii + jj].name + '_PWM'] = leg_list[ii + jj].calc_PWM(
                    leg_list[ii + jj].calc_trajectory(movement_dict[leg_list[ii + jj].name]))
            self.gait_list.append(movement_dict)

    def _generate_coord_offset(self, leg, offset: float = 0, turn_movement=False) -> tuple:
        if not turn_movement:
            return (leg.init_x_f + offset * self.step_length_x,
                    leg.init_y_f + offset * self.step_length_y,
                    leg.init_z_f)
        else:
            return (self.r * np.sin(leg.init_phi + offset * self.step_length_phi),
                    self.r * np.cos(leg.init_phi + offset * self.step_length_phi),
                    leg.init_z_f)


class Pose:
    def __init__(self, robot_model: RobotModel, movement_goal: dict, name: str, n=2):
        self.robot_model = robot_model
        assert n > 1, 'pose movement must at least contain 2 samples (start and end point)'
        self.n = n  # number of samples until final pose
        self.name = name
        self.movement_goal = movement_goal

    def calc_pose_dict(self):
        movement_dict = {}
        for leg in self.robot_model.legs:
            x, y, z = self.robot_model.generate_straight_line((leg.cur_x_f, leg.cur_y_f, leg.cur_z_f),
                                                              self.movement_goal[leg.name], n=self.n)
            movement_dict[leg.name] = list(zip(x, y, z))
            movement_dict[leg.name + '_PWM'] = leg.calc_PWM(leg.calc_trajectory(movement_dict[leg.name]))
        return movement_dict
