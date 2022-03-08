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
        self.phi_max = 2 * np.pi
        self.phi_min = -2 * np.pi
        self.theta_max = 2 * np.pi
        self.theta_min = -2 * np.pi
        self.calc_limits(np.deg2rad(phi_0))
        self.cur_phi = np.deg2rad(phi_0)
        self.cur_theta = self.calc_theta(self.cur_phi)
        self.phi_0 = np.deg2rad(phi_0)
        self.pwm_0 = 300
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
        pwm_value = self.pwm_0 + self.actuator_direction * (phi - self.phi_0) * 400 / np.pi
        if pwm_value < 100:
            pwm_value = 100
        elif pwm_value > 500:
            pwm_value = 500
        return int(pwm_value)

    def calc_theta(self, phi):
        """
        method to calculate theta (in rad) from phi (in rad)
        """
        l_ag = np.sqrt(self.l_sa ** 2 + self.l_sg ** 2 - 2 * self.l_sa * self.l_sg * np.cos(phi))
        assert self.phi_min <= phi <= self.phi_max, 'Movement not possible, because l_bs > l_ab + l_sa: {0} >  ' \
                                                    '{1} + {2}'.format(l_ag, self.l_ab, self.l_gb)
        theta_1 = np.arccos((self.l_sg ** 2 + l_ag ** 2 - self.l_sa ** 2) / (2 * self.l_sg * l_ag))
        theta_2 = np.arccos((self.l_gb ** 2 + l_ag ** 2 - self.l_ab ** 2) / (2 * self.l_gb * l_ag))
        theta = theta_1 + theta_2
        return theta

    def calc_phi(self, theta):
        """
        method to calculate phi (in rad) from theta (in rad)
        """
        l_bs = np.sqrt(self.l_gb ** 2 + self.l_sg ** 2 - 2 * self.l_gb * self.l_sg * np.cos(theta))
        assert self.theta_min <= theta <= self.theta_max, 'Movement not possible, because l_bs > l_ab + l_sa: {0} > ' \
                                                          '{1} + {2}'.format(l_bs, self.l_ab, self.l_sa)

        phi_1 = np.arccos((self.l_sg ** 2 + l_bs ** 2 - self.l_gb ** 2) / (2 * self.l_sg * l_bs))
        phi_2 = np.arccos((self.l_sa ** 2 + l_bs ** 2 - self.l_ab ** 2) / (2 * self.l_sa * l_bs))
        phi = phi_1 + phi_2
        return phi

    def calc_limits(self, phi_0):
        """Sets the movement limits to meaningful values defined by a) boundaries of the actuator of +/- 90 deg from the
        initial position. b) by calculating the angle at which the conneceed bars are in a straight line. Further
        movement would then revert the direction of a change in the other angle!"""
        l_ag_max = self.l_ab + self.l_gb
        phi_max = np.arccos((self.l_sa ** 2 + self.l_sg ** 2 - l_ag_max ** 2) / (2 * self.l_sa * self.l_sg))
        if np.isnan(phi_max):
            phi_max = np.pi
        if phi_max > phi_0 + np.pi / 2:  # Correct value, if it is out of servo movement range!
            self.phi_max = phi_0 + np.pi / 2
        else:
            self.phi_max = phi_max

        self.theta_min = self.calc_theta(self.phi_max)

        l_bs_max = self.l_ab + self.l_sa
        theta_max = np.arccos((self.l_gb ** 2 + self.l_sg ** 2 - l_bs_max ** 2) / (2 * self.l_gb * self.l_sg))
        if np.isnan(theta_max):
            theta_max = np.pi
        self.theta_max = theta_max

        phi_min = self.calc_phi(self.theta_max)
        if np.isnan(phi_min):
            phi_min = 0

        if phi_min < phi_0 - np.pi / 2:
            self.phi_min = phi_0 - np.pi / 2
        else:
            self.phi_min = phi_min

    def plot_current_state(self, ax=None, linestyle='-', color='blue', cs_rot_angle: float = 30,
                           g_offset=None, inv_x=1, inv_y=1):
        if g_offset is None:
            g_offset = [0, 0]
        if ax is None:
            fig = plt.figure()
            fig.add_subplot(111)
            ax = fig.axes[0]
        x_list = np.array([0, -self.l_sg, -self.l_sg + self.l_sa * np.cos(self.cur_phi),
                           - self.l_gb * np.cos(self.cur_theta), 0])
        y_list = np.array([0, 0, self.l_sa * np.sin(self.cur_phi),
                           self.l_gb * np.sin(self.cur_theta), 0])
        rot = np.array([[np.cos(np.deg2rad(-cs_rot_angle)), -np.sin(np.deg2rad(-cs_rot_angle))],
                        [np.sin(np.deg2rad(-cs_rot_angle)), np.cos(np.deg2rad(-cs_rot_angle))]])
        [x_list, y_list] = np.dot(rot, [x_list, y_list])
        x_list = inv_x * x_list + g_offset[0]
        y_list = inv_y * y_list + g_offset[1]
        ax.plot(x_list, y_list, linestyle=linestyle, color=color)
        # print('calculated length l_AB = {}'.format(np.sqrt((self.l_sg - self.l_gb * np.cos(self.cur_theta) -
        #                                                     self.l_sa * np.cos(self.cur_phi)) ** 2 +
        #                                                    (self.l_gb * np.sin(self.cur_theta) -
        #                                                     self.l_sa * np.sin(self.cur_phi)) ** 2)))
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
        self.init_phi = np.arctan2(self.init_x_f, self.init_y_f)
        self.cur_x_f, self.cur_y_f, self.cur_z_f = self.init_x_f, self.init_y_f, self.init_z_f

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

    def visualize_state(self, phi_2, phi_3, ax=None, linestyle='-', color='blue'):
        if ax is None:
            fig = plt.figure()
            fig.add_subplot(111)
            ax = fig.axes[0]
        self.actuator2.cur_phi = phi_2
        self.actuator3.cur_phi = phi_3
        self.actuator2.cur_theta = self.actuator2.calc_theta(phi_2)
        self.actuator3.cur_theta = self.actuator3.calc_theta(phi_3)
        ax = self.actuator2.plot_current_state(ax=ax, color=color, linestyle=linestyle,
                                               cs_rot_angle=9.5, g_offset=[self.r_g, self.z_g])
        ax = self.actuator3.plot_current_state(ax=ax, color=color, linestyle=linestyle, cs_rot_angle=9.5,
                                               g_offset=[self.r_g, self.z_g], inv_y=-1)
        parallelogram = FourBarLinkage(self.l_gp, self.actuator3.l_gb, self.l_gp,
                                       self.actuator3.l_gb, phi_0=np.degrees(self.psi_0 - self.actuator3.cur_theta -
                                                                             self.actuator2.cur_theta + self.theta_0))
        z_p = self.z_g - self.l_gp * np.sin(self.actuator2.cur_theta - self.theta_0)
        r_p = self.r_g + self.l_gp * np.cos(self.actuator2.cur_theta - self.theta_0)
        ax = parallelogram.plot_current_state(ax=ax, color=color, linestyle=linestyle,
                                              cs_rot_angle=-np.degrees(self.actuator2.cur_theta - self.theta_0),
                                              g_offset=[r_p, z_p], inv_y=-1)
        z_2b = self.z_g + self.actuator2.l_gb * np.sin(self.actuator2.cur_theta + np.deg2rad(9.5))
        r_2b = self.r_g - self.actuator2.l_gb * np.cos(self.actuator2.cur_theta + np.deg2rad(9.5))

        r_e = r_p + self.actuator3.l_gb * np.cos(self.psi_0 - self.actuator3.cur_theta)
        z_e = z_p - self.actuator3.l_gb * np.sin(self.psi_0 - self.actuator3.cur_theta)

        x_f, y_f, z_f = self.forward_transform(self.actuator1.cur_phi, phi_2, phi_3)
        r_f = np.sqrt((x_f - self.x_j) ** 2 + (y_f - self.y_j) ** 2)

        ax.plot([r_2b, r_p, r_f, r_e], [z_2b, z_p, z_f, z_e], linestyle=linestyle, color=color)

        return ax

    def update_cur_pos(self, phi_1, phi_2, phi_3):
        self.actuator1.cur_phi, self.actuator2.cur_phi, self.actuator3.cur_phi = phi_1, phi_2, phi_3
        self.actuator1.cur_theta = self.actuator1.calc_theta(phi_1)
        self.actuator2.cur_theta = self.actuator2.calc_theta(phi_2)
        self.actuator3.cur_theta = self.actuator3.calc_theta(phi_3)
        self.cur_x_f, self.cur_y_f, self.cur_z_f = self.forward_transform(self.actuator1.cur_phi,
                                                                          self.actuator2.cur_phi,
                                                                          self.actuator3.cur_phi)


class RobotModel:
    def __init__(self, FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                 HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm,
                 FLB_direction=1, FLM_direction=-1, FLE_direction=-1, FRB_direction=-1, FRM_direction=1,
                 FRE_direction=1, HLB_direction=-1, HLM_direction=1, HLE_direction=1, HRB_direction=1,
                 HRM_direction=-1, HRE_direction=-1):

        self.forward_left_leg = SpiderLeg(x_j=43.5, y_j=-42, dir_x=1, dir_y=-1, name='LFL')
        self.backward_left_leg = SpiderLeg(x_j=-43.5, y_j=-42, dir_x=-1, dir_y=-1, name='LBL')
        self.forward_right_leg = SpiderLeg(x_j=43.5, y_j=42, dir_x=1, dir_y=1, name='RFL')
        self.backward_right_leg = SpiderLeg(x_j=-43.5, y_j=42, dir_x=-1, dir_y=1, name='RBL')
        self.legs = [self.forward_left_leg,
                     self.backward_left_leg,
                     self.forward_right_leg,
                     self.backward_right_leg]
        self.forward_left_leg.actuator1.set_pwm_init(FLE_init_pwm, FLE_direction)
        self.forward_left_leg.actuator2.set_pwm_init(FLB_init_pwm, FLB_direction)
        self.forward_left_leg.actuator3.set_pwm_init(FLM_init_pwm, FLM_direction)
        self.backward_left_leg.actuator1.set_pwm_init(HLE_init_pwm, HLE_direction)
        self.backward_left_leg.actuator2.set_pwm_init(HLB_init_pwm, HLB_direction)
        self.backward_left_leg.actuator3.set_pwm_init(HLM_init_pwm, HLM_direction)
        self.forward_right_leg.actuator1.set_pwm_init(FRE_init_pwm, FRE_direction)
        self.forward_right_leg.actuator2.set_pwm_init(FRB_init_pwm, FRB_direction)
        self.forward_right_leg.actuator3.set_pwm_init(FRM_init_pwm, FRM_direction)
        self.backward_right_leg.actuator1.set_pwm_init(HRE_init_pwm, HRE_direction)
        self.backward_right_leg.actuator2.set_pwm_init(HRB_init_pwm, HRB_direction)
        self.backward_right_leg.actuator3.set_pwm_init(HRM_init_pwm, HRM_direction)

        self.step_height = 10
        self.step_length_x = 60
        self.step_length_y = 20
        self.step_length_turn = 60

        self.move_forward = Gait(self, step_length=self.step_length_x,
                                 velocity=120, freq=24, turn_movement=False, direction='+x')
        self.move_backward = Gait(self, step_length=self.step_length_x,
                                  velocity=120, freq=24, turn_movement=False, direction='-x')
        self.move_right = Gait(self, step_length=self.step_length_y,  # mögliche Schrittlänge in y ist nur
                          # 1/3 des Wertes in x  -> deshalb is auch die mögliche Geschwindigkeit nur 1/3!
                               velocity=40, freq=24, turn_movement=False, direction='+y')
        self.move_left = Gait(self, step_length=self.step_length_y,
                              velocity=40, freq=24, turn_movement=False, direction='-y')
        self.turn_clockwise = Gait(self, step_length=self.step_length_turn,
                                   velocity=120, freq=24, turn_movement=True, direction='+')
        self.turn_counterclockwise = Gait(self, step_length=self.step_length_turn,
                                          velocity=120, freq=24, turn_movement=True, direction='-')

    def vizualize_feet_vs_cog(self, fig=None):
        if fig is None:
            fig = plt.figure()
            fig.add_subplot(111)
        fig.axes[0].plot([self.forward_left_leg.cur_x_f, self.backward_left_leg.cur_x_f,
                          self.backward_right_leg.cur_x_f, self.forward_left_leg.cur_x_f],
                         [self.forward_left_leg.cur_y_f, self.backward_left_leg.cur_y_f,
                          self.backward_right_leg.cur_y_f, self.forward_left_leg.cur_y_f])
        fig.axes[0].plot([0], [0])
        return fig

    def generate_step(self, starting_point: tuple, end_point: tuple, n: int):
        l_step = np.sqrt((end_point[0] - starting_point[0]) ** 2 + (end_point[1] - starting_point[1]) ** 2)
        x = np.linspace(starting_point[0], end_point[0], n)
        y = np.linspace(starting_point[1], end_point[1], n)
        s = np.linspace(0, l_step, n)
        z = starting_point[2] - np.sqrt(1 - 4 * (s - l_step / 2) ** 2 / l_step ** 2) * self.step_height
        return x, y, z

    @staticmethod
    def generate_straight_line(starting_point: tuple, end_point: tuple, n: int):
        x = np.linspace(starting_point[0], end_point[0], n)
        y = np.linspace(starting_point[1], end_point[1], n)
        z = starting_point[2] * np.ones_like(x)
        return x, y, z

    @staticmethod
    def generate_partial_circle(starting_point: tuple, end_point: tuple, n: int):
        r = np.sqrt(starting_point[0] ** 2 + starting_point[1] ** 2)
        r_end = np.sqrt(end_point[0] ** 2 + end_point[1] ** 2)
        assert np.isclose(r, r_end), 'end_point and start_point have different radius with respect to COG!'
        phi_0 = np.arctan2(starting_point[1], starting_point[0])
        phi_end = np.arctan2(end_point[1], end_point[0])
        phi = np.linspace(phi_0, phi_end, n)
        x = r * np.cos(phi)
        y = r * np.sin(phi)
        z = starting_point[2] * np.ones_like(x)
        return x, y, z

    def calc_reset_move(self, n):
        gait_list = []
        for moving_leg in self.legs:
            if not (np.isclose(moving_leg.cur_x_f, moving_leg.init_x_f) and
                    np.isclose(moving_leg.cur_y_f, moving_leg.init_y_f) and
                    np.isclose(moving_leg.cur_z_f, moving_leg.init_z_f)):
                movement_dict = {}
                for leg in self.legs:
                    if moving_leg.name == leg.name:
                        if np.isclose(moving_leg.cur_z_f, moving_leg.init_z_f):
                            x, y, z = self.generate_step((moving_leg.cur_x_f, moving_leg.cur_y_f, moving_leg.cur_z_f),
                                                         (moving_leg.init_x_f, moving_leg.init_y_f, moving_leg.init_z_f),
                                                         n=n)
                        else:
                            x, y, z = self.generate_straight_line((moving_leg.cur_x_f,
                                                                   moving_leg.cur_y_f,
                                                                   moving_leg.cur_z_f),
                                                                  (moving_leg.init_x_f,
                                                                   moving_leg.init_y_f,
                                                                   moving_leg.init_z_f), n=n)
                        movement_dict[leg.name] = list(zip(x, y, z))

                    else:
                        movement_dict[leg.name] = list(zip(np.ones(n)*leg.cur_x_f,
                                                           np.ones(n)*leg.cur_y_f,
                                                           np.ones(n)*leg.cur_z_f))
                    movement_dict[leg.name + '_PWM'] = leg.calc_trajectory(movement_dict[leg.name])
                gait_list.append(movement_dict)
        return gait_list


class Gait:
    def __init__(self, robot_model: RobotModel, step_length=60,
                 velocity=120, freq=24, turn_movement=False, direction='+x'):
        self.robot_model = robot_model
        self.velocity = velocity  # target velocity in mm/s
        self.T = 4 * step_length / velocity  # Dauer kompletter Schrittzyklus
        self.freq = freq  # Update Frequenz für die Servos. Annahme: ~ Halbe PWM Frequenz = ~24 Hz
        self.n = int(self.T / 4 * self.freq)
        self.init_gait_list = []
        self.gait_list = []
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
            elif 'y' in direction:
                self.step_length_x = 0
                self.step_length_y = multiplier * step_length
            else:
                raise ValueError('direction must be "+x", "-x", "+y" or "-y", when turn_movement is False or "+","-",' +
                                 ' when turn_movement is True')
        else:
            r = []
            phi = []
            for leg in self.robot_model.legs:
                r.append(np.sqrt(leg.init_x_f ** 2 + leg.init_y_f ** 2))
                phi.append(2 * np.arcsin(step_length/2/r[-1]))
            self.r = np.mean(r)
            self.step_length_phi = multiplier * np.mean(phi)

        self._generate_init_gait_sequence(turn_movement)
        self._generate_gait_sequence(turn_movement)

    def _generate_init_gait_sequence(self, turn_movement):
        if not turn_movement:
            leg_list = [self.robot_model.backward_right_leg,
                        self.robot_model.backward_left_leg,
                        self.robot_model.forward_right_leg]
            movement_dict = {}
            x, y, z = self.robot_model.generate_step((self.robot_model.forward_left_leg.init_x_f,
                                                      self.robot_model.forward_left_leg.init_y_f,
                                                      self.robot_model.forward_left_leg.init_z_f),
                                                     (self.robot_model.forward_left_leg.init_x_f +
                                                      self.step_length_x / 6,
                                                      self.robot_model.forward_left_leg.init_y_f +
                                                      self.step_length_y / 6,
                                                      self.robot_model.forward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])
            for leg in leg_list:
                x, y, z = self.robot_model.generate_straight_line((leg.init_x_f, leg.init_y_f, leg.init_z_f),
                                                                  (leg.init_x_f - self.step_length_x / 6,
                                                                   leg.init_y_f - self.step_length_y / 6,
                                                                   leg.init_z_f), self.n)
                movement_dict[leg.name] = list(zip(x, y, z))
                movement_dict[leg.name + '_PWM'] = leg.calc_trajectory(movement_dict[leg.name])
            self.init_gait_list.append(movement_dict)

            movement_dict={}
            leg_list = [self.robot_model.backward_left_leg,
                        self.robot_model.forward_right_leg]
            x, y, z = self.robot_model.generate_straight_line((self.robot_model.forward_left_leg.init_x_f +
                                                               self.step_length_x / 6,
                                                               self.robot_model.forward_left_leg.init_y_f +
                                                               self.step_length_y / 6,
                                                               self.robot_model.forward_left_leg.init_z_f),
                                                              (self.robot_model.forward_left_leg.init_x_f,
                                                               self.robot_model.forward_left_leg.init_y_f,
                                                               self.robot_model.forward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])
            x, y, z = self.robot_model.generate_step((self.robot_model.backward_right_leg.init_x_f -
                                                      self.step_length_x / 6,
                                                      self.robot_model.backward_right_leg.init_y_f -
                                                      self.step_length_y / 6,
                                                      self.robot_model.backward_right_leg.init_z_f),
                                                     (self.robot_model.backward_right_leg.init_x_f +
                                                      self.step_length_x / 3,
                                                      self.robot_model.backward_right_leg.init_y_f +
                                                      self.step_length_y / 3,
                                                      self.robot_model.backward_right_leg.init_z_f), self.n)
            movement_dict[self.robot_model.backward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_right_leg.name + '_PWM'] = \
                self.robot_model.backward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_right_leg.name])
            for leg in leg_list:
                x, y, z = self.robot_model.generate_straight_line((leg.init_x_f - self.step_length_x / 6,
                                                                   leg.init_y_f - self.step_length_y / 6,
                                                                   leg.init_z_f),
                                                                  (leg.init_x_f - self.step_length_x / 3,
                                                                   leg.init_y_f - self.step_length_y / 3,
                                                                   leg.init_z_f), self.n)
                movement_dict[leg.name] = list(zip(x, y, z))
                movement_dict[leg.name +'_PWM'] = leg.calc_trajectory(movement_dict[leg.name])
            self.init_gait_list.append(movement_dict)

            movement_dict = {}
            x, y, z = self.robot_model.generate_straight_line((self.robot_model.forward_left_leg.init_x_f,
                                                               self.robot_model.forward_left_leg.init_y_f,
                                                               self.robot_model.forward_left_leg.init_z_f),
                                                              (self.robot_model.forward_left_leg.init_x_f -
                                                               self.step_length_x / 6,
                                                               self.robot_model.forward_left_leg.init_y_f -
                                                               self.step_length_y / 6,
                                                               self.robot_model.forward_left_leg.init_z_f),
                                                              self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])
            x, y, z = self.robot_model.generate_straight_line((self.robot_model.backward_right_leg.init_x_f +
                                                               self.step_length_x / 3,
                                                               self.robot_model.backward_right_leg.init_y_f +
                                                               self.step_length_y / 3,
                                                               self.robot_model.backward_right_leg.init_z_f),
                                                              (self.robot_model.backward_right_leg.init_x_f +
                                                               self.step_length_x / 6,
                                                               self.robot_model.backward_right_leg.init_y_f +
                                                               self.step_length_y / 6,
                                                               self.robot_model.backward_right_leg.init_z_f), self.n)
            movement_dict[self.robot_model.backward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_right_leg.name + '_PWM'] = \
                self.robot_model.backward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_right_leg.name])

            x, y, z = self.robot_model.generate_step((self.robot_model.forward_right_leg.init_x_f -
                                                      self.step_length_x / 3,
                                                      self.robot_model.forward_right_leg.init_y_f -
                                                      self.step_length_y / 3,
                                                      self.robot_model.forward_right_leg.init_z_f),
                                                     (self.robot_model.forward_right_leg.init_x_f +
                                                      self.step_length_x / 2,
                                                      self.robot_model.forward_right_leg.init_y_f +
                                                      self.step_length_y / 2,
                                                      self.robot_model.forward_right_leg.init_z_f),
                                                     self.n)
            movement_dict[self.robot_model.forward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_right_leg.name + '_PWM'] = \
                self.robot_model.forward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_right_leg.name])

            x, y, z = self.robot_model.generate_straight_line((self.robot_model.backward_left_leg.init_x_f -
                                                               self.step_length_x / 3,
                                                               self.robot_model.backward_left_leg.init_y_f -
                                                               self.step_length_y / 3,
                                                               self.robot_model.backward_left_leg.init_z_f),
                                                              (self.robot_model.backward_left_leg.init_x_f -
                                                               self.step_length_x / 2,
                                                               self.robot_model.backward_left_leg.init_y_f -
                                                               self.step_length_y / 2,
                                                               self.robot_model.backward_left_leg.init_z_f),
                                                              self.n)
            movement_dict[self.robot_model.backward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_left_leg.name + '_PWM'] = \
                self.robot_model.backward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_left_leg.name])
            self.init_gait_list.append(movement_dict)
        else:
            leg_list = [self.robot_model.backward_right_leg,
                        self.robot_model.backward_left_leg,
                        self.robot_model.forward_right_leg]
            movement_dict = {}
            x, y, z = self.robot_model.generate_step((self.robot_model.forward_left_leg.init_x_f,
                                                      self.robot_model.forward_left_leg.init_y_f,
                                                      self.robot_model.forward_left_leg.init_z_f),
                                                     (self.r * np.sin(self.robot_model.forward_left_leg.init_phi +
                                                                      self.step_length_phi / 6),
                                                      self.r * np.cos(self.robot_model.forward_left_leg.init_phi +
                                                                      self.step_length_phi / 6),
                                                      self.robot_model.forward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])
            for leg in leg_list:
                x, y, z = self.robot_model.generate_partial_circle((leg.init_x_f, leg.init_y_f, leg.init_z_f),
                                                                   (self.r * np.sin(leg.init_phi -
                                                                                    self.step_length_phi / 6),
                                                                    self.r * np.cos(leg.init_phi -
                                                                                    self.step_length_phi / 6),
                                                                    leg.init_z_f), self.n)
                movement_dict[leg.name] = list(zip(x, y, z))
                movement_dict[leg.name + '_PWM'] = leg.calc_trajectory(movement_dict[leg.name])
            self.init_gait_list.append(movement_dict)

            movement_dict = {}
            leg_list = [self.robot_model.backward_left_leg,
                        self.robot_model.forward_right_leg]
            x, y, z = self.robot_model.generate_partial_circle((self.r * np.sin(self.robot_model.forward_left_leg.init_phi +
                                                                                self.step_length_phi / 6),
                                                                self.r * np.cos(self.robot_model.forward_left_leg.init_phi +
                                                                                self.step_length_phi / 6),
                                                                self.robot_model.forward_left_leg.init_z_f),
                                                               (self.robot_model.forward_left_leg.init_x_f,
                                                                self.robot_model.forward_left_leg.init_y_f,
                                                                self.robot_model.forward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])

            x, y, z = self.robot_model.generate_step((self.r * np.sin(self.robot_model.backward_right_leg.init_phi -
                                                                      self.step_length_phi / 6),
                                                      self.r * np.cos(self.robot_model.backward_right_leg.init_phi -
                                                                      self.step_length_phi / 6),
                                                      self.robot_model.backward_right_leg.init_z_f),
                                                     (self.r * np.sin(self.robot_model.backward_right_leg.init_phi +
                                                                      self.step_length_phi / 3),
                                                      self.r * np.cos(self.robot_model.backward_right_leg.init_phi +
                                                                      self.step_length_phi / 3),
                                                      self.robot_model.backward_right_leg.init_z_f), self.n)
            movement_dict[self.robot_model.backward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_right_leg.name + '_PWM'] = \
                self.robot_model.backward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_right_leg.name])
            for leg in leg_list:
                x, y, z = self.robot_model.generate_partial_circle((self.r * np.sin(leg.init_phi -
                                                                                    self.step_length_phi / 6),
                                                                    self.r * np.cos(leg.init_phi -
                                                                                    self.step_length_phi / 6),
                                                                    leg.init_z_f),
                                                                   (self.r * np.sin(leg.init_phi -
                                                                                    self.step_length_phi / 3),
                                                                    self.r * np.cos(leg.init_phi -
                                                                                    self.step_length_phi / 3),
                                                                    leg.init_z_f), self.n)
                movement_dict[leg.name] = list(zip(x, y, z))
                movement_dict[leg.name + '_PWM'] = leg.calc_trajectory(movement_dict[leg.name])
            self.init_gait_list.append(movement_dict)

            movement_dict = {}
            x, y, z = self.robot_model.generate_partial_circle((self.robot_model.forward_left_leg.init_x_f,
                                                                self.robot_model.forward_left_leg.init_y_f,
                                                                self.robot_model.forward_left_leg.init_z_f),
                                                               (self.r * np.sin(self.robot_model.forward_left_leg.init_phi -
                                                                                self.step_length_phi / 6),
                                                                self.r * np.cos(self.robot_model.forward_left_leg.init_phi -
                                                                                self.step_length_phi / 6),
                                                                self.robot_model.forward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_left_leg.name + '_PWM'] = \
                self.robot_model.forward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_left_leg.name])
            x, y, z = self.robot_model.generate_partial_circle((self.r * np.sin(self.robot_model.backward_right_leg.init_phi +
                                                                                self.step_length_phi / 3),
                                                                self.r * np.cos(self.robot_model.backward_right_leg.init_phi +
                                                                                self.step_length_phi / 3),
                                                                self.robot_model.backward_right_leg.init_z_f),
                                                               (self.r * np.sin(self.robot_model.backward_right_leg.init_phi +
                                                                                self.step_length_phi / 6),
                                                                self.r * np.cos(self.robot_model.backward_right_leg.init_phi +
                                                                                self.step_length_phi / 6),
                                                                self.robot_model.backward_right_leg.init_z_f), self.n)
            movement_dict[self.robot_model.backward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_right_leg.name + '_PWM'] = \
                self.robot_model.backward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_right_leg.name])

            x, y, z = self.robot_model.generate_step((self.r * np.sin(self.robot_model.forward_right_leg.init_phi -
                                                                      self.step_length_phi / 3),
                                                      self.r * np.cos(self.robot_model.forward_right_leg.init_phi -
                                                                      self.step_length_phi / 3),
                                                      self.robot_model.forward_right_leg.init_z_f),
                                                     (self.r * np.sin(self.robot_model.forward_right_leg.init_phi +
                                                                      self.step_length_phi / 2),
                                                      self.r * np.cos(self.robot_model.forward_right_leg.init_phi +
                                                                      self.step_length_phi / 2),
                                                      self.robot_model.forward_right_leg.init_z_f), self.n)
            movement_dict[self.robot_model.forward_right_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.forward_right_leg.name + '_PWM'] = \
                self.robot_model.forward_right_leg.calc_trajectory(
                    movement_dict[self.robot_model.forward_right_leg.name])

            x, y, z = self.robot_model.generate_partial_circle((self.r * np.sin(self.robot_model.backward_left_leg.init_phi -
                                                                                self.step_length_phi / 3),
                                                                self.r * np.cos(self.robot_model.backward_left_leg.init_phi -
                                                                                self.step_length_phi / 3),
                                                                self.robot_model.backward_left_leg.init_z_f),
                                                               (self.r * np.sin(self.robot_model.backward_left_leg.init_phi -
                                                                                self.step_length_phi / 2),
                                                                self.r * np.cos(self.robot_model.backward_left_leg.init_phi -
                                                                                self.step_length_phi / 2),
                                                                self.robot_model.backward_left_leg.init_z_f), self.n)
            movement_dict[self.robot_model.backward_left_leg.name] = list(zip(x, y, z))
            movement_dict[self.robot_model.backward_left_leg.name + '_PWM'] = \
                self.robot_model.backward_left_leg.calc_trajectory(
                    movement_dict[self.robot_model.backward_left_leg.name])
            self.init_gait_list.append(movement_dict)

    def _generate_gait_sequence(self, turn_movement):
        leg_list = [self.robot_model.backward_left_leg,
                    self.robot_model.forward_left_leg,
                    self.robot_model.backward_right_leg,
                    self.robot_model.forward_right_leg,
                    self.robot_model.backward_left_leg,
                    self.robot_model.forward_left_leg,
                    self.robot_model.backward_right_leg]
        if not turn_movement:
            for ii in range(4):
                movement_dict = {}
                x, y, z = self.robot_model.generate_step((leg_list[ii].init_x_f - self.step_length_x / 2,
                                                          leg_list[ii].init_y_f - self.step_length_y / 2,
                                                          leg_list[ii].init_z_f),
                                                         (leg_list[ii].init_x_f + self.step_length_x / 2,
                                                          leg_list[ii].init_y_f + self.step_length_y / 2,
                                                          leg_list[ii].init_z_f), self.n)
                movement_dict[leg_list[ii].name] = list(zip(x, y, z))
                movement_dict[leg_list[ii].name + '_PWM'] = leg_list[ii].calc_trajectory(
                    movement_dict[leg_list[ii].name])
                for jj in range(1, 4):
                    x, y, z = self.robot_model.generate_straight_line(
                        (leg_list[ii + jj].init_x_f - self.step_length_x / 2 + jj * self.step_length_x / 3,
                         leg_list[ii + jj].init_y_f - self.step_length_y / 2 + jj * self.step_length_y / 3,
                         leg_list[ii + jj].init_z_f),
                        (leg_list[ii + jj].init_x_f - self.step_length_x / 2 + (jj - 1) * self.step_length_x / 3,
                         leg_list[ii + jj].init_y_f - self.step_length_y / 2 + (jj - 1)  * self.step_length_y / 3,
                         leg_list[ii + jj].init_z_f), self.n)
                    movement_dict[leg_list[ii + jj].name] = list(zip(x, y, z))
                    movement_dict[leg_list[ii + jj].name + '_PWM'] = leg_list[ii + jj].calc_trajectory(
                        movement_dict[leg_list[ii + jj].name])
                self.gait_list.append(movement_dict)
        else:
            for ii in range(4):
                movement_dict = {}

                x, y, z = self.robot_model.generate_step((self.r * np.sin(leg_list[ii].init_phi -
                                                                          self.step_length_phi / 2),
                                                          self.r * np.cos(leg_list[ii].init_phi -
                                                                          self.step_length_phi / 2),
                                                          leg_list[ii].init_z_f),
                                                         (self.r * np.sin(leg_list[ii].init_phi +
                                                                          self.step_length_phi / 2),
                                                          self.r * np.cos(leg_list[ii].init_phi +
                                                                          self.step_length_phi / 2),
                                                          leg_list[ii].init_z_f), self.n)
                movement_dict[leg_list[ii].name] = list(zip(x, y, z))
                movement_dict[leg_list[ii].name + '_PWM'] = leg_list[ii].calc_trajectory(
                    movement_dict[leg_list[ii].name])
                for jj in range(1, 4):
                    x, y, z = self.robot_model.generate_partial_circle(
                        (self.r * np.sin(leg_list[ii + jj].init_phi - self.step_length_phi / 2 +
                                         jj * self.step_length_phi / 3),
                         self.r * np.cos(leg_list[ii + jj].init_phi - self.step_length_phi / 2 +
                                         jj * self.step_length_phi / 3),
                         leg_list[ii].init_z_f),
                        (self.r * np.sin(leg_list[ii + jj].init_phi - self.step_length_phi / 2 +
                                         (jj - 1) * self.step_length_phi / 3),
                         self.r * np.cos(leg_list[ii + jj].init_phi - self.step_length_phi / 2 +
                                         (jj - 1) * self.step_length_phi / 3),
                         leg_list[ii].init_z_f), self.n)
                    movement_dict[leg_list[ii + jj].name] = list(zip(x, y, z))
                    movement_dict[leg_list[ii + jj].name + '_PWM'] = leg_list[ii + jj].calc_trajectory(
                        movement_dict[leg_list[ii + jj].name])
                self.gait_list.append(movement_dict)
