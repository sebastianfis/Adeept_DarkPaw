from own_code.SpiderKinematics import FourBarLinkage, SpiderLeg, RobotModel, Gait
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



FLB_init_pwm = 313
FLM_init_pwm = 305
FLE_init_pwm = 313

FRB_init_pwm = 325
FRM_init_pwm = 281
FRE_init_pwm = 301

HLB_init_pwm = 312
HLM_init_pwm = 287
HLE_init_pwm = 260

HRB_init_pwm = 305
HRM_init_pwm = 195
HRE_init_pwm = 340

FLB_direction = 1
FLM_direction = -1
FLE_direction = -1

FRB_direction = -1
FRM_direction = 1
FRE_direction = 1

HLB_direction = -1
HLM_direction = 1
HLE_direction = 1

HRB_direction = 1
HRM_direction = -1
HRE_direction = -1


def test_linkage_implementation():
    test_linkage = FourBarLinkage(l_sg=20, l_sa=17, l_ab=30, l_gb=15, phi_0=90)
    figure = test_linkage.plot_current_state()
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--')

    test_linkage.cur_phi = np.deg2rad(120)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='red')
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='red')

    test_linkage.cur_phi = np.deg2rad(60)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='orange')
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='orange')

    test_linkage.cur_phi = np.deg2rad(175)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='black')
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='black')

    test_linkage.cur_phi = np.deg2rad(135)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='violet')
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='violet')

    figure.axes[0].set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.show()


def test_leg_implementation():
    test_leg = SpiderLeg()
    figure = test_leg.visualize_state(test_leg.actuator2.cur_phi, test_leg.actuator3.cur_phi,
                                      linestyle=':', color='black')
    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f, 52.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    phi_1_old = phi1
    phi_2_old = phi2
    phi_3_old = phi3
    figure = test_leg.visualize_state(phi2, phi3, figure, color='blue')

    x, y, z = test_leg.cur_x_f+40, test_leg.cur_y_f, 37.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    print(np.degrees(phi1-phi_1_old), np.degrees(phi2-phi_2_old), np.degrees(phi3-phi_3_old))
    figure = test_leg.visualize_state(phi2, phi3, figure, color='green')

    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f+30, 45.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    # phi1, phi2, phi3 = test_leg.actuator1.cur_phi, test_leg.actuator2.phi_max, test_leg.actuator3.phi_max
    figure = test_leg.visualize_state(phi2, phi3, figure, color='red')

    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f + 20, 52.5
    # phi1, phi2, phi3 = test_leg.actuator1.cur_phi, test_leg.actuator2.phi_min, test_leg.actuator3.phi_max
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    figure = test_leg.visualize_state(phi2, phi3, figure, color='orange')
    figure.axes[0].set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.show()


def test_single_step_implementation():
    robot_model = RobotModel(FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                 HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm)
    x, y, z = robot_model.generate_step((robot_model.forward_left_leg.cur_x_f,
                                         robot_model.forward_left_leg.cur_y_f-20,
                                         robot_model.forward_left_leg.cur_z_f),
                                        (robot_model.forward_left_leg.cur_x_f,
                                         robot_model.forward_left_leg.cur_y_f+10,
                                         robot_model.forward_left_leg.cur_z_f),
                                        n=10)
    phis = robot_model.forward_left_leg.calc_trajectory(zip(x, y, z))

    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_ylim((-75, 60), auto=False)
    ax.set_xlim((20, 150), auto=False)
    ax.invert_yaxis()
    robot_model.forward_left_leg.visualize_state(phis[0][1], phis[0][2], ax)

    def animate(i):
        ax.cla()
        ax.set_aspect('equal')
        ax.set_ylim((-75, 60), auto=False)
        ax.set_xlim((20, 150), auto=False)
        ax.invert_yaxis()
        axis = robot_model.forward_left_leg.visualize_state(phis[i][1], phis[i][2], ax)
        lines = axis.get_lines()  # update the data.
        return lines

    ani = animation.FuncAnimation(fig, animate, frames=10, interval=100, blit=False, repeat=True)

    plt.show()

def test_gait_implementation():
    robot_model = RobotModel(FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                             HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_zlim([53, 0])
    LFL = []
    LBL = []
    RFL = []
    RBL = []
    # for item in robot_model.turn_clockwise.init_gait_list + robot_model.turn_clockwise.gait_list:
    # for item in robot_model.turn_counterclockwise.gait_list:
    for item in robot_model.move_forward.gait_list:
        for entry in item['LFL']:
            LFL.append(entry)
        for entry in item['LBL']:
            LBL.append(entry)
        for entry in item['RFL']:
            RFL.append(entry)
        for entry in item['RBL']:
            RBL.append(entry)

    def animate(i):
        ax.cla()
        lim = i-10
        ax.set_xlim((-120, 120), auto=False)
        ax.set_ylim((-155, 155), auto=False)
        ax.set_zlim((53, 0), auto=False)
        ax.plot(*list(zip(*LFL[max([0, lim]):i+1])), color='blue')
        robot_model.forward_left_leg.cur_x_f, \
        robot_model.forward_left_leg.cur_y_f, \
        robot_model.forward_left_leg.cur_z_f = LFL[i]
        ax.plot(*list(zip(*LBL[max([0, lim]):i+1])), color='red')
        robot_model.backward_left_leg.cur_x_f, \
        robot_model.backward_left_leg.cur_y_f, \
        robot_model.backward_left_leg.cur_z_f = LBL[i]
        ax.plot(*list(zip(*RFL[max([0, lim]):i+1])), color='green')
        robot_model.forward_right_leg.cur_x_f, \
        robot_model.forward_right_leg.cur_y_f, \
        robot_model.forward_right_leg.cur_z_f = RFL[i]
        ax.plot(*list(zip(*RBL[max([0, lim]):i+1])), color='black')
        robot_model.backward_right_leg.cur_x_f, \
        robot_model.backward_right_leg.cur_y_f, \
        robot_model.backward_right_leg.cur_z_f = RBL[i]
        lines = ax.get_lines()  # update the data.
        return lines
    ani = animation.FuncAnimation(fig, animate, frames=len(LFL), interval=100, blit=False, repeat=True)
    plt.show()
    print('')

if __name__ == '__main__':
    # test_linkage_implementation()
    # test_leg_implementation()
    # test_single_step_implementation()
    test_gait_implementation()


