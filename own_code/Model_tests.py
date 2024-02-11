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
    figure = test_linkage.plot_current_state(act1_theta=np.pi/180*10)
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--')

    test_linkage.cur_phi = np.deg2rad(120)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='red', act1_theta=np.pi/180*20)
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='red')

    test_linkage.cur_phi = np.deg2rad(60)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='orange', act1_theta=np.pi/180*30)
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='orange')

    test_linkage.cur_phi = np.deg2rad(175)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='black', act1_theta=np.pi/180*40)
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='black')

    test_linkage.cur_phi = np.deg2rad(135)
    test_linkage.cur_theta = test_linkage.calc_theta(test_linkage.cur_phi)
    figure = test_linkage.plot_current_state(figure, color='violet', act1_theta=np.pi/180*50)
    test_linkage.cur_phi = test_linkage.calc_phi(test_linkage.cur_theta)
    figure = test_linkage.plot_current_state(figure, linestyle='--', color='violet')

#    figure.axes.set_aspect('equal')
    figure.axes.set_ylim((-75, 60), auto=False)
    figure.axes.set_xlim((20, 150), auto=False)
    plt.gca().invert_yaxis()
    plt.show()


def test_leg_implementation():

    test_leg = SpiderLeg()
    figure = test_leg.visualize_state(test_leg.actuator1.cur_phi, test_leg.actuator2.cur_phi, test_leg.actuator3.cur_phi,
                                      linestyle=':', color='black')
    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f, 52.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    phi_1_old = phi1
    phi_2_old = phi2
    phi_3_old = phi3
    figure = test_leg.visualize_state(phi1, phi2, phi3, figure, color='blue')

    x, y, z = test_leg.cur_x_f+40, test_leg.cur_y_f, 37.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    print(np.degrees(phi1-phi_1_old), np.degrees(phi2-phi_2_old), np.degrees(phi3-phi_3_old))
    figure = test_leg.visualize_state(phi1, phi2, phi3, figure, color='green')

    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f+30, 45.5
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    # phi1, phi2, phi3 = test_leg.actuator1.cur_phi, test_leg.actuator2.phi_max, test_leg.actuator3.phi_max
    figure = test_leg.visualize_state(phi1, phi2, phi3, figure, color='red')

    x, y, z = test_leg.cur_x_f, test_leg.cur_y_f + 20, 52.5
    # phi1, phi2, phi3 = test_leg.actuator1.cur_phi, test_leg.actuator2.phi_min, test_leg.actuator3.phi_max
    phi1, phi2, phi3 = test_leg.backward_transform(x, y, z)
    figure = test_leg.visualize_state(phi1, phi2, phi3, figure, color='orange')
#    figure.axes.set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.show()


def test_single_step_implementation():
    robot_model = RobotModel(FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                             HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm)
    x, y, z = robot_model.generate_step((robot_model.forward_left_leg.cur_x_f-20,
                                         robot_model.forward_left_leg.cur_y_f,#-10,
                                         robot_model.forward_left_leg.cur_z_f),
                                        (robot_model.forward_left_leg.cur_x_f+20,
                                         robot_model.forward_left_leg.cur_y_f,#+10,
                                         robot_model.forward_left_leg.cur_z_f),
                                        step_height=10, n=10)
    phis = robot_model.forward_left_leg.calc_trajectory(zip(x, y, z))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim((20, 150), auto=False)
    ax.set_ylim((-20, -150), auto=False)
    ax.set_zlim((-75, 60), auto=False)
    robot_model.forward_left_leg.visualize_state(*phis[0], ax)

    def animate(i):
        ax.cla()
        ax.set_xlim((20, 150), auto=False)
        ax.set_ylim((-20, -155), auto=False)
        ax.set_zlim((-75, 60), auto=False)

        axis = robot_model.forward_left_leg.visualize_state(*phis[i], ax)
        lines = axis.get_lines()  # update the data.
        return lines

    ani = animation.FuncAnimation(fig, animate, frames=10, interval=100, blit=False, repeat=True)

    plt.show()

def gait_implementation_test():
    robot_model = RobotModel(FLB_init_pwm, FLM_init_pwm, FLE_init_pwm, FRB_init_pwm, FRM_init_pwm, FRE_init_pwm,
                             HLB_init_pwm, HLM_init_pwm, HLE_init_pwm, HRB_init_pwm, HRM_init_pwm, HRE_init_pwm)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_zlim([53, -75])
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
        ax.set_zlim((53, -75), auto=False)
        ax.plot(*list(zip(*LFL[max([0, lim]):i+1])), color='blue')
        robot_model.forward_left_leg.cur_x_f, \
        robot_model.forward_left_leg.cur_y_f, \
        robot_model.forward_left_leg.cur_z_f = LFL[i]
        robot_model.forward_left_leg.update_cur_phi(*LFL[i])
        ax.plot(*list(zip(*LBL[max([0, lim]):i+1])), color='red')
        robot_model.backward_left_leg.cur_x_f, \
        robot_model.backward_left_leg.cur_y_f, \
        robot_model.backward_left_leg.cur_z_f = LBL[i]
        robot_model.backward_left_leg.update_cur_phi(*LBL[i])
        ax.plot(*list(zip(*RFL[max([0, lim]):i+1])), color='green')
        robot_model.forward_right_leg.cur_x_f, \
        robot_model.forward_right_leg.cur_y_f, \
        robot_model.forward_right_leg.cur_z_f = RFL[i]
        robot_model.forward_right_leg.update_cur_phi(*RFL[i])
        ax.plot(*list(zip(*RBL[max([0, lim]):i+1])), color='purple')
        robot_model.backward_right_leg.cur_x_f, \
        robot_model.backward_right_leg.cur_y_f, \
        robot_model.backward_right_leg.cur_z_f = RBL[i]
        robot_model.backward_right_leg.update_cur_phi(*RBL[i])
        robot_model.forward_left_leg.visualize_state(robot_model.forward_left_leg.actuator1.cur_phi,
                                                     robot_model.forward_left_leg.actuator2.cur_phi,
                                                     robot_model.forward_left_leg.actuator3.cur_phi,
                                                     ax=ax, color='black')
        robot_model.backward_left_leg.visualize_state(robot_model.backward_left_leg.actuator1.cur_phi,
                                                      robot_model.backward_left_leg.actuator2.cur_phi,
                                                     robot_model.backward_left_leg.actuator3.cur_phi,
                                                     ax=ax, color='black')
        robot_model.backward_right_leg.visualize_state(robot_model.backward_right_leg.actuator1.cur_phi,
                                                       robot_model.backward_right_leg.actuator2.cur_phi,
                                                       robot_model.backward_right_leg.actuator3.cur_phi,
                                                       ax=ax, color='black')
        robot_model.forward_right_leg.visualize_state(robot_model.forward_right_leg.actuator1.cur_phi,
                                                      robot_model.forward_right_leg.actuator2.cur_phi,
                                                      robot_model.forward_right_leg.actuator3.cur_phi,
                                                      ax=ax, color='black')
        lines = ax.get_lines()  # update the data.
        return lines

    ani = animation.FuncAnimation(fig, animate, frames=len(LFL), interval=100, blit=False, repeat=True)
    plt.show()
    print('')

if __name__ == '__main__':
    # test_linkage_implementation()
    # test_leg_implementation()
    # test_single_step_implementation()
    gait_implementation_test()
