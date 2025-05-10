#include "RobotModel.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"
#include "math.h"

RobotModel::RobotModel(SpiderLeg* leg_list[4]): 
    move_forward(leg_list),
    move_backward(leg_list), 
    move_right(leg_list), 
    move_left(leg_list), 
    turn_right(leg_list), 
    turn_left(leg_list),
    neutral(leg_list, "pne"),
    look_up(leg_list, "plu"), 
    look_down(leg_list, "pld"), 
    lean_right(leg_list, "plr"), 
    lean_left(leg_list, "pll"), 
    high(leg_list, "phi"), 
    low(leg_list, "plo") {
        this->leg_list[0] = leg_list[0];
        this->leg_list[1] = leg_list[1];
        this->leg_list[2] = leg_list[2];
        this->leg_list[3] = leg_list[3];
        this->gait_list[0] = &move_forward;
        this->gait_list[1] = &move_backward;
        this->gait_list[2] = &move_right;
        this->gait_list[3] = &move_left;
        this->gait_list[4] = &turn_right;
        this->gait_list[5] = &turn_left;
        this->pose_list[0] = &neutral;
        this->pose_list[1] = &look_up;
        this->pose_list[2] = &look_down;
        this->pose_list[3] = &lean_right;
        this->pose_list[4] = &lean_left;
        this->pose_list[5] = &high;
        this->pose_list[6] = &low;
}

void RobotModel::init() {
    this->update_freq = 50;
    this->step_height = 14;
    this->z_max = 68;
    this->z_min = 37;
    
    char dir = 'x';
    float step_length = 80;
    this->v_max_x = step_length / 16 / 2 * this->update_freq; // note: expanded n_min = 4 to n_min = 8, hence the division by 2
    this->v_min_x = step_length / 100 * this->update_freq;
    this->move_forward.init(dir, false, step_length, this->step_height, this->v_max_x, this->update_freq);
    this->move_backward.init(dir, true, step_length, this->step_height, this->v_max_x, this->update_freq);
    
    dir ='y';
    step_length = 20;
    this->v_max_y = step_length / 16 * this->update_freq;
    this->v_min_y = step_length / 100 * this->update_freq;
    this->move_right.init(dir, false, step_length, this->step_height, this->v_max_y, this->update_freq);
    this->move_left.init(dir, true, step_length, this->step_height, this->v_max_y, this->update_freq);
    
    dir ='t';
    step_length = 80;
    this->v_max_t = step_length / 16 / 2 * this->update_freq; // note: expanded n_min = 4 to n_min =8, hence the division by 2
    this->v_min_t = step_length / 100 * this->update_freq;
    this->turn_right.init(dir, false, step_length, this->step_height, this->v_max_t, update_freq);
    this->turn_left.init(dir, true, step_length, this->step_height, this->v_max_t, update_freq);

    float neutral_target[4][3];
    this->calc_leg_pos_from_body_angles(neutral_target, 0, 0, 0);
    this->neutral.init(neutral_target);
    float look_up_target[4][3];
    this->calc_leg_pos_from_body_angles(look_up_target, 10, 0, 0);
    this->look_up.init(look_up_target);
    float look_down_target[4][3];
    this->calc_leg_pos_from_body_angles(look_down_target, -10, 0, 0);
    this->look_down.init(look_down_target);
    float lean_right_target[4][3];
    this->calc_leg_pos_from_body_angles(lean_right_target, 0, 5.5, 0);
    this->lean_right.init(lean_right_target);
    float lean_left_target[4][3];
    this->calc_leg_pos_from_body_angles(lean_left_target, 0, -5.5, 0);
    this->lean_left.init(lean_left_target);
    float high_target[4][3];
    this->calc_leg_pos_from_body_angles(high_target, 0, 0, this->z_max);
    this->high.init(high_target);
    float low_target[4][3];
    this->calc_leg_pos_from_body_angles(low_target, 0, 0, this->z_min);
    this->low.init(low_target);


}

void RobotModel::get_body_angles(float angles[2]) {
    // it could happen, that this method is called, when one leg is airborne, so it should be checked, that all legs
    // are approx. in one plane -> solution: calculate for left/right and front/back and choose smaller value!
    // SpiderLeg *leg_list[] = {&RF, &LF, &RB, &LB};
    float theta[4];
    float c1[3];
    float c2[3];
    for (short ii; ii<2; ++ii) {
        this->leg_list[ii]->get_cur_pos(c1); // right/left forward
        this->leg_list[ii+2]->get_cur_pos(c2); // right/left backward
        theta[ii] = atan2f((c1[2]-c2[2]), (c1[0]-c2[0]));
        this->leg_list[2 * ii]->get_cur_pos(c1); // right forward/backward
        this->leg_list[2*ii+1]->get_cur_pos(c2); // left forward/backward
        theta[ii+2] = - atan2f((c1[2]-c2[2]), (c1[1]-c2[1]));
    }
    angles[0]= (theta[0]+theta[1])/ 2 / M_PI * 180;
    angles[1]= (theta[2]+theta[3])/ 2 / M_PI * 180;
}

void RobotModel::calc_leg_pos_from_body_angles(float movement_goal[4][3],float theta_x, float theta_y, float z_0 = 0) {
    float coordinates[3];
    if (z_0 == 0) {
        for (short leg_no = 0; leg_no < 4; ++leg_no) {
            this->leg_list[leg_no] ->get_init_pos(coordinates);
            z_0 += coordinates[2];
        }
        z_0 = z_0 / 4;
        }
    for (short leg_no = 0; leg_no < 4; ++leg_no) {
        this->leg_list[leg_no] ->get_init_pos(coordinates);
        movement_goal[leg_no][0] = coordinates[0]*cos(theta_x * M_PI / 180);
        movement_goal[leg_no][1] = coordinates[1]*cos(theta_y * M_PI / 180);
        movement_goal[leg_no][2] = z_0 + tanf(theta_x * M_PI / 180) * coordinates[0] - tanf(theta_y * M_PI / 180) * coordinates[1];
    }
}

void RobotModel::calc_reset_step() {
    float start[3];
    float target[3];
    float coordinates[8][3];
    float angles[8][3];
    short PWM_values[8][3];
    float distance;
    short n;

    for (short leg_no = 0; leg_no < 4; ++leg_no){
        this->leg_list[leg_no]->get_cur_pos(start);
        this->leg_list[leg_no]->get_init_pos(target);
        if (start[2]<target[2]){
            target[0]=start[0];
            target[1]=start[1];
            generate_straight_line(start,target,coordinates,2);
        }
        else {
            generate_straight_line(start,start,coordinates,2);
        }
        this->leg_list[leg_no]->calc_trajectory(coordinates, angles, 2);
        this->leg_list[leg_no]->calc_PWM(angles, PWM_values, 2);
        for (short sample = 0; sample < 2; ++sample){
            for (short ii = 0; ii < 3; ++ii){
                this->reset_coord_list[0][sample][leg_no][ii]=coordinates[sample][ii];
                this->reset_pwm_list[0][sample][leg_no][ii]=PWM_values[sample][ii];
            }
        }
        for (short sample = 2; sample < 8; ++sample){
            for (short ii = 0; ii < 3; ++ii){
                this->reset_coord_list[0][sample][leg_no][ii]=-1000;
                this->reset_pwm_list[0][sample][leg_no][ii]=-1;
            }
        }
    }

    for(short step; step < 4; ++step){
        for (short leg_no = 0; leg_no < 4; ++leg_no){
            // start = letztes Ziel :-)
            for (short ii = 0; ii < 3; ++ii){
                start[ii] = this->reset_coord_list[0][1][leg_no][ii];
            }
            this->leg_list[leg_no]->get_init_pos(target);
            distance = sqrtf(sq(target[0]-start[0])+sq(target[1]-start[1]));
            n = round(distance/10);
            if (n < 4){
                n = 4;
            }
            else if (n > 8){
                n = 8;
            }
            if (leg_no == step) {
                generate_step(start, target, this->step_height, coordinates, n);
            }
            else if (leg_no < step) {
                generate_straight_line(target, target, coordinates, n);
            }
            else {
                generate_straight_line(start, start, coordinates, n);
            }
            this->leg_list[leg_no]->calc_trajectory(coordinates, angles, n);
            this->leg_list[leg_no]->calc_PWM(angles, PWM_values, n);
            for (short sample = 0; sample < n; ++sample){
                for (short ii = 0; ii  <3; ++ii){
                    this->reset_coord_list[step+1][sample][leg_no][ii]=coordinates[sample][ii];
                    this->reset_pwm_list[step+1][sample][leg_no][ii]=PWM_values[sample][ii];
                }
            }
            if (n < 8) {
                for (short sample = n; sample < 8; ++sample){
                    for (short ii = 0; ii  <3; ++ii){
                        this->reset_coord_list[step+1][sample][leg_no][ii]=-1000;
                        this->reset_pwm_list[step+1][sample][leg_no][ii]=-1;
                    }
                }
            }
        }
    }
}

float RobotModel::get_reset_coord(short step, short sample, short leg, short ii){
    return this->reset_coord_list[step][sample][leg][ii];
}

short RobotModel::get_reset_pwm(short step, short sample, short leg, short ii){
    return this->reset_pwm_list[step][sample][leg][ii];
}

void RobotModel::set_velocity(short percentage) {
    float v_max;
    float v_min;
    float v;
    for(short ii=0; ii<6; ++ii){
        if (ii < 2){
            v = this->v_max_x * float(percentage)/100;
            v_max = this->v_max_x;
            v_min = this->v_min_x;
        }
        else if (ii < 4){
            v = this->v_max_y * float(percentage)/100;
            v_max = this->v_max_y;
            v_min = this->v_min_y;
        }
        else {
            v = this->v_max_t * float(percentage)/100;
            v_max = this->v_max_t;
            v_min = this->v_min_t;
        }
        if (v > v_max) {
            v = v_max;
        }
        else if (v < v_min) {
            v = v_min;
        }
        this->gait_list[ii]->set_velocity(v);
    }
}