#include "RobotModel.h"
#include "SpiderLeg.h"
#include "Gait.h"
#include "Pose.h"

RobotModel::RobotModel(SpiderLeg* leg_list[4]): 
    move_forward(leg_list),
    move_backward(leg_list), 
    move_right(leg_list), 
    move_left(leg_list), 
    turn_right(leg_list), 
    turn_left(leg_list),
    neutral(leg_list), 
    look_up(leg_list), 
    look_down(leg_list), 
    lean_right(leg_list), 
    lean_left(leg_list), 
    high(leg_list), 
    low(leg_list) {
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
    // init pwm for actuators here
    this->update_freq = 50;
    this->step_height = 14;
    this->z_max = 69;
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

    // init poses here
}

//write functions for getting and setting the body angels! Also required for pose init


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