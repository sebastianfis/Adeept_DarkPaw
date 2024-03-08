#include "Gait.h"
#include "SpiderLeg.h"
#include "math.h"

const short n_max = 25; // Set the max number of steps
const short n_min = 4; // Set the min number of steps


void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n) {
    // float l_step = sqrt(sq(end_point[0] - starting_point[0]) + sq(end_point[1] - starting_point[1]));
    for (int i = 0; i < n; i++) {
        float t = i  / (n - 1);
        result[i][0] = starting_point[0] + t * (end_point[0] - starting_point[0]);
        result[i][1] = starting_point[1] + t * (end_point[1] - starting_point[1]);
        result[i][2] = max(starting_point[2], end_point[2]) - sqrt(1 - 4 * sq(t - 0.5)) * step_height;
    }
}

void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n) {
    float delta_x = (end_point[0] - starting_point[0]) / (n-1);
    float delta_y = (end_point[1] - starting_point[1]) / (n-1);
    float delta_z = (end_point[2] - starting_point[2]) / (n-1);

    for (int i = 0; i < n; i++) {
        result[i][0] = starting_point[0] + i * delta_x;
        result[i][1] = starting_point[1] + i * delta_y;
        result[i][2] = starting_point[2] + i * delta_z;
    }
}

void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n) {
    float r = sqrt(sq(starting_point[0]) + sq(starting_point[1]));
    float phi_0 = atan2(starting_point[1], starting_point[0]);
    float phi_end = atan2(end_point[1], end_point[0]);
    float delta_phi = (phi_end - phi_0) / (n - 1);

    for (int i = 0; i < n; i++) {
        float phi = phi_0 + i * delta_phi;
        result[i][0] = r * cos(phi);
        result[i][1] = r * sin(phi);
        result[i][2] = starting_point[2] + i * (end_point[2] - starting_point[2]) / (n - 1);
    }
}

Gait::Gait(SpiderLeg* leg_list[4], float step_length, float step_height, float velocity, float freq=50, bool inv_direction= false, char direction = 'x'){
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    this->step_length = step_length;
    this->step_height = step_height;
    this->velocity =  velocity;
    this->inv_direction = inv_direction;
    this->freq = freq;
    this->direction = direction;
    this->cur_step_no = 0;
    float r =0;
    for (short leg =0; leg<4; leg++){
      r+= leg_list[0]->get_init_r();
    }
    this->r = r/4;
}

void Gait::init(float init_pos_alloc[][3][4][3], short init_pwm_alloc[][3][4][3], float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]) {
  this->generate_init_gait_sequence(init_pos_alloc, init_pwm_alloc);
  this->generate_gait_sequence(pos_alloc, pwm_alloc);
}

void Gait::generate_coord_offset(short leg_no, float offset, float result[3]){
  this->leg_list[leg_no]->get_init_pos(result);
  if (this->direction == 'x'){
    result[0] += offset * this->step_length;
  }
  else if (this->direction == 'y'){
    result[1] += offset * this->step_length;
  }
  else if (this->direction == 't'){
    result[0] = this->r * sin(this->leg_list[leg_no]->get_init_phi() + offset * this->step_length);
    result[1] = this->r * cos(this->leg_list[leg_no]->get_init_phi() + offset * this->step_length);
  }
}

void Gait::generate_init_gait_sequence(float init_pos_alloc[][3][4][3],short init_pwm_alloc[][3][4][3]) {
//finish implementing this!
}

void Gait::generate_gait_sequence(float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]){
//finish implementing this!
}

void Gait::get_next_coordinate(float coord_list[4][3], short coord_list_PWM[4][3]){
//finish implementing this!
}

void Gait::set_velocity(float velocity, float init_pos_alloc[][3][4][3], short init_pwm_alloc[][3][4][3], float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]) {
  short value =short(round(this->step_length / 4 / velocity * this->freq));
  if (value < n_min){
    this->total_samples_per_step =n_min;
    this->velocity = this->step_length / 4 / n_min * this->freq;
  }
  else if (value > n_max){
    this->total_samples_per_step =n_max;
    this->velocity = this->step_length / 4 / n_max * this->freq;
  }
  else {
    this->total_samples_per_step = value;
    this->velocity = velocity;
  }
  this->init(init_pos_alloc, init_pwm_alloc, pos_alloc, pwm_alloc);
}

short Gait::get_cur_step_no(){
  return this->cur_step_no;
}

char* Gait::get_name() {
  char name[2];
  if (this->inv_direction){
    name[0] = '-';
  }
  else {
    name[0] = '+';
  }
  name[1] = this->direction;
  return name;
}
