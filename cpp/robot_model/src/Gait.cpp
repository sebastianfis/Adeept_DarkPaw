#include "Gait.h"
#include "SpiderLeg.h"
#include "math.h"

const short n_gait_max = 25; // Set the max number of samples
const short n_gait_min = 4; // Set the min number of samples
const short gait_sequence[] = {0, 1, 2, 3, 0, 1, 2}; // which leg needs to do the step -> ... RF, LF, RB, LB, ...

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

void write_data_to_lists(SpiderLeg* leg, short leg_no, float coordinates[][3], float coord_list[][4][3], short coord_list_PWM[][4][3], short n_samples) {
  float angles[n_gait_max][3];
  short PWM_values[n_gait_max][3];
  leg->calc_trajectory(coordinates, angles, n_samples);
  leg->calc_PWM(angles, PWM_values, n_samples);
  for (short sample = 0; sample < n_samples; ++sample) {  
    for (short ii = 0; ii < 3; ++ii) {
      coord_list[sample][leg_no][ii] = coordinates[sample][ii];
      coord_list_PWM[sample][leg_no][ii] = PWM_values[sample][ii];
    }
  }
}


Gait::Gait(SpiderLeg* leg_list[4], float freq=50, bool inv_direction= false, char direction = 'x'){
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
    this->inv_direction = inv_direction;
    this->freq = freq;
    this->direction = direction;
}

void Gait::init(float step_length, float step_height, float velocity) {
  this->step_height = step_height;
  float r =0;
  float phi = 0;
  for (short leg =0; leg < 4; leg++){
    float r_i= leg_list[leg]->get_init_r();
    r += r_i;
    phi += 2 * asin(step_length / 2 /r_i);
  }
  this->r = r/4;
  if (this->direction == 't') {
    step_length = phi/4;
  }
  if (~this->inv_direction) {
    this->step_length = step_length;
  }
  else {
    this->step_length = - step_length;
  }
  this->set_velocity(velocity);

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

void Gait::generate_init_gait_sequence(short n_samples) {
  // init_gait_sequnce[] = {1, 2, 3}; // which leg needs to do the step -> LF, RB, LB, ... 
  float coordinates[n_samples][3];
  float coord_temp[n_samples][4][3];
  short pwm_temp[n_samples][4][3];
  float start[3];
  float target[3];
  for (short init_step = 0; init_step < 3; init_step++) {
    for (short leg = 0; leg < 4; leg++) {
      if (leg == init_step + 1){
        this->generate_coord_offset(leg, - init_step/6, start);
        this->generate_coord_offset(leg, (init_step + 1)/6, target);
        // generate_step(start, target, this->step_height, coordinates, n_samples);
      }
      else if (leg > init_step + 1 || leg == 0) { //this is only true for the leg that has not made a step yet!
        this->generate_coord_offset(leg, - init_step/6, start);
        this->generate_coord_offset(leg, - (init_step+1)/6, target);
        if (this->direction == 't'){
          generate_partial_circle(start, target, coordinates, n_samples);
        }
        else {
          generate_straight_line(start, target, coordinates, n_samples);
        }
      }
      else {
        this->generate_coord_offset(leg,(-init_step + 2*leg)/6, start);
        this->generate_coord_offset(leg,(-init_step + 2*leg - 1)/6, target);
        if (this->direction == 't'){
          generate_partial_circle(start, target, coordinates, n_samples);
        }
        else {
          generate_straight_line(start, target, coordinates, n_samples);
        }
      }
      write_data_to_lists(this->leg_list[leg], leg, coordinates, coord_temp, pwm_temp, n_samples);
      for (short sample = 0; sample < n_samples; sample++) {
        for (short ii = 0; ii < 3; ii++) {
          this->init_coord_list[init_step][sample][leg][ii] = coord_temp[sample][leg][ii];
          this->init_pwm_list[init_step][sample][leg][ii] = pwm_temp[sample][leg][ii];
        }
      }
    }
    if (n_samples < n_gait_max) {
      for (short sample = n_samples; sample < n_gait_max; ++sample) {
        for (short leg = 0; leg < 4; ++leg) {
          for (short ii = 0; ii < 3; ++ii) {
            this->init_coord_list[init_step][sample][leg][ii] = -1000;
            this->init_pwm_list[init_step][sample][leg][ii] = -1;
          }
        }
      }
    }
  }
}

void Gait::generate_gait_sequence(short n_samples){
  const short gait_sequence[7] = {0, 1, 2, 3, 0, 1, 2}; // which leg needs to do the step -> ... RF, LF, RB, LB, ...
  float coordinates[n_samples][3];
  float coord_temp[n_samples][4][3];
  short pwm_temp[n_samples][4][3];
  float start[3];
  float target[3];
  for (short step = 0; step <4; step++) {
    this->generate_coord_offset(gait_sequence[step], -1/2, start);
    this->generate_coord_offset(gait_sequence[step], 1/2, target);
    generate_step(start, target, this->step_height, coordinates, n_samples);
    write_data_to_lists(this->leg_list[gait_sequence[step]], step, coordinates, coord_temp, pwm_temp, n_samples);
    for (short leg = 1; leg < 4; leg++) {
      this->generate_coord_offset(gait_sequence[step + leg], -1/2 + leg/3, start);
      this->generate_coord_offset(gait_sequence[step + leg], -1/2 + (leg - 1)/3, target);
      if (this->direction == 't'){
        generate_partial_circle(start, target, coordinates, n_samples);
      }
      else {
        generate_straight_line(start, target, coordinates, n_samples);
      }
      write_data_to_lists(this->leg_list[gait_sequence[step + leg]], gait_sequence[step + leg], coordinates, coord_temp, pwm_temp, n_samples);
    }
    for (short sample = 0; sample < n_samples; sample++){
      for (short leg = 0; leg < 4; leg++) {
        for (short ii = 0; ii < 3; ii++) {
          this->coord_list[step][sample][leg][ii] = coord_temp[sample][leg][ii];
          this->pwm_list[step][sample][leg][ii] = pwm_temp[sample][leg][ii];
        }
      }
    }
    if (n_samples < n_gait_max) {
      for (short sample = n_samples; sample < n_gait_max; ++sample) {
        for (short leg = 0; leg < 4; ++leg) {
          for (short ii = 0; ii < 3; ++ii) {
            this->coord_list[step][sample][leg][ii] = -1000;
            this->pwm_list[step][sample][leg][ii] = -1;
          }
        }
      }
    }
  }
}

float Gait::get_coordinate_from_list(short step, short sample, short leg, short ii, bool init) {
  if (init) {
    return this->init_coord_list[step][sample][leg][ii];
  }
  else {
    return this->coord_list[step][sample][leg][ii];
  }
}

short Gait::get_pwm_from_list(short step, short sample, short leg, short ii, bool init) {
  if (init) {
    return this->init_pwm_list[step][sample][leg][ii];
  }
  else {
    return this->pwm_list[step][sample][leg][ii];
  }
}

short Gait::get_sample_no() {
  return this->total_samples_per_step;
}

void Gait::set_velocity(float velocity) {

  short value = short(round(this->step_length / 4 / velocity * this->freq));
  if (value < n_gait_min){
    this->total_samples_per_step = n_gait_min;
    this->velocity = abs(this->step_length) / 4 / n_gait_min * this->freq;
  }
  else if (value > n_gait_max){
    this->total_samples_per_step =n_gait_max;
    this->velocity = abs(this->step_length) / 4 / n_gait_max * this->freq;
  }
  else {
    this->total_samples_per_step = value;
    this->velocity = velocity;
  }
  // these calls seems to crash stuff!
  this->generate_init_gait_sequence(this->total_samples_per_step);
  this->generate_gait_sequence(this->total_samples_per_step);
}

void Gait::get_name(char name[2]) {

  if (this->inv_direction){
    name[0] = '-';
  }
  else {
    name[0] = '+';
  }
  name[1] = this->direction;
}
