#include "Gait.h"
#include "SpiderLeg.h"
#include "math.h"

const short n_gait_max = 25; // Set the max number of samples
const short n_gait_min = 4; // Set the min number of samples
const short gait_sequence[] = {0, 1, 2, 3, 0, 1, 2}; // which leg needs to do the step -> ... RF, LF, RB, LB, ...

void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n) {
    // float l_step = sqrt(sq(end_point[0] - starting_point[0]) + sq(end_point[1] - starting_point[1]));
    //float delta_x = (end_point[0] - starting_point[0]) / (n-1);
    //float  = (end_point[1] - starting_point[1]) / (n-1);
    float z_0;
    z_0 = max(starting_point[2], end_point[2]);
    for (short i = 0; i < n; i++) {
      result[i][0] = starting_point[0] + float(i)/(n-1) * (end_point[0] - starting_point[0]);
      result[i][1] = starting_point[1] + float(i)/(n-1) * (end_point[1] - starting_point[1]);
      // next line is still not working!!!
      result[i][2] = z_0  - sqrt(1 - 4 * sq(float(i)/(n-1) - 0.5) + float(1e-10)) * step_height; // 1e-10 to avoid floating number rounding resulting in < 0 number!
    }
}

void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n) {
    float delta_x = (end_point[0] - starting_point[0]) / (n-1);
    float delta_y = (end_point[1] - starting_point[1]) / (n-1);
    float delta_z = (end_point[2] - starting_point[2]) / (n-1);

    for (short i = 0; i < n; i++) {
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

    for (short i = 0; i < n; i++) {
        float phi = phi_0 + i * delta_phi;
        result[i][0] = r * cos(phi);
        result[i][1] = r * sin(phi);
        result[i][2] = starting_point[2] + float(i) * (end_point[2] - starting_point[2]) / (n - 1);
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
  for (short leg = 0; leg < 4; leg++){
    float r_i= leg_list[leg]->get_init_r();
    r += r_i;
    phi += 2 * asin(step_length / 2 /r_i);
  }
  this->r = r/4;
  if (this->direction == 't') {
    step_length = phi/4;
  }
  if (this->inv_direction) {
    this->step_length = - step_length;
  }
  else {
    this->step_length = step_length;
  }
  this->set_velocity(velocity);

}

void Gait::generate_coord_offset(short leg_no, float offset, float result[3]){
  char x = 'x';
  char y = 'y';
  char t = 't';
  this->leg_list[leg_no]->get_init_pos(result);
  if (this->direction == x){
    result[0] += offset * this->step_length;
  }
  else if (this->direction == y){
    result[1] += offset * this->step_length;
  }
  else if (this->direction == t){
    result[0] = this->r * sin(this->leg_list[leg_no]->get_init_phi() + offset * this->step_length);
    result[1] = this->r * cos(this->leg_list[leg_no]->get_init_phi() + offset * this->step_length);
  }
}

void Gait::write_data_to_lists(float coordinates[][3], short step_no, short leg_no, short n_samples, bool init){
  float angles[n_samples][3];
  short PWM_values[n_samples][3];
  this->leg_list[leg_no]->calc_trajectory(coordinates, angles, n_samples);
  this->leg_list[leg_no]->calc_PWM(angles, PWM_values, n_samples);
  for (short sample = 0; sample < n_samples; ++sample) {  
    for (short ii = 0; ii < 3; ++ii) {
      if (init) {
        this->init_coord_list[step_no][sample][leg_no][ii] = coordinates[sample][ii];
        this->init_pwm_list[step_no][sample][leg_no][ii] = PWM_values[sample][ii];
      }
      else {
        this->coord_list[step_no][sample][leg_no][ii] = coordinates[sample][ii];
        this->pwm_list[step_no][sample][leg_no][ii] = PWM_values[sample][ii];
      }
    }
  }
  if (n_samples < n_gait_max) {
    for (short sample = n_samples; sample < n_gait_max; ++sample) {        
      for (short ii = 0; ii < 3; ++ii) {
        if (init) {
        this->init_coord_list[step_no][sample][leg_no][ii] = -1000;
        this->init_pwm_list[step_no][sample][leg_no][ii] = -1;
        }
        else {
          this->coord_list[step_no][sample][leg_no][ii] = -1000;
          this->pwm_list[step_no][sample][leg_no][ii] = -1;
        }    
      }
    } 
  }
}

void Gait::generate_init_gait_sequence(short n_samples) {
  // init_gait_sequnce[] = {1, 2, 3}; // which leg needs to do the step -> LF, RB, LB, ... 
  float coordinates[n_samples][3];
  float start[3];
  float target[3];
  char t = 't';
  short comp;
  float start_offset;
  float target_offset;
  for (short init_step = 0; init_step < 3; init_step++) {
    comp = init_step + 1; 
    // stepping into the conditions does not yet work as expected: 
    // Steps are NEVER excuted, instead final value is immediatly target, no interpolation is happening, z stays constant...
    // if ((leg_no == 0) || (leg_no > comp)) evalutes to true, as expected, but no offset is added...
    // else clause seems to work
    for (short leg_no = 0; leg_no < 4; leg_no++) {
      // CALCULATE OFFSET
      if (leg_no == comp) { // this leg needs to make the step!
        start_offset = - float(init_step)/6;
        target_offset = (float(init_step) + 1)/6;
      }
      else if ((leg_no == 0) || (leg_no > comp)) { //this is only true for legs that have not made a step yet!
        start_offset = - float(init_step)/6;
        target_offset = - float(init_step+1)/6;
      }
      else {//all reamining legs have already made a step!
        start_offset = float(-init_step + 2*leg_no)/6;
        target_offset = float(-init_step + 2*leg_no - 1)/6;
      }
      // CALCULATE STEP MOVEMENT
      this->generate_coord_offset(leg_no, start_offset, start);
      this->generate_coord_offset(leg_no, target_offset, target);
      if (leg_no == comp) { // this leg needs to make the step!
        generate_step(start, target, this->step_height, coordinates, n_samples);
      }
      else { // all others move on the ground
        if (this->direction == t){
          generate_partial_circle(start, target, coordinates, n_samples);
        }
        else {
          generate_straight_line(start, target, coordinates, n_samples);
        }
      }
      this->write_data_to_lists(coordinates, init_step, leg_no, n_samples, true); 
    }
  }
}

void Gait::generate_gait_sequence(short n_samples){
  const short gait_sequence[7] = {0, 1, 2, 3, 0, 1, 2}; // which leg needs to do the step -> ... RF, LF, RB, LB, ...
  float coordinates[n_samples][3];
  float start[3];
  float target[3];
  for (short step = 0; step < 4; step++) {
    this->generate_coord_offset(gait_sequence[step], float(-1)/2, start);
    this->generate_coord_offset(gait_sequence[step], float(1)/2, target);
    generate_step(start, target, this->step_height, coordinates, n_samples);
    this->write_data_to_lists(coordinates, step, gait_sequence[step], n_samples, false);
    for (short leg_no = 1; leg_no < 4; leg_no++) {
      this->generate_coord_offset(gait_sequence[step + leg_no], float(-1)/2 + float(leg_no)/3, start);
      this->generate_coord_offset(gait_sequence[step + leg_no], float(-1)/2 + float(leg_no - 1)/3, target);
      if (this->direction == 't'){
        generate_partial_circle(start, target, coordinates, n_samples);
      }
      else {
        generate_straight_line(start, target, coordinates, n_samples);
      }
      this->write_data_to_lists(coordinates, step, gait_sequence[step + leg_no], n_samples, false);
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
