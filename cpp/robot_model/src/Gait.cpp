#include "Gait.h"
#include "SpiderLeg.h"
#include "math.h"

/*
 * Limits on the number of samples per gait step.
 * These bounds keep trajectories smooth but computationally reasonable.
 */
const short n_gait_max = 25; // Maximum number of samples per step
const short n_gait_min = 4; // Minimum number of samples per step

/*
 * Leg stepping order.
 * Indices correspond to legs:
 * 0 = RF, 1 = LF, 2 = RB, 3 = LB
 *
 * This sequence defines which leg performs the swing phase.
 */
const short gait_sequence[] = {0, 1, 2, 3, 0, 1, 2};

/*
 * Generate a stepping trajectory from starting_point to end_point.
 *
 * - X and Y move linearly
 * - Z follows a smooth arch (swing phase)
 * - step_height defines the maximum foot clearance
 */
void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n) {
    // Ground reference height (use the higher of start/end)
    float z_0;
    z_0 = max(starting_point[2], end_point[2]);

    for (short i = 0; i < n; i++) {
      // Linear interpolation in X
      result[i][0] = starting_point[0] + float(i)/(n-1) * (end_point[0] - starting_point[0]);
      // Linear interpolation in Y
      result[i][1] = starting_point[1] + float(i)/(n-1) * (end_point[1] - starting_point[1]);
        /*
         * Z trajectory:
         * Creates a smooth arc using a semi-circular profile:
         *
         *   z = z0 - sqrt(1 - 4*(t-0.5)^2) * step_height
         *
         * 1e-10 prevents numerical issues due to floating-point rounding.
         */
      result[i][2] = z_0  - sqrtf(1 - 4 * sq(float(i)/(n-1) - 0.5) + float(1e-10)) * step_height;
    }
}

/*
 * Generate a straight-line trajectory between two points.
 * Used for stance phase where the foot remains on the ground.
 */
void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n) {
    // Per-sample increments
    float delta_x = (end_point[0] - starting_point[0]) / (n-1);
    float delta_y = (end_point[1] - starting_point[1]) / (n-1);
    float delta_z = (end_point[2] - starting_point[2]) / (n-1);

    for (short i = 0; i < n; i++) {
        result[i][0] = starting_point[0] + i * delta_x;
        result[i][1] = starting_point[1] + i * delta_y;
        result[i][2] = starting_point[2] + i * delta_z;
    }
}

/*
 * Generate a partial circular trajectory in the XY plane.
 * Used for turning (rotational gait).
 */
void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n) {
    // Radius from origin
    float r = sqrt(sq(starting_point[0]) + sq(starting_point[1]));
    // Initial and final polar angles
    float phi_0 = atan2(starting_point[1], starting_point[0]);
    float phi_end = atan2(end_point[1], end_point[0]);
    float delta_phi = (phi_end - phi_0) / (n - 1);

    for (short i = 0; i < n; i++) {
        float phi = phi_0 + i * delta_phi;
        // Circular motion in XY
        result[i][0] = r * cos(phi);
        result[i][1] = r * sin(phi);
        // Linear interpolation in Z
        result[i][2] = starting_point[2] + float(i) * (end_point[2] - starting_point[2]) / (n - 1);
    }
}

/*
 * Gait constructor.
 * Stores pointers to the four SpiderLeg objects.
 */
Gait::Gait(SpiderLeg* leg_list[4]){
    this->leg_list[0] = leg_list[0];
    this->leg_list[1] = leg_list[1];
    this->leg_list[2] = leg_list[2];
    this->leg_list[3] = leg_list[3];
}

/*
 * Initialize gait parameters.
 *
 * - direction: 'x', 'y', or 't' (translation or turning)
 * - inv_direction: reverse direction flag
 * - step_length: linear or angular step size
 * - step_height: foot lift height
 * - velocity: desired body velocity
 * - freq: controller update frequency
 */
void Gait::init(char direction, bool inv_direction, float step_length, float step_height, float velocity, float freq=50) {
  this->inv_direction = inv_direction;
  this->freq = freq;
  this->direction = direction;
  this->step_height = step_height;
  float r =0;
  float phi = 0;

    /*
     * Compute average leg radius and equivalent angular step
     * for turning gaits.
     */
  for (short leg = 0; leg < 4; leg++){
    float r_i= this->leg_list[leg]->get_init_r();
    r += r_i;
    phi += 2 * asin(step_length / 2 /r_i);
  }
  this->r = r/4;
  // Convert linear step length to angular step for turning
  if (this->direction == 't') {
    step_length = phi/4;
  }
  // Apply direction inversion
  if (this->inv_direction) {
    this->step_length = - step_length;
  }
  else {
    this->step_length = step_length;
  }
  // Compute timing and generate gait tables
  this->set_velocity(velocity);
}

/*
 * Compute a foot position offset from the initial position.
 * The offset is normalized in the range [-0.5, 0.5].
 */
void Gait::generate_coord_offset(short leg_no, float offset, float result[3]){
  char x = 'x';
  char y = 'y';
  char t = 't';
  // Start from initial foot position
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

/*
 * Convert a trajectory into actuator angles and PWM signals,
 * then store them in the gait tables.
 */
void Gait::write_data_to_lists(float coordinates[][3], short step_no, short leg_no, short n_samples, bool init){
  float angles[n_samples][3];
  short PWM_values[n_samples][3];
  // Inverse kinematics
  this->leg_list[leg_no]->calc_trajectory(coordinates, angles, n_samples);
  // Angle → PWM conversion
  this->leg_list[leg_no]->calc_PWM(angles, PWM_values, n_samples);
  // Store valid samples
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
  // Pad unused samples with sentinel values
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

/*
 * Generate the initial gait sequence.
 *
 * This function creates a smooth transition from the robot's
 * initial standing pose into a stable walking gait.
 *
 * The initialization consists of 3 preparatory steps where
 * legs are repositioned one by one to reach a valid phase
 * offset for the cyclic gait.
 */
void Gait::generate_init_gait_sequence(short n_samples) {
  // Coordinates of the generated trajectory
  float coordinates[n_samples][3];
  // Start and end positions for each leg
  float start[3];
  float target[3];
  char t = 't';   // turning gait identifier
  short comp;     // index of the leg that performs the swing phase
  float start_offset;
  float target_offset;
  /*
   * The initialization consists of 3 steps.
   * In each step, a different leg performs a swing motion.
   */
  for (short init_step = 0; init_step < 3; init_step++) {
    // The leg that moves during this initialization step
    comp = init_step + 1;
    for (short leg_no = 0; leg_no < 4; leg_no++) {

      /*
       * Compute phase offsets depending on:
       * - which leg is currently stepping
       * - which legs have already stepped
       *
       * Offsets are fractions of the step length and
       * determine the relative foot position along the gait cycle.
       */
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
      /*
       * Convert offsets into actual Cartesian coordinates
       */
      this->generate_coord_offset(leg_no, start_offset, start);
      this->generate_coord_offset(leg_no, target_offset, target);

      /*
       * Generate the trajectory:
       * - Swing leg → stepping trajectory (arched)
       * - Other legs → ground trajectory (straight or circular)
       */
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
      // Store generated data in initialization buffers
      this->write_data_to_lists(coordinates, init_step, leg_no, n_samples, true);
    }
  }
}

/*
 * Generate the cyclic walking gait sequence.
 *
 * This function produces the repeating gait pattern used
 * after initialization.
 */
void Gait::generate_gait_sequence(short n_samples){
  const short gait_sequence[7] = {0, 1, 2, 3, 0, 1, 2}; // which leg needs to do the step -> ... RF, LF, RB, LB, ...
  float coordinates[n_samples][3];
  float start[3];
  float target[3];

  /*
   * Each gait cycle consists of 4 steps,
   * where exactly one leg is in swing phase.
   */
  for (short step = 0; step < 4; step++) {
    // Generate swing trajectory for the stepping leg
    this->generate_coord_offset(gait_sequence[step], float(-1)/2, start);
    this->generate_coord_offset(gait_sequence[step], float(1)/2, target);
    generate_step(start, target, this->step_height, coordinates, n_samples);
    this->write_data_to_lists(coordinates, step, gait_sequence[step], n_samples, false);
    /*
     * Remaining legs perform stance phase motion
     * to compensate body movement.
     */
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

/*
 * Retrieve a Cartesian coordinate from the gait tables.
 *
 * Parameters select:
 * - step number
 * - sample index
 * - leg index
 * - coordinate index (x/y/z)
 * - whether to use initialization or normal gait
 */
float Gait::get_coordinate_from_list(short step, short sample, short leg, short ii, bool init) {
  if (init) {
    return this->init_coord_list[step][sample][leg][ii];
  }
  else {
    return this->coord_list[step][sample][leg][ii];
  }
}

/*
 * Retrieve a PWM value from the gait tables.
 */
short Gait::get_pwm_from_list(short step, short sample, short leg, short ii, bool init) {
  if (init) {
    return this->init_pwm_list[step][sample][leg][ii];
  }
  else {
    return this->pwm_list[step][sample][leg][ii];
  }
}

/*
 * Get the number of samples per gait step.
 */
short Gait::get_sample_no() {
  return this->total_samples_per_step;
}

/*
 * Set gait velocity.
 *
 * This function:
 * - Converts velocity into sample count
 * - Clamps sample count to allowed limits
 * - Regenerates gait trajectories accordingly
 */
void Gait::set_velocity(float velocity) {
  float step_length;
  // Convert angular step length into linear distance if turning
  if (this->direction == 't') {
    step_length = sin(this->step_length / 2) * 2 * this->r;
  }
  else{
    step_length = this->step_length;
  }

  /*
   * Compute required number of samples per step
   * based on desired velocity and update frequency.
   */
  short value = short(round(step_length / 4 / velocity * this->freq));
  if (value < n_gait_min){
    this->total_samples_per_step = n_gait_min;
    this->velocity = abs(step_length) / 4 / n_gait_min * this->freq;
  }
  else if (value > n_gait_max){
    this->total_samples_per_step =n_gait_max;
    this->velocity = abs(step_length) / 4 / n_gait_max * this->freq;
  }
  else {
    this->total_samples_per_step = value;
    this->velocity = velocity;
  }

  // Regenerate gait tables
  this->generate_init_gait_sequence(this->total_samples_per_step);
  this->generate_gait_sequence(this->total_samples_per_step);
}

/*
 * Generate a human-readable gait name.
 *
 * Example:
 * "+x" → forward
 * "-y" → backward
 * "+t" → clockwise rotation
 */
void Gait::get_name(char name[2]) {
  if (this->inv_direction){
    name[0] = '-';
  }
  else {
    name[0] = '+';
  }
  name[1] = this->direction;
}
