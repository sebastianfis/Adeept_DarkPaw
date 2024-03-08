#ifndef Gait_h
#define Gait_h
#include "SpiderLeg.h"

void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n);
void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n);
void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n);

class Gait {
  //finish implementing this!
  private:
    bool inv_direction;
    float freq; 
    char direction;
    float step_length;
    float step_height;
    float velocity;
    float r;
    void generate_init_gait_sequence(float init_pos_alloc[][3][4][3],short init_pwm_alloc[][3][4][3]);
    void generate_gait_sequence(float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]);
    void generate_coord_offset(short leg_no, float offset, float result[3]);
    short cur_step_no;
    short total_samples_per_step;

  public:
    SpiderLeg* leg_list[4];
    Gait(SpiderLeg* leg_list[4], float step_length, float step_height, float velocity, float freq, bool inv_direction, char direction);
    void init(float init_pos_alloc[][3][4][3],short init_pwm_alloc[][3][4][3], float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]);
    void get_next_coordinate(float coord_list[4][3], short coord_list_PWM[4][3]);
    void set_velocity(float velocity, float init_pos_alloc[][3][4][3], short init_pwm_alloc[][3][4][3], float pos_alloc[][4][4][3], short pwm_alloc[][4][4][3]);
    short get_cur_step_no();
    char* get_name();
};
#endif