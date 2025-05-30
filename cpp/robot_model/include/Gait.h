#ifndef Gait_h
#define Gait_h
#include "SpiderLeg.h"

void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n);
void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n);
void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n);

class Gait {
  private:
    bool inv_direction;
    float freq; 
    char direction;
    float step_length;
    float step_height;
    float velocity;
    float r;
    void generate_init_gait_sequence(short n_samples);
    void generate_gait_sequence(short n_samples);
    short total_samples_per_step;
    float init_coord_list[3][25][4][3];
    short init_pwm_list[3][25][4][3];
    float coord_list[4][25][4][3];
    short pwm_list[4][25][4][3];

  public:
    SpiderLeg* leg_list[4];
    Gait(SpiderLeg* leg_list[4]);
    void init(char direction, bool inv_direction, float step_length, float step_height, float velocity, float freq);
    float get_coordinate_from_list(short step, short sample, short leg, short ii, bool init);
    short get_pwm_from_list(short step, short sample, short leg, short ii, bool init);
    short get_sample_no();
    void write_data_to_lists(float coordinates[][3], short step_no, short leg_no, short n_samples, bool init);
    void generate_coord_offset(short leg_no, float offset, float result[3]);
    void set_velocity(float velocity);
    void get_name(char name[2]);
};
#endif