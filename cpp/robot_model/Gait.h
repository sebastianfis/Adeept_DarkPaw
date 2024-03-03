#ifndef Gait_h
#define Gait_h
#include "SpiderLeg.h"

void generate_step(float starting_point[3], float end_point[3], float step_height, float result[][3], short n);
void generate_straight_line(float starting_point[3], float end_point[3], float result[][3], short n);
void generate_partial_circle(float starting_point[3], float end_point[3], float result[][3], short n);

// class Gait {
//   private:
    
//   public:

// };
#endif