#include "SpiderLeg.h"
#include "Gait.h"

const int n = 100; // Set the number of steps

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
