#ifndef GROUND_TRUTH_LAYER_CONFIG_H
#define GROUND_TRUTH_LAYER_CONFIG_H

#define NUM_MAPPING_THREADS 4
#define NUM_FREE_SPACE_CELLS_SKIPPED 4
// how big is the obstacle given by each laser beam (in pixel)
#define LASER_BEAM_WIDTH 1

#define LOG_OCCUPANCY 2.5 // log(p(z=1 | s = 1) / p(z=1 | s = 0))
#define LOG_FREE -1     // log(p(z=0 | s = 1) / p(z=0 | s = 0))

#define LOG_OCCUPANCY_MAX 50
#define LOG_OCCUPANCY_MIN -50

#endif // GROUND_TRUTH_LAYER_CONFIG_H
