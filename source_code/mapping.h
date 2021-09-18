#ifndef MAPPING
#define MAPPING


typedef struct mapping *Mapping;


int mp_init(Mapping *mp);
int mp_close(Mapping *mp);

int mp_loadMaps(Mapping mp, char *name);
int mp_freeMaps(Mapping mp);
int mp_setQtdLayers(Mapping mp, int layers);
int mp_getDiscreteDimension(Mapping mp, int **x, int **y);
int mp_getContinuousDimension(Mapping mp, double **x, double **y);
int mp_getPhysicalMapPointer(Mapping mp, int ***pointer);
int mp_getPheromoneMapPointer(Mapping mp, double ***pointer, int layer);

int mp_setNeighborhoodType(Mapping mp, int neighborhood_type);
int mp_getNeighborhoodType(Mapping mp, int *neighborhood_type);

int mp_setDetectionRadius(Mapping mp, int detection_radius);
int mp_getDetectionRadius(Mapping mp, int *detection_radius);

int mp_setDecisionRadius(Mapping mp, int decision_radius);
int mp_getDecisionRadius(Mapping mp, int *decision_radius);

int mp_setChoiceStrategy(Mapping mp, int choice_strategy);
int mp_getChoiceStrategy(Mapping mp, int *choice_strategy);

int mp_setStochasticPercentage(Mapping mp, double stochastic_perc);
int mp_getStochasticPercentage(Mapping mp, double *stochastic_perc);

int mp_setElitistPercentage(Mapping mp, double elitist_perc);
int mp_getElitistPercentage(Mapping mp, double *elitist_perc);

int mp_setInertialForce(Mapping mp, double inertial_force);
int mp_getInertialForce(Mapping mp, double *inertial_force);

int mp_setEvaporationRate(Mapping mp, double evaporation_rate, int layer);
int mp_getEvaporationRate(Mapping mp, double *evaporation_rate, int layer);


int mp_pheromoneDetection(Mapping mp, int *current_position, int layer);
int mp_cellDecision(Mapping mp, int *current_position, int *goal_position, int **blocked_positions, int size);
int mp_pheromoneDiffusion(Mapping mp, int *current_position, int layer);
int mp_pheromoneEvaporation(Mapping mp);

int mp_checkRoom(Mapping mp, int *cur_pos, int *room);

void mp_printMaps(Mapping mp, int **blocked_positions);

#endif

