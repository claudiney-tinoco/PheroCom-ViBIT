/* 
 * 17/09/2021
 * Claudiney R. Tinoco
 */
 
#include "mapping.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#define PI 3.1415926535
#define e 2.7182818284


struct cell {
	int x, y;
	double pheromone;
};

struct mapping {
	char *name;
	double xC, yC;
	int xD, yD, layers;
	int **physical_map;
	double ***pheromone_map;
	
	int neighborhood_type;
	int detection_radius;
	int decision_radius;
	
	int neighbor_size;
	struct cell *neighbor_cells;
	
	int choice_strategy;
	double elitist_perc;
	double stochastic_perc;
	double inertial_force;
	
	double *evaporation_rates;
};


int mp_toDiscreteCoordinates(Mapping mp, double *current_position, int *goal_position);
int mp_toContinuousCoordinates(Mapping mp, int *current_position, double *goal_position);
int mp_getQtdNeighborCells(Mapping mp, int radius, int *qtdCells);
int mp_getQtdBorderCells(Mapping mp, int radius, int *qtdCells);
int mp_sortCellVector(struct cell *vector, int size);
int mp_swapCells(struct cell *vector, int a, int b);
void mp_removeBlockedPositions(struct cell *vet1, int *size1, int **vet2, int size2);
int mp_returnDirection(int *current_position, int x, int y, int *direction);


/* Input:
 * Process:
 * Output:
 */
int mp_init(Mapping *mp)
{
	if((*mp == NULL) && (*mp = (struct mapping *) malloc(sizeof(struct mapping))) != NULL) 
	{
		(*mp)->name = NULL;
		(*mp)->xC = 0.0;
		(*mp)->yC = 0.0;
		(*mp)->xD = 0;
		(*mp)->yD = 0;
		(*mp)->layers = 1;
		(*mp)->physical_map = NULL;
		(*mp)->pheromone_map = NULL;
		
		(*mp)->neighborhood_type = 1;
		(*mp)->detection_radius = 1;
		(*mp)->decision_radius = 1;
		
		(*mp)->neighbor_size = 0;
		(*mp)->neighbor_cells = NULL;
		
		(*mp)->choice_strategy = 1;
		(*mp)->elitist_perc = 0.3;
		(*mp)->stochastic_perc = 0.3;
		(*mp)->inertial_force = 2.0;
		
		if(!mp_setDetectionRadius(*mp, 1)) {
			mp_close(mp);
			return 0;
		}
		if(((*mp)->evaporation_rates = (double *) malloc(sizeof(double))) != NULL) {
			(*mp)->evaporation_rates[0] = 0.005;
			return 1;
		}
	}
	mp_close(mp);
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_close(Mapping *mp)
{
	if(*mp != NULL)
	{
		mp_freeMaps(*mp);
		free((*mp)->neighbor_cells);
		free((*mp)->evaporation_rates);
		free(*mp);
		*mp = NULL;
		return 1;
	}
	return 0;
}


// ############################################################################################################
/* Input:
 * Process:
 * Output:
 */
int mp_loadMaps(Mapping mp, char *name)
{
	int fail = 0;
	int i, j, k;
	char buffer1[20], buffer2[20];
	FILE *file;
	

	if((mp != NULL) && ((file = fopen(name, "r")) != NULL))
	{
		mp_freeMaps(mp);
		
		mp->name = (char *) malloc((strlen(name) + 2) * sizeof(char));
		sprintf(mp->name, "%s", name);
		
		fscanf(file, "%lf %lf", &(mp->xC), &(mp->yC));
		fscanf(file, "%d %d", &(mp->xD), &(mp->yD));
		sprintf(buffer1, "%.4lf", (mp->xC / mp->xD));
		sprintf(buffer2, "%.4lf", (mp->yC / mp->yD));
		if((strcmp(buffer1, buffer2) != 0) || (strcmp(buffer1, "0.2560") != 0)) {
			fail = 1;
		}
		
		if((!fail) && ((mp->physical_map = (int **) malloc(mp->yD * sizeof(int *))) == NULL)) {
			fail = 1;
		}
		for(i = 0; ((!fail) && (i < mp->yD)); i++) {
			if((mp->physical_map[i] = (int *) malloc(mp->xD * sizeof(int))) == NULL) {
				fail = 1;
			}
			for(j = 0; ((!fail) && (j < mp->xD)); j++) {
				fscanf(file, "%d", &(mp->physical_map[i][j]));
			}
		}
		
		if((!fail) && ((mp->pheromone_map = (double ***) malloc(mp->layers * sizeof(double **))) == NULL)) {
			fail = 1;
		}
		for(i = 0; ((!fail) && (i < mp->layers)); i++) {
			if((mp->pheromone_map[i] = (double **) malloc(mp->yD * sizeof(double *))) == NULL) {
				fail = 1;
			}
			for(j = 0; ((!fail) && (j < mp->yD)); j++) {
				if((mp->pheromone_map[i][j] = (double *) malloc(mp->xD * sizeof(double))) == NULL) {
					fail = 1;
				}
				for(k = 0; ((!fail) && (k < mp->xD)); k++) {
					mp->pheromone_map[i][j][k] = 0.00001;
				}
			}
		}
		
		fclose(file);
		if(!fail) {
			return 1;
		}
		mp_freeMaps(mp);
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_freeMaps(Mapping mp)
{
	int i, j;
	
	
	if(mp != NULL)
	{
		free(mp->name);
		if(mp->physical_map != NULL)
		{
			for(i = 0; i < mp->yD; i++) {
				free(mp->physical_map[i]); 
			}
			free(mp->physical_map);
		}
		
		if(mp->pheromone_map != NULL)
		{
			for(i = 0; i < mp->layers; i++) {
				for(j = 0; j < mp->yD; j++) {
					free(mp->pheromone_map[i][j]);
				}
				free(mp->pheromone_map[i]);
			}
			free(mp->pheromone_map);
		}
		
		mp->name = NULL;
		mp->xC = 0.0;
		mp->yC = 0.0;
		mp->xD = 0;
		mp->yD = 0;
		mp->physical_map = NULL;
		mp->pheromone_map = NULL;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setQtdLayers(Mapping mp, int layers)
{
	char buffer[30];
	
	
	if((mp != NULL) && (layers > 0))
	{
		if(mp->layers == layers) {
			return 1;
		}
		if(mp->name != NULL) {
			strcpy(buffer, mp->name);
			mp_freeMaps(mp);
			mp->layers = layers;
			return mp_loadMaps(mp, buffer);
		}
		mp->layers = layers;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getDiscreteDimension(Mapping mp, int **x, int **y)
{
	if(mp != NULL) 
	{
		*x = &(mp->xD);
		*y = &(mp->yD);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getContinuousDimension(Mapping mp, double **x, double **y)
{
	if(mp != NULL) 
	{
		*x = &(mp->xC);
		*y = &(mp->yC);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getPhysicalMapPointer(Mapping mp, int ***pointer)
{
	if((mp != NULL) && (mp->physical_map != NULL)) 
	{
		*pointer = mp->physical_map;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getPheromoneMapPointer(Mapping mp, double ***pointer, int layer)
{
	if((mp != NULL) && (mp->pheromone_map != NULL)) 
	{
		if((layer >= 0) && (layer < mp->layers)) {
			*pointer = &(mp->pheromone_map[layer][0]);
			return 1;
		}
	}
	return 0;
}


// ############################################################################################################
/* Input:
 * Process:
 * Output:
 */
int mp_setNeighborhoodType(Mapping mp, int neighborhood_type)
{	
	if((mp != NULL) && (neighborhood_type >= 1) && (neighborhood_type <= 1))
	{
		if(mp->neighborhood_type != neighborhood_type) {
			mp->neighborhood_type = neighborhood_type;
			mp_setDetectionRadius(mp, mp->detection_radius);
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getNeighborhoodType(Mapping mp, int *neighborhood_type)
{
	if(mp != NULL)
	{
		*neighborhood_type = mp->neighborhood_type;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setDetectionRadius(Mapping mp, int detection_radius)
{
	int qtdCells;
	
	
	if((mp != NULL) && (detection_radius > 0))
	{
		if((mp->detection_radius == detection_radius) && (mp->neighbor_cells != NULL)) {
			mp->neighbor_size = 0;
			return 1;
		}
		
		free(mp->neighbor_cells);
		mp_getQtdNeighborCells(mp, detection_radius, &qtdCells);
		if((mp->neighbor_cells = (struct cell *) malloc(qtdCells * sizeof(struct cell))) == NULL) {
			mp->detection_radius = -1;
			return 0;
		}
		
		mp->neighbor_size = 0;
		mp->detection_radius = detection_radius;
		
		if(mp->decision_radius > mp->detection_radius) {
			mp->decision_radius = mp->detection_radius;
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getDetectionRadius(Mapping mp, int *detection_radius)
{
	if(mp != NULL) 
	{
		*detection_radius = mp->detection_radius;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setDecisionRadius(Mapping mp, int decision_radius)
{	
	if((mp != NULL) && (decision_radius > 0))
	{
		if(decision_radius <= mp->detection_radius) {
			mp->decision_radius = decision_radius;
		}
		else {
			mp->decision_radius = mp->detection_radius;
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getDecisionRadius(Mapping mp, int *decision_radius)
{
	if(mp != NULL) 
	{
		*decision_radius = mp->decision_radius;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setChoiceStrategy(Mapping mp, int choice_strategy)
{
	if((mp != NULL) && (choice_strategy >= 1) && (choice_strategy <= 5)) 
	{
		mp->choice_strategy = choice_strategy;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getChoiceStrategy(Mapping mp, int *choice_strategy)
{
	if(mp != NULL) 
	{
		*choice_strategy = mp->choice_strategy;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setStochasticPercentage(Mapping mp, double stochastic_perc)
{
	if((mp != NULL) && (stochastic_perc >= 0.0) && (stochastic_perc <= 1.0)) 
	{
		mp->stochastic_perc = stochastic_perc;
		if((mp->stochastic_perc + mp->elitist_perc) > 1.0) {
			mp->elitist_perc = 1.0 - mp->stochastic_perc;
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getStochasticPercentage(Mapping mp, double *stochastic_perc)
{
	if(mp != NULL) 
	{
		*stochastic_perc = mp->stochastic_perc;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setElitistPercentage(Mapping mp, double elitist_perc)
{
	if((mp != NULL) && (elitist_perc >= 0.0) && (elitist_perc <= 1.0)) 
	{
		mp->elitist_perc = elitist_perc;
		if((mp->elitist_perc + mp->stochastic_perc) > 1.0) {
			mp->stochastic_perc = 1.0 - mp->elitist_perc;
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getElitistPercentage(Mapping mp, double *elitist_perc)
{
	if(mp != NULL) 
	{
		*elitist_perc = mp->elitist_perc;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_setInertialForce(Mapping mp, double inertial_force)
{
	if((mp != NULL) && (inertial_force >= 1.0)) 
	{
		mp->inertial_force = inertial_force;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getInertialForce(Mapping mp, double *inertial_force)
{
	if(mp != NULL) 
	{
		*inertial_force = mp->inertial_force;
		return 1;
	}
	return 0;
}



/* Input:
 * Process:
 * Output:
 */
int mp_setEvaporationRate(Mapping mp, double evaporation_rate, int layer)
{
	if((mp != NULL) && (evaporation_rate >= 0.0) && (evaporation_rate <= 1.0)) 
	{
		if((layer >= 0) && (layer < mp->layers)) {
			mp->evaporation_rates[layer] = evaporation_rate;
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getEvaporationRate(Mapping mp, double *evaporation_rate, int layer)
{
	if(mp != NULL)
	{
		if((layer >= 0) && (layer < mp->layers)) {
			*evaporation_rate = mp->evaporation_rates[layer];
			return 1;
		}
	}
	return 0;
}


// ############################################################################################################
/* Input:
 * Process:
 * Output:
 */
int mp_pheromoneDetection(Mapping mp, int *current_position, int layer)
{
	int i, j;
	int i_start, i_end, j_start, j_end;
	int verify, robot_room, cell_room;
	int aux_vet[2];
	
	
	if((mp != NULL) && (mp->pheromone_map != NULL) && (current_position != NULL)) 
	{
		mp->neighbor_size = 0;
		i_start = 0; i_end = (mp->yD - 1); j_start = 0; j_end = (mp->xD - 1);
		
		if((current_position[1] - mp->detection_radius) > 0) {
			i_start = current_position[1] - mp->detection_radius;
		}
		if((current_position[1] + mp->detection_radius) < (mp->yD - 1)) {
			i_end = current_position[1] + mp->detection_radius;
		}
		if((current_position[0] - mp->detection_radius) > 0) {
			j_start = current_position[0] - mp->detection_radius;
		}
		if((current_position[0] + mp->detection_radius) < (mp->xD - 1)) {
			j_end = current_position[0] + mp->detection_radius;
		}
		
		mp_checkRoom(mp, current_position, &robot_room);
		for(i = i_start; i <= i_end; i++) {
			for(j = j_start; j <= j_end; j++)
			{
				aux_vet[0] = j;
				aux_vet[1] = i;
				mp_checkRoom(mp, aux_vet, &cell_room);
				
				verify = 0;
				if(mp->detection_radius > 1)
				{
					if((cell_room == robot_room) && (mp->physical_map[i][j] != 2)) {
						verify = 1;
					}
				}
				else if(mp->physical_map[i][j] != 2) {
						verify = 1;
				}
				
				if(verify) {
					mp->neighbor_cells[mp->neighbor_size].x = j;
					mp->neighbor_cells[mp->neighbor_size].y = i;
					mp->neighbor_cells[mp->neighbor_size].pheromone = mp->pheromone_map[layer][i][j];
					mp->neighbor_size++;
				}
			}
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_cellDecision(Mapping mp, int *current_position, int *goal_position, int **blocked_positions, int size)
{	
	int i, j, k;
	int draw, verify, direction;
	double aux_sum, alpha, roulette;
	
	int border_size, border_count;
	struct cell *border_pheromone;
	int radius, qt_stochastic, qt_elitist;
	
	
	if((mp != NULL) && (current_position != NULL) && (goal_position != NULL))
	{
		mp_getQtdBorderCells(mp, mp->decision_radius, &border_size);
		if((border_pheromone = (struct cell *) malloc(border_size * sizeof(struct cell))) == NULL) {
			return 0;
		}
		
		for(k = 0, border_count = 0; k < mp->neighbor_size; k++)
		{
			i = mp->neighbor_cells[k].y;
			j = mp->neighbor_cells[k].x;
			radius = fmax(fabs(i - current_position[1]), fabs(j - current_position[0]));
			
			if((radius == mp->decision_radius) && (mp->physical_map[i][j] == 0)) {
				border_pheromone[border_count].x = j;
				border_pheromone[border_count].y = i;
				border_pheromone[border_count].pheromone = 1.0 - mp->neighbor_cells[k].pheromone;
				border_count++;
			}
		}
		mp_removeBlockedPositions(border_pheromone, &border_count, blocked_positions, size);
		mp_sortCellVector(border_pheromone, border_count);
		
		// ====================================================================================================
		switch(mp->choice_strategy) {
			case 1: //Aleatorio (Todas celulas da borda tem a mesma probabilidade)
				for(i = 0; i < border_count; i++) {
					border_pheromone[i].pheromone = 0.01;
				}
				break;
				
			case 2: //Estrategia gulosa
				if(border_count > 1) {
					border_count = 1;
				}
				break;
				
			case 3: //Estrategia estocastica
				break;
				
			case 4: //Estrategia elitista
				qt_elitist = ceil(mp->elitist_perc * border_count);
				qt_stochastic = floor(mp->stochastic_perc * border_count);
				for(i = qt_elitist; i < (qt_elitist + qt_stochastic); i++) 
				{
					draw = (rand() % (border_count - i)) + i;
					if(draw > i) {
						mp_swapCells(border_pheromone, i, draw);
					}
				}
				border_count = qt_elitist + qt_stochastic;
				break;
			
			case 5: //Estrategia inercial Discreta (raio 1)
				qt_elitist = ceil(mp->elitist_perc * border_count);
				qt_stochastic = floor(mp->stochastic_perc * border_count);
				
				for(i = 0, verify = 0; i < qt_elitist; i++) 
				{
					mp_returnDirection(current_position, border_pheromone[i].x, border_pheromone[i].y, &direction);
					if(direction == current_position[2]) {
						border_pheromone[i].pheromone *= mp->inertial_force;
						verify = 1;
					}
				}
				for(i = qt_elitist; ((verify == 0) && (qt_stochastic > 0) && (i < border_count)); i++)
				{
					mp_returnDirection(current_position, border_pheromone[i].x, border_pheromone[i].y, &direction);
					if(direction == current_position[2]) {
						mp_swapCells(border_pheromone, qt_elitist, i);
						verify = 2;
					}
				}
				for(i = (verify == 2) ? (qt_elitist+1) : qt_elitist; i < (qt_elitist + qt_stochastic); i++)
				{
					draw = (rand() % (border_count - i)) + i;
					if(draw > i) {
						mp_swapCells(border_pheromone, draw, i);
					}
				}
				border_count = qt_elitist + qt_stochastic;
				break;
			
			default:
				return 0;
		}
		// ====================================================================================================
		for(i = 0, aux_sum = 0.0; i < border_count; i++) {
			aux_sum += border_pheromone[i].pheromone;
		}
		for(i = 0; i < border_count; i++) {
			border_pheromone[i].pheromone /= aux_sum;
		}
		
		roulette = (((double) rand()) / ((double) RAND_MAX));
		for(i = 0, aux_sum = 0.0; i < border_count; i++) {
			aux_sum += border_pheromone[i].pheromone;
			if(aux_sum >= roulette) {
				break;
			}
		}
		i = (i < border_count) ? i : (i - 1);
		
		mp_returnDirection(current_position, border_pheromone[i].x, border_pheromone[i].y, &direction);
		goal_position[0] = border_pheromone[i].x;
		goal_position[1] = border_pheromone[i].y;
		goal_position[2] = direction;
		
		free(border_pheromone);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_pheromoneDiffusion(Mapping mp, int *current_position, int layer)
{
	int i, j, k;
	int radius;
	double diffusion;
	
	
	if((mp != NULL) && (mp->pheromone_map != NULL) && (current_position != NULL))
	{	
		if((layer >= 0) && (layer < mp->layers))
		{
			for(k = 0; k < mp->neighbor_size; k++) 
			{
				i = mp->neighbor_cells[k].y;
				j = mp->neighbor_cells[k].x;
				
				
				radius = fmax(fabs(i - current_position[1]), fabs(j - current_position[0]));
				diffusion = 0.5 * pow((0.1 * e), (2 * (radius/PI)));

				mp->pheromone_map[layer][i][j] += ((1.0 - mp->pheromone_map[layer][i][j]) * diffusion);
				
				if(mp->pheromone_map[layer][i][j] > 1.0) {
					mp->pheromone_map[layer][i][j] = 1.0;
				}
			}
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_pheromoneEvaporation(Mapping mp)
{
	int i, j, k;
	
	
	if((mp != NULL) && (mp->pheromone_map != NULL))
	{
		for(i = 0; i < mp->layers; i++) {
			for(j = 0; j < mp->yD; j++) {
				for(k = 0; k < mp->xD; k++) 
				{
					mp->pheromone_map[i][j][k] -= (mp->pheromone_map[i][j][k] * mp->evaporation_rates[i]);
					if(mp->pheromone_map[i][j][k] < 0.00001) {
						mp->pheromone_map[i][j][k] = 0.00001;
					}
				}
			}
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getQtdNeighborCells(Mapping mp, int radius, int *qtdCells)
{
	if(mp != NULL) 
	{
		if(mp->neighborhood_type == 1) {
			*qtdCells = pow(((2 * radius) + 1), 2);
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_getQtdBorderCells(Mapping mp, int radius, int *qtdCells)
{
	if(mp != NULL) 
	{
		if(mp->neighborhood_type == 1) {
			*qtdCells = (radius > 0) ? (radius * 8) : 1;
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int mp_sortCellVector(struct cell *vector, int size)
{
	int i, j;
	
	
	for(i = 0; i < size; i++) {
		for(j = (i+1); j < size; j++) 
		{
			if (vector[i].pheromone < vector[j].pheromone) {
				mp_swapCells(vector, i, j);
			}
		}
	}
	return 1;
}


/* Input:
 * Process:
 * Output:
 */
int mp_swapCells(struct cell *vector, int a, int b)
{
	int x_aux, y_aux;
	double pheromone_aux;
	
	
	if(vector != NULL)
	{
		x_aux = vector[a].x;
		y_aux = vector[a].y;
		pheromone_aux = vector[a].pheromone;

		vector[a].x = vector[b].x;
		vector[a].y = vector[b].y;
		vector[a].pheromone = vector[b].pheromone;

		vector[b].x = x_aux;
		vector[b].y = y_aux;
		vector[b].pheromone = pheromone_aux;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
void mp_removeBlockedPositions(struct cell *vet1, int *size1, int **vet2, int size2)
{
	int i, j;
	int verify = 0;

	
	for(i = 0; i < *size1; i++) {
		for(j = 0; j < size2; j++) 
		{
			if((vet1[i].x == vet2[0][j]) && (vet1[i].y == vet2[1][j]))
			{
				vet1[i].x = vet1[(*size1) - 1].x;
				vet1[i].y = vet1[(*size1) - 1].y;
				vet1[i].pheromone = vet1[(*size1) - 1].pheromone;
				(*size1)--;
				verify = 1;
				break;
			}
		}
		if(verify) {
			i = i - 1;
			verify = 0;
		}
	}
	return;
}


/* Input:
 * Process:
 * Output:
 */
int mp_returnDirection(int *current_position, int x, int y, int *direction)
{
	if(((current_position[0] - 1) == x) && ((current_position[1] - 1) == y)) {
		*direction = 1;
	}
	else if((current_position[0] == x) && ((current_position[1] - 1) == y)) {
		*direction = 2;
	}
	else if(((current_position[0] + 1) == x) && ((current_position[1] - 1) == y)) {
		*direction = 3;
	}
	else if(((current_position[0] - 1) == x) && (current_position[1] == y)) {
		*direction = 4;
	}
	else if(((current_position[0] + 1) == x) && (current_position[1] == y)) {
		*direction = 5;
	}
	else if(((current_position[0] - 1 ) == x) && ((current_position[1] + 1) == y)) {
		*direction = 6;
	}
	else if((current_position[0] == x) && ((current_position[1] + 1) == y)) {
		*direction = 7;
	}
	else if(((current_position[0] + 1) == x) && ((current_position[1] + 1) == y)) {
		*direction = 8;
	}
	else {
		*direction = 0;
	}
	return 1;
}


/* Input:
 * Process:
 * Output:
 */
int mp_checkRoom(Mapping mp, int *cur_pos, int *room)
{
	char buffer1[10], buffer2[10];
	int i, n = 6, actual_position, times;
	double possible_sizes[] = {0.008, 0.016, 0.032, 0.064, 0.128, 0.256};
	double cell_side = mp->xC / mp->xD;
	
	
	if((mp != NULL) && (mp->physical_map != NULL) && (cur_pos != NULL))
	{
		sprintf(buffer1, "%.4lf", cell_side);
		for(i = 0; i < n; i++) {
			sprintf(buffer2, "%.4lf", possible_sizes[i]);
			if(strcmp(buffer1, buffer2) == 0) {
				actual_position = i;
				break;
			}
		}
		
		times = (int) pow(2, ((n-1) - actual_position));
		if(cur_pos[0] == 14 && cur_pos[1] == 12){
			times = times;
		}
		if((cur_pos[0] >= (1*times)) && (cur_pos[0] <= (9*times + (times-1))) && 
		   (cur_pos[1] >= (1*times)) && (cur_pos[1] <= (6*times + (times-1)))) {
			*room = 1;
		}
		else if((cur_pos[0] >= (11*times)) && (cur_pos[0] <= (20*times + (times-1))) && 
				(cur_pos[1] >= (1*times)) && (cur_pos[1] <= (5*times + (times-1)))) {
			*room = 2;
		}
		else if((cur_pos[0] >= (22*times)) && (cur_pos[0] <= (28*times + (times-1))) && 
				(cur_pos[1] >= (1*times)) && (cur_pos[1] <= (11*times + (times-1)))) {
			*room = 3;
		}
		else if((cur_pos[0] >= (1*times)) && (cur_pos[0] <= (8*times + (times-1))) && 
				(cur_pos[1] >= (8*times)) && (cur_pos[1] <= (11*times + (times-1)))) {
			*room = 4;
		}
		else if(((cur_pos[0] >= (11*times)) && (cur_pos[0] <= (20*times + (times-1))) && 
				(cur_pos[1] >= (7*times)) && (cur_pos[1] <= (11*times + (times-1)))) ||
				((cur_pos[0] >= (10*times)) && (cur_pos[0] <= (10*times + (times-1))) && 
				(cur_pos[1] >= (8*times)) && (cur_pos[1] <= (11*times + (times-1))))) {
					*room = 5;
				}
		else if((cur_pos[0] >= (1*times)) && (cur_pos[0] <= (13*times + (times-1))) &&
				(cur_pos[1] >= (13*times)) && (cur_pos[1] <= (18*times + (times-1)))) {
			*room = 6;
		}
		else if((cur_pos[0] >= (15*times)) && (cur_pos[0] <= (28*times + (times-1))) &&
				(cur_pos[1] >= (13*times)) && (cur_pos[1] <= (18*times + (times-1)))) {
			*room = 7;
		}
		else {
			*room = 0;
		}
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
void mp_printMaps(Mapping mp, int **blocked_positions)
{
	int i, j, k;
	char vet[50000];
	

	vet[0] = '\0';
	strcat(vet, "\n");
	for(i = 0; i < mp->yD; i++) 
	{
		strcat(vet, "  ");
		for(j = 0; j < mp->xD; j++) 
		{
			if(mp->physical_map[i][j] == 2) {
				strcat(vet, "\e[01;37m# ");
			}
			else {
				if(mp->pheromone_map[0][i][j] < 0.05) {
					strcat(vet, "\e[00;34mX ");
				}
				else if((mp->pheromone_map[0][i][j] >= 0.05) && (mp->pheromone_map[0][i][j] < 0.15)) {
					strcat(vet, "\e[01;34mX ");
				}
				else if((mp->pheromone_map[0][i][j] >= 0.15) && (mp->pheromone_map[0][i][j] < 0.25)) {
					strcat(vet, "\e[01;36mX ");
				}
				else if((mp->pheromone_map[0][i][j] >= 0.25) && (mp->pheromone_map[0][i][j] < 0.4)) {
					strcat(vet, "\e[01;32mX ");
				}
				else if((mp->pheromone_map[0][i][j] >= 0.4) && (mp->pheromone_map[0][i][j] < 0.75)) {
					strcat(vet, "\e[01;33mX ");
				}
				else if((mp->pheromone_map[0][i][j] >= 0.75) && (mp->pheromone_map[0][i][j] < 0.9)) {
					strcat(vet, "\e[01;31mX ");
				}
				else if(mp->pheromone_map[0][i][j] >= 0.9) {
					strcat(vet, "\e[00;31mX ");
				}
			}
		}
		strcat(vet, "  ");
		for(j = 0; j < mp->xD; j++) 
		{
			if(mp->physical_map[i][j] == 2) {
				strcat(vet, "# ");
			}
			else {
				for(k = 0; k < 3; k++) {
					if((i == blocked_positions[1][k]) && (j == blocked_positions[0][k])) {
						strcat(vet, "o ");
						break;
					}
				}
				if(k == 3) {
					strcat(vet, "  ");
				}
			}
		}
		strcat(vet, "\n");
	}
	printf("%s\n", vet);
	usleep(10000);
}

