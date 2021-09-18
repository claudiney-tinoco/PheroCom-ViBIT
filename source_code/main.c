#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <time.h>
#include <unistd.h>
#include <windows.h>
#include "mapping.h"
#include "communication.h"


struct robot {
	int cur_pos[3];
	int goal_pos[3];
	Mapping mp;
	Communication com;
};
typedef struct robot Robot;


int rpt = 100, itr = 10000;
int mpi = 20, mpj = 30;
int *random = NULL, **blocked_positions = NULL;

void rb_init(Robot *rb, int rb_qtd, int dir);
void rb_fsm(Robot *rb, int rb_qtd, Pool pl, int id);
void rb_shuffle(int *array, int size);

void rb_initPool(Robot *rb, Pool pl);
void rb_saveTime(double elapsedtime, int dir);
void rb_initExperiments(int rb_qtd);
void rb_backupExperiments(Robot *rb, int rb_qtd, int dir);
void rb_backupVisitas(int rb_qtd, int dir);


/* Input:
 * Process:
 * Output:
 */
int main()
{
	int i, j, a, dir;
	int rb_qtd = 3;
	Robot *rb;
	Pool pl = NULL;
	
	LARGE_INTEGER frequency;
	LARGE_INTEGER inicio,fim;
	double elapsedtime;
	
	
	for(dir = -1; dir <= 10; dir++)
	{
		rb = NULL; pl = NULL;
		com_poolInit(&pl);
		rb = (Robot *) malloc(rb_qtd * sizeof(Robot));
		random = NULL; blocked_positions = NULL;
		
		for(j = 1; j <= rpt; j++) 
		{
			srand(j);
			rb_init(rb, rb_qtd, dir);
			rb_initPool(rb, pl);
			rb_initExperiments(rb_qtd);
			
			// =================================================================
			QueryPerformanceFrequency(&frequency);
			QueryPerformanceCounter(&inicio);
			
			for(i = 0; i < itr; i++) {
				rb_fsm(rb, rb_qtd, pl, i);
			}
			
			QueryPerformanceCounter(&fim);
			elapsedtime = (fim.QuadPart - inicio.QuadPart) * 1000.0 / frequency.QuadPart;
			printf(" %2d - Time: %.16lf ms\n", j, elapsedtime);
			// =================================================================
			
			if(j != rpt) {
				for(a = 0; a < rb_qtd; a++) {
					mp_close(&rb[a].mp);
					com_close(&rb[a].com);
				}
			}
		}
	}
	return 1;
}


/* Input:
 * Process:
 * Output:
 */
void rb_fsm(Robot *rb, int rb_qtd, Pool pl, int id)
{
	int i, j, aux, aux2;
	
	
	rb_shuffle(random, rb_qtd);
	for(i = 0, aux = random[i]; i < rb_qtd; i++, aux = random[i])
	{		
		com_MEMreceivePheromoneMap(rb[aux].com, rb[aux].cur_pos, pl);
		
		mp_pheromoneDetection(rb[aux].mp, rb[aux].cur_pos, 0);
		mp_cellDecision(rb[aux].mp, rb[aux].cur_pos, rb[aux].goal_pos, blocked_positions, rb_qtd);
		mp_pheromoneDiffusion(rb[aux].mp, rb[aux].cur_pos, 0);
		mp_pheromoneEvaporation(rb[aux].mp);
		
		com_MEMbroadcastPheromoneMap(rb[aux].com, rb[aux].cur_pos, pl);
		
		rb[aux].cur_pos[0] = rb[aux].goal_pos[0];
		rb[aux].cur_pos[1] = rb[aux].goal_pos[1];
		rb[aux].cur_pos[2] = rb[aux].goal_pos[2];
		
		blocked_positions[0][aux] = rb[aux].goal_pos[0];
		blocked_positions[1][aux] = rb[aux].goal_pos[1];
	}
	system("cls");
	mp_printMaps(rb[0].mp, blocked_positions);
	
	for(i = 0; i < 2; i++) {
		for(j = 0; j < rb_qtd; j++) {
			blocked_positions[i][j] = -1;
		}
	}
	return;
}


/* Input:
 * Process:
 * Output:
 */
void rb_init(Robot *rb, int rb_qtd, int dir)
{
	char buffer[30];
	int i;
	int *xD, *yD;
	double **pointer;
	
	
	for(i = 0; i < rb_qtd; i++) 
	{
		rb[i].mp = NULL;
		mp_init(&rb[i].mp);
		mp_loadMaps(rb[i].mp, "input_map.txt");
		
		rb[i].com = NULL;
		com_init(&rb[i].com);

		mp_getDiscreteDimension(rb[i].mp, &xD, &yD);
		mp_getPheromoneMapPointer(rb[i].mp, &pointer, 0);

		sprintf(buffer, "epuck-%d", i);
		com_setCommunicationName(rb[i].com, buffer);
		com_setPheromoneMapDimensions(rb[i].com, xD, yD);
		com_setPheromoneMapPointer(rb[i].com, pointer);
		
		com_setLinkRadius(rb[i].com, dir);
		com_setMenssageRadius(rb[i].com, dir);
		
		buffer[0] = '\0';
		sprintf(buffer, "communication/epuck-%d.txt", i);
		remove(buffer);
	}
	
	rb[0].cur_pos[0] = 2;
	rb[0].cur_pos[1] = 2;
	rb[0].cur_pos[2] = 1;
	
	rb[1].cur_pos[0] = 27;
	rb[1].cur_pos[1] = 2;
	rb[1].cur_pos[2] = 1;
	
	rb[2].cur_pos[0] = 27;
	rb[2].cur_pos[1] = 17;
	rb[2].cur_pos[2] = 1;
	
	mp_setChoiceStrategy(rb[0].mp, 2);
	mp_setChoiceStrategy(rb[1].mp, 2);
	mp_setChoiceStrategy(rb[2].mp, 2);
	
	return;
}


/* Input:
 * Process:
 * Output:
 */
void rb_shuffle(int *array, int size)
{
	int i, j;
	int aux;
	
	
	if(size > 1)
	{
		for (i = 0; i < (size-1); i++) 
		{
			j = i + rand() / (RAND_MAX / (size-i) + 1);
			aux = array[j];
			array[j] = array[i];
			array[i] = aux;
		}
	}
	return;
}


/* Input:
 * Process:
 * Output:
 */
void rb_initPool(Robot *rb, Pool pl)
{
	int *xD, *yD;
	
	
	if((rb != NULL) && (pl != NULL))
	{
		mp_getDiscreteDimension(rb[0].mp, &xD, &yD);
		com_poolClear(pl, *yD);
	}
}


/* Input:
 * Process:
 * Output:
 */
void rb_initExperiments(int rb_qtd)
{
	int i, j;
	
	
	if(random == NULL) {
		random = (int *) malloc(rb_qtd * sizeof(int));
	}
	for(i = 0; i < rb_qtd; i++) {
		random[i] = i;
	}
	rb_shuffle(random, rb_qtd);
	
	if(blocked_positions == NULL) {
		blocked_positions = (int **) malloc(2 * sizeof(int*));
		for(i = 0; i < 2; i++) {
			blocked_positions[i] = (int *) malloc(rb_qtd * sizeof(int));
			for(j = 0; j < rb_qtd; j++) {
				blocked_positions[i][j] = -1;
			}
		}
	}
	
	return;
}

