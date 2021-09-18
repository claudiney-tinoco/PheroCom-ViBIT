/* 
 * 05/03/2018
 * Claudiney Ramos Tinoco
 */

#include "communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dirent.h>


struct link {
	char *name;
	int check;
	unsigned long int hush;
	unsigned long int time_stamp;
	struct link *prox;
};

struct communication {
	int *xD, *yD;
	double **pheromone_map;
	
	int link_radius;
	int msg_radius;
	struct link *link_list;
};


struct message {
	char *name;
	int current_position[2];
	unsigned long int time_stamp;
	int i_start, i_end, j_start, j_end;
	double **map;
	struct message *prox;
};

struct pool {
	unsigned long int bc_times;
	unsigned long int bc_data;
	unsigned long int rc_times;
	unsigned long int rc_data;
	int size;
	struct message *msgs;
};


int com_inLinkRange(Communication com, int *a_center, int *b_center);
int com_verifyNewLink(Communication com, char *name, unsigned long int time_stamp);
int com_computeLinkHush(Communication com);
int com_clearLinkList(Communication com);


/* Input:
 * Process:
 * Output:
 */
int com_init(Communication *com)
{
	if((*com == NULL) && ((*com = (struct communication *) malloc(sizeof(struct communication))) != NULL))
	{
		(*com)->xD = NULL;
		(*com)->yD = NULL;
		(*com)->pheromone_map = NULL;
		
		(*com)->link_radius = 6; //Value equal to -1 represents a infinite link range
		(*com)->msg_radius = 6; //Value equal to -1 represents a whole map message

		if(((*com)->link_list = (struct link *) malloc(sizeof(struct link))) != NULL) {
			(*com)->link_list->name = NULL;
			(*com)->link_list->check = 0;
			(*com)->link_list->hush = 0;
			(*com)->link_list->time_stamp = 0;
			(*com)->link_list->prox = NULL;
			return 1;
		}
	}
	com_close(com);
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_close(Communication *com)
{
	struct link *aux1, *aux2;
	
	
	if(com != NULL)
	{
		for(aux1 = (*com)->link_list; aux1 != NULL; aux1 = aux2) {
			aux2 = aux1->prox;
			free(aux1->name);
			free(aux1);
		}
		free(*com);
		*com = NULL;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_setLinkRadius(Communication com, int link_radius)
{
	if((com != NULL) && (link_radius >= -1))
	{
		com->link_radius = link_radius;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_setMenssageRadius(Communication com, int msg_radius)
{
	if((com != NULL) && (msg_radius >= -1))
	{
		com->msg_radius = msg_radius;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_setPheromoneMapDimensions(Communication com, int *xD, int *yD)
{
	if((com != NULL) && (*xD > 0) && (*yD > 0))
	{
		com->xD = xD;
		com->yD = yD;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_setPheromoneMapPointer(Communication com, double **pointer)
{
	if((com != NULL) && (pointer != NULL))
	{
		com->pheromone_map = pointer;
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_setCommunicationName(Communication com, char *name)
{
	struct link *aux;
	
	
	if((com != NULL) && (name != NULL))
	{
		aux = com->link_list;
		
		if(aux->name != NULL) {
			free(aux->name);
		}
		aux->name = (char *) malloc((strlen(name) + 6) * sizeof(char));
		sprintf(aux->name, "%s.txt", name);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_inLinkRange(Communication com, int *a_center, int *b_center)
{
	int temp1, temp2;
	
	
	if(com->link_radius == -1) {
		return 1;
	}
	if((a_center[0] < 0) || (a_center[1] < 0) || (b_center[0] < 0) || (b_center[1] < 0)) {
		return 0;
	}
	
	temp1 = (int) fabs(a_center[0] - b_center[0]);
	temp2 = (int) fabs(a_center[1] - b_center[1]);
	return ((temp1 <= com->link_radius) && (temp2 <= com->link_radius)) ? 1 : 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_verifyNewLink(Communication com, char *name, unsigned long int time_stamp)
{
	struct link *aux;
	
	
	if((com != NULL) && (strcmp(com->link_list->name, name) != 0))
	{
		for(aux = com->link_list->prox; aux != NULL; aux = aux->prox) 
		{
			if(strcmp(aux->name, name) == 0) 
			{
				if(time_stamp > aux->time_stamp) {
					aux->check = 1;
					aux->time_stamp = time_stamp;
					return 1;
				}
				return 0;
			}
		}
		
		if(((aux = (struct link *) malloc(sizeof(struct link))) != NULL) &&
		   ((aux->name = (char *) malloc((strlen(name) + 2) * sizeof(char))) != NULL))
		{
			strcpy(aux->name, name);
			aux->check = 1;
			aux->hush = 0;
			aux->time_stamp = time_stamp;
			
			aux->prox = com->link_list->prox;
			com->link_list->prox = aux;
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_computeLinkHush(Communication com)
{
	struct link *aux;
	
	
	if(com != NULL)
	{
		for(aux = com->link_list->prox; aux != NULL; aux = aux->prox) 
		{
			if(aux->check == 1) {
				aux->check = 0;
				aux->hush = 0;
			}
			else {
				aux->hush = ((aux->hush + 1) < 1000000) ? (aux->hush + 1) : aux->hush;
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
int com_clearLinkList(Communication com)
{
	struct link *aux1, *aux2;
	
	
	if(com != NULL)
	{
		for(aux1 = com->link_list->prox; aux1 != NULL; aux1 = aux2) {
			aux2 = aux1->prox;
			free(aux1->name);
			free(aux1);
		}
		com->link_list->prox = NULL;
		return 1;
	}
	return 0;
}


// ######################################################################################
/* Input:
 * Process:
 * Output:
 */
int com_MEMbroadcastPheromoneMap(Communication com, int *current_position, Pool pool)
{
	int i_start, i_end, j_start, j_end;
	
	
	if((com != NULL) && (com->pheromone_map != NULL) && (current_position != NULL) && (pool != NULL)) 
	{
		i_start = 0; i_end = (*(com->yD) - 1); j_start = 0; j_end = (*(com->xD) - 1);
		
		if(com->msg_radius > -1)
		{
			if((current_position[1] - com->msg_radius) > 0) {
				i_start = current_position[1] - com->msg_radius;
			}
			if((current_position[1] + com->msg_radius) < (*(com->yD) - 1)) {
				i_end = current_position[1] + com->msg_radius;
			}
			if((current_position[0] - com->msg_radius) > 0) {
				j_start = current_position[0] - com->msg_radius;
			}
			if((current_position[0] + com->msg_radius) < (*(com->xD) - 1)) {
				j_end = current_position[0] + com->msg_radius;
			}
		}
		
		if(!com_poolVerifyCommunication(com, pool)) 
		{
			com_poolNewCommunication(com, pool);
		}
		com_poolUpdateCommunication(com, pool, current_position, i_start, i_end, j_start, j_end);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_MEMreceivePheromoneMap(Communication com, int *current_position, Pool pool)
{
	int peer_position[2];
	int i, j, i_start, i_end, j_start, j_end;
	long unsigned int aux_ts;
	struct message *aux;
	
	
	if((com != NULL) && (com->pheromone_map != NULL) && (pool != NULL))  
	{
		for(aux = pool->msgs; aux != NULL; aux = aux->prox) 
		{
			i_start = aux->i_start;
			i_end = aux->i_end;
			j_start = aux->j_start;
			j_end = aux->j_end;
			
			peer_position[0] = aux->current_position[0];
			peer_position[1] = aux->current_position[1];
			
			aux_ts = aux->time_stamp;
			
			if(com_inLinkRange(com, current_position, peer_position) && 
			   com_verifyNewLink(com, aux->name, aux_ts) &&
			   (com->link_radius != 0))
			{
				pool->rc_times++;
				for(i = i_start; i <= i_end; i++) {
					for(j = j_start; j <= j_end; j++) 
					{
						pool->rc_data++;
						com->pheromone_map[i][j] = fmax(com->pheromone_map[i][j], aux->map[i][j]);
					}
				}
			}
		}
		com_computeLinkHush(com);
		return 1;
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_poolInit(Pool *pool)
{
	if(((*pool) == NULL) && (((*pool) = (struct pool *) malloc(sizeof(struct pool))) != NULL)) {
		(*pool)->bc_times = 0;
		(*pool)->bc_data = 0;
		(*pool)->rc_times = 0;
		(*pool)->rc_data = 0;
		(*pool)->msgs = NULL;
		return 1;
	}
	else {
		return 0;
	}
}


/* Input:
 * Process:
 * Output:
 */
int com_poolClear(Pool pool, int y_size)
{
	int i;
	struct message *aux1, *aux2;
	
	
	if(pool != NULL)
	{
		for(aux1 = pool->msgs; aux1 != NULL; aux1 = aux2) 
		{
			free(aux1->name);
			for(i = 0; i < 20; i++) {
				free(aux1->map[i]);
			}
			aux2 = aux1->prox;
			free(aux1->map);
			free(aux1);
		}
		pool->size = 0;
		pool->bc_times = 0;
		pool->bc_data = 0;
		pool->rc_times = 0;
		pool->rc_data = 0;
		pool->msgs = NULL;
		return 1;
	}	
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_poolVerifyCommunication(Communication com, Pool pool)
{
	struct message *aux;
	
	
	for(aux = pool->msgs; aux != NULL; aux = aux->prox) 
	{
		if(strcmp(com->link_list->name, aux->name) == 0) {
			return 1;
		}
	}
	return 0;
}


/* Input:
 * Process:
 * Output:
 */
int com_poolNewCommunication(Communication com, Pool pool)
{
	int i;
	struct message *node, *aux;
	
	
	if(((node = (struct message *) malloc(sizeof(struct message))) == NULL) ||
	   ((node->name = (char *) malloc((strlen(com->link_list->name) + 2) * sizeof(char))) == NULL)) {
		return 0;
	}
	
	node->map = (double **) malloc(*(com->yD) * sizeof(double *));
	for(i = 0; i < *(com->yD); i++) {
		node->map[i] = (double *) malloc(*(com->xD) * sizeof(double));
	}
	
	strcpy(node->name, com->link_list->name); 
	node->time_stamp = 0;
	node->prox = NULL;
	
	if(pool->msgs == NULL) {
		pool->msgs = node;
	}
	else {
		for(aux = pool->msgs; aux->prox != NULL; aux = aux->prox);
		aux->prox = node;
	}
	return 1;
}


/* Input:
 * Process:
 * Output:
 */
int com_poolUpdateCommunication(Communication com, Pool pool, int *current_position, int i1, int i2, int j1, int j2)
{
	int i, j;
	struct message *aux;
	
	
	for(aux = pool->msgs; aux != NULL; aux = aux->prox) 
	{
		if(strcmp(com->link_list->name, aux->name) == 0) 
		{
			aux->current_position[0] = current_position[0];
			aux->current_position[1] = current_position[1];
			
			aux->i_start = i1;
			aux->i_end = i2;
			aux->j_start = j1;
			aux->j_end = j2;
			
			pool->bc_times++;
			for(i = i1; i <= i2; i++) {
				for(j = j1; j <= j2; j++) {
					pool->bc_data++;
					aux->map[i][j] = com->pheromone_map[i][j];
				}
			}
			aux->time_stamp++;
			return 1;
		}
	}
	return 0;
}

