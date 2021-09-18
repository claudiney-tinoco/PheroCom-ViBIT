#ifndef COMMUNICATION
#define COMMUNICATION


typedef struct communication *Communication;
typedef struct pool *Pool;

int com_init(Communication *com);
int com_close(Communication *com);

int com_setLinkRadius(Communication com, int link_radius);
int com_setMenssageRadius(Communication com, int msg_radius);
int com_setPheromoneMapDimensions(Communication com, int *xD, int *yD);
int com_setPheromoneMapPointer(Communication com, double **pointer);
int com_setCommunicationName(Communication com, char *name);

int com_broadcastPheromoneMap(Communication com, int *current_position);
int com_receivePheromoneMap(Communication com, int *current_position);

int com_MEMbroadcastPheromoneMap(Communication com, int *current_position, Pool pool);
int com_MEMreceivePheromoneMap(Communication com, int *current_position, Pool pool);
int com_poolInit(Pool *pool);
int com_poolClear(Pool pool, int y_size);
int com_poolVerifyCommunication(Communication com, Pool pool);
int com_poolNewCommunication(Communication com, Pool pool);
int com_poolUpdateCommunication(Communication com, Pool pool, int *current_position, int i1, int i2, int j1, int j2);
void com_savePool(Pool pool, int dir);
void com_printPool(Pool pool);

#endif

