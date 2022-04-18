
#ifndef TURN_H
#define TURN_H

#include "mazeblaze.h"
#include "line_following.h"
#include "tuning_http_server.h"
#include "node_detection.h"

#define RIGHT 1
#define LEFT 2
#define UTURN 3
#define STRAIGHT 4
#define LEFT_ 5
#define END 6
#define KUCH_NAI 7


extern TaskHandle_t Maze_explore;
extern bool complete;
void turn(void *arg);
void stop();
void take_turn(int Turn);

#endif
