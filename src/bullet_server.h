#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>

// Bullet
#include <RobotSimulator/b3RobotSimulatorClientAPI.h>
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Utils/b3Clock.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "bullet_hyumm.h"

// Robot Definition
#include "xddp_packet.h"
#include "PropertyDefinition.h"

int sockfd_nom, sockfd_act;

pthread_t bullet_thread;
pthread_attr_t bullet_attr;
struct sched_param bullet_param;

size_t BUFLEN_BULLET = sizeof(packet::JointState);

struct packet::JointState *bullet_nom = (packet::JointState *)malloc(BUFLEN_BULLET);
struct packet::JointState *bullet_act = (packet::JointState *)malloc(BUFLEN_BULLET);

b3RobotSimulatorClientAPI* b3sim;
Bullet_Hyumm* b3robot_nom;
Bullet_Hyumm* b3robot_act;

const int CONTROL_RATE = 100;
b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);