#pragma once

#include <stdbool.h>

#define B3_JOINT_LDL_MAX_ROWS 1024

typedef struct b3StepContext b3StepContext;

// Solve hard joint equalities as a linear system on each joint-connected
// island: skyline LDL for typical islands, matrix-free PCG when the island is
// large. Contacts stay sequential-impulse.
void b3SolveJoints_Direct( b3StepContext* context, bool useBias, bool resetImpulses );

void b3ProjectJointPositions( b3StepContext* context );
