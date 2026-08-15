#pragma once

#include <stdbool.h>

typedef struct b3StepContext b3StepContext;

// Solve hard joint equalities as a linear system on each joint-connected
// island: dense LDL for typical islands, matrix-free PCG when the island is
// large. Contacts stay sequential-impulse.
void b3SolveJoints_Direct( b3StepContext* context, bool useBias );
