#ifndef __STATE_CONTROL_MACHINE__
#define __STATE_CONTROL_MACHINE__

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ll_macros.h"

typedef enum {
    START,
    FLAG,
    A,
    C,
    BCC,
    STOP
} ControlState;

typedef struct {
    ControlState state;
    uint8_t a;
    uint8_t c;
} ControlMachine;

ControlMachine* control_machine_init();
void control_machine_update(ControlMachine* machine, uint8_t byte);
bool control_machine_is_finished(ControlMachine* machine);
void control_machine_destroy(ControlMachine* machine);

#endif // __STATE_CONTROL_MACHINE__