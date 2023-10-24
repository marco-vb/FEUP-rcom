#ifndef __DATA_STATE_MACHINE_H__
#define __DATA_STATE_MACHINE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "ll_macros.h"

#define MAX_DATA_SIZE 3000

typedef enum {
    DATA_START,
    DATA_FLAG,
    DATA_A,
    DATA_C,
    DATA_BCC1,
    DATA_DATA,
    DATA_ESCAPE,
    DATA_FAIL,
    DATA_STOP
} DataState;

typedef struct {
    DataState state;
    uint8_t data[MAX_DATA_SIZE];
    uint16_t data_size;
    uint8_t a, c;
} DataMachine;

DataMachine* data_machine_init();
void data_machine_update(DataMachine* machine, uint8_t byte);
bool data_machine_is_finished(DataMachine* machine);
bool data_machine_is_failed(DataMachine* machine);
void data_machine_destroy(DataMachine* machine);

#endif // __DATA_STATE_MACHINE_H__