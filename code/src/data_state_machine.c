#include "data_state_machine.h"


DataMachine* data_machine_init() {
    DataMachine* dm = malloc(sizeof(DataMachine));
    dm->state = DATA_START;
    dm->a = dm->c = dm->data_size = 0;
    memset(dm->data, 0, MAX_DATA_SIZE);
    return dm;
}

void data_machine_update(DataMachine* machine, uint8_t byte) {
    switch (machine->state) {
    case DATA_START:
        if (byte == FLAG_BYTE) {
            machine->state = DATA_FLAG;
        }
        break;
    case DATA_FLAG:
        if (byte == A_BYTE) {
            machine->state = DATA_A;
            machine->a = byte;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = DATA_FLAG;
        }
        else {
            machine->state = DATA_START;
        }
        break;
    case DATA_A:
        if (byte == C_I0 || byte == C_I1) {
            machine->state = DATA_C;
            machine->c = byte;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = DATA_FLAG;
        }
        else {
            machine->state = DATA_START;
        }
        break;
    case DATA_C:
        if (byte == (machine->a ^ machine->c)) {
            machine->state = DATA_BCC1;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = DATA_FLAG;
        }
        else {
            machine->state = DATA_START;
        }
        break;
    case DATA_BCC1:
        if (byte == FLAG_BYTE) {    // no data in the frame
            machine->state = DATA_FAIL;
        }
        else if (byte == ESCAPE_BYTE) {
            machine->state = DATA_ESCAPE;
        }
        else {
            machine->state = DATA_DATA;
            machine->data[machine->data_size++] = byte;
        }
        break;
    case DATA_DATA:
        if (byte == FLAG_BYTE) {
            uint8_t xor = 0;

            for (int i = 0; i < machine->data_size - 1; i++) {
                xor ^= machine->data[i];
            }

            if (machine->data[machine->data_size - 1] == xor) {
                machine->state = DATA_STOP;
                machine->data_size--;
            }
            else {
                machine->state = DATA_FAIL;
            }
        }
        else if (byte == ESCAPE_BYTE) {
            machine->state = DATA_ESCAPE;
        }
        else {
            machine->data[machine->data_size++] = byte;
        }
        break;
    case DATA_ESCAPE:
        if (byte == ESCAPED_FLAG_BYTE) {
            machine->data[machine->data_size++] = FLAG_BYTE;
            machine->state = DATA_DATA;
        }
        else if (byte == ESCAPED_ESCAPE_BYTE) {
            machine->data[machine->data_size++] = ESCAPE_BYTE;
            machine->state = DATA_DATA;
        }
        else {
            machine->state = DATA_FAIL;
        }
        break;
    case DATA_STOP:
        break;
    case DATA_FAIL:
        break;
    default:
        break;
    }
}

bool data_machine_is_finished(DataMachine* machine) {
    return machine->state == DATA_STOP || machine->state == DATA_FAIL;
}

bool data_machine_is_failed(DataMachine* machine) {
    return machine->state == DATA_FAIL;
}

void data_machine_destroy(DataMachine* machine) {
    free(machine);
}
