#include "state_control_machine.h"

ControlMachine* control_machine_init() {
    ControlMachine* cm = malloc(sizeof(ControlMachine));
    cm->state = START;
    cm->a = 0;
    cm->c = 0;

    return cm;
}

void control_machine_update(ControlMachine* machine, uint8_t byte) {
    switch (machine->state) {
    case START:
        if (byte == FLAG_BYTE) {
            machine->state = FLAG;
        }
        break;
    case FLAG:
        if (byte == A_BYTE) {
            machine->state = A;
            machine->a = byte;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = FLAG;
        }
        break;
    case A:
        if (byte == C_SET || byte == C_UA || byte == C_DISC ||
            byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 ||
            byte == C_REJ1 || byte == C_I0 || byte == C_I1) {
            machine->state = C;
            machine->c = byte;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = FLAG;
        }
        break;
    case C:
        if (byte == (machine->a ^ machine->c)) {
            machine->state = BCC;
        }
        else if (byte == FLAG_BYTE) {
            machine->state = FLAG;
        }
        break;
    case BCC:
        if (byte == FLAG_BYTE) {
            machine->state = STOP;
        }
        break;
    case STOP:
        break;
    default:
        machine->state = START;
    }
}

bool control_machine_is_finished(ControlMachine* machine) {
    return machine->state == STOP;
}

void control_machine_destroy(ControlMachine* machine) {
    free(machine);
}
