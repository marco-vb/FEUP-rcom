#ifndef __STATE_CONTROL_MACHINE__
#define __STATE_CONTROL_MACHINE__

#include <stdbool.h>
#include <stdint.h>

#define FLAG_BYTE 0x7E
#define A_BYTE 0x03
#define C_SET 0x03
#define C_UA 0x07
#define C_RR 0x05
#define C_REJ 0x01
#define C_DISC 0x0B


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