#ifndef __ACTION_H__
#define __ACTION_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

typedef struct {
    uint8_t control;
    void (*actionFunction)();
} ControlAction;


typedef struct {
    ControlAction **actions;
    int size;
} Actions;

ControlAction *createAction(uint8_t control, void (*actionFunction)());
Actions *createActions(int numArgs, ...);
/**
 * Performs an action.
 * Returns the index of the action performed, or -1 if not performed.
 * The index corresponds to the order given in createActions.
 */
int performAction(Actions *actions, uint8_t control);
void destroyActions(Actions *actions);

#endif // __ACTION_H__
