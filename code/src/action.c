#include "action.h"
#include <stdio.h>

ControlAction *createAction(uint8_t control, void (*actionFunction)()) {
    ControlAction *action = (ControlAction*)malloc(sizeof(ControlAction));
    action->control = control;
    action->actionFunction = actionFunction;
    return action;
}
void destroyAction(ControlAction *action) {
    free(action);
}

Actions *createActions(int numArgs, ...) {
    Actions *actions = (Actions*)malloc(sizeof(Actions));
    actions->size = numArgs;

    if (numArgs > 0) {
        actions->actions = (ControlAction**)malloc(numArgs * sizeof(ControlAction*));

        va_list args;
        va_start(args, numArgs);

        for (int i = 0; i < numArgs; i++) {
            actions->actions[i] = va_arg(args, ControlAction*);
        }

        va_end(args);
    } else {
        actions->actions = NULL;
    }

    return actions;
}

int performAction(Actions *actions, uint8_t control) {
    for (int i = 0; i < actions->size; i++) {
        if (actions->actions[i]->control == control) {
            if (actions->actions[i]->actionFunction) actions->actions[i]->actionFunction();
            return i;
        }
    }
    return -1;
}

void destroyActions(Actions *actions) {
    for (int i = 0; i < actions->size; i++) {
        destroyAction(actions->actions[i]);
    }
    free(actions->actions);
    free(actions);
}
