// Link layer protocol implementation
#include "ll_macros.h"
#include "data_state_machine.h"
#include "link_layer.h"
#include "state_control_machine.h"
#include "action.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>


int fd;
struct termios oldtio;
struct termios newtio;

LinkLayer parameters;

// current sequence number for I frames (0 or 1)
int currentSequenceNumber = 0;  // it's the next number that the writer will send

int alarmEnabled = FALSE;
int alarmCount = 0;

int totalFramesInfo = 0;
int totalFramesControl = 0;
int totalAlarmTimeouts = 0;
int totalREJ = 0;
time_t currTime;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    totalAlarmTimeouts++;
    printf("Alarm #%d\n", alarmCount);
}

int addByteStuffing(uint8_t* frame, int frameSize, uint8_t* stuffed);
int writeWithTimeout(const uint8_t* frame, int frameSize, Actions *actions);

// actions
void toggleSequenceNumber();
void stopAlarm();
void resetAlarm();


// Sends control frame and waits for response
void sendControlFrame(uint8_t control) {
    uint8_t ans;
    if (control == C_SET || (control == C_DISC && parameters.role == LlRx)) ans = C_UA;
    if (control == C_DISC && parameters.role == LlTx) ans = C_DISC;

    uint8_t a = parameters.role == LlTx ? A_BYTE : A_BYTE_OTHER;
    uint8_t buf [] = { FLAG_BYTE, a, control, a ^ control, FLAG_BYTE };
    
    Actions *actions = createActions(1, createAction(ans, stopAlarm));
    if (writeWithTimeout(buf, 5, actions) == -1) {
        destroyActions(actions);
        exit(-1);
    }
    totalFramesControl++;
    destroyActions(actions);
}

// Sends response frame without timeout
void sendResponseFrame(uint8_t control) {
    uint8_t buf [] = { FLAG_BYTE, A_BYTE, control, A_BYTE ^ control, FLAG_BYTE };
    write(fd, buf, 5);
}

// Receives control frame with state machine and returns the control byte
uint8_t receiveControlFrame() {
    ControlMachine* cm = control_machine_init();
    uint8_t response;

    while (!control_machine_is_finished(cm)) {
        if (read(fd, &response, 1) > 0) {
            // printf("Received byte %02x\n", response);
            control_machine_update(cm, response);
        }
        if (parameters.role == LlTx && !alarmEnabled) {
            // write with timeout alarm received -> stop reading and try to write again
            control_machine_destroy(cm);
            printf("Timed out\n");
            return 0xFF;
        }
    }
    uint8_t result = cm->c;
    control_machine_destroy(cm);
    printf("Received control frame %02x\n", result);
    return result;
}

uint8_t* buildInformationFrame(const uint8_t* buf, int bufSize, int* newFrameSize) {
    int frameSize = bufSize + 6;
    uint8_t frame[frameSize];
    memset(frame, 0, frameSize);

    frame[0] = FLAG_BYTE;
    frame[1] = A_BYTE;
    frame[2] = currentSequenceNumber ? C_I1 : C_I0;
    frame[3] = frame[1] ^ frame[2];
    frame[frameSize - 1] = FLAG_BYTE;

    int dataBccIndex = frameSize - 2;

    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        frame[dataBccIndex] ^= buf[i];
    }

    uint8_t* stuffed = (uint8_t*) malloc(frameSize * 2);
    memset(stuffed, 0, frameSize * 2);
    *newFrameSize = addByteStuffing(frame, frameSize, stuffed);

    return stuffed;
}

// Stuffs the information frame and returns the stuffed array size
int addByteStuffing(uint8_t* frame, int frameSize, uint8_t* stuffed) {
    int stuffedSize = 0;
    stuffed[stuffedSize++] = FLAG_BYTE;
    for (int i = 1; i < frameSize - 1; i++) {  // avoid flag at beginning and end
        if (frame[i] == FLAG_BYTE || frame[i] == ESCAPE_BYTE) {
            stuffed[stuffedSize++] = ESCAPE_BYTE;
            stuffed[stuffedSize++] = frame[i] ^ ESCAPE_XOR;
        }
        else {
            stuffed[stuffedSize++] = frame[i];
        }
    }
    stuffed[stuffedSize++] = FLAG_BYTE;
    return stuffedSize;
}

int stopTimeout = FALSE;

void toggleSequenceNumber() {
    currentSequenceNumber ^= 1;
    stopAlarm();
}
void resetAlarm() {
    printf("Received REJ, reset alarm\n");
    totalREJ++;
    alarmCount = 0;
    alarmEnabled = FALSE;
    alarm(parameters.timeout);
}
void stopAlarm() {
    printf("Stopped alarm\n");
    alarm(0);
    stopTimeout = TRUE;
}

int writeWithTimeout(const uint8_t* frame, int frameSize, Actions *actions) {
    alarmCount = 0;
    alarmEnabled = FALSE;
    stopTimeout = FALSE;
    while (alarmCount < parameters.nRetransmissions && !stopTimeout) {
        if (!alarmEnabled) {
            write(fd, frame, frameSize);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }
        performAction(actions, receiveControlFrame());
    }

    if (alarmCount == parameters.nRetransmissions) {
        printf("Failed to send frame\n");
        exit(-1);
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    parameters = connectionParameters;
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = connectionParameters.role == LlTx ? 10 : 0;
    newtio.c_cc[VMIN] = connectionParameters.role == LlTx ? 0 : 1;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    printf("New termios structure set\n");
    (void)signal(SIGALRM, alarmHandler);

    if (connectionParameters.role == LlTx) {
        
        sendControlFrame(C_SET);
        printf("Sent SET and received UA\n");
    }
    else {
        while (receiveControlFrame() != C_SET) printf("Received something that is not SET\n");
        sendResponseFrame(C_UA);
        totalFramesControl++;
    }
    currTime = time(NULL);
    return 1;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const uint8_t* buf, int bufSize) {

    int newFrameSize;
    uint8_t* frame = buildInformationFrame(buf, bufSize, &newFrameSize);
    totalFramesInfo++;
    Actions *actions = createActions(2,
                createAction(currentSequenceNumber ? C_RR0 : C_RR1, toggleSequenceNumber),
                createAction(currentSequenceNumber ? C_REJ1 : C_REJ0, resetAlarm));
    if (writeWithTimeout(frame, newFrameSize, actions) == -1) {
        free(frame);
        destroyActions(actions);
        printf("Failed to send frame with timeout\n");
        exit(-1);
    }
    destroyActions(actions);
    free(frame);
    return bufSize;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(uint8_t* packet) {
    DataMachine* dm = data_machine_init();
    uint8_t response;

    while (!data_machine_is_finished(dm)) {
        if (read(fd, &response, 1) > 0) {
            // printf("Received byte %2x\n", response);
            data_machine_update(dm, response);
        }
    }

    if (data_machine_is_failed(dm)) {
        if (dm->c == C_I0 && currentSequenceNumber == 0) {
            printf("Something failed, sending REJ\n");
            sendResponseFrame(C_REJ0);
            totalREJ++;
        }
        else if (dm->c == C_I1 && currentSequenceNumber == 1) {
            printf("Something failed, sending REJ\n");
            sendResponseFrame(C_REJ1);
            totalREJ++;
        }
        else {
            // error in I frame, but we have already received it before (RR got lost)
            sendResponseFrame(dm->c == C_I0 ? C_RR1 : C_RR0);
        }        

        data_machine_destroy(dm);
        return -1;
    }

    int correctSequenceNumber = 0;
    if (dm->c == C_I0) {
        sendResponseFrame(C_RR1);
        correctSequenceNumber = currentSequenceNumber == 0;
    }
    else if (dm->c == C_I1) {
        sendResponseFrame(C_RR0);
        correctSequenceNumber = currentSequenceNumber == 1;
        totalFramesInfo++;
    }
    if (!correctSequenceNumber) {
        data_machine_destroy(dm);
        return -1;
    }

    memcpy(packet, dm->data, dm->data_size);
    int result = dm->data_size;
    data_machine_destroy(dm);

    currentSequenceNumber ^= 1;

    return result;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    if (parameters.role == LlTx) {
        sendControlFrame(C_DISC);
        sendResponseFrame(C_UA);
    }
    else {
        while (receiveControlFrame() != C_DISC) printf("Received something that is not DISC\n");
        sendControlFrame(C_DISC);
        totalFramesControl++;
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    if (showStatistics) {
        if (parameters.role == LlTx) {
            printf("Total frames sent: %d\n", totalFramesInfo + totalFramesControl);
            printf("Total frames sent (info): %d\n", totalFramesInfo);
            printf("Total frames sent (control): %d\n", totalFramesControl);
            printf("Total alarm timeouts: %d\n", totalAlarmTimeouts);
            printf("Total REJ received: %d\n", totalREJ);
            printf("Total time spent on information frames: %ld seconds\n", time(NULL) - currTime);
        }
        else {
            printf("Total frames received: %d\n", totalFramesInfo + totalFramesControl);
            printf("Total frames received (info): %d\n", totalFramesInfo);
            printf("Total frames received (control): %d\n", totalFramesControl);
            printf("Total REJ sent: %d\n", totalREJ);
            printf("Total time spent on information frames: %ld seconds\n", time(NULL) - currTime);
            
            printf("Total information frames received with errors: %d vs without errors: %d\n", totalREJ, totalFramesInfo - totalREJ);
        }
    }
    return 1;
}
