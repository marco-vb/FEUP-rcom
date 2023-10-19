// Link layer protocol implementation

#include "state_control_machine.h"
#include "data_state_machine.h"
#include "link_layer.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

int fd;
struct termios oldtio;
struct termios newtio;

LinkLayer parameters;

// current sequence number for I frames (0 or 1)
int currentSequenceNumber = 0;  // it's the next number that the writer will send

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

int addByteStuffing(uint8_t* frame, int frameSize, uint8_t* stuffed);
int writeWithTimeout(const uint8_t* frame, int frameSize, uint8_t control);

// Sends control frame and waits for response
void sendControlFrame(uint8_t control) {
    uint8_t ans;
    if (control == C_SET) ans = C_UA;
    if (control == C_DISC) ans = C_DISC;

    uint8_t buf [] = { FLAG, A, control, A ^ control, FLAG };
    writeWithTimeout(buf, 5, ans);
}

// Sends response frame without timeout
void sendResponseFrame(uint8_t control) {
    uint8_t buf [] = { FLAG, A, control, A ^ control, FLAG };
    write(fd, buf, 5);
}

// Receives control frame with state machine and returns the control byte
uint8_t receiveControlFrame() {
    ControlMachine* cm = control_machine_init();
    uint8_t response;

    while (!control_machine_is_finished(cm)) {
        read(fd, &response, 1);
        control_machine_update(cm, response);
    }

    // if (tries == 7) {
    //     printf("Failed to receive control frame\n");
    //     control_machine_destroy(cm);
    //     return 0;
    // }

    uint8_t result = cm->c;
    control_machine_destroy(cm);

    return result;
}

uint8_t* buildInformationFrame(const uint8_t* buf, int bufSize, int* newFrameSize) {
    int frameSize = bufSize + 6;
    uint8_t frame[frameSize];
    memset(frame, 0, frameSize);

    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = currentSequenceNumber ? C_I1 : C_I0;
    frame[3] = frame[1] ^ frame[2];
    frame[frameSize - 1] = FLAG;

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
    stuffed[stuffedSize++] = FLAG;
    for (int i = 1; i < frameSize - 1; i++) {  // avoid flag at beginning and end
        if (frame[i] == FLAG || frame[i] == ESCAPE) {
            stuffed[stuffedSize] = ESCAPE;
            stuffed[stuffedSize + 1] = frame[i] ^ ESCAPE_XOR;
            stuffedSize += 2;
        }
        else {
            stuffed[stuffedSize] = frame[i];
            stuffedSize++;
        }
    }
    stuffed[stuffedSize++] = FLAG;
    return stuffedSize;
}

int writeWithTimeout(const uint8_t* frame, int frameSize, uint8_t control) {
    alarmCount = 0;
    alarmEnabled = FALSE;

    while (alarmCount < parameters.nRetransmissions) {
        if (!alarmEnabled) {
            write(fd, frame, frameSize);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }

        if (receiveControlFrame() == control) {
            alarm(0);
            break;
        }
    }

    if (alarmCount == parameters.nRetransmissions) {
        printf("Failed to send frame\n");
        return -1;
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
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    printf("New termios structure set\n");

    if (connectionParameters.role == LlTx) {
        sendControlFrame(C_SET);
        printf("Sent SET and received UA\n");
    }
    else {
        while (receiveControlFrame() != C_SET) printf("Received something that is not SET\n");
        printf("Received SET\n");
        sendResponseFrame(C_UA);
        printf("Sent UA\n");
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const uint8_t* buf, int bufSize) {

    int newFrameSize;
    uint8_t* frame = buildInformationFrame(buf, bufSize, &newFrameSize);
    writeWithTimeout(frame, newFrameSize, currentSequenceNumber ? C_RR1 : C_RR0);

    printf("Writing %d bytes\n", newFrameSize);
    currentSequenceNumber ^= 1;
    free(frame);

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(uint8_t* packet) {
    DataMachine* dm = data_machine_init();
    uint8_t response;

    while (!data_machine_is_finished(dm)) {
        read(fd, &response, 1);
        data_machine_update(dm, response);
    }

    if (data_machine_is_failed(dm)) {
        sendResponseFrame(dm->c == C_I0 ? C_REJ0 : C_REJ1);
        printf("Something failed, sending REJ\n");
        data_machine_destroy(dm);
        return -1;
    }


    if (dm->c == C_I0) {
        sendResponseFrame(C_RR0);
    }
    else if (dm->c == C_I1) {
        sendResponseFrame(C_RR1);
    }

    printf("Read %d bytes\n", dm->data_size);
    printf("Last byte: %x\n", dm->data[dm->data_size - 1]);

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
    if (showStatistics) {
        printf("No statistics\n");
    }

    if (parameters.role == LlTx) {
        sendControlFrame(C_DISC);
        sendResponseFrame(C_UA);
    }
    else {
        while (receiveControlFrame() != C_DISC) printf("Received something that is not DISC\n");
        sendResponseFrame(C_DISC);
        while (receiveControlFrame() != C_UA) printf("Received something that is not UA\n");
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
