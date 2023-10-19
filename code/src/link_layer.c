// Link layer protocol implementation

#include "state_control_machine.h"
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

// Sends control frame and waits for response
void sendControlFrame(uint8_t control) {
    uint8_t ans;
    if (control == C_SET) ans = C_UA;

    uint8_t buf [] = { FLAG, A, control, A ^ control, FLAG };
    writeWithTimeout(buf, 5, ans);
}

// Receives control frame with state machine and returns the control byte
uint8_t receiveControlFrame() {
    ControlMachine* cm = control_machine_init();
    uint8_t response;

    while (!control_machine_is_finished(cm)) {
        read(fd, &response, 1);
        control_machine_update(cm, response);
    }

    uint8_t result = cm->c;
    control_machine_destroy(cm);

    return result;
}


// stuffed must have size >= 2 * frame_size
int addByteStuffing(char* frame, int frameSize, char* stuffed) {
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
            ssize_t bytes = write(fd, frame, frameSize);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }

        if (receiveControlFrame() == control) {
            alarm(0);
            break;
        }
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
    }
    else {
        receiveControlFrame(fd, C_SET, processFrameSET);
        sendControlFrame(fd, C_UA);
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const uint8_t* buf, int bufSize) {
    // builds frame with data given by buf, and writes it to the serial port, using STOP & WAIT
    int frameSize = bufSize + 6;
    uint8_t frame[frameSize];
    memset(frame, 0, frameSize);

    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = currentSequenceNumber == 0 ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];
    frame[frameSize - 1] = FLAG;

    int dataBccIndex = frameSize - 2;

    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        frame[dataBccIndex] ^= buf[i];
    }

    writeWithTimeout(fd, frame, frameSize);
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(uint8_t* packet) {
    // needs a function to read from serial port
    // process frame first, then send something
    // readFromSerialPort(fd, &response, 1);  // TODO: do this with a state machine
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    // print statistics?
    if (showStatistics) {
        printf("Suka blyad\n");
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
