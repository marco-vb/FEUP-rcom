// Link layer protocol implementation

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

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    C_RCV_SET,
    C_RCV_UA,   
    BCC_OK,
    STOP
} State;


int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void processFrameSET(State *state, char byte) {
    switch (*state) {
        case C_RCV_SET:
            if (byte == (A ^ C_SET)) {
                *state = BCC_OK;
            }
            
            else {
                *state = START;
            }
            break;

        case BCC_OK:
            if (byte == FLAG) {
                // send_ua_frame(fd);   // ?
                *state = STOP;
            }
            else {
                *state = START;
            }
            break;
    }
}

void processFrameUA(State *st, char byte) {
    switch (*st) {
        case START:
            if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == A) {
                *st = A_RCV;
            }
            else if (byte != FLAG) {
                *st = START;
            }
            break;
        case A_RCV:
            if (byte == C_UA) {
                *st = C_RCV_UA;
            }
            else if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            else {
                *st = START;
            }
            break;
        case C_RCV_UA:
            if (byte == (A ^ C_UA)) {
                *st = BCC_OK;
            }
            else {
                *st = START;
            }
            break;
        case BCC_OK:
            if (byte == FLAG) {
                *st = STOP; // received UA frame correctly
            }
            else {
                *st = START;
            }
            break;
    }
}


void processInfoFrame(State *st, char byte) {
    static int receivedSequenceNumber;
    switch (*st) {
        case START:
            if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == A) {
                *st = A_RCV;
            }
            else if (byte != FLAG) {
                *st = START;
            }
            break;
        case A_RCV:
            if (byte == C_I0 || byte == C_I1) {
                *st = C_RCV;
                receivedSequenceNumber = byte == C_I0 ? 0 : 1;
            }
            else if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            else {
                *st = START;
            }
            break;
        case C_RCV:
            if (byte == (A ^ byte)) {
                *st = BCC_OK;
            }
            else if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            else {
                *st = START;
            }
            break;
        case BCC_OK:
            if (byte == FLAG) {
                *st = STOP;
                if (currentSequenceNumber != receivedSequenceNumber) {  // REJ
                    sendControlFrame(fd, receivedSequenceNumber == 0 ? C_REJ0 : C_REJ1, FALSE, NULL);
                }
                else sendControlFrame(fd, receivedSequenceNumber == 0 ? C_RR1 : C_RR0, FALSE, NULL);
            }
            else {
                *st = START;
            }
            break;
        default:
            break;
    }
}

void processFrameInfoResponse(State *st, char byte) {
    static int c;
    switch (*st) {
        case START:
            if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == A) {
                *st = A_RCV;
            }
            else if (byte != FLAG) {
                *st = START;
            }
            break;
        case A_RCV:
            if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                *st = C_RCV;
                c = byte;
            }
            else if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            else {
                *st = START;
            }
            break;
        case C_RCV:
            if (byte == (A ^ c)) {
                *st = BCC_OK;
            }
            else if (byte == FLAG) {
                *st = FLAG_RCV;
            }
            else {
                *st = START;
            }
            break;
        case BCC_OK:
            if (byte == FLAG) {
                *st = STOP;
                if (c == C_RR0 || c == C_REJ0) currentSequenceNumber = 0;
                else currentSequenceNumber = 1; // C_RR1 or C_REJ1
            }
            else {
                *st = START;
            }
            break;
        default:
            break;
    }
}

// stuffed must have size >= 2 * frame_size
int add_byte_stuffing(char *frame, int frame_size, char *stuffed) {
    int stuffed_size = 0;
    stuffed[stuffed_size++] = FLAG;
    for (int i = 1; i < frame_size - 1; i++) {  // avoid flag at beginning and end
        if (frame[i] == FLAG || frame[i] == ESCAPE) {
            stuffed[stuffed_size] = ESCAPE;
            stuffed[stuffed_size + 1] = frame[i] ^ ESCAPE_XOR;
            stuffed_size += 2;
        }
        else {
            stuffed[stuffed_size] = frame[i];
            stuffed_size++;
        }
    }
    stuffed[stuffed_size++] = FLAG;
    return stuffed_size;
}

int writeWithTimeout(int fd, const char *frame, int frame_size, void (*process_frame_rcv)(State *, char)) {
    // char stuffed_frame[2 * frame_size];
    // int size = add_byte_stuffing(frame, frame_size, stuffed_frame);

    int STOP = FALSE;
    State st = START;
    
    alarmCount = 0;
    alarmEnabled = FALSE;
    while (alarmCount < parameters.nRetransmissions && !STOP) {
        if (!alarmEnabled) {
            int bytes = write(fd, frame, frame_size);
            printf("%d bytes written\n", bytes);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }
        char response;
        int bytes = read(fd, &response, 1);
        printf("%d bytes read\n", bytes);

        process_frame_rcv(&st, response);
        if (st == STOP) {
            STOP = TRUE;
            alarm(0);
        }
        printf("%d bytes read\n", bytes);
    }
    return 0;
}


void receiveControlFrame(int fd, char control, void (*process_rcv_frame)(State *, char)) {
    char response;
    
    int STOP = FALSE;
    State st = START;
    while (!STOP) {
        int bytes = read(fd, &response, 1);
        printf("%d bytes read\n", bytes);
        process_rcv_frame(&st, response);

        if (st == STOP) {
            STOP = TRUE;
        }
    }
}

void sendControlFrame(int fd, char control, int needsTimeout, void (*process_rcv_frame)(State *, char)) {
    char buf[5] = {FLAG, A, control, A ^ control, FLAG};
    if (needsTimeout) writeWithTimeout(fd, buf, 5, process_rcv_frame);
    else write(fd, buf, 5);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    parameters = connectionParameters;
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
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

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    printf("New termios structure set\n");

    if (connectionParameters.role == LlTx) {
        sendControlFrame(fd, C_SET, TRUE, processFrameUA);
    }
    else {
        char buf[5];
        // needs a function to read from serial port -> process frame first, then send something (here we send UA)
        // readFromSerialPort(fd, buf, 5);  // TODO: do this with a state machine
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // builds frame with data given by buf, and writes it to the serial port, using STOP & WAIT
    int frame_size = bufSize + 6;
    unsigned char frame[frame_size];

    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = currentSequenceNumber == 0 ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];
    int data_bcc_index = frame_size - 2;
    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        frame[data_bcc_index] ^= buf[i];
    }
    frame[frame_size - 1] = FLAG;

    writeWithTimeout(fd, frame, frame_size, processFrameInfoResponse);    // make sure it gets sent correctly
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // needs a function to read from serial port
    // process frame first, then send something
    // readFromSerialPort(fd, &response, 1);  // TODO: do this with a state machine
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // print statistics?
    if (showStatistics) {
        printf("Suka blyad\n");
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
