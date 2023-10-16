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

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV_SET,
    C_RCV_UA,
    BCC_OK,
    STOP
} State;

void process_set_frame(State *state, char byte) {
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


void process_ua_frame(State *st, char byte) {
    // after receiving C = UA
    switch (*st) {        
        case C_RCV_UA:
            if (byte == (A ^ C_UA)) {
                *st = BCC_OK;
            }
            else {
                *st = START;        // error -> might need to do something else
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



// Here frame does not include the actual flags at the beginning and end of the frame
// stuffed must have size >= 2 * frame_size
int add_byte_stuffing(char *frame, int frame_size, char *stuffed) {
    int stuffed_size = 0;
    for (int i = 0; i < frame_size; i++) {
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
    return stuffed_size;
}

int remove_byte_stuffing(char *frame, int frame_size, char *unstuffed) {
    int unstuffed_size = 0;
    for (int i = 0; i < frame_size; i++) {
        if (frame[i] == ESCAPE) {   // there is always something after an escape (what if there are errors)
            unstuffed[unstuffed_size] = frame[i + 1] ^ ESCAPE_XOR;
            unstuffed_size++;
            i++;
        }
        else {
            unstuffed[unstuffed_size] = frame[i];
            unstuffed_size++;
        }
    }
    return unstuffed_size;
}

void send_control_frame(int fd, char control, int needs_timeout) {
    char buf[5];
    buf[0] = FLAG;
    buf[1] = A;
    buf[2] = control;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    if (needs_timeout) write_with_timeout(fd, buf, 5);
    else write(fd, buf, 5);
}

int write_with_timeout(int fd, const char *frame, int frame_size) {
    char response[5];

    int STOP = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;
    while (alarmCount < parameters.nRetransmissions && !STOP) {
        if (!alarmEnabled) {
            int bytes = write(fd, frame, frame_size);
            printf("%d bytes written\n", bytes);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }
        int bytes = read(fd, response, 5);
        printf("%d bytes read\n", bytes);
        if (TRUE) { // process bytes with a state machine (for now we assume they are correct)
            // set -> ua; i -> rr, disc -> disc

            STOP = TRUE;
            alarm(0);
        }
        printf("%d bytes read\n", bytes);
    }
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
        char buf[5];
        buf[0] = FLAG;
        buf[1] = A;
        buf[2] = C_SET;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;
        write_with_timeout(buf, 5, fd);
    }
    else {
        char buf[5];
        read_from_serial_port(fd, buf, 5);
        // send UA
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
    frame[2] = C_SET;
    frame[3] = frame[1] ^ frame[2];
    int data_bcc_index = frame_size - 2;
    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        frame[data_bcc_index] ^= buf[i];
    }
    frame[frame_size - 1] = FLAG;

    write_with_timeout(fd, frame, frame_size);    // make sure it gets sent correctly

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    int max_frame_size = MAX_PAYLOAD_SIZE + 6;
    char frame[max_frame_size];
    int bytes_read = read(fd, frame, max_frame_size);
    if (bytes_read == -1) {
        perror("read");
        exit(-1);
    }
    printf("%d bytes read\n", bytes_read);
    // process bytes_read with a state machine, then send response accordingly
    // for now, we just send an acknowledgment for the set frame (UA)
    // send_ua_frame(fd);
    send_control_frame(fd, C_UA, FALSE);
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
