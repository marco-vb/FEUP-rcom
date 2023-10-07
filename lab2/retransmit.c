#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#define FALSE 0
#define TRUE 1

int alarmEnabled = FALSE;
int alarmCount = 0;

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5

volatile int STOP = FALSE;

int fd;
unsigned char buf[BUF_SIZE] = { 0 };

// Alarm function handler
void alarmHandler(int signal) {
    alarmEnabled = FALSE;

    int bytes = write(fd, buf, BUF_SIZE);
    printf("Repeating: %d bytes written\n", bytes);

    // Wait until all bytes have been written to the serial port
    sleep(1);

    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int main(int argc, char* argv []) {
    // Program usage: Uses either COM1 or COM2
    const char* serialPortName = argv[1];
    int wait_time = atoi(argv[2]);
    int repeat = atoi(argv[3]);

    if (argc < 4) {
        printf("Incorrect program usage\n"
            "Usage: %s <SerialPort> <WaitTime> <Repeat>\n"
            "Example: %s /dev/ttyS1 1 3\n",
            argv[0],
            argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    buf[0] = 0x7E;  // flag
    buf[1] = 0x03;  // addr
    buf[2] = 0x03;  // control
    buf[3] = buf[1] ^ buf[2];   // BCC
    buf[4] = 0x7E;

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    //buf[5] = '\n';

    // Set alarm function handler
    (void) signal(SIGALRM, alarmHandler);

    int bytes = write(fd, buf, BUF_SIZE);
    printf("First: %d bytes written\n", bytes);

    sleep(1); // wait for bytes to be sent

    while (alarmCount < repeat) {
        if (!alarmEnabled) {
            alarm(wait_time); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }
    }

    printf("Sent message a total of %d times.\n", repeat + 1);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}

