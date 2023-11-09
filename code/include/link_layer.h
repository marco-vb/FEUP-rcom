// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <stdint.h>
typedef enum {
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct {
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// SIZE of maximum acceptable payload by the link layer.
// Maximum number of bytes that application layer should send to link layer
// For facilitation purposes, the application layer can send up to 20 more bytes than this value
#define MAX_PAYLOAD_SIZE 1000
#define MAX_DATA_SIZE (MAX_PAYLOAD_SIZE + 20)

// MISC
#define FALSE 0
#define TRUE 1

#define _POSIX_SOURCE 1 // POSIX compliant source


// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char* buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char* packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int showStatistics);

#endif // _LINK_LAYER_H_
