// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

void sendFile(const char *filename)
{
    int fd = open(filename, O_RDONLY);

    if (fd == -1)
    {
        printf("Error opening file\n");
        exit(-1);
    }

    unsigned char buf[MAX_PAYLOAD_SIZE];
    int bytesRead = 0;
    int packetNumber = 1;

    while ((bytesRead = read(fd, buf, MAX_PAYLOAD_SIZE)) > 0)
    {
        // build packet before writing it to serial port [TODO]
        llwrite(buf, bytesRead);
        packetNumber++;
    }

    close(fd);
}

void receiveFile(const char *filename)
{
    int fd = open(filename, O_WRONLY);

    if (fd == -1)
    {
        printf("Error opening file\n");
        exit(-1);
    }

    unsigned char buf[MAX_PAYLOAD_SIZE];
    int bytesRead = 0;
    int packetNumber = 1;

    while ((bytesRead = llread(buf)) > 0)
    {
        // process frame before writing to file
        write(fd, buf, bytesRead);
        packetNumber++;
    }

    close(fd);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole llRole = strcmp(role, "tx") == 0 ? LlTx : LlRx;

    LinkLayer link_layer;
    link_layer.baudRate = baudRate;
    link_layer.nRetransmissions = nTries;
    link_layer.role = llRole;
    link_layer.timeout = timeout;
    strcpy(link_layer.serialPort, serialPort);

    if (llopen(link_layer) == -1)
    {
        printf("Error opening link layer\n");
        exit(-1);
    }

    if (llRole == LlTx)
    {
        sendFile(filename);
    }
    else
    {
        receiveFile(filename);
    }

    if (llclose(1) == -1)
    {
        printf("Error closing link layer\n");
        exit(-1);
    }

    printf("Application layer finished\n");
}
