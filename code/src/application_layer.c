// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>


void sendControlPacket(uint8_t controlField, const char* field) {
    uint8_t buf[MAX_PAYLOAD_SIZE];

    buf[0] = controlField;
    buf[1] = strlen(field);
    strcpy((char*) &buf[2], field);

    llwrite(buf, strlen(field) + 2);
}

void sendDataPacket(uint8_t* buf, ssize_t size) {
    uint8_t packet[MAX_PAYLOAD_SIZE + 3];

    packet[0] = DATA;
    packet[1] = (size >> 8) & 0xFF;
    packet[2] = size & 0xFF;
    memcpy(&packet[3], buf, size);

    llwrite(packet, size + 3);
}

void sendFile(const char* filename) {
    int fd = open(filename, O_RDONLY);

    if (fd == -1) {
        printf("Error opening file\n");
        exit(-1);
    }

    sendControlPacket(START, filename);

    uint8_t buf[MAX_PAYLOAD_SIZE];
    ssize_t bytesRead = 0, packetNumber = 1;

    while ((bytesRead = read(fd, buf, MAX_PAYLOAD_SIZE)) > 0) {
        sendDataPacket(buf, bytesRead);
        packetNumber++;
    }

    sendControlPacket(END, filename);
    close(fd);
}

void receiveFile(const char* filename) {
    int fd = open(filename, O_WRONLY);

    if (fd == -1) {
        printf("Error opening file\n");
        exit(-1);
    }

    uint8_t buf[MAX_PAYLOAD_SIZE];
    ssize_t bytesRead = 0, packetNumber = 1;

    while ((bytesRead = llread(buf)) > 0) {
        assert(buf[0] == DATA);
        size_t bytesToWrite = (buf[1] << 8) | buf[2];
        write(fd, &buf[3], bytesToWrite);
        packetNumber++;
    }

    close(fd);
}

void applicationLayer(const char* serialPort, const char* role, int baudRate,
    int nTries, int timeout, const char* filename) {

    LinkLayer link_layer = {
        .role = strcmp(role, "tx") == 0 ? LlTx : LlRx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };

    strcpy(link_layer.serialPort, serialPort);

    if (llopen(link_layer) == -1) {
        printf("Error opening link layer\n");
        exit(-1);
    }

    if (link_layer.role == LlTx) sendFile(filename);
    else receiveFile(filename);

    if (llclose(1) == -1) {
        printf("Error closing link layer\n");
        exit(-1);
    }

    printf("Application layer finished\n");
}
