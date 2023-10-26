// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>


void sendControlPacket(uint8_t controlField, size_t filesize) {
    uint8_t buf[MAX_PAYLOAD_SIZE];

    buf[0] = controlField;
    buf[1] = FILESIZE;
    buf[2] = 4;
    buf[3] = filesize >> 24;
    buf[4] = filesize >> 16;
    buf[5] = filesize >> 8;
    buf[6] = filesize;

    llwrite(buf, 7);
}

void sendDataPacket(uint8_t* buf, ssize_t size) {
    uint8_t packet[MAX_PAYLOAD_SIZE + 3];

    packet[0] = DATA;
    packet[1] = (size >> 8) & 0xFF;
    packet[2] = size & 0xFF;
    memcpy(&packet[3], buf, size);

    llwrite(packet, size + 3);
}

size_t receiveControlPacket(uint8_t controlField, uint8_t flag) {
    uint8_t buf[MAX_PAYLOAD_SIZE];
    while (llread(buf) == -1);

    if (buf[0] != controlField) {
        printf("Error receiving control packet\n");
        exit(-1);
    }

    if (buf[1] != flag) {
        printf("Error receiving control packet\n");
        exit(-1);
    }

    if (buf[2] != 4) {
        printf("Error receiving control packet\n");
        exit(-1);
    }

    size_t field = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];

    printf("Received control packet with field %ld\n", field);

    return field;
}

int receiveDataPacket(uint8_t* buf) {
    uint8_t packet[MAX_PAYLOAD_SIZE + 3];
    while (llread(packet) == -1);

    if (packet[0] != DATA) {
        printf("Error receiving data packet\n");
        printf("Expected DATA, got %d\n", packet[0]);
        exit(-1);
    }

    int size = (packet[1] << 8) | packet[2];
    memcpy(buf, &packet[3], size);

    return size;
}

void sendFile(const char* filename) {
    printf("Sending file %s\n", filename);
    int fd = open(filename, O_RDONLY);

    if (fd == -1) {
        printf("Error opening file\n");
        exit(-1);
    }

    size_t filesize = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);

    sendControlPacket(START, filesize);

    uint8_t buf[MAX_PAYLOAD_SIZE];
    ssize_t bytesRead = 0, packetNumber = 1;

    while ((bytesRead = read(fd, buf, MAX_PAYLOAD_SIZE)) > 0) {
        sendDataPacket(buf, bytesRead);
        packetNumber++;
        sleep(1);
    }

    sendControlPacket(END, filesize);

    close(fd);
}

void receiveFile(const char* filename) {
    printf("Receiving file %s\n", filename);
    int fd = open(filename, O_WRONLY | O_CREAT, 0666);

    if (fd == -1) {
        printf("Error opening file\n");
        exit(-1);
    }

    size_t filesize = receiveControlPacket(START, FILESIZE);

    uint8_t buf[MAX_PAYLOAD_SIZE];
    ssize_t bytesRead = 0, bytesWritten = 0;

    while (bytesWritten < filesize) {
        bytesRead = receiveDataPacket(buf);
        bytesWritten += write(fd, buf, bytesRead);
        printf("Wrote %ld bytes into the file\n", bytesRead);
        sleep(1);
    }

    printf("Wrote a total of %ld bytes into the file\n", bytesWritten);
    receiveControlPacket(END, FILESIZE);

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
