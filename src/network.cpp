#include <sys/ioctl.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "network.h"

#define BSD_BASE_PORT 0
#define MYNAME "192.168.1.6"     // my IP address (onboard computer)
#define REMOTENAME "192.168.1.5" // IP address from the desktop in 001
#define THISPORTNUM 9001

struct sockaddr_in ma;
struct sockaddr_in ra;
struct sockaddr_in bma;
struct sockaddr_in bra;

#define BUFFERSIZE 1024
struct port_ref {
    void* sockDevice;
    int portNum;
    int isServer;
    int myport;
    int remoteport;
    int portIsOpen;
    int bytesread;
    int fd;
    unsigned int sent;
    unsigned int received;
    unsigned char buffer[BUFFERSIZE];
};
struct port_ref thisPort = {
  0,
  THISPORTNUM,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  {0},
};

struct datalinkHeader_ref {
    unsigned char sync1; /*  */
    unsigned char sync2; /*  */
    unsigned char sync3; /*  */
    unsigned char spare; /*  */
    int messageID; /* id # */
    int messageSize; /* including header */
    unsigned int hcsum; /*  */
    unsigned int csum; /*  */
};
struct datalinkHeader_ref datalinkHeader = {
  0xa1,
  0xb2,
  0xc3,
  0,
  0,
  sizeof(struct datalinkHeader_ref),
  0,
  0,
};

#define DATALINK_MESSAGE_OPTITRACK 230
struct onboardMocapClient_ref {
    unsigned char sync1; /*  */
    unsigned char sync2; /*  */
    unsigned char sync3; /*  */
    unsigned char spare; /*  */
    int messageID; /* id # */
    int messageSize; /* including header */
    unsigned int hcsum; /*  */
    unsigned int csum; /*  */
    float pos_x; /*  */
    float pos_y; /*  */
    float pos_z; /*  */
    float qx; /*  */
    float qy; /*  */
    float qz; /*  */
    float qw; /*  */
    int frameNum; /*  */
    int valid; /*   */
};
struct onboardMocapClient_ref onboardMocapClient = {
  0xa1,
  0xb2,
  0xc3,
  0,
  DATALINK_MESSAGE_OPTITRACK,
  sizeof(struct onboardMocapClient_ref),
  0,
  0,
  0,0,0,
  0,0,0,0,
  0,
  0,
};

unsigned int datalinkCheckSumCompute(unsigned char* buf, int byteCount) {

    unsigned int sum1 = 0xffff;
    unsigned int sum2 = 0xffff;
    unsigned int tlen = 0;
    unsigned int shortCount = byteCount / sizeof(short);
    unsigned int oddLength = byteCount % 2;

    /* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

    while (shortCount) {
        /* 360 is the largest number of sums that can be performed without overflow */
        tlen = shortCount > 360 ? 360 : shortCount;
        shortCount -= tlen;
        do {
            sum1 += *buf++;
            sum1 += (*buf++ << 8);
            sum2 += sum1;
        } while (--tlen);

        /* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
        if ((oddLength == 1) && (shortCount < 1)) {
            sum1 += *buf++;
            sum2 += sum1;
        }

        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
    }

    /* Second reduction step to reduce sums to 16 bits */
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);

    return(sum2 << 16 | sum1);
}

static int getaddr(struct sockaddr_in* ad, char* hostname, unsigned short port) {

    struct hostent* hp;

    memset(ad, 0, sizeof(struct sockaddr_in)); /* clear our address */
    hp = gethostbyname(hostname);
    if (NULL == hp) {
        printf("getaddr: lookup of %s failed\n", hostname);
        return(-1);
    }
    memcpy((char*)(&(ad->sin_addr)), hp->h_addr, hp->h_length);
    ad->sin_family = AF_INET;
    ad->sin_port = htons(port);
    return 0;
}

int openPort(void) {

    struct port_ref* s = &thisPort;
    char remotename[120];
    char myname[120];

    sprintf(myname, MYNAME);
    sprintf(remotename, REMOTENAME);

    // formulate addresses of my port and remote ports
    if (s->isServer) {
        s->myport = BSD_BASE_PORT + s->portNum + 1000;
        s->remoteport = BSD_BASE_PORT + s->portNum;
    }
    else {
        s->myport = BSD_BASE_PORT + s->portNum;
        s->remoteport = BSD_BASE_PORT + s->portNum + 1000;
    }

    if (getaddr(&bma, myname, s->myport) != 0) return -1;
    if (getaddr(&bra, remotename, s->remoteport) != 0) return -1;
    memcpy(&ma, &bma, sizeof(bma));
    memcpy(&ra, &bra, sizeof(bra));

    // create the socket
    s->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == s->fd) {
        printf("network: openSock Cannot make socket\n");
        return -1;
    }
    else {
        unsigned long one = 1;
        ioctl(s->fd, FIONBIO, &one);
    }

    if (bind(s->fd, (struct sockaddr*)(&ma), sizeof(ma)) == -1) {
        printf("network: open Cannot bind socket to %s:%d (unable to connect to %s:%d), errno=%s\n",
            myname, s->myport, remotename, s->remoteport, strerror(errno));
        s->portIsOpen = 0;
        return(-1);
    }
    else {
        printf("network: UDP Open SUCCESS (listening at %s:%d to %s:%d)\n",
            myname, s->myport, remotename, s->remoteport);
    }
    s->portIsOpen = 1;
    return 0;
}

static int readSock(struct port_ref* s, void* buf, int nbytes) {
    int totalNew = 0;
    if (s->portIsOpen) {
        int toread = 0, status;
        socklen_t sizeofsockaddr = sizeof(struct sockaddr);
        memcpy(&ra, &bra, sizeof(bra));
        do {
            status = ioctl(s->fd, FIONREAD, &toread);
            if (status == -1) {
                printf("network: problem receiving\n");
                return totalNew;
            }
            status = recvfrom(s->fd, (char*)buf, (toread < nbytes ? toread : nbytes), 0, (struct sockaddr*)(&ra), &sizeofsockaddr);
            if (status > 0) {
                buf = (void*)((char*)buf + status);
                totalNew += status;
                s->received += status;
            }
        } while (status > 0);
    }
    return totalNew;
}

Vector5d readDatalink() {
    struct port_ref* s = &thisPort;
    int gotPacket = 0, newBytes;
    int done = 0, index = 0;
    unsigned char* bf;

    // Initialize return matrix with zeros or NaNs to indicate "no data"
    Vector5d result = Vector5d::Zero();

    newBytes = readSock(s, s->buffer + s->bytesread, BUFFERSIZE - s->bytesread);
    s->bytesread += newBytes;

    while ((index <= s->bytesread - (int)sizeof(datalinkHeader_ref)) && !done)
    {
        if ((s->buffer[index] == 0xa3) &&
            (s->buffer[index + 1] == 0xb2) &&
            (s->buffer[index + 2] == 0xc1))
        {
            bf = &(s->buffer[index]);
            memcpy(&datalinkHeader, bf, sizeof(datalinkHeader_ref));

            if (datalinkCheckSumCompute(bf, sizeof(struct datalinkHeader_ref) - sizeof(int) * 2) == datalinkHeader.hcsum &&
                datalinkHeader.messageSize >= sizeof(struct datalinkHeader_ref) &&
                datalinkHeader.messageSize < BUFFERSIZE)
            {
                if (datalinkHeader.messageSize + index <= s->bytesread)
                {
                    if (datalinkCheckSumCompute(&bf[sizeof(struct datalinkHeader_ref)], datalinkHeader.messageSize - sizeof(struct datalinkHeader_ref)) == datalinkHeader.csum)
                    {
                        switch (datalinkHeader.messageID)
                        {
                        case DATALINK_MESSAGE_OPTITRACK:
                            if (datalinkHeader.messageSize == sizeof(struct onboardMocapClient_ref)) {
                                memcpy(&onboardMocapClient, bf, datalinkHeader.messageSize);

                                double qx = (double)onboardMocapClient.qx;
                                double qy = (double)onboardMocapClient.qy;
                                double qz = (double)onboardMocapClient.qz;
                                double qw = (double)onboardMocapClient.qw;
                                double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

                                // Map struct values to Eigen Matrix
                                result(0) = (double)onboardMocapClient.pos_y; // North
                                result(1) = (double)onboardMocapClient.pos_x; // East
                                result(2) = (double)onboardMocapClient.pos_z * -1.0; // Up converted to down
                                result(3) = yaw;
                                result(4) = (double)onboardMocapClient.valid;  // 0 for non valid, 1 for valid
                            }
                            break;
                        }
                    }
                    index += datalinkHeader.messageSize - 1;
                }
                else {
                    index--;
                    done = 1;
                }
            }
            else {
                index += sizeof(datalinkHeader_ref);
            }
        }
        index++;

        if (index < 0) index = BUFFERSIZE - 1;
    }

    // Buffer cleanup logic (remains the same)
    if (index != 0) {
        if (index >= s->bytesread) {
            s->bytesread = 0;
        }
        else {
            memmove(s->buffer, &(s->buffer[index]), s->bytesread - index);
            s->bytesread -= index;
        }
    }

    return result;
}