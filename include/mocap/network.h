int openPort( void );
int readDatalink( void );

// network.h  – add this
struct onboardMocapClient_ref {
    unsigned char sync1, sync2, sync3, spare;
    int          messageID, messageSize;
    unsigned int hcsum, csum;
    float pos_x, pos_y, pos_z;
    float qx, qy, qz, qw;
    int   frameNum, valid;
};
extern onboardMocapClient_ref onboardMocapClient;