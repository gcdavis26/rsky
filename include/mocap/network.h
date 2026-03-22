int openPort( void );
int readDatalink( void );

// network.h  – add this
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
extern onboardMocapClient_ref onboardMocapClient;