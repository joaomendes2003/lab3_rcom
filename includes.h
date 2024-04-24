//STATES
#define START 0 
#define FLAG_RCV 1 
#define A_RCV 2 
#define C_RCV 3
#define BCC1_OK 4
#define STATE_STOP 5
#define BCC2_OK 6
#define D_RCV 7

//VALUES
#define FLAG 0x5c
#define A 0x03
#define C_SET 0x07
#define C_UA 0x06
#define C_RR 0x11
#define C_I 0xC0
#define D 0x00
#define BCC1_SET A^C_SET
#define BCC1_UA A^C_UA
#define BCC1_RR A^C_RR
#define BCC1_I A^C_I
#define BCC2_I A^D



