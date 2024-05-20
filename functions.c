
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "linklayer.h"

typedef struct linkLayer{
    char serialPort[50];
    int role; // 0- Trans 1- Rec
    int baudRate;
    int numTries;
    int timeOut;
} linkLayer;

//CONNECTION default values
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

//BYTES
#define FLAG 0x5C
#define A_TRANS 0x01
#define A_REC 0x03

//Control bytes
#define C_SET 0x07
#define C_UA 0x06
#define C_DISC 0x0A
#define C_I0 0x80 
#define C_I1 0xC0
#define C_RR0 0x01
#define C_RR1 0x11
#define C_REJ0 0x05
#define C_REJ1 0x15

int fd;

struct linkLayer global_parameters; //for connection parameters

struct termios oldtio, newtio;

time_t start, end;

#define START 0 
#define FLAG_RCV 1 
#define A_RCV 2 
#define C_RCV 3
#define BCC1_OK 4
#define STOP 5
#define BCC2_OK 6
#define D_RCV 7

//use the given code to set the serial port defaults
void serial_port(struct termios *oldtio, struct termios *newtio, int fd) 
    {
    if (tcgetattr(fd,oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    bzero(newtio, sizeof(*newtio));
    newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio->c_iflag = IGNPAR;
    newtio->c_oflag = 0;

    newtio->c_lflag = 0;

    newtio->c_cc[VTIME] = 0;
    newtio->c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
}

int state_machine(_Bool timer, unsigned char control, char address){
    
    int state = START;
    unsigned char aux;
    bool flag_rej = 0;

    while(state != STOP){
        
        if(timer) //1 passed by call
        {
            end = clock();
            if((end - start) >= (global_parameters.timeOut * 1000)) return -2; //return in timeout
        }
        read(fd, aux, 1);
        switch(state){
            case START:
                if( aux == FLAG) state=FLAG_RCV;
                break;

            case FLAG_RCV:
                if(aux == address) //A received correctly
                    state = A_RCV;
            
                else if (aux == FLAG) state=FLAG_RCV;
                else state=START;

                break;
                
            case A_RCV:
                if(aux == control) state = C_RCV;
    
                else if(aux==FLAG) state = FLAG_RCV;
                else
                {
                    if(((control == C_RR0) || (control == C_RR1)) && ((aux == C_REJ0) || (aux == C_REJ1))) 
                    {
                        flag_rej=!flag_rej;
                        control = aux;
                    }
                    state = START;
                }
                
                break;

            case C_RCV:
                if(aux == (address ^ control)) state = BCC1_OK; //A^C
                
                else if(aux==FLAG) state = FLAG_RCV;
                else state = START;
                
                break;

            case BCC1_OK:
                if( aux == FLAG) state=STOP;
                else state=START;
                break;
        }
    }
    if(flag_rej) return 1;
}

//Opens a connection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer machine){ //machine -> connection parameters
    global_parameters = machine;

    //Open connection
    fd = open(global_parameters.serialPort, O_RDWR | O_NOCTTY ); 
    if (fd < 0) {perror(global_parameters.serialPort); exit(-1); }

    serial_port(&oldtio, &newtio, fd);

    unsigned char buf_SET[5] = {FLAG, A_TRANS, C_SET, (A_TRANS ^ C_SET), FLAG};
    unsigned char buf_UA[5] = {FLAG, A_TRANS, C_UA, (A_TRANS ^ C_UA), FLAG};

    switch(machine.role){
        case 0: //Transmitter
            //sends SET message
            start = clock();
            write(fd, buf_SET, 5);
            //printf("")

            int n_transmissions = 0;
            //receives UA message
            
            //Retransmission in case of timeout
            while(state_machine(1, C_UA, A_TRANS) == -2){
                if(n_transmissions == machine.numTries) return -1;
                start = clock();
                write(fd, buf_SET, 5);
                n_transmissions++;
            }
            return 1;

        case 1: //Receiver

            //receives SET message
            state_machine(0, C_SET, A_TRANS); //0 to not enter timer if

            //sends UA message
            write(fd, buf_UA, 5);
            
            return 1;
        default:
            return -1;
    }
}

// Closes previously opened connection; if statistics_print==TRUE, link layer should print statistics in the console on close
int llclose(linkLayer machine, int statistics_print){
    
    int n_transmissions = 0;

    switch(machine.role){
        case 0: //Transmitter

            unsigned char buf_DISC_T[5] = {FLAG, A_TRANS, C_DISC, (A_TRANS ^ C_DISC), FLAG};
            //sends DISC message
            write(fd, buf_DISC_T, 5);

            //receives DISC message
            
            while(state_machine(1, C_DISC, A_REC) == -2){
                
                if(n_transmissions == machine.numTries) return -1;
                write(fd, buf_DISC_T, 5);
                n_transmissions++;
            }
            
            //sends UA message
            unsigned char buf_UA[5] = {FLAG, A_REC, C_UA, (A_REC ^ C_UA), FLAG};
            write(fd, buf_UA, 5);
            break;
        case 1: //Receiver
            //receives DISC message
            state_machine(0, C_DISC, A_TRANS);

            //sends DISC message
            unsigned char buf_DISC_R[5] = {FLAG, A_REC, C_DISC, (A_REC ^ C_DISC), FLAG};
            write(fd, buf_DISC_R, 5);

            //printf("UA\t");
            //receives UA message
            while(state_machine(1, C_UA, A_REC) == -2){
                //printf("\nDISC (%d)\t", n_transmissions);
                if(n_transmissions == machine.numTries) return -1;
                write(fd, buf_DISC_R, 5);
                n_transmissions++;
            }
            break;
        default:
            return -1;
    }
    
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);

    return 1;
}



/*How does REJ work? Should the program look for the previous packet sent and compare the two and, if equal, send REJ?*/
/*Missing retransmissions at I frames*/
/*Timer should not start at reception of response*/