#ifndef LINKLAYER
#define LINKLAYER

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


#define FLAG 0x5C
#define A_TRANS 0x01
#define A_REC 0x03

#define C_SET 0x07
#define C_UA 0x06
#define C_DISC 0x0A
#define C_I0 0x80 
#define C_I1 0xC0
#define C_RR0 0x01
#define C_RR1 0x11
#define C_REJ0 0x05
#define C_REJ1 0x15

//Byte stuffing
#define ESC 0x5D
#define XOR_BYTE 0x20 

int fd;
_Bool nr = 1, ns = 0;

struct linkLayer global_parameters; //for connection parameters
struct termios oldtio, newtio;

time_t start, end;



//typedef enum {START, FLAG_RCV, A_RCV, C_RCV, BCC1_OK, D_N_RCV, ESC_REC, BCC2_OK, STOP} STATE;
//States
#define START 0 
#define FLAG_RCV 1 
#define A_RCV 2 
#define C_RCV 3
#define BCC1_OK 4
#define STOP 5
#define BCC2_OK 6
#define D_N_RCV 7
#define ESC_REC 8 

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

//State machine for control messages (SET, UA, DISC,...)
int state_machine(_Bool timer, unsigned char control, char address){
    int state = START;
    unsigned char aux[1];

    _Bool rej_flag = 0;


    while(state != STOP){
        
        if(timer) //1 passed by call
        {
            end = clock();
            if((end - start) >= (global_parameters.timeOut * 1000)) return -2;
        }
        read(fd, aux, 1);
        switch(state){
            case START:
                if( aux[0] == FLAG) state=FLAG_RCV;
                break;

            case FLAG_RCV:
                if(aux[0] == address) //A received correctly
                    state = A_RCV;
            
                else if (aux[0] == FLAG) state=FLAG_RCV;
                else state=START;

                break;
                
            case A_RCV:
                if(aux[0] == control) state = C_RCV;
    
                else if(aux[0]==FLAG) state = FLAG_RCV;
                else
                {
                    if(((control == C_RR0) || (control == C_RR1)) && ((aux[0] == C_REJ0) || (aux[0] == C_REJ1))) 
                    {
                        !rej_flag;
                        control = aux[0];
                    }
                    state = START;
                }
                break;

            case C_RCV:
                if(aux[0] == (address ^ control)) state = BCC1_OK; //A^C
                
                else if(aux[0]==FLAG) state = FLAG_RCV;
                else state = START;
                
                break;

            case BCC1_OK:
                if( aux[0] == FLAG) state=STOP;
                else state=START;
                break;
        }
    }
    if(rej_flag) return 1;
    //printf("\n");
}

// Opens a connection using the parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess
int llopen(linkLayer connectionParameters){ //connectionParameters -> connection parameters
    global_parameters = connectionParameters;

    //Open connection
    fd = open(global_parameters.serialPort, O_RDWR | O_NOCTTY ); 
    if (fd < 0) {perror(global_parameters.serialPort); exit(-1); }

    serial_port(&oldtio, &newtio, fd);

    unsigned char buf_SET[5] = {FLAG, A_TRANS, C_SET, (A_TRANS ^ C_SET), FLAG};
    unsigned char buf_UA[5] = {FLAG, A_TRANS, C_UA, (A_TRANS ^ C_UA), FLAG};

    switch(connectionParameters.role){
        case 0: //Transmitter
            //sends SET message
            start = clock();
            write(fd, buf_SET, 5);
            //printf("before while\n");

            int n_transmissions = 0;
            //receives UA message
            //printf("UA\t");

            //Retransmission in case of timeout
            while(state_machine(1, C_UA, A_TRANS) == -2){
                printf("in while\n");
                //printf("\nUA (%d)\t", n_transmissions);
                if(n_transmissions == connectionParameters.numTries) return -1;
                start = clock();
                write(fd, buf_SET, 5);
                n_transmissions++;
            }
            return 1;

        case 1: //Receiver
            //receives SET message
            //printf("SET\t");
            state_machine(0, C_SET, A_TRANS); //0 to not enter timer if

            //sends UA message
            write(fd, buf_UA, 5);
            
            return 1;
        default:
            return -1;
    }
}

// Sends data in buf with size bufSize
int llwrite(unsigned char* buf, int bufSize){

    int packetSize = bufSize + 6;
    unsigned char *packet = (unsigned char*)malloc(packetSize * sizeof(unsigned char)); //creates space for all 6 control bytes and for all data bytes
    int packet_index = 4;
    unsigned char bcc2;


    unsigned char c;
    switch(ns){ //depending on Ns a different C message is sent
        case 0:
            c = C_I0;
            break;
        case 1:
            c = C_I1;
    }

    packet[0] = FLAG; //controll messages
    packet[1] = A_TRANS;
    packet[2] = c;
    packet[3] = A_TRANS ^ c;

    
    for(int i = 0; i < bufSize; i++){ //this for loop will fill the packet array with the buffer
       
        switch(i){
            case 0:
                bcc2 = buf[0];
                break;
            default:
                bcc2 = bcc2 ^ buf[i]; //always calculatting a new bcc based on last D frame to send
        }
        switch(buf[i]){ //BITE STUFFING
            case FLAG: //If a frame equal to flag - send an ESC + 0x7c (flag^0x20)
                packetSize++;
                packet = (unsigned char*)realloc(packet, packetSize * sizeof(unsigned char));
                packet[packet_index++] = ESC;
                packet[packet_index++] = FLAG ^ XOR_BYTE;
                break;
            case ESC: //If an ESC is to send we should send ESC + 0x7d (ESC^0x20)
                packetSize++;
                packet = (unsigned char*)realloc(packet, packetSize * sizeof(unsigned char));
                packet[packet_index++] = ESC;
                packet[packet_index++] = ESC ^ XOR_BYTE;
                break;
            default:
                packet[packet_index++] = buf[i]; //default case, stores on packet the frame to be sent
        }
    }

    //parses last 2 bytes of packet array
    packet[packetSize - 2] = bcc2;
    packet[packetSize - 1] = FLAG;

    //After getting the packet ready, send it to the receiver

    write(fd, packet, packetSize);

    c = C_RR0;
    if(nr) c = C_RR1;

    int n_transmissions = 0;

    //receives RR(nr) message
    state_machine(1, c, A_TRANS);

    free(packet);
    nr = !nr;
    ns = !ns;
    //returns number of written characters
    return packetSize;
}

// Receive data in packet
int llread(unsigned char* packet){
    int state = START;
    int packetSize = 0;
    unsigned char* packetBuf = (unsigned char*)malloc(sizeof(unsigned char));
    unsigned char bcc2;
    int packet_index = 0;
    unsigned char aux[1];
    unsigned char carry;

    while(state != STOP){
        read(fd, aux, 1);
        
        switch(state){ //Ver maquina de estados caderno
            case START:
                if (aux[0]==FLAG) state=FLAG_RCV;
                else state=START;
                break;

            case FLAG_RCV:
                if( aux[0] == FLAG) state=FLAG_RCV;
                else if( aux[0] == A_TRANS) state=A_RCV;
                else state=START;
                break;               
                
            case A_RCV:
                if( aux[0] == FLAG) state=FLAG_RCV;
                else if( aux[0] == C_I0 || aux[0] == C_I1) state=C_RCV;
                else state=START;
                break;

            case C_RCV:
                if( aux[0] == FLAG) state=FLAG_RCV;
                else if( aux[0] == A_TRANS^C_I0 || aux[0] == A_TRANS^C_I1) 
                    state=BCC1_OK;
                else state=START;
                break;
                
            case BCC1_OK:
                if(aux[0]==FLAG) state=STOP;
                else if(aux[0] == ESC) state=ESC_REC;
                else
                {
                    bcc2 = aux[0];
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = aux[0];
                    state = D_N_RCV;
                }
                break;

            case D_N_RCV:
                if(aux[0] == bcc2)
                {
                    state = BCC2_OK;
                if(aux[0] == bcc2)
                    carry = aux[0];
                }
                if(aux[0]==FLAG) state=STOP;
                else if(aux[0]==ESC) state=ESC_REC;
                else
                {
                    bcc2 = bcc2 ^ aux[0];
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = aux[0];
                }
                break;

            case ESC_REC: //read the frame after de ESC frame, in order to decide what to store in de packet array
                if (aux[0]==FLAG ^ XOR_BYTE) //means that the frame was originaly a FLAG
                {
                    bcc2 = bcc2 ^ FLAG;
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = FLAG;
                }
                else if (aux[0]==ESC ^ XOR_BYTE) //means that the original frame was an ESC
                {
                    bcc2 = bcc2 ^ ESC;
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = ESC;
                }
                state = D_N_RCV;
                break;

            case BCC2_OK:
                if(aux[0]==FLAG) state=STOP;
                else //Caso de D ser um frame igual ao BCC
                {
                    //atualiza o BCC 2x para atualizar com a mensagem anterior(igual ao BCC) e com a atual (qualquer)
                    bcc2 = bcc2 ^ carry;
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = carry;

                    bcc2 = bcc2 ^ aux[0];
                    packetSize++;
                    packetBuf = (unsigned char*)realloc(packetBuf, packetSize * sizeof(unsigned char));
                    packetBuf[packet_index++] = aux[0];
                    state = D_N_RCV; 
                }
                break;
            
        }
    }

    memcpy(packet, packetBuf, packetSize); //guardar o pacote recebido na variavel packet
    

    unsigned char c = C_RR0;

    if(nr) c = C_RR1;

    unsigned char buf_RR[5] = {FLAG, A_TRANS, c, (A_TRANS ^ c), FLAG}; //Send RR message back to the transmitter
    write(fd, buf_RR, 5);

    nr = !nr;
    ns = !ns;
    return packetSize;
}

// Closes previously opened connection; if showStatistics==TRUE, link layer should print statistics in the console on close
int llclose(linkLayer connectionParameters, int showStatistics){
    
    int n_transmissions = 0;

    unsigned char buf_DISC_T[5] = {FLAG, A_TRANS, C_DISC, (A_TRANS ^ C_DISC), FLAG};

    switch(connectionParameters.role){
        case 0: //Transmitter
            //sends DISC message
            write(fd, buf_DISC_T, 5);

            //receives DISC message
            while(state_machine(1, C_DISC, A_REC) == -2){ //timeout retransmissions
                if(n_transmissions == connectionParameters.numTries) return -1;
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
                if(n_transmissions == connectionParameters.numTries) return -1;
                write(fd, buf_DISC_R, 5);
                n_transmissions++;
            }
            break;
        default:
            return -1;
    }
    
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);

    return 1;
}

#endif