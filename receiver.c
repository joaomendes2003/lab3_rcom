/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include "includes.h"
#include "linklayer.h"

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

int llopen(linkLayer machine){

    unsigned char buf[255];
    unsigned char aux;
    int res, fd;
    struct termios oldtio,newtio;
    int state = START;

    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */
    fd = open(machine.serialPort, O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(machine.serialPort); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    if (machine.role==0)
    {
        buf[0]=FLAG;
        buf[1]=A;
        buf[2]=C_SET;
        buf[3]=BCC1_SET;
        buf[4]=FLAG;   
    
        res = write(fd,buf,5);
        printf("%d bytes written - SET message sent\n", res);


        unsigned char aux;
        int state = START;

        /* Receive UA */
        while (state!=STATE_STOP) {   
            
            res = read(fd,&aux,1);   /* returns after 1 chars have been input */
                
            switch (state)
            {
            case START:
                if( aux == FLAG) state=FLAG_RCV;
                break;

            case FLAG_RCV:          
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == A) state=A_RCV;
                else state=START;
                break;

            case A_RCV:           
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == C_UA) state=C_RCV;
                else state=START;
                break;

            case C_RCV:
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == BCC1_UA) state=BCC1_OK;
                else state=START;
                break;

            case BCC1_OK:       
                if( aux == FLAG) state=STATE_STOP;
                else state=START;
                break;

            default:
                break;
            }        
        }

        printf("UA message recieved\n");
        }

    else if (machine.role==1)
    {
        while (state!=STATE_STOP) {       /* Receive SET */
            res = read(fd,&aux,1);   
                    
            state=START;
            switch (state)
            {
            case START:
                if( aux == FLAG) state=FLAG_RCV;
                break;

            case FLAG_RCV:
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == A) state=A_RCV;
                else state=START;
                break;

            case A_RCV:
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == C_SET) state=C_RCV;
                else state=START;
                break;

            case C_RCV:
                if( aux == FLAG) state=FLAG_RCV;
                else if( aux == BCC1_SET) state=BCC1_OK;
                else state=START;
                break;

            case BCC1_OK:
                if( aux == FLAG) state=STATE_STOP;
                else state=START;
                break;

            default:
                break;
            }        
        }

        printf("SET message received\n");

        //Send UA
        buf[0]=FLAG;
        buf[1]=A;
        buf[2]=C_UA;
        buf[3]=BCC1_UA;
        buf[4]=FLAG;   
    
        res = write(fd,buf,5);
        printf("%d bytes written - UA message sent\n", res);
    }

    else
        return -1;

    return 1;
}



int main(int argc, char** argv)
{
    linkLayer _machine_;
    _machine_.role=1;
    int fd,c, res;
    unsigned char aux;
    struct termios oldtio,newtio;
    unsigned char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    
    printf("New termios structure set\n");


    if (llopen(_machine_)==1)
        printf("Connection Stablished\n");
    else
         printf("ERROR while Establishing connection\n");
    

    

    //Receive I
    int state = START;
    while (state!=STATE_STOP) {       /* Receive I */

        res = read(fd,&aux,1);   /* returns after 1 chars have been input */
                   

        switch (state)
        {
        case START:
            if( aux == FLAG) state=FLAG_RCV;
            break;

        case FLAG_RCV:
            if( aux == FLAG) state=FLAG_RCV;
            else if( aux == A) state=A_RCV;
            else state=START;
            break;

        case A_RCV:
            if( aux == FLAG) state=O_RDWR | O_NOCTTY FLAG_RCV;
            else if( aux == C_SET) state=C_RCV;
            else state=START;
            break;

        case C_RCV:
            if( aux == FLAG) state=FLAG_RCV;
            else if( aux == BCC1_SET) state=BCC1_OK;
            else state=START;
            break;

        case BCC1_OK:
            if( aux == D) state=D_RCV;
            if( aux == FLAG) state=FLAG_RCV;
            else state=START;
            break;

        case D_RCV:
            if( aux == BCC2_I) state=BCC2_OK;
            if( aux == FLAG) state=FLAG_RCV;
            else state=START;
            break;

        case BCC2_OK:
            if( aux == FLAG) state=STATE_STOP;
            else state=START;
            break;

        default:
            break;
        }        
    }

    printf("I message received\n");

    //Send RR
    buf[0]=FLAG;
    buf[1]=A;
    buf[2]=C_RR;
    buf[3]=BCC1_RR;
    buf[4]=FLAG;   
   
    res = write(fd,buf,5);
    printf("%d bytes written - RR message sent\n", res);

    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
