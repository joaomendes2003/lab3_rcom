/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define START 0 
#define FLAG_RCV 1 
#define A_RCV 2 
#define C_RCV 3
#define BCC_OK 4
#define STATE_STOP 5


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





volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255];
    int i, sum = 0, speed = 0;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */



    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


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
            else if( aux == BCC1_UA) state=BCC_OK;
            else state=START;
            break;

        case BCC_OK:       
            if( aux == FLAG) state=STATE_STOP;
            else state=START;
            break;

        default:
            break;
        }        
    }

    printf("UA message recieved\n");


    /* Send I */
    
    buf[0]=FLAG;
    buf[1]=A;
    buf[2]=C_I;
    buf[3]=BCC1_I;
    buf[4]=D; 
    buf[5]= BCC2_I;
    buf[6]= FLAG;
   
    res = write(fd,buf,5);
    printf("%d bytes written - I message sent\n", res);

   
    /* Receive RR */
    state = START;

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
            else if( aux == C_RR) state=C_RCV;
            else state=START;
            break;

        case C_RCV:
            if( aux == FLAG) state=FLAG_RCV;
            else if( aux == BCC1_RR) state=BCC_OK;
            else state=START;
            break;

        case BCC_OK:       
            if( aux == FLAG) state=STATE_STOP;
            else state=START;
            break;

        default:
            break;
        }        
    }

    printf("RR message recieved\n");
   


    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */


    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
