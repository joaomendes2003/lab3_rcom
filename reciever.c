/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define START 0 
#define FlAG_RCV 1 
#define A_RCV 2 
#define C_RCV 3
#define BCC_OK 4
#define STOP 5


#define FLAG 0x5c
#define A 0x03
#define C 0x08
#define BCC1 A^C

volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255];

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

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
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

    printf("New termios structure set\n");


    int cont=0;
    unsigned char aux;
    int state = START;
    while (state!=STOP) {       /* loop for input */
        res = read(fd,aux,1);   /* returns after 1 chars have been input */
                   

        switch (state)
        {
        case START:
            if( aux[0] == FLAG) state=FlAG_RCV;
            break;

        case FlAG_RCV:
            if( aux[0] == FLAG) state=FlAG_RCV;
            else if( aux[0] == A) state=A_RCV;
            else state=START;
            break;

        case A_RCV:
            if( aux[0] == FLAG) state=FlAG_RCV;
            else if( aux[0] == C) state=C_RCV;
            else state=START;
            break;

        case C_RCV:
            if( aux[0] == FLAG) state=FlAG_RCV;
            else if( aux[0] == BCC1) state=BCC_OK;
            else state=START;
            break;

        case BCC_OK:
            if( aux[0] == FLAG) state=STOP;
            else state=START;
            break;

        default:
            break;
        }        
    }

    printf("SET message recieved\n")


    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
