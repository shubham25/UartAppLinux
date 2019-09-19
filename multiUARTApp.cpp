#include<stdio.h>
#include<fcntl.h>
#include <stdlib.h>
#include <iostream>
#include<unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include<termios.h>   // using the termios.h library
#include <pthread.h>
#include <sys/ioctl.h>


// #define SAME

// uart max 3.6864 Mbps
// B4000000    B50            B75         B110        B134
// B150        B200           B300        B600        B1200
// B1800       B2400          B4800       B9600       B19200
// B38400      B57600         B115200     B230400     B460800    
// B500000     B576000        B921600     B1000000    B1152000
// B1500000    B2000000       B2500000    B3000000    B3500000   
#define BAUD B2000000
#define DEFAULT_CNT 200
#define DEFAULT_BUF 6
#define RESPONSE_BUF 60

struct UART
{
  int file;
  int id;
  struct termios options;
  speed_t baud;
  struct termios saved_options;
  unsigned char receiveBuff[500];
};

//pthread_t writeThrdID_1,writeThrdID_2,writeThrdID_4,writeThrdID_5 , readThrdID_1 , readThrdID_2 , readThrdID_5 ,readThrdID_4; 
pthread_t obccThreadID, scubiThreadID;
// pthread_mutex_t wrMtx1;
struct UART *uart1 , *uart2, *uart4, *uart5;

int pauseth = 0 , pauseth_wr = 0;
int tc1=0;//,tc2=0,tc4=0,tc5=0;
int loopCount = DEFAULT_CNT;
int buffLen = DEFAULT_BUF;

//bool RxFlag_1 = false;
//bool RxFlag_5 = true;

unsigned char cmdBuff[8] = {0xff, 0xa,0xb,0xc,0xd,0xe,0x1,0x2};

//unsigned char receive_1[200] , receive_2[200], receive_4[200], receive_5[200];

void *obccThread(void *arg);
void *scubiThread(void *arg);
//void *readThread_1(void *arg);
//void *readThread_2(void *arg);
//void *readThread_4(void *arg);
//void *readThread_5(void *arg);
//void *writeThread_1(void *arg);
//void *writeThread_5(void *arg);
void reset_input_mode (struct UART *uart);
int uartOpen( struct UART *uart, char* name);
void uartClose( struct UART *uart);
int setupUART(struct UART *uart,speed_t spd, unsigned int vmin , unsigned int vtime);

int gsleep (int usDelay)
{
    int ret;
    struct timespec request, remain, initTime;
    clock_gettime (CLOCK_MONOTONIC, &initTime);
    request.tv_sec =initTime.tv_sec + (usDelay/1000000);
    long delayNs = (usDelay % 1000000) * 1000;
    if (1e9 > (initTime.tv_nsec + delayNs ))
        request.tv_nsec = initTime.tv_nsec + delayNs;
    else {
        request.tv_sec++;
        request.tv_nsec = initTime.tv_nsec + delayNs - 1e9;
    }
    do {
        ret = clock_nanosleep (CLOCK_MONOTONIC, TIMER_ABSTIME, &request, &remain);
        if ((ret == EFAULT) || (ret == EINVAL))  break;
        /// ret will never be EFAULT as request and remain are local variables with valid addresses
        /// ret will never be EINVAL because it is checked against 1e9
    }while (ret != 0);

    return ret;
}

int needQuit(pthread_mutex_t *mtx)
{
  switch(pthread_mutex_trylock(mtx)) {
    case 0: /* if we got the lock, unlock and return 1 (true) */
      pthread_mutex_unlock(mtx);
      return 1;
    case EBUSY: /* return 0 (false) if the mutex was locked */
      return 0;
  }
  return 1;
}

void printInitMSG()
{
  printf("---- Multi uart Application  ---\n");
  printf("usage : ./multiUART _loopCount _buffLength\n");
  printf("Controls :  w - write thread . y - print read stat . x - exit \n");
}

int main(int argc, char *argv[])
{

	//printf("Hello World1\n");
  uart1 = new UART;
  //uart2 = new UART;
  //uart4 = new UART;
  uart5 = new UART;
  printInitMSG();
  if( uartOpen(uart1, "/dev/ttyS1") < 0 )
    printf("uart1 open error %d , %s\n", uart1->file , strerror(errno));
  //if( uartOpen(uart2, "/dev/ttyS2") < 0 )
    //printf("uart2 open error %d , %s\n", uart2->file , strerror(errno));
  //if( uartOpen(uart4, "/dev/ttyS4") < 0 )
    //printf("uart4 open error %d , %s\n", uart4->file , strerror(errno));
  if( uartOpen(uart5, "/dev/ttyS5") < 0 )
  {
   printf("uart5 open error %d , %s\n", uart5->file , strerror(errno));
	}

   if(argc>=2)
   {
    loopCount = atoi(argv[1]);
    }
   else
   {
    loopCount = DEFAULT_CNT;
    }

	if(argc>=8)
	{
		unsigned char byte;
		for(int i=0;i<6;i++)
		{
			sscanf(argv[i+2],"%hhx",&byte);
			cmdBuff[i+1] = byte;
		}
	}
	for(int i=0;i<7;i++)
		printf("%x ",cmdBuff[i]);
	
	printf("\n"); 
	
  // if(argc>=3)
  //   buffLen = atoi(argv[2]);
  //  else
  //   buffLen = DEFAULT_BUF;

  setupUART( uart1,BAUD, 6,1 );
  //setupUART( uart2,BAUD );
  setupUART( uart5,BAUD,2,1 );  
  //setupUART( uart4,BAUD );

  uart1->id = 1;
  //uart2->id = 2;
  //uart4->id = 4;
  uart5->id = 5;
  printf("UART setup done\n");

	
  
 
  if(pthread_create(&obccThreadID, NULL, &obccThread, uart5 ))
  {
    printf("Failed to create obcc thread %s \n", strerror(errno));
    // close file
    reset_input_mode(uart5);
    uartClose(uart5);
    return -1;
  }

	
  if(pthread_create(&scubiThreadID, NULL, &scubiThread, uart1 ))
  {
    printf("Failed to create scubi thread %s \n", strerror(errno));
    // close file
    reset_input_mode(uart1);
    uartClose(uart1);
    return -1;
  }
  
  printf("Threads active\n");

  while(1)
  {
    char a;
    std::cin >>a;
    if( (a == 'x') || (a == 'X') ) break;
    if( (a == 'Y') || (a == 'y') ) 
    {
      printf("Total bytes rx1: %d\n", tc1);
      fflush(stdout);
    }
    if( (a == 'a') || (a == 'A') ) pauseth=1;
    if( (a == 'W') || (a == 'w') ) pauseth_wr=1;
  }


  pthread_cancel(obccThreadID);
  pthread_cancel(scubiThreadID);

  pthread_join(obccThreadID,NULL);
  pthread_join(scubiThreadID,NULL);
  
  printf("Total bytes rx1: %d\n", tc1);
  
  //reset_input_mode(uart2);
  reset_input_mode(uart1);
  //reset_input_mode(uart4);
  reset_input_mode(uart5);
  //uartClose(uart2);
  uartClose(uart1);
  //uartClose(uart4);
  uartClose(uart5);

  delete uart1;
  //delete uart2;
  //delete uart4;
  delete uart5;

  return 0;
}

void *obccThread(void *arg)
{
  struct UART *uart = (struct UART*) arg;
  int file = uart->file;
  // free after copying to local mem
  // free(arg); 
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  unsigned char *receive = uart->receiveBuff;
  
  while( !pauseth_wr) gsleep(1000);
  //unsigned char send[8];
  //send[0] = 0xff;
  int count_wr =0;
  int count_rd=0;
  int total_count=0;
  for(int k=0;k<loopCount;k++)
  { 
  
     count_wr = write(file, cmdBuff, 7);
     // count = dprintf(file,"%x ",i);
     // as vmin != 0 , count not 0
      // i++;
      // write(file, &gap, 1);
     total_count += count_wr;
    // gsleep(15); // provide triple dalay for same channel
   // }while(i<255);  
     gsleep(500);
     
     // read now
	memset(receive,'\0',500);
    count_rd = read(file, receive, 500);
    if (count_rd <= 0)
      printf("Failed to read: %d, %s \n", count_rd, strerror(errno) );
    else
    {
    tc1 += count_rd;
    for(int j=0;j<count_rd;j++)
    {
     printf("%x ",  receive[j]);
    }
    printf("\n");
    fflush(stdout);
    }
    }
    printf("send bytes : %d\n" , total_count);
    pthread_exit(NULL);
}


void *scubiThread(void *arg)
{
  // pthread_mutex_t *mx = (pthread_mutex_t*)arg;
  struct UART *uart = (struct UART*) arg;
  int file = uart->file;
  // free after copying to local mem
  // free(arg); 
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  unsigned char send[RESPONSE_BUF+2] ;
  int count;
  //int total_count = 0;
  unsigned char *receive = uart->receiveBuff;
//printf("Hello World1\n");
  unsigned char gap = 0xFF;
  unsigned char cost = 0x01;
  send[0] = gap;
  for(int k = 1;k<=RESPONSE_BUF;k+=1)
  {
    send[k] = cost;
    cost = cost  +1;
  }
  //printf("Hello World3\n");
  while( !pauseth) gsleep(1000);
  //printf("Hello World4\n");
  printf("Write started pauseth : %d\n", pauseth);
  
  do
  {
  memset(receive,'\0',10);
  count = read(file, receive, 10);
  printf("scubi rx: ");
  for(int j=0;j<count;j++)
    {
     printf("%x ",  receive[j]);
    }
    printf("\n");
    
    count = write(file, send, RESPONSE_BUF+1);
    
    
  }while(1);
 
  // free(arg);
  pthread_exit(NULL);
  return (void *)0; 
} 




// ----- UART Functions -----
int uartOpen( struct UART *uart, char* name)
{
  int file = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
  if ( file <0){
    // printf("file open error %d , %s\n", file , strerror(errno));
    // return file;
  }
  else
  fcntl(file, F_SETFL, 0);
  uart->file = file;
  return file;
}

void uartClose( struct UART *uart)
{
  int file = uart->file;
  close(file);
}

void reset_input_mode (struct UART* uart)
{
  int rc;
  int file = uart->file;
  struct termios saved_options = uart->saved_options;
  // tcflush wr
  #ifdef SAME
		rc = tcdrain(file);
		if( rc < 0)
		{
		  printf("file drain error %d , %s\n",rc , strerror(errno) );
		}
	#else
	  rc = tcflush(file , TCIOFLUSH);
	  if( rc < 0)
	  {
	    printf("file flush error %d , %s\n",rc , strerror(errno) );
	  }
  #endif

  rc = tcsetattr(file, TCSANOW , &saved_options); 
  if( rc < 0)
  {
    printf("attrb SET error %d , %s\n",rc , strerror(errno) );
  }
  return;
}

int setupUART(struct UART *uart,speed_t spd , unsigned int vmin , unsigned int vtime)
{
  int file = uart->file;
  struct termios options;
  int rc;
  rc = tcgetattr(file, &options); 
  if( rc < 0)
  {
    printf("attrb get error %d , %s\n",rc , strerror(errno) );
    return -2;
  }

  uart->saved_options = options;
  // atexit (reset_input_mode);

  // change terminal attributes
  #ifndef SAME
  	cfsetospeed(&options, (speed_t)spd);
	#endif
  
  cfsetispeed(&options, (speed_t)spd);

  #ifndef SAME
	  cfmakeraw(&options);    // use raw bsd mode
  #endif

  // 8N1
  // options.c_cflag &= ~PARENB;
  // options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag |= PARENB;
  options.c_cflag &= ~PARODD;
  options.c_cflag |= PARODD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag |= CSTOPB;
  options.c_cflag &= ~CSIZE;  // mask for character size
  options.c_cflag |= CS8;
  
  //Enable the receiver and set local mode.
  options.c_cflag &= ~(CLOCAL | CREAD);
  options.c_cflag |= (CLOCAL | CREAD);
  // options.c_cflag &= ~CREAD;   // read disable

  // options.c_lflag |= (ICANON | ECHO | ECHOE);  // canonical form
  // options.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */ // non canonical form
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOK | ECHOCTL | ECHOKE);  // raw mode ECHOK | ECHOCTL | ECHOKE);


  options.c_cflag &= ~CRTSCTS; // disable hardware control
  // options.c_cflag |= CRTSCTS;  // enable hw control

  options.c_iflag &= ~(IGNPAR | ICRNL | ISTRIP | IGNBRK | IMAXBEL);   // IGNBRK | IMAXBEL);
  // options.c_iflag &= ~(IXON | IXOFF); 	
  options.c_iflag |= (IGNPAR | IGNBRK) ;    //ignore partity errors, CR -> newline IGNBRK) ; 

  options.c_oflag &= ~(ONLCR);   // minicom

  // options.c_iflag |= (IXON | IXOFF);
  // options.c_iflag |= (IXON );
  // options.c_iflag |= (IXOFF);

  // timeouts
  #ifndef SAME 
	  options.c_cc[VMIN]  = vmin;
	  options.c_cc[VTIME] = vtime;
  #endif

  uart->options = options;
  uart->baud = (speed_t) spd;
  // flush wr
	#ifdef SAME
		rc = tcflush(file , TCIFLUSH);
		if( rc < 0)
		{
		  printf("file flush error %d , %s\n",rc , strerror(errno) );
		}
	#else
	  rc = tcflush(file , TCIOFLUSH);
	  if( rc < 0)
	  {
	    printf("file flush error %d , %s\n",rc , strerror(errno) );
	  }
  #endif

  rc = tcsetattr(file, TCSANOW , &options); 
  if( rc < 0)
  {
    printf("attrb SET error %d , %s\n",rc , strerror(errno) );
    return -2;
  }
  return 0;
}

