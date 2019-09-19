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

struct UART
{
  int file;
  int id;
  struct termios options;
  speed_t baud;
  struct termios saved_options;
  unsigned char receiveBuff[500];
};

pthread_t writeThrdID , readThrdID_1 , readThrdID_2 , readThrdID_5 ,readThrdID_4; 
// pthread_mutex_t wrMtx;
struct UART *uart1 , *uart2, *uart4, *uart5;

int pauseth = 0 , pauseth_wr = 0;
int tc1=0,tc2=0,tc4=0,tc5=0;
int loopCount = DEFAULT_CNT;
int buffLen = DEFAULT_BUF;

unsigned char receive_1[200] , receive_2[200], receive_4[200], receive_5[200];

void *readThread_1(void *arg);
void *readThread_2(void *arg);
void *readThread_4(void *arg);
void *readThread_5(void *arg);
void *writeThread(void *arg);
void reset_input_mode (struct UART *uart);
int uartOpen( struct UART *uart, char* name);
void uartClose( struct UART *uart);
int setupUART(struct UART *uart,speed_t spd);

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
  printf("Controls : a - for read threads start , w - write thread . y - print read stat . x - exit \n");
}

int main(int argc , char* argv[])
{
  uart1 = (struct UART*)malloc(sizeof( struct UART));
  uart2 = (struct UART*)malloc(sizeof( struct UART));
  uart4 = (struct UART*)malloc(sizeof( struct UART));
  uart5 = (struct UART*)malloc(sizeof( struct UART));
  printInitMSG();
  if( uartOpen(uart1, "/dev/ttyO1") < 0 )
    printf("uart1 open error %d , %s\n", uart1->file , strerror(errno));
  if( uartOpen(uart2, "/dev/ttyO2") < 0 )
    printf("uart2 open error %d , %s\n", uart2->file , strerror(errno));
  if( uartOpen(uart4, "/dev/ttyO4") < 0 )
    printf("uart4 open error %d , %s\n", uart4->file , strerror(errno));
  if( uartOpen(uart5, "/dev/ttyO5") < 0 )
    printf("uart5 open error %d , %s\n", uart5->file , strerror(errno));

   if(argc>=2)
    loopCount = atoi(argv[1]);
   else
    loopCount = DEFAULT_CNT;

  // if(argc>=3)
  //   buffLen = atoi(argv[2]);
  //  else
  //   buffLen = DEFAULT_BUF;

  setupUART( uart1,BAUD );
  setupUART( uart2,BAUD );
  setupUART( uart5,BAUD );  
  setupUART( uart4,BAUD );

  uart1->id = 1;
  uart2->id = 2;
  uart4->id = 4;
  uart5->id = 5;
  printf("UART setup done\n");

  // use of malloc to avoid dangling pointer incase of asynchronous thread kill
  // struct UART *arg1 = (struct UART*)malloc(sizeof( struct UART));
  // memcpy(  arg1, &uart2 ,sizeof( struct UART) );

  // struct UART *arg2 = (struct UART*)malloc(sizeof( struct UART));
  // memcpy(  arg2, &uart1 ,sizeof( struct UART) );
  if(pthread_create(&readThrdID_1, NULL, &readThread_1, uart1 ))
  {
    printf("Failed to create read thread 1 %s \n", strerror(errno));
    // close file
    reset_input_mode(uart1);
    uartClose(uart1);
    return -1;
  }
  if(pthread_create(&readThrdID_2, NULL, &readThread_2, uart2 ))
  {
    printf("Failed to create read thread 2 %s \n", strerror(errno));
    // close file
    reset_input_mode(uart2);
    uartClose(uart2);
    return -1;
  }
  if(pthread_create(&readThrdID_4, NULL, &readThread_4, uart4 ))
  {
    printf("Failed to create read thread 4 %s \n", strerror(errno));
    // close file
    reset_input_mode(uart4);
    uartClose(uart4);
    return -1;
  }
  if(pthread_create(&readThrdID_5, NULL, &readThread_5, uart5 ))
  {
    printf("Failed to create read thread 5 %s \n", strerror(errno));
    // close file
    reset_input_mode(uart5);
    uartClose(uart5);
    return -1;
  }
  if(pthread_create(&writeThrdID, NULL, &writeThread, uart2 ))
  {
    printf("Failed to create write thread %s \n", strerror(errno));
    // close file
    reset_input_mode(uart2);
    uartClose(uart2);
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
      printf("Total bytes rx2: %d\n", tc2);
      printf("Total bytes rx4: %d\n", tc4);
      printf("Total bytes rx5: %d\n", tc5);
      fflush(stdout);
    }
    if( (a == 'a') || (a == 'A') ) pauseth=1;
    if( (a == 'W') || (a == 'w') ) pauseth_wr=1;
  }

  pthread_cancel(readThrdID_1);
  pthread_cancel(readThrdID_2);
  pthread_cancel(readThrdID_4);
  pthread_cancel(readThrdID_5);
  pthread_cancel(writeThrdID);
  pthread_join(readThrdID_1,NULL);
  pthread_join(readThrdID_2,NULL);
  pthread_join(readThrdID_4,NULL);
  pthread_join(readThrdID_5,NULL);
  pthread_join(writeThrdID,NULL);
  printf("Total bytes rx1: %d\n", tc1);
  printf("Total bytes rx2: %d\n", tc2);
  printf("Total bytes rx4: %d\n", tc4);
  printf("Total bytes rx5: %d\n", tc5);
  reset_input_mode(uart2);
  reset_input_mode(uart1);
  reset_input_mode(uart4);
  reset_input_mode(uart5);
  uartClose(uart2);
  uartClose(uart1);
  uartClose(uart4);
  uartClose(uart5);

  free(uart1);
  free(uart2);
  free(uart4);
  free(uart5);

  return 0;
}

void *writeThread(void *arg)
{
  // pthread_mutex_t *mx = (pthread_mutex_t*)arg;
  struct UART *uart = (struct UART*) arg;
  int file = uart->file;
  // free after copying to local mem
  // free(arg); 
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  unsigned char send[buffLen+2] ;
  int count;
  int total_count = 0;

  unsigned char gap = 0xFF;
  unsigned char cost = 0x55;
  send[0] = gap;
  for(int k = 1;k<=buffLen;k+=1)
  {
    send[k] = cost;
    cost = cost  +1;
  }
  while( !pauseth_wr) gsleep(1000);
  printf("Write started pauseth : %d\n", pauseth_wr);
  for(int k=0;k<loopCount;k++)
 { 
  // unsigned char i=0x00;
  // do 
   // {
     count = write(file, send, buffLen+1);
     // count = dprintf(file,"%x ",i);
     // as vmin != 0 , count not 0
      // i++;
      // write(file, &gap, 1);
     total_count += count;
    // gsleep(15); // provide triple dalay for same channel
   // }while(i<255);  
     gsleep(100);
  }
  printf("sent %d\n", total_count );
  // free(arg);
  pthread_exit(NULL);
  return (void *)0; 
} 

void *readThread_1(void *arg)
{
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  struct UART *uart = (struct UART*) arg;
  int count;
  int file = uart->file;
  unsigned char *receive = uart->receiveBuff;
  // free(arg);
  while( !pauseth) gsleep(100);

  unsigned char last_ch = 0xAA;
  printf("Read started pauseth : %d\n", pauseth);
  do 
  {
    // ioctl(file, FIONREAD, &bytes);
    // printf("FIFO Size : %d bytes\n", bytes);
    memset(receive,'\0',500);
    count = read(file, receive, 500);

    // as vmin != 0 , count not 0
    if (count <= 0)
      // continue;
      printf("Failed to read: %d, %s \n", count, strerror(errno) );
    else
    {
      // printf("Last ch before : %x\n",last_ch );
      // for(int j=0;j<count;j++)
      // {
      // printf("%x ",  receive[j]);
      // }
      // printf("\n");
      tc1 += count;
      // fflush(stdout);
      for(int j=0;j<count;j++)
      {
        if( (receive[j] == 0xff ) )
          continue;
         if(receive[j] != last_ch)
           printf("Error , expected : %x, got : %x\n", last_ch , receive[j] );
         printf("Last ch : %x , rx:  %x\n",last_ch , receive[j] );
        
        last_ch += 1;
        if(last_ch >= buffLen + 0x55) last_ch = 0x55; 
      }
      // printf("Last ch after1  : %x , adding %x\n",last_ch , receive[count-1] );
      // last_ch = receive[count-1] +1;
      // printf("Last ch after : %x\n",last_ch );
      // fflush(stdout);
    }
    // if(tc1%100 == 0) printf(" uartid: %d , rx: %d\n", uart->id, tc1);
    // printf("count %d\n", count );
    // fflush(stdout);
  } while (1);
  // }while(!needQuit(mx));
  pthread_exit(NULL);
  return (void *)0; 
} 

void *readThread_2(void *arg)
{
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  struct UART *uart = (struct UART*) arg;
  int count;
  int file = uart->file;
  unsigned char *receive = uart->receiveBuff;
  // free(arg);
  while( !pauseth) gsleep(100);

  unsigned char last_ch = 0x55;
  printf("Read started pauseth : %d\n", pauseth);
  do 
  {
    // ioctl(file, FIONREAD, &bytes);
    // printf("FIFO Size : %d bytes\n", bytes);
    memset(receive,'\0',500);
    count = read(file, receive, 500);

    // as vmin != 0 , count not 0
    if (count <= 0)
      printf("Failed to read: %d, %s \n", count, strerror(errno) );
    else
    {
      // printf("Last ch before : %x\n",last_ch );
      // for(int j=0;j<count;j++)
      // {
      // printf("%x ",  receive[j]);
      // }
      // printf("\n");
      tc2 += count;
      // fflush(stdout);
      for(int j=0;j<count;j++)
      {
        if( (receive[j] == 0xff ) )
          continue;
        if(receive[j] != last_ch)
          printf("Error , expected : %x, got : %x\n", last_ch , receive[j] );
        // printf("Last ch : %x , rx:  %x\n",last_ch , receive[j] );
        
        last_ch += 1;
        if(last_ch >= buffLen + 0x55) last_ch = 0x55;  
      }
      // printf("Last ch after1  : %x , adding %x\n",last_ch , receive[count-1] );
      // last_ch = receive[count-1] +1;
      // printf("Last ch after : %x\n",last_ch );
      // fflush(stdout);
    }
    // if(tc2%100 == 0) printf(" uartid: %d , rx: %d\n", uart->id, tc2);
    // fflush(stdout);
  } while (1);
  // }while(!needQuit(mx));
  pthread_exit(NULL);
  return (void *)0; 
} 

void *readThread_4(void *arg)
{
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  struct UART *uart = (struct UART*) arg;
  int count;
  int file = uart->file;
  unsigned char *receive = uart->receiveBuff;
  // free(arg);
  while( !pauseth) gsleep(100);

  unsigned char last_ch = 0x55;
  printf("Read started pauseth : %d\n", pauseth);
  do 
  {
    // ioctl(file, FIONREAD, &bytes);
    // printf("FIFO Size : %d bytes\n", bytes);
    memset(receive,'\0',500);
    count = read(file, receive, 500);

    // as vmin != 0 , count not 0
    if (count <= 0)
      printf("Failed to read: %d, %s \n", count, strerror(errno) );
    else
    {
      // printf("Last ch before : %x\n",last_ch );
      // for(int j=0;j<count;j++)
      // {
      // printf("%x ",  receive[j]);
      // }
      // printf("\n");
      tc4 += count;
      // fflush(stdout);
      for(int j=0;j<count;j++)
      {
        if( (receive[j] == 0xff ) )
          continue;
        if(receive[j] != last_ch)
          printf("Error , expected : %x, got : %x\n", last_ch , receive[j] );
        // printf("Last ch : %x , rx:  %x\n",last_ch , receive[j] );
        
        last_ch += 1;
        if(last_ch >= buffLen + 0x55) last_ch = 0x55;  
      }
      // printf("Last ch after1  : %x , adding %x\n",last_ch , receive[count-1] );
      // last_ch = receive[count-1] +1;
      // printf("Last ch after : %x\n",last_ch );
      // fflush(stdout);
    }
    // if(tc4%100 == 0) printf(" uartid: %d , rx: %d\n", uart->id, tc4);
    // fflush(stdout);
  } while (1);
  // }while(!needQuit(mx));
  pthread_exit(NULL);
  return (void *)0; 
} 

void *readThread_5(void *arg)
{
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  struct UART *uart = (struct UART*) arg;
  int count;
  int file = uart->file;
  unsigned char *receive = uart->receiveBuff;
  // free(arg);
  while( !pauseth) gsleep(100);

  unsigned char last_ch = 0x55;
  printf("Read started pauseth : %d\n", pauseth);
  do 
  {
    // ioctl(file, FIONREAD, &bytes);
    // printf("FIFO Size : %d bytes\n", bytes);
    memset(receive,'\0',500);
    count = read(file, receive, 500);

    // as vmin != 0 , count not 0
    if (count <= 0)
      printf("Failed to read: %d, %s \n", count, strerror(errno) );
    else
    {
      // printf("Last ch before : %x\n",last_ch );
      // for(int j=0;j<count;j++)
      // {
      // printf("%x ",  receive[j]);
      // }
      // printf("\n");
      tc5 += count;
      // fflush(stdout);
      for(int j=0;j<count;j++)
      {
        if( (receive[j] == 0xff ) )
          continue;
        if(receive[j] != last_ch)
          printf("Error , expected : %x, got : %x\n", last_ch , receive[j] );
        // printf("Last ch : %x , rx:  %x\n",last_ch , receive[j] );
        
        last_ch += 1;
        if(last_ch >= buffLen + 0x55) last_ch = 0x55; 
      }
      // printf("Last ch after1  : %x , adding %x\n",last_ch , receive[count-1] );
      // last_ch = receive[count-1] +1;
      // printf("Last ch after : %x\n",last_ch );
      // fflush(stdout);
    }
    // if(tc5%100 == 0) printf(" uartid: %d , rx: %d\n", uart->id, tc5);
    // fflush(stdout);
  } while (1);
  // }while(!needQuit(mx));
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

int setupUART(struct UART *uart,speed_t spd)
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
	  options.c_cc[VMIN]  = 1;
	  options.c_cc[VTIME] = 5;
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

