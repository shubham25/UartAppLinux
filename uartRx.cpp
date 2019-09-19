#include<stdio.h>
#include<fcntl.h>
#include <stdlib.h>
#include <iostream>
#include<unistd.h>
#include <string.h>
#include <errno.h>
#include<termios.h>   // using the termios.h library
#include <pthread.h>
#include <sys/ioctl.h>

struct termios options, saved_options;
int file;
pthread_t readThrdID;
pthread_mutex_t rdMtx;
int pauseth = 0;
int tc = 0;
// #define SAME
    unsigned char receive[200];
// B4000000    B50            B75         B110        B134
// B150        B200           B300        B600        B1200
// B1800       B2400          B4800       B9600       B19200
// B38400      B57600         B115200     B230400     B460800    
// B500000     B576000        B921600     B1000000    B1152000
// B1500000    B2000000       B2500000    B3000000    B3500000   
#define BAUD B2000000

void *readThread(void *arg);
void reset_input_mode (void);
int setupUART(speed_t spd);

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

int main(){
   // FILE* file=NULL;
	 int rc;
   // if ( (file = fopen("/dev/ttyO2", "r") )==NULL ){
   // file = open("/dev/ttyO1", O_RDWR | O_NOCTTY );
   file = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY);
   if (file <0){
      printf("file open error %d , %s\n", file , strerror(errno));
      return -1;
   }
   else
    fcntl(file, F_SETFL, 0);

  printf("File opened\n");
  setupUART( BAUD );
  printf("UART setup done\n");  
  // read thread
  // pthread_mutex_init(&rdMtx,NULL);
  // pthread_mutex_lock(&rdMtx);

  // if(pthread_create(&readThrdID, NULL, &readThread, &rdMtx))
  if(pthread_create(&readThrdID, NULL, &readThread, NULL))
  {
    printf("Failed to create read thread %s \n", strerror(errno));
    // close file
    reset_input_mode();
    close(file);
    return -1;
  }
printf("Thread active\n");

  while(1)
  {
    char a;
    std::cin >>a;
    if( (a == 'x') || (a == 'X') ) break;
    if( (a == 'a') || (a == 'A') ) pauseth=1;
  }

  // thread kill and join
  // pthread_mutex_unlock(&rdMtx);
  pthread_cancel(readThrdID);
  pthread_join(readThrdID,NULL);
  printf("Total bytes rx: %d\n", tc);
  reset_input_mode();
  close(file);
  return 0;
}

void *readThread(void *arg)
{
  // pthread_mutex_t *mx = (pthread_mutex_t*)arg;
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  int count;
  // int bytes;

  while( !pauseth){};
  unsigned char last_ch = 0xAA;
  printf("Read started pauseth : %d\n", pauseth);
  do 
  {
    // ioctl(file, FIONREAD, &bytes);
    // printf("FIFO Size : %d bytes\n", bytes);
    memset(receive,'\0',200);
    count = read(file, receive, 200);

    // as vmin != 0 , count not 0
    if (count <= 0)
      printf("Failed to read: %d, %s \n", count, strerror(errno) );
    else
    {
      // for(int j=0;j<count;j++)
      // {
      // printf("%x ",  receive[j]);
      // }
      // printf("\n");
      tc += count;
      // fflush(stdout);
      for(int j=0;j<count;j++)
      {
        if( (receive[j] == 0xff ) )
          continue;
        if(receive[j] != last_ch)
          printf("Error , expected : %x, got : %x\n", last_ch , receive[j] );
        // printf("Last ch : %x , rx:  %x\n",last_ch , receive[j] );
        
        // last_ch += 1;
        if(last_ch == 0xff) last_ch = 0; 
      }
      // printf("Last ch after1  : %x , adding %x\n",last_ch , receive[count-1] );
      // last_ch = receive[count-1] +1;
      // printf("Last ch after : %x\n",last_ch );
      // fflush(stdout);
    }

    if(tc%100 == 0) printf("rx: %d\n", tc);
    fflush(stdout);
  } while (1);
  // }while(!needQuit(mx));
  pthread_exit(NULL);
  return (void *)0; 
} 


void reset_input_mode (void)
{
  int rc;
  // flush rd
  #ifdef SAME
    rc = tcflush(file , TCOFLUSH);
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

  rc = tcsetattr(file, TCSANOW , &saved_options); 
  if( rc < 0)
  {
    printf("attrb SET error %d , %s\n",rc , strerror(errno) );
  }
  return;
}

int setupUART(speed_t spd)
{
  int rc;
  rc = tcgetattr(file, &options); 
  if( rc < 0)
  {
    printf("attrb get error %d , %s\n",rc , strerror(errno) );
    return -2;
  }

  saved_options = options;
  atexit (reset_input_mode);

  // change terminal attributes
  #ifndef SAME
    cfsetispeed(&options, (speed_t)spd);
  #endif

  cfsetospeed(&options, (speed_t)spd);

  #ifndef SAME
    cfmakeraw(&options);    // use raw bsd mode
  #endif

  // 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  
  //Enable the receiver and set local mode.
  options.c_cflag &= ~(CLOCAL | CREAD);
  options.c_cflag |= (CLOCAL | CREAD);

  // options.c_lflag |= (ICANON | ECHO | ECHOE);  // canonical form
  // options.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */ // non canonical form
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOK | ECHOCTL | ECHOKE);  // raw mode ECHOK | ECHOCTL | ECHOKE);

  // options.c_cflag |= CRTSCTS;  // enable hw control
  options.c_cflag &= ~CRTSCTS; // disable hardware control

  options.c_iflag &= ~(IGNPAR | ICRNL | ISTRIP | IGNBRK | IMAXBEL);   // IGNBRK | IMAXBEL);
  // options.c_iflag &= ~(IXON | IXOFF);  
  options.c_iflag |= (IGNPAR | IGNBRK) ;    //ignore partity errors, CR -> newline IGNBRK) ; 

  options.c_oflag &= ~(ONLCR);   // minicom

  // options.c_iflag |= (IXON | IXOFF);
  // options.c_iflag |= (IXON );
  // options.c_iflag |= (IXOFF);


  // timeouts
  options.c_cc[VMIN]  = 1;
  options.c_cc[VTIME] = 5;

  // rc = tcflush(file , TCIOFLUSH);
  // if( rc < 0)
  // {
  //   printf("attrb SET error %d , %s\n",rc , strerror(errno) );
  //   return -2;
  // }
  // flush rd
  #ifdef SAME
    rc = tcflush(file , TCOFLUSH);
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

