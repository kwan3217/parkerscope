/**
 * Record a bunch of data from a serial port. This data is expected to 
 * come in the form of packets. Record the data into a series of files 
 * with the date and time in their filename. Record to a new log file 
 * periodically.
 *
 * Record whole packets into a log before breaking the log.
 */

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <cmath>
#include <time.h>
#include <sys/time.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>

FILE* ouf;
char oufn[256];
char* prefix;
int old_min;

void open_log() {
  const char oufnpat[]="%s/%s_%02d%02d%02d_%02d%02d%02d.csv";
  time_t rawtime;
  struct tm * ptm;

  time ( &rawtime );

  ptm = gmtime ( &rawtime );

  snprintf(oufn,sizeof(oufn)-1,oufnpat,prefix,prefix,ptm->tm_year%100,ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,ptm->tm_min,ptm->tm_sec);
  old_min=ptm->tm_min/10;
  ouf=fopen(oufn,"wb");
  printf("Opening %s\n",oufn);
}

void rotate() {
  time_t rawtime;
  struct tm * ptm;

  time ( &rawtime );

  ptm = gmtime ( &rawtime  );
  
  if(ptm->tm_min/10!=old_min) {
    char cmd[256+15];
    fclose(ouf);
    snprintf(cmd, sizeof(cmd) - 1, "bzip2 -9v \"%s\" &", oufn);
    system(cmd);
    open_log();
  }
}

int open_serial(char* device_name) {
  int serial_port=open(device_name,O_RDWR);
  if(serial_port<0) {
    printf("Couldn't open serial port: %d %s",errno,strerror(errno));
  }
  // Get current serial port settings. We will change 
  // this structure and write it back
  struct termios tty;
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
  } 
  // Set serial port to 8N1
  tty.c_cflag&=~PARENB; //disable parity by clearing PARENB bit
  tty.c_cflag&=~CSTOPB; //set 1 stop bit by clearing CSTOPB bit
  tty.c_cflag&=~CSIZE; //clear all byte size flags
  tty.c_cflag|=CS8;    //select 8-bit bytes
  
  tty.c_cflag&=~CRTSCTS;//Disable hardware (RTS/CTS) flow control
  tty.c_cflag|=(CREAD|CLOCAL); //turn on read and local control (disable modem control lines and SIGHUP)
  
  tty.c_cflag&=~ICANON; //Disable canonical mode. In canonical mode, we process input on newline and treat control characters special.
  tty.c_cflag&=~ECHO;   //disable echo
  tty.c_cflag&=~ECHOE;  //disable echo erasure
  tty.c_cflag&=~ECHONL; //disable echo new lines
  
  tty.c_cflag&=~ISIG; //disable interpretation of Ctrl-C etc from serial port
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  
  tty.c_cc[VTIME] = 20;    // Wait for up to given deciseconds...
  tty.c_cc[VMIN] = 0;      //  returning as soon as any number of bytes are received.
  
  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  return serial_port;
}

int main(int argc, char** argv) {
  if(argc<3) {
    fprintf(stderr,"Usage: %s /dev/serialport prefix\n",argv[0]);
    exit(1);
  }
  int serial_port=open_serial(argv[1]);  
  prefix=argv[2];
  open_log();
  bool started=false;
  bool timestamp_next_char=false;
  for(;;) {
    char read_buf[256];
    int n=read(serial_port,read_buf,sizeof(read_buf)-1);
    if(n>=0) {
      for(auto i=0;i<n;i++) {
        char c=read_buf[i];
	if(c==0x0a) {
          fputc(c,stdout);
          fputc(c,ouf);
          rotate();
          timestamp_next_char=true;
        } else if(timestamp_next_char) {
	  struct timeval tv;
	  gettimeofday(&tv,NULL);
	  struct tm ptm;
	  gmtime_r ( &tv.tv_sec,&ptm );
	  fprintf(ouf, "%04d-%02d-%02dT%02d:%02d:%02d.%06ldZ ",ptm.tm_year+1900,ptm.tm_mon+1,ptm.tm_mday,
	                                                ptm.tm_hour,ptm.tm_min,ptm.tm_sec,tv.tv_usec);
	   printf(     "%04d-%02d-%02dT%02d:%02d:%02d.%06ldZ ",ptm.tm_year+1900,ptm.tm_mon+1,ptm.tm_mday,
	                                                ptm.tm_hour,ptm.tm_min,ptm.tm_sec,tv.tv_usec);
	  timestamp_next_char=false;
          fputc(c,stdout);
          fputc(c,ouf);
	} else {
          fputc(c,stdout);
          fputc(c,ouf);
        }
        fflush(ouf);
        fflush(stdout);       
      } 
    }
  }
}

