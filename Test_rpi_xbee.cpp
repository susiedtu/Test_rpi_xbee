/* 
 * File:   main.cpp
 * Author: Wen (reference.osch)
 *Read the Xbee end AT without checkSum function
 * Created on 10. NOV 2015, 11:15
 */
#include <iostream>
//#include <wiringSerial.h>   // wiring library https://projects.drogon.net/raspberry-pi/wiringpi/serial-library/
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <bcm2835.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
using namespace std;

#define idle 1
#define node1on 2
#define node1off 3

volatile char state= 0;


int Xbeetransmit(int ref, unsigned char *packet,  int length);  //Xbee transmit function call declaration
unsigned char checkSum (unsigned char* packet_tx, int length);  //checkSum for the API frame 

unsigned char packet_tx_on[] ={0x7E, 0x00, 0x0F, 0x10, 0x01, 0x00, 0x13, 0xA2, 0x00, 0x40, 0xAD, 0xD0, 0xE9, 0xFF, 0xFE, 0x00, 0x00, 0x7E, 0x18};   //uart transmit request21 bytes
void setup(int uart0_filestream ){                              //setup for serial transmission
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);  
}

int Xbeetransmit(int ref, unsigned char *packet, int packetLength) 
{
    std::cout << "transmit" << std::endl; 
    // Write to the port
    int n = write(ref, packet, packetLength);
    std::cout << n<< std::endl;
    if (n < 0) {
    //perror("Write failed - ");
        std::cout << "error" << std::endl; 
    return -1;
    }
  return n;
}

int main(int argc, char** argv) {                               //main loop


     int i=0;                                                   //index
 //int analogHigh=0;
 int fd;
 int n=0;
  // Open the Port. We want read/write, no "controlling tty" status, and open it no matter what state DCD is in
  fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);  //serialOpen ("/dev/ttyAMA0", 9600) ;used wiringpi
  if (fd == -1) {
      std::cout << "no uart" << std::endl; 
    //perror("open_port: Unable to open /dev/ttyAMA0 - ");
    return(-1);
  } else {
   std::cout << " uart" << std::endl;
   setup(fd);}
   fcntl(fd, F_SETFL, 0);
  // Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
    char buf[256];
  
    delay(5000);
    std::cout << "start" << std::endl; 
   
   
	 buf[0]='\0';
         printf("idle"); 
         if((Xbeetransmit(fd, packet_tx_on, packet_tx_on[2]+4))>-1) {//add 4 because of the 7E, 0xXXXX-length, 0x10 - type and check sum 0xXX 
             
             delay(1000);
             n = read(fd, (void*)buf, sizeof(buf));                 //read the receive data, whole function return to an integer
             delay(1000); 
             
         } 
       
	 
	 std::cout << "receive" << std::endl;     //18 bytes returned in remote response
                                                       
         if (n < 0) {
            perror("Read failed - ");
            return -1;
         } 
         else if (n == 0){
             printf("No data on port\n");
         }
         else {
            buf[n] = '\0';
            for (int i=0; i<=(n-1); i++){
                if((buf[8] == 0) && (i > 10)){                                //check the response status 00 OK 
                  printf("%i bytes read : %X\n", i, buf[i]);
                  delay(10);
                    
                }
            }
            printf("checksum : %X\n", buf[n-1]);                                // n-1 the last element is the checksum
            buf[0]='\0';
            }          
            i=0; 
            
            delay(5000);
            // Wait until we have a mouthful of data
             close(fd);
    std::cout << "slut" << std::endl; 
    return 0;
           
    }

   


