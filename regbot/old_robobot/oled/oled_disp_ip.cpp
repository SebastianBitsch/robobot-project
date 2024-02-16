/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x32|64 size display using SPI or I2C to communicate
4 or 5 pins are required to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

02/18/2013 	Charles-Henri Hallard (http://hallard.me)
						Modified for compiling and use on Raspberry ArduiPi Board
						LCD size and connection are now passed as arguments on 
						the command line (no more #define on compilation needed)
						ArduiPi project documentation http://hallard.me/arduipi

						
*********************************************************************/
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>
#include <readline/readline.h>

#include "ArduiPi_OLED_lib.h"
#include "Adafruit_GFX.h"
#include "ArduiPi_OLED.h"

#include <getopt.h>

#define PRG_NAME        "oled_demo"
#define PRG_VERSION     "1.1"

// Instantiate the display
ArduiPi_OLED * display = NULL;


// Config Option
struct s_opts
{
	int oled;
	int verbose;
} ;

int sleep_divisor = 1 ;
	
// default options values
s_opts opts = {
	OLED_ADAFRUIT_SPI_128x32,	// Default oled
  false										// Not verbose
};

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static unsigned char logo16_glcd_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  uint8_t icons[NUMFLAKES][3];
  srandom(666);     // whatever seed
 
  // initialize
  for (uint8_t f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS] = random() % display->width();
    icons[f][YPOS] = 0;
    icons[f][DELTAY] = random() % 5 + 1;
    
    printf("x: %d", icons[f][XPOS]);
    printf("y: %d", icons[f][YPOS]);
    printf("dy: %d\n", icons[f][DELTAY]);
  }

  while (1) {
    // draw each icon
    for (uint8_t f=0; f< NUMFLAKES; f++) {
      display->drawBitmap(icons[f][XPOS], icons[f][YPOS], logo16_glcd_bmp, w, h, WHITE);
    }
    display->display();
    usleep(100000/sleep_divisor);
    
    // then erase it + move it
    for (uint8_t f=0; f< NUMFLAKES; f++) {
      display->drawBitmap(icons[f][XPOS], icons[f][YPOS],  logo16_glcd_bmp, w, h, BLACK);
      // move it
      icons[f][YPOS] += icons[f][DELTAY];
      // if its gone, reinit
      if (icons[f][YPOS] > display->height()) {
	icons[f][XPOS] = random() % display->width();
	icons[f][YPOS] = 0;
	icons[f][DELTAY] = random() % 5 + 1;
      }
    }
   }
}


void testdrawchar(void) {
  display->setTextSize(1);
  display->setTextColor(WHITE);
  display->setCursor(0,0);

  for (uint8_t i=0; i < 168; i++) {
    if (i == '\n') continue;
    display->write(i);
    if ((i > 0) && (i % 21 == 0))
      display->print("\n");
  }    
  display->display();
}

void testdrawcircle(void) {
  for (int16_t i=0; i<display->height(); i+=2) {
    display->drawCircle(display->width()/2, display->height()/2, i, WHITE);
    display->display();
  }
}

void testfillrect(void) {
  uint8_t color = 1;
  for (int16_t i=0; i<display->height()/2; i+=3) {
    // alternate colors
    display->fillRect(i, i, display->width()-i*2, display->height()-i*2, color%2);
    display->display();
    color++;
  }
}

void testdrawtriangle(void) {
  for (int16_t i=0; i<min(display->width(),display->height())/2; i+=5) {
    display->drawTriangle(display->width()/2, display->height()/2-i,
                     display->width()/2-i, display->height()/2+i,
                     display->width()/2+i, display->height()/2+i, WHITE);
    display->display();
  }
}

void testfilltriangle(void) {
  uint8_t color = WHITE;
  for (int16_t i=min(display->width(),display->height())/2; i>0; i-=5) {
    display->fillTriangle(display->width()/2, display->height()/2-i,
                     display->width()/2-i, display->height()/2+i,
                     display->width()/2+i, display->height()/2+i, WHITE);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display->display();
  }
}

void testdrawroundrect(void) {
  for (int16_t i=0; i<display->height()/2-2; i+=2) {
    display->drawRoundRect(i, i, display->width()-2*i, display->height()-2*i, display->height()/4, WHITE);
    display->display();
  }
}

void testfillroundrect(void) {
  uint8_t color = WHITE;
  for (int16_t i=0; i<display->height()/2-2; i+=2) {
    display->fillRoundRect(i, i, display->width()-2*i, display->height()-2*i, display->height()/4, color);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display->display();
  }
}
   
void testdrawrect(void) {
  for (int16_t i=0; i<display->height()/2; i+=2) {
    display->drawRect(i, i, display->width()-2*i, display->height()-2*i, WHITE);
    display->display();
  }
}

void testdrawline() {  
  for (int16_t i=0; i<display->width(); i+=4) {
    fprintf(stdout, "# testdrawline::1 %d\n", i);
    display->drawLine(0, 0, i, display->height()-1, WHITE);
    display->display();
  }
  for (int16_t i=0; i<display->height(); i+=4) {
    fprintf(stdout, "# testdrawline::2 %d\n", i);
    display->drawLine(0, 0, display->width()-1, i, WHITE);
    display->display();
  }
  usleep(250000/sleep_divisor);
  
  display->clearDisplay();
  for (int16_t i=0; i<display->width(); i+=4) {
    display->drawLine(0, display->height()-1, i, 0, WHITE);
    display->display();
  }
  for (int16_t i=display->height()-1; i>=0; i-=4) {
    display->drawLine(0, display->height()-1, display->width()-1, i, WHITE);
    display->display();
  }
  usleep(250000/sleep_divisor);
  
  display->clearDisplay();
  for (int16_t i=display->width()-1; i>=0; i-=4) {
    display->drawLine(display->width()-1, display->height()-1, i, 0, WHITE);
    display->display();
  }
  for (int16_t i=display->height()-1; i>=0; i-=4) {
    display->drawLine(display->width()-1, display->height()-1, 0, i, WHITE);
    display->display();
  }
  usleep(250000/sleep_divisor);

  display->clearDisplay();
  for (int16_t i=0; i<display->height(); i+=4) {
    display->drawLine(display->width()-1, 0, 0, i, WHITE);
    display->display();
  }
  for (int16_t i=0; i<display->width(); i+=4) {
    display->drawLine(display->width()-1, 0, i, display->height()-1, WHITE); 
    display->display();
  }
  usleep(250000/sleep_divisor);
}

void testscrolltext(void) {
  display->setTextSize(2);
  display->setTextColor(WHITE);
  display->setCursor(10,0);
  display->clearDisplay();
  
  if (opts.oled == OLED_SH1106_I2C_128x64)
    display->print("No scroll\non SH1106");
  else
    display->print("scroll");
  display->display();
 
  display->startscrollright(0x00, 0x0F);
  sleep(2);
  display->stopscroll();
  sleep(1);
  display->startscrollleft(0x00, 0x0F);
  sleep(2);
  display->stopscroll();
  sleep(1);    
  display->startscrolldiagright(0x00, 0x07);
  sleep(2);
  display->startscrolldiagleft(0x00, 0x07);
  sleep(2);
  display->stopscroll();
}


/* ======================================================================
Function: usage
Purpose : display usage
Input 	: program name
Output	: -
Comments: 
====================================================================== */
void usage( char * name)
{
	printf("%s\n", name );
	printf("Usage is: %s --oled type [options]\n", name);
	printf("  --<o>led type\nOLED type are:\n");
	for (int i=0; i<OLED_LAST_OLED;i++)
		printf("  %1d %s\n", i, oled_type_str[i]);
	
	printf("Options are:\n");
	printf("  --<v>erbose  : speak more to user\n");
	printf("  --<h>elp\n");
	printf("<?> indicates the equivalent short option.\n");
	printf("Short options are prefixed by \"-\" instead of by \"--\".\n");
	printf("Example :\n");
	printf( "%s -o 1 use a %s OLED\n\n", name, oled_type_str[1]);
	printf( "%s -o 4 -v use a %s OLED being verbose\n", name, oled_type_str[4]);
}


/* ======================================================================
Function: parse_args
Purpose : parse argument passed to the program
Input 	: -
Output	: -
Comments: 
====================================================================== */
void parse_args(int argc, char *argv[])
{
	static struct option longOptions[] =
	{
		{"oled"	  , required_argument,0, 'o'},
		{"verbose", no_argument,	  	0, 'v'},
		{"help"		, no_argument, 			0, 'h'},
		{0, 0, 0, 0}
	};

	int optionIndex = 0;
	int c;

	while (1) 
	{
		/* no default error messages printed. */
		opterr = 0;

    c = getopt_long(argc, argv, "vho:", longOptions, &optionIndex);

		if (c < 0)
			break;

		switch (c) 
		{
			case 'v': opts.verbose = true	;	break;

			case 'o':
				opts.oled = (int) atoi(optarg);
				
				if (opts.oled < 0 || opts.oled >= OLED_LAST_OLED )
				{
						fprintf(stderr, "--oled %d ignored must be 0 to %d.\n", opts.oled, OLED_LAST_OLED-1);
						fprintf(stderr, "--oled set to 0 now\n");
						opts.oled = 0;
				}
			break;

			case 'h':
				usage(argv[0]);
				exit(EXIT_SUCCESS);
			break;
			
			case '?':
			default:
				fprintf(stderr, "Unrecognized option.\n");
				fprintf(stderr, "Run with '--help'.\n");
				exit(EXIT_FAILURE);
		}
	} /* while */

	if (opts.verbose)
	{
		printf("%s v%s\n", PRG_NAME, PRG_VERSION);
		printf("-- OLED params -- \n");
		printf("Oled is    : %s\n", oled_type_str[opts.oled]);
		printf("-- Other Stuff -- \n");
		printf("verbose is : %s\n", opts.verbose? "yes" : "no");
		printf("\n");
	}	
}


/* ======================================================================
Function: main
Purpose : Main entry Point
Input 	: -
Output	: -
Comments: 
====================================================================== */
int main(int argc, char **argv)
{
  int i;
  display = new ArduiPi_OLED();
  const int MIPS = 10;
  char * ips[MIPS] = {NULL, NULL, NULL, NULL, NULL}; // list of IP strings
  int ipsCnt, loopCnt = 0;
  const int MIPSLEN = 100;
  opts.oled = 1; // default option
  // Oled supported display in ArduiPi_SSD1306.h
  // Get OLED type
  parse_args(argc, argv);
  //
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  bool done = false;
  // SPI change parameters to fit to your LCD
  if ( !display->init(OLED_SPI_DC,OLED_SPI_RESET,OLED_SPI_CS, opts.oled) )
  {
    printf("# SPI access failed - aborting\n");
    exit(EXIT_FAILURE);
  }
  // display setup using SPI connection
  display->begin();
  // init done - clear local buffer
  display->clearDisplay();   // clears the screen  buffer
  display->display();        // display it (clear display)
  //
  while (true)
  { // continue until 2 IPs are found - or timeout
    getifaddrs(&ifAddrStruct);
    ipsCnt = 0;
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) 
    {
      if (ifa->ifa_addr->sa_family == AF_INET) 
      { // is a valid IP4 Address
        if (ips[ipsCnt] == NULL)
          ips[ipsCnt] = (char*)malloc(MIPSLEN);
        tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
        //
        printf("found IP %d: %s\n", ipsCnt, ifa->ifa_name);
        //
        char addressBuffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
        if (strcmp(ifa->ifa_name, "lo") != 0)
        { // not loop back - loopback is skipped
          snprintf(ips[ipsCnt], MIPSLEN, "%5s:%s\n", ifa->ifa_name, addressBuffer); 
          if (ipsCnt < MIPS - 1)
            ipsCnt++;
        }
      } 
      else if (ifa->ifa_addr->sa_family == AF_INET6) 
      { // check it is IP6
        // is a valid IP6 Address
//         printf("ip V6 - not used\n");
        // not used
      } 
    }
//     if (ifAddrStruct!=NULL) 
//       freeifaddrs(ifAddrStruct);
    done = ipsCnt >= 2 or loopCnt >= 20;
    if (not done)
    {
      if (ips[ipsCnt] == NULL)
        ips[ipsCnt] = (char*)malloc(MIPSLEN);
      snprintf(ips[ipsCnt], MIPSLEN, "\nWaiting for wifi (%d)\n", loopCnt);
      printf("\nWaiting for wifi (%d), ipsCnt=%d\n", loopCnt, ipsCnt);
      ipsCnt++;
    }
    // init done
    display->clearDisplay();   // clears the screen  buffer
    display->display();        // display it (clear display)
    
    display->setTextSize(1);
    display->setTextColor(WHITE);
    display->setCursor(0,0);
    display->printf("RoboBot\n");
    //
    fprintf(stdout, "Sending to display:\n");
    fprintf(stdout, "RoboBot\n");
    for (int i = 0; i < ipsCnt; i++)
    {
      display->printf(ips[i]);
      fprintf(stdout,ips[i]);
    }
    display->display();
    printf("Loop %d ended with %d addresses\n", loopCnt, ipsCnt);
    loopCnt++;
    if (done)
    { // do not wait for more IP adresses
      printf("Waiting no longer - finished or timeout\n");
      break;
    }
    sleep(2);
  }
  printf("# display disconnected\n");
  for (i = 0; i < MIPS; i++)
  { // release ip-names
    free(ips[i]);
  }
  // Free PI GPIO ports
  display->close();
  free(display);
}


