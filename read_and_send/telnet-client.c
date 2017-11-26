 /*
 * Sean Middleditch
 * sean@sourcemud.org
 *
 * The author or authors of this code dedicate any and all copyright interest
 * in this code to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and successors. We
 * intend this dedication to be an overt act of relinquishment in perpetuity of
 * all present and future rights to this code under copyright law.
 */

#if !defined(_POSIX_SOURCE)
#	define _POSIX_SOURCE
#endif
#if !defined(_BSD_SOURCE)
#	define _BSD_SOURCE
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <unistd.h>

#include <pthread.h>
#include <wiringPi.h>

#ifdef HAVE_ZLIB
#include "zlib.h"
#endif

#include "libtelnet.h"

#define  RoAPin    0
#define  RoBPin    1
#define  RoSPin    2

#define Button1 25
#define Button2 29

static volatile int globalCounter = 1;

unsigned char globalCounterPrev;
unsigned char flag;
unsigned char Last_RoB_Status;
unsigned char Button_value1;
unsigned char Button_prev_value1;
unsigned char Button_value2;
unsigned char Button_prev_value2;
unsigned char Button_prev_value1_send;
unsigned char Button_prev_value2_send;
unsigned char Current_RoB_Status;

#define LIMIT_ID_LIN2 59
#define LIMIT_ID_LIN1 15


void *readButton1();
void *readButton2();
void *readCoder();
void rotaryClear(void);
void rotaryDeal(void);
void SetupDevice();
void *SendLin();
void *SendCan();


void Network_read(int sock, unsigned char *dataBuf, unsigned int *dataSize)
{
    unsigned int tmpDataSize;
    int actDataSize = 0;

    tmpDataSize = *dataSize;

    while(tmpDataSize > 0U)
    {
        actDataSize = recv(sock, (void *)dataBuf, tmpDataSize, 0);
        
        dataBuf += actDataSize;
        tmpDataSize -= (unsigned int)actDataSize;
    }
}


enum LIN_MODE 
{
	LIN_1X = 1,
	LIN_2X = 2
};
static struct termios orig_tios;
static telnet_t *telnet;
static int do_echo;

enum lin_alias{LOM1 = 0, LOM2 = 1, LOS1, LOS2, LSR, LRQ, LOF, LSRF, LRQF, LC, LON, LOFF, RON, ROFF};

enum can_alias{CUO = 0, CUC = 1, CUS, CUA, CUT, CUF, CH1, CH2, S1, S2, S3, S4, S5, S6, S7, S8, S9, CL};

static char* lin_command[] = { 
	"LIN OPEN MASTER1X ",
	"LIN OPEN MASTER2X ",
	"LIN OPEN SLAVE1X\r",
	"LIN OPEN SLAVE2X\r",
	"LIN SR ",
	"LIN RQ ",
	"LIN OPEN FREE 20000\r\n",
	"LIN TX ",
	"LIN RP ",
	"LIN CLOSE\r",
	"LIN RP 21 00\r\n",
	"LIN RP 22 00\r\n",
	"LIN RP 23 00\r\n",
	"LIN RP 24 00\r\n"
};

static char* can_command[] = {
	"CAN USER OPEN CH2 500K\r\n",
	"CAN USER CLOSE ",
	"CAN USER STATUS ",
	"CAN USER ALIGN",
	"CAN USER TX ",
	"CAN USER FILTER CH2 0 FFFF\r\n",
	"CH1",
	"CH2",
	"CAN USER TX CH2 1111 01\r\n",
	"CAN USER TX CH2 1111 02\r\n",
	"CAN USER TX CH2 1111 03\r\n",
	"CAN USER TX CH2 1111 04\r\n",
	"CAN USER TX CH2 1111 05\r\n",
	"CAN USER TX CH2 1111 06\r\n",
	"CAN USER TX CH2 1111 07\r\n",
	"CAN USER TX CH2 1111 08\r\n",
	"CAN USER TX CH2 1111 09\r\n",
	"CAN USER CLOSE CH2\r\n"
};

static const telnet_telopt_t telopts[] = {
	{ TELNET_TELOPT_ECHO,		TELNET_WONT, TELNET_DO   },
	{ TELNET_TELOPT_TTYPE,		TELNET_WILL, TELNET_DONT },
	{ TELNET_TELOPT_COMPRESS2,	TELNET_WONT, TELNET_DO   },
	{ TELNET_TELOPT_MSSP,		TELNET_WONT, TELNET_DO   },
	{ -1, 0, 0 }
};

#if 0
unsigned char calcParityBit(unsigned char id, enum LIN_MODE mode)
{

	if ((id > LIMIT_ID_LIN2 && LIN_2X == mode) || (id > LIMIT_ID_LIN1 && LIN_1X == mode)){
		printf("Selected id is out of range!\n");
		return -1;
		}

	// PID0

	unsigned char ID0 = id << 6;
	ID0 &= 64;
	unsigned char ID1 = id << 5;
	ID1 &= 64;
	unsigned char ID2 = id << 4;
	ID2 &= 64;
	unsigned char ID4 = id << 2;
	ID4 &= 64;
	unsigned char P0 = ID0 ^ ID1 ^ ID2 ^ ID4;

	//printf("ID0: %d, ID1: %d, ID2: %d, ID4: %d\n", ID0>>6, ID1>>6, ID2>>6, ID4>>6);
	//printf("P0: %d\n", P0>>6);

	//PID1
	ID1 <<= 1;
	unsigned char ID3 = id << 4;
	ID3 &= 128;
	ID4 <<= 1;
	unsigned char ID5 = id << 2;
	ID5 &= 128;
	unsigned char P1 = ~ (ID1 ^ ID3 ^ ID4 ^ ID5);
    P1 &= 128;

	//printf("ID1: %d, ID3: %d, ID4: %d, ID5: %d\n", ID1>>7, ID3>>7, ID4>>7, ID5>>7);
	//printf("P1: %d\n", P1>>7);

	unsigned char pB = P1 ^ P0;

return pB;
}

#endif

#if 0
unsigned char calcParity(unsigned char id, enum LIN_MODE mode, int dataLen)
{
	unsigned char pB;
	unsigned char PID;
	unsigned char L5L4 = -1;

	pB = calcParityBit(id, mode);
		if (mode == LIN_2X){

			id &= 63;
			PID = pB | id;
		}

		else {

			switch (dataLen){
				case 2: L5L4 = 0;
						break;
				case 4: L5L4 = 32;
						break;
				case 8: L5L4 = 48;
						break;
				default: printf("NOT good\n");
						break;
			}

			id &= 15;
			if (L5L4 != -1)
			PID = pB | L5L4 | id;

		}

	return PID;

}

#endif


#if 0
unsigned char calcCheckSum(enum LIN_MODE mode, int dataLen, char* data, unsigned char PID)
{
	unsigned short preCheckSum = 0;
	unsigned char postCheckSum= 0;
	int i;
			if (mode == LIN_2X){
				preCheckSum += PID;
			}
				//printf("suma pre oduzimanja : %d\n", preCheckSum);
			for (i = 0; i < dataLen; i++){

				preCheckSum += data[i];
				//printf("suma pre oduzimanja : %d\n", preCheckSum);
				}

				while (preCheckSum > 256)

					preCheckSum -= 255;
		postCheckSum = preCheckSum;
		//printf("post check sum : %d\n", postCheckSum);
	return ~postCheckSum;
}

#endif


#if 0
unsigned char calcID (enum LIN_MODE mode, unsigned char PID){

	if (mode == LIN_2X)

		return PID & 63;
	else
		return PID & 15;

}

#endif


static void _cleanup(void) 
{
	tcsetattr(STDOUT_FILENO, TCSADRAIN, &orig_tios);
}

static void _input(char *buffer, int size) 
{
	static char crlf[] = { '\r', '\n' };
	int i;
	for (i = 0; i != size; ++i) 
	{
		/* if we got a CR or LF, replace with CRLF
		 * NOTE that usually you'd get a CR in UNIX, but in raw
		 * mode we get LF instead (not sure why)
		 */
		if (buffer[i] == '\r' || buffer[i] == '\n') 
		{
			if (do_echo)
				printf("\r\n");
			telnet_send(telnet, crlf, 2);
		} 
		else 
		{
			if (do_echo)
				putchar(buffer[i]);
			telnet_send(telnet, buffer + i, 1);
		}
	}
	fflush(stdout);
}

static void lin_master_speed_setup(int master_select)
{
	int rs;

	rs = strlen(lin_command[master_select]);
	_input(lin_command[master_select], rs);
}

static void can_speed_setup()
{
	int rs;

	rs = strlen(can_command[CUO]);
	_input(can_command[CUO], rs);
}

static void can_filter_setup()
{
	int rs;

	rs = strlen(can_command[CUF]);
	_input(can_command[CUF], rs);
}


static void _send(int sock, const char *buffer, size_t size) 
{
	int rs;

	/* send data */
	while (size > 0) 
	{
		if ((rs = send(sock, buffer, size, 0)) == -1) 
		{
			fprintf(stderr, "send() failed: %s\n", strerror(errno));
			exit(1);
		} 
		else if (rs == 0) 
		{
			fprintf(stderr, "send() unexpectedly returned 0\n");
			exit(1);
		}

		/* update pointer and size to see if we've got more to send */
		buffer += rs;
		size -= rs;
	}
}

static void _event_handler(telnet_t *telnet, telnet_event_t *ev,
		void *user_data) 
{
	int sock = *(int*)user_data;

	switch (ev->type) 
	{
	/* data received */
	case TELNET_EV_DATA:
		if (ev->data.size && fwrite(ev->data.buffer, 1, ev->data.size, stdout) != ev->data.size) 
		{
              		fprintf(stderr, "ERROR: Could not write complete buffer to stdout");
		}
		fflush(stdout);
		break;
	/* data must be sent */
	case TELNET_EV_SEND:
		_send(sock, ev->data.buffer, ev->data.size);
		break;
	/* request to enable remote feature (or receipt) */
	case TELNET_EV_WILL:
		/* we'll agree to turn off our echo if server wants us to stop */
		if (ev->neg.telopt == TELNET_TELOPT_ECHO)
			do_echo = 0;
		break;
	/* notification of disabling remote feature (or receipt) */
	case TELNET_EV_WONT:
		if (ev->neg.telopt == TELNET_TELOPT_ECHO)
			do_echo = 1;
		break;
	/* request to enable local feature (or receipt) */
	case TELNET_EV_DO:
		break;
	/* demand to disable local feature (or receipt) */
	case TELNET_EV_DONT:
		break;
	/* respond to TTYPE commands */
	case TELNET_EV_TTYPE:
		/* respond with our terminal type, if requested */
		if (ev->ttype.cmd == TELNET_TTYPE_SEND) 
		{
			telnet_ttype_is(telnet, getenv("TERM"));
		}
		break;
	/* respond to particular subnegotiations */
	case TELNET_EV_SUBNEGOTIATION:
		break;
	/* error */
	case TELNET_EV_ERROR:
		fprintf(stderr, "ERROR: %s\n", ev->error.msg);
		exit(1);
	default:
		/* ignore */
		break;
	}
}


static unsigned char getCmdFromCanResponse(char* buffer)
{
	return strtol(buffer + 9, NULL, 16);
}

static unsigned char getCmdFromLinResponse(char* buffer)
{
	return strtol(buffer + 7, NULL, 16);
}



int main(int argc, char **argv) 
{
	unsigned int rs;
	int sock;
	struct sockaddr_in addr;
	struct pollfd pfd[2];
	struct addrinfo *ai;
	struct addrinfo hints;

	pthread_t readCod, readButt1, readButt2, sendLin, sendCan;

	if(wiringPiSetup() < 0){
		fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
	}
	
	/* for Coder */
	pinMode(RoAPin, INPUT);
	pinMode(RoBPin, INPUT);
	pinMode(RoSPin, INPUT);
	pullUpDnControl(RoSPin, PUD_UP);

	/* for Button1 */
	pinMode(Button1, INPUT);
	pullUpDnControl(Button1, PUD_DOWN);
	
	/* for Button2 */
	pinMode(Button2, INPUT);
	pullUpDnControl(Button2, PUD_DOWN);


	/* check usage */
	if (argc != 3) {
		fprintf(stderr, "Usage:\n ./telnet-client <host> <port>\n");
		return 1;
	}
		
	
	/* look up server host */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	if ((rs = getaddrinfo(argv[1], argv[2], &hints, &ai)) != 0) {
		fprintf(stderr, "getaddrinfo() failed for %s: %s\n", argv[1],
				gai_strerror(rs));
		return 1;
	}

	/* create server socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		fprintf(stderr, "socket() failed: %s\n", strerror(errno));
		return 1;
	}

	/* bind server socket */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		fprintf(stderr, "bind() failed: %s\n", strerror(errno));
		return 1;
	}

	/* connect */
	if (connect(sock, ai->ai_addr, ai->ai_addrlen) == -1) {
		fprintf(stderr, "connect() failed: %s\n", strerror(errno));
		return 1;
	}

	/* free address lookup info */
	freeaddrinfo(ai);

	/* set input echoing on by default */
	do_echo = 0;


	/* initialize telnet box */
	telnet = telnet_init(telopts, _event_handler, 0, &sock);


	/* initialize poll descriptors */
	memset(pfd, 0, sizeof(pfd));
	pfd[0].fd = STDIN_FILENO;
	pfd[0].events = POLLIN;
	pfd[1].fd = sock;
	pfd[1].events = POLLIN;


	/* Open LIN FREE2X, baud rate 20000 */
	lin_master_speed_setup(LOF);
	usleep(400000);
	can_speed_setup();
	usleep(400000);


	pthread_create(&sendCan, NULL, SendCan, NULL);
	pthread_create(&sendLin, NULL, SendLin, NULL);
	pthread_create(&readButt1, NULL, readButton1, NULL);
	pthread_create(&readButt2, NULL, readButton2, NULL);
	pthread_create(&readCod, NULL, readCoder, NULL);


	pthread_join(sendCan,NULL);
	pthread_join(sendLin,NULL);
	pthread_join(readCod,NULL);	
	pthread_join(readButt1,NULL);
	pthread_join(readButt2,NULL);


	/* clean up */
	telnet_free(telnet);
	close(sock);


	return 0;
}

void *SendCan()
{
	globalCounterPrev = globalCounter;

	unsigned char rs;

	while(1)
	{	
		if(globalCounter != globalCounterPrev)
		{

			rs = strlen(can_command[S1]);

			switch(globalCounter)
			{
				case 1:
					_input(can_command[S1], rs);
					break;
				case 2:
					_input(can_command[S2], rs);
					break;
				case 3:
					_input(can_command[S3], rs);
					break;
				case 4:
					_input(can_command[S4], rs);
					break;
				case 5:
					_input(can_command[S5], rs);
					break;
				case 6:
					_input(can_command[S6], rs);
					break; 
				case 7:
					_input(can_command[S7], rs);
					break;
				case 8:
					_input(can_command[S8], rs);
					break;
				case 9:
					_input(can_command[S9], rs);
					break;
			}

			globalCounterPrev = globalCounter;
		}

		delay(20);
	}
}

void *SendLin()
{
	
	Button_prev_value1_send = Button_value1;
	Button_prev_value2_send = Button_value2;
	unsigned char rs;

	while(1)
	{
		if (Button_value1 != Button_prev_value1_send)
		{
			if (Button_value1 == 0)
			{
				rs = strlen(lin_command[LOFF]);
				_input(lin_command[LOFF], rs);
			}

			if (Button_value1 == 1)
			{
				rs = strlen(lin_command[LON]);
				_input(lin_command[LON], rs);
			}

			Button_prev_value1_send = Button_value1;
		}

		if (Button_value2 != Button_prev_value2_send)
		{
			if (Button_value2 == 0)
			{
				rs = strlen(lin_command[ROFF]);
				_input(lin_command[ROFF], rs);
			}

			if (Button_value2 == 1)
			{
				rs = strlen(lin_command[RON]);
				_input(lin_command[RON], rs);
			}

			Button_prev_value2_send = Button_value2;
		}

		delay(100);
	}
}

void rotaryDeal(void)
{
	Last_RoB_Status = digitalRead(RoBPin);

	while(!digitalRead(RoAPin)){
		Current_RoB_Status = digitalRead(RoBPin);
		flag = 1;
	}

	if(flag == 1){
		flag = 0;
		if((Last_RoB_Status == 0)&&(Current_RoB_Status == 1)){
			globalCounter ++;
			if (globalCounter >= 10)
				globalCounter= 1;
			printf("Coder_Value : %d\n",globalCounter);
		}
		if((Last_RoB_Status == 1)&&(Current_RoB_Status == 0)){
			globalCounter --;
			if (globalCounter <= 0)
				globalCounter = 9;
			printf("Coder_Value : %d\n",globalCounter);
		}

	}
}

void rotaryClear(void)
{
	if(digitalRead(RoSPin) == 0)
	{
		globalCounter = 0;
		printf("Coder_Value : %d\n",globalCounter);
		delay(1000);
	}
}

void *readCoder()
{
	while(1)
	{
		rotaryDeal();
		rotaryClear();
	}
}

void *readButton1()
{	
	while(1)
	{
		
		Button_value1 = digitalRead(Button1);

		if (Button_value1 != Button_prev_value1)
		{	
			printf("Button1 : %d\n",Button_value1);
			Button_prev_value1 = Button_value1;
		}

		delay(100);
	}
}

void *readButton2()
{	
	while(1)
	{	
		Button_value2 = digitalRead(Button2);
		
		if (Button_value2 != Button_prev_value2)
		{	
			printf("Button2 : %d\n",Button_value2);
			Button_prev_value2 = Button_value2;
		}

		delay(100);
	}
}


