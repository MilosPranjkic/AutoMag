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

#define LIMIT_ID_LIN2 59
#define LIMIT_ID_LIN1 15

#define SegA 28
#define SegB 27
#define SegC 24
#define SegD 29
#define SegE 5
#define SegF 7
#define SegG 31

#define blinkerL 8
#define blinkerR 9

unsigned char blink[2];
unsigned char dispState;

void dispStateF();
void setPinMode();
void clear_7seg();
void blinkS();
void blinkL();
void control();
void set1();
void set2();
void set3();
void set4();
void set5();
void set6();
void set7();
void set8();
void set9();

pthread_mutex_t lock, blinker_lock, disp_lock;


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

unsigned int readResponse(int sock, unsigned char *dataBuf)
{
	unsigned int dataSize = 15; /* LIN response */
	unsigned int tmpDataSize = 4;
	
	Network_read(sock, dataBuf, &tmpDataSize);
	
	if(dataBuf[0] == 'O' && dataBuf[1] == 'K') /* OK response */
	{
		return 4;
	}
	else if(dataBuf[0] == 'R' && dataBuf[1] == 'X') /* CAN response */
	{
		tmpDataSize = dataSize - tmpDataSize - 2;
		Network_read(sock, dataBuf + 4, &tmpDataSize);
		return dataSize - 2;
	}
	else if(dataBuf[0] == 'L' && dataBuf[1] == 'I') /* LIN response */
	{
		printf("LIN DETECTED\n");
		tmpDataSize = dataSize - tmpDataSize;
		Network_read(sock, dataBuf + 4, &tmpDataSize);
		return dataSize;
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

enum lin_alias{LOM1 = 0, LOM2 = 1, LOS1, LOS2, LSR, LRQ, LOF, LSRF, LRQF, LC};

enum can_alias{CUO = 0, CUC = 1, CUS, CUA, CUT, CUF, CH1, CH2};

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
	"LIN CLOSE\r"
};

static char* can_command[] = {
	"CAN USER OPEN CH2 500K\r\n",
	"CAN USER CLOSE ",
	"CAN USER STATUS ",
	"CAN USER ALIGN",
	"CAN USER TX ",
	"CAN USER FILTER CH2 0 FFFF\r\n",
	"CH1",
	"CH2"
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
	if ((id > LIMIT_ID_LIN2 && LIN_2X == mode) || (id > LIMIT_ID_LIN1 && LIN_1X == mode))
	{
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

		//PID1
		ID1 <<= 1;
		unsigned char ID3 = id << 4;
		ID3 &= 128;
		ID4 <<= 1;
		unsigned char ID5 = id << 2;
		ID5 &= 128;
		unsigned char P1 = ~ (ID1 ^ ID3 ^ ID4 ^ ID5);
	    P1 &= 128;

		unsigned char pB = P1 ^ P0;

	return pB;
}

unsigned char calcParity(unsigned char id, enum LIN_MODE mode, int dataLen)
{
	unsigned char pB;
	unsigned char PID;
	unsigned char L5L4 = -1;

	pB = calcParityBit(id, mode);
		if (mode == LIN_2X)
		{
			id &= 63;
			PID = pB | id;
		}

		else 
		{

			switch (dataLen)
			{
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

unsigned char calcCheckSum(enum LIN_MODE mode, int dataLen, char* data, unsigned char PID)
{
	unsigned short preCheckSum = 0;
	unsigned char postCheckSum= 0;
	int i;
			if (mode == LIN_2X)
			{
				preCheckSum += PID;
			}
				//printf("suma pre oduzimanja : %d\n", preCheckSum);
			for (i = 0; i < dataLen; i++)
			{
				preCheckSum += data[i];
				//printf("suma pre oduzimanja : %d\n", preCheckSum);
			}

			while (preCheckSum > 256)

					preCheckSum -= 255;
		postCheckSum = preCheckSum;
		
	return ~postCheckSum;
}


unsigned char calcID (enum LIN_MODE mode, unsigned char PID)
{
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
	while (size > 0) {
		if ((rs = send(sock, buffer, size, 0)) == -1) {
			fprintf(stderr, "send() failed: %s\n", strerror(errno));
			exit(1);
		} else if (rs == 0) {
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

	switch (ev->type) {
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

static void process_cmd(unsigned char cmd)
{	
	printf("Komanda: %d\n", cmd);
	switch(cmd)
	{
		case 33:
			pthread_mutex_lock(&blinker_lock);
			blink[0] = 1;
			blink[1] = 0;
			pthread_mutex_unlock(&blinker_lock);
			break;
		case 34:
			pthread_mutex_lock(&blinker_lock);
			blink[0] = 0;
			blink[1] = 0;
			pthread_mutex_unlock(&blinker_lock);
			break;
		case 35:
			pthread_mutex_lock(&blinker_lock);
			blink[0] = 0;
			blink[1] = 1;
			pthread_mutex_unlock(&blinker_lock);
			break;
		case 36:
			pthread_mutex_lock(&blinker_lock);
			blink[0] = 0;
			blink[1] = 0;
			pthread_mutex_unlock(&blinker_lock);
			break;
		case 1:
			pthread_mutex_lock(&disp_lock);		
			dispState = 1;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 2:
			pthread_mutex_lock(&disp_lock);		
			dispState = 2;		
			pthread_mutex_unlock(&disp_lock); 
			break; 
		case 3:
			pthread_mutex_lock(&disp_lock);		
			dispState = 3;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 4:
			pthread_mutex_lock(&disp_lock);		
			dispState = 4;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 5:
			pthread_mutex_lock(&disp_lock);		
			dispState = 5;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 6:
			pthread_mutex_lock(&disp_lock);		
			dispState = 6;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 7:
			pthread_mutex_lock(&disp_lock);		
			dispState = 7;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 8:
			pthread_mutex_lock(&disp_lock);		
			dispState = 8;		
			pthread_mutex_unlock(&disp_lock); 
			break;
		case 9:
			pthread_mutex_lock(&disp_lock);		
			dispState = 9;		
			pthread_mutex_unlock(&disp_lock); 
			break; 
	}
}


static unsigned char getCmdFromCanResponse(char* buffer)
{	
	printf("buffer = %s\n", buffer);
	return strtol(buffer + 9, NULL, 16);
}

static unsigned char getCmdFromLinResponse(char* buffer)
{	
	return strtol(buffer + 7, NULL, 16);
}

void *print_message_function( void *ptr )
{
	int sock = *(int*)ptr;
	char buffer[20];
	unsigned int charRead = 0;
	unsigned char cmd = 0;
	
	while(1)
	{
		charRead = readResponse(sock, buffer);
		
		if(charRead == 13) /* CAN response */
		{
			/* There is a whole CAN response message inside the buffer */
			cmd = getCmdFromCanResponse(buffer);
		}
		else if(charRead == 15) /* LIN response */
		{
			cmd = getCmdFromLinResponse(buffer);
		}
	
		telnet_recv(telnet, buffer, charRead);
		
		process_cmd(cmd);
	}
}


int main(int argc, char **argv) 
{
	unsigned int rs;
	int sock;
	struct sockaddr_in addr;
	struct pollfd pfd[2];
	struct addrinfo *ai;
	struct addrinfo hints;
	
	pthread_t thread1;

	pthread_t dispStateh, blS;

	if(wiringPiSetup() < 0)
	{
		fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
		return 1;
	}	

	if (pthread_mutex_init(&lock, NULL) != 0)
    {
		printf("\n mutex init failed\n");
		return 1;
    }

	if (pthread_mutex_init(&blinker_lock, NULL) != 0)
    {
		printf("\n mutex init failed\n");
		return 1;
    }
	
	if (pthread_mutex_init(&disp_lock, NULL) != 0)
    {
		printf("\n mutex init failed\n");
		return 1;
    }
	

	/* check usage */
	if (argc != 3) 
	{
		fprintf(stderr, "Usage:\n ./telnet-client <host> <port>\n");
		return 1;
	}

	
	/* look up server host */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	if ((rs = getaddrinfo(argv[1], argv[2], &hints, &ai)) != 0) 
	{
		fprintf(stderr, "getaddrinfo() failed for %s: %s\n", argv[1],
				gai_strerror(rs));
		return 1;
	}

	/* create server socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{
		fprintf(stderr, "socket() failed: %s\n", strerror(errno));
		return 1;
	}

	/* bind server socket */
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) 
	{
		fprintf(stderr, "bind() failed: %s\n", strerror(errno));
		return 1;
	}

	/* connect */
	if (connect(sock, ai->ai_addr, ai->ai_addrlen) == -1) 
	{
		fprintf(stderr, "connect() failed: %s\n", strerror(errno));
		return 1;
	}

	/* free address lookup info */
	freeaddrinfo(ai);

	do_echo = 0;

	/* initialize telnet box */
	telnet = telnet_init(telopts, _event_handler, 0, &sock);
	
	pthread_create(&thread1, NULL, print_message_function, &sock);

	/* initialize poll descriptors */
	memset(pfd, 0, sizeof(pfd));
	pfd[0].fd = STDIN_FILENO;
	pfd[0].events = POLLIN;
	pfd[1].fd = sock;
	pfd[1].events = POLLIN;
	
	/* setPinMode */
	pinMode(SegA, OUTPUT);
	pinMode(SegB, OUTPUT);
	pinMode(SegC, OUTPUT);
	pinMode(SegD, OUTPUT);
	pinMode(SegE, OUTPUT);
	pinMode(SegF, OUTPUT);
	pinMode(SegG, OUTPUT);


	pinMode(blinkerL, OUTPUT);
	pinMode(blinkerR, OUTPUT);

	digitalWrite(blinkerL, LOW);
	digitalWrite(blinkerR, LOW);
	clear_7seg();
	set1();

	/* Open LIN SLAVE2X, baud rate 20000 */
	lin_master_speed_setup(LOF);
	usleep(400000);
	can_speed_setup();
	usleep(400000);
	can_filter_setup();
	usleep(400000);

	pthread_create(&dispStateh, NULL, (void*)&dispStateF, NULL);
	pthread_create(&blS, NULL, (void*)&blinkS, NULL);

	blink[0] = 0;
	blink[1] = 0;


	/* clean up */
	pthread_join(dispStateh,NULL);
	pthread_join(blS,NULL);

    pthread_join( thread1, NULL);
	telnet_free(telnet);
	close(sock);

	return 0;
}


void blinkS()
{	
	unsigned char blink_onR, blink_onL;
	
	while(1)
	{	
		pthread_mutex_lock(&blinker_lock);
		
		blink_onL = blink[0];
		blink_onR = blink[1];

		pthread_mutex_unlock(&blinker_lock);		
		
		if(blink_onR == 1 && blink_onL == 0)
		{
			digitalWrite(blinkerR, HIGH);
			delay(400);
			digitalWrite(blinkerR, LOW);
			delay(400);
		}
		
		else if(blink_onR == 0 && blink_onL == 1)
		{
			digitalWrite(blinkerL, HIGH);
			delay(400);
			digitalWrite(blinkerL, LOW);
			delay(400);
		}
		else if(blink_onR == 1 && blink_onL == 1)
		{
			digitalWrite(blinkerR, HIGH);
			digitalWrite(blinkerL, HIGH);
			delay(400);
			digitalWrite(blinkerR, LOW);
			digitalWrite(blinkerL, LOW);
			delay(400);
		}
		else
		{
			digitalWrite(blinkerR, LOW);
			digitalWrite(blinkerL, LOW);
			delay(100);
		}
	}
}


void dispStateF()
{		
	unsigned char state;	
	
	while(1)
	{		
		pthread_mutex_lock(&disp_lock);		
		state = dispState;		
		pthread_mutex_unlock(&disp_lock); 
		
		switch(state)
		{
			case 1:
				set1();
				break;
			case 2:
				set2();
				break;
			case 3:
				set3();
				break;
			case 4:
				set4();
				break;
			case 5:
				set5();
				break;
			case 6:
				set6();
				break;
			case 7:
				set7();
				break;
			case 8:
				set8();
				break;
			case 9:
				set9();
				break;	
		} 
		
		delay(200);
	}
	
}

void setPinMode()
{	
	pinMode(SegA, OUTPUT);
	pinMode(SegB, OUTPUT);
	pinMode(SegC, OUTPUT);
	pinMode(SegD, OUTPUT);
	pinMode(SegE, OUTPUT);
	pinMode(SegF, OUTPUT);
	pinMode(SegG, OUTPUT);

	pinMode(blinkerL, OUTPUT);
	pinMode(blinkerR, OUTPUT);
}

void clear_7seg()
{	
	pthread_mutex_lock(&lock);

	digitalWrite(SegA, LOW);
	digitalWrite(SegB, LOW);
	digitalWrite(SegC, LOW);
	digitalWrite(SegD, LOW);
	digitalWrite(SegE, LOW);
	digitalWrite(SegF, LOW);
	digitalWrite(SegG, LOW);

	pthread_mutex_unlock(&lock);
}

void set1()
{	
	clear_7seg();
	pthread_mutex_lock(&lock);
	
	digitalWrite(SegB, HIGH);
	digitalWrite(SegC, HIGH);	
	
	pthread_mutex_unlock(&lock);	
}

void set2()
{	

	clear_7seg();
	pthread_mutex_lock(&lock);

	
	digitalWrite(SegA, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegG, HIGH);
	digitalWrite(SegE, HIGH);
	digitalWrite(SegD, HIGH);
	
	pthread_mutex_unlock(&lock);	
}

void set3()
{

	clear_7seg();
	pthread_mutex_lock(&lock);

	digitalWrite(SegA, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegG, HIGH);
	digitalWrite(SegC, HIGH);
	digitalWrite(SegD, HIGH);
	
	pthread_mutex_unlock(&lock);
}

void set4()
{
	clear_7seg();	

	pthread_mutex_lock(&lock);

	digitalWrite(SegF, HIGH);
	digitalWrite(SegG, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegC, HIGH);	
	
	pthread_mutex_unlock(&lock);
}

void set5()
{	
	clear_7seg();	

	pthread_mutex_lock(&lock);

	digitalWrite(SegA, HIGH);
	digitalWrite(SegF, HIGH);
	digitalWrite(SegG, HIGH);
	digitalWrite(SegC, HIGH);	
	digitalWrite(SegD, HIGH);
	
	pthread_mutex_unlock(&lock);
}

void set6()
{
	clear_7seg();
	
	pthread_mutex_lock(&lock);

	digitalWrite(SegA, HIGH);
	digitalWrite(SegF, HIGH);
	digitalWrite(SegG, HIGH);
	digitalWrite(SegC, HIGH);	
	digitalWrite(SegD, HIGH);
	digitalWrite(SegE, HIGH);

	pthread_mutex_unlock(&lock);
}

void set7()
{
	clear_7seg();

	pthread_mutex_lock(&lock);	

	digitalWrite(SegA, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegC, HIGH);

	pthread_mutex_unlock(&lock);		
}

void set8()
{	
	clear_7seg();	

	pthread_mutex_lock(&lock);	

	digitalWrite(SegA, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegC, HIGH);
	digitalWrite(SegD, HIGH);
	digitalWrite(SegE, HIGH);
	digitalWrite(SegF, HIGH);
	digitalWrite(SegG, HIGH);
	
	pthread_mutex_unlock(&lock);
}

void set9()
{	
	clear_7seg();

	pthread_mutex_lock(&lock);	

	digitalWrite(SegA, HIGH);
	digitalWrite(SegB, HIGH);
	digitalWrite(SegC, HIGH);
	digitalWrite(SegD, HIGH);
	digitalWrite(SegF, HIGH);
	digitalWrite(SegG, HIGH);

	pthread_mutex_unlock(&lock);
}



