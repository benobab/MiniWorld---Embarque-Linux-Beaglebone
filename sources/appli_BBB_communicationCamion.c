#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//		     VARIABLES			//
	int tty_fd;

int addr_low_entree = 0;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//		     STRUCTURES			//
struct tx_packet {
   uint8_t length; /* (1 octet, = N + 2) */
   uint8_t addr_low; /* (un octet) */
   uint8_t addr_high; /* (un octet) */
   uint8_t data[61]; /* (N octets) */
};

 struct rx_packet {
   uint8_t length; /* Taille du paquet - 1 : toujours 15 */
   uint8_t bcast; /* Always 0xFF */
   uint16_t addr; /* Big endian */
   uint8_t status;  /* Vehicule leds state, 8 bits */
   uint8_t anticolision;
   uint8_t speed_target;
   uint8_t motor_speed;
   uint8_t Vbat; /* tension batterie */
   uint8_t magnets_to_action;
   uint16_t nb_magnets;
   uint32_t message_num;

} __attribute__ ((__packed__));


//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//		FONCTIONS PERSO			//
struct tx_packet* creer_tx_Packet (uint8_t l, uint8_t al, uint8_t ah, uint8_t* d) {
	struct tx_packet* packet = malloc(sizeof(struct tx_packet));
	packet->length = l+2;
	packet->addr_low = al;
	packet->addr_high = ah;
	memcpy(packet->data,d, l); 
	
	printf("add_low : %d\n", packet->addr_low);

	return packet;
}

void envoyer_msg() {

}


void analyse_entree(char* s) {



}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int main(int argc, char** argv)
{
	struct termios tio;

	printf("Please start with %s /dev/ttyS1 (for example)\n", argv[0]);

	tty_fd = open(argv[1], O_RDWR | O_NONBLOCK);      
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CREAD | CLOCAL;           // 8n1, see termios.h for more information
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;
	cfsetospeed(&tio, B115200);            // 115200 baud
	cfsetispeed(&tio, B115200);            // 115200 baud
	tcsetattr(tty_fd, TCSANOW, &tio);


	sscanf(argv[2],"%d",&addr_low_entree);

	printf("Adresse du véhicule : %d\n",addr_low_entree);

	while (1) {
		write(STDOUT_FILENO, "\nmenu : \n",10);
		write(STDOUT_FILENO, "\n",1);
		write(STDOUT_FILENO, "\n",1);
		write(STDOUT_FILENO, "\n",1);
		
		char s[61] = ""; 
		scanf("%s",s);
		
		if (strcmp(s,"quit") == 0) {
			break;
		} else {
			
			int loop = 0;
			//analyse_entree(s);
			uint8_t d[61];
			int ret = 0;
			struct tx_packet* p = NULL;
			memcpy(d,s,strlen(s));
			d[strlen(s)] = '\n';
			int add; 
			p = creer_tx_Packet((uint8_t)strlen(d), addr_low_entree, 0,d);

			ret = write(tty_fd, p, (p->length + 1));
			printf("sending: (%d/%d):'%s', returned %d\n", strlen(d), strlen(s), d, ret);

			//Pour boucler tant qu'on n'a pas de réponse du camion
			while (1) {
				unsigned char buffer[128] = "";
				int len = 0;
				loop++;
				len = read(tty_fd, buffer, sizeof(struct rx_packet));
				if (len > 0) {
					struct rx_packet* msg_recu;
		
					msg_recu = (struct rx_packet*) buffer;
					write(STDOUT_FILENO, "Adresse camion : %d.\n",msg_recu->addr);

					break;

				} 
			}
			if (p != NULL) {
				free(p);
			}
		}
	}

	close(tty_fd);
}

