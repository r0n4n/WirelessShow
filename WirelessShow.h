#ifndef WirelessShow
#define WirelessShow

#include "Arduino.h"
#include <RFM69.h> //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS      50.0


//*********************************************************************************************
// *********** RFM69 Configuration *********************** *************
//*********************************************************************************************
#define NETWORKID 100 // The same on all nodes that talk to each other
#define NODEID 1 // The unique identifier of this node
#define ENCRYPTKEY "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define NODERECEIVE 2

//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY RF69_433MHZ
#define IS_RFM69HCW true // set to 'true' if you are using an RFM69HCW module

#define RFM69_CS 10 // Chip select
#define RFM69_IRQ 2 // interruption
#define RFM69_IRQN 0 // Pin 2 is IRQ 0!
#define RFM69_RST 9 // Reset
//********************************************************************************************

/******************************Config réseau récepteur***************************************/
// ***************** Configuration réseau *********************
#define TRANSMITTERID 1 // L'adresse réseau de l'émetteur
#define BROADCASTID 2 // L'adresse où le DMX est broadcasté
#define NODEID 3 // L'adresse réseau du récepteur
//***********************************************************************
/********************************************************************/


/************************************** WirelessShow macros ************************************/
#define NO_DATA_SINCE 3000
#define PACKET_SIZE 60 // number of channels to send per packet
#define PACKET_SIZE_PLUS_ID  PACKET_SIZE+1 // total size of a packet with the ID
#define PACKET_NBR 8 // nombre paquet que l'on souhaite envoyer
#define PACKET_AVAILABLE 8 // nombre de paquets qui dÃ©composent l'ensemble des canaux DMX

#define RFM69_COM_LED 13



/*********************************************************************************************/


/********* HARDWARE CONFIG **************/
// HARDWARE OUTPUTS
#define LED1 6
#define LED2 7
#define LED3 8
#define BANDE1 A3 // pin pour contrôler la bande Led
//********************************************************************************


//*********************  Renommage des entrées sorties **************************

#define RECEPTION LED3 // la led clignote dès que le récepteur reçoit un message
/************************************************************************************/

class NodeShow {
    public:
/**********Recepteur fonctions *************/
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, BANDE1, NEO_GRB + NEO_KHZ800); // Création de l'objet pixel qui gère la bande numérique du pin BANDE1
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

bool _noDataSince_rcv(void);
void printReception(void);
void listenRadio(void);
void checkCom(void);
void traitement(void);
void wireless_init_rcv(void);
void printDMX(void);
void find_index(void);
void print_config(void);
void prepare_pixel_color1(int start_indice, int stop_indice, int packet_ID) ;
void stripLed_init(void);
void black_strip(void);
void fullRed(void);
void stripAuto();
/***************************************///

/**********Emeteur fonctions *************/
void wireless_init(void);
void sendPackets(uint8_t  *trame);
void build_packet(int packet_id, uint8_t  *trame , uint8_t  *packet );
void delete_zeros(uint8_t *packet, int i);
void _noDataSince();
void checkRFMReception();
/**************************/



/** Variables****/
typedef struct {
  uint8_t           packetId; //store this nodeId
  uint8_t packet[PACKET_SIZE];
} Payload;
Payload theData;

/**Communications status **/

unsigned long last_reception = 0 ;
float debit = 0;
int broadcast_RSSI = 0;
int trameCntOk = 0;
int packetIdBuff[PACKET_NBR];
bool bPacketRcv = false;

/****** Variables émetteur **/
int state ; // L'id du paquet qui est attendu
int packet_id ; // L'id du paquet qui vient d'être reçu
int last_packet_id ; // ID du dernier paquet reçu

// Variables config paquets
int nbr_paquet_perdu = 0; // compteur du nombre de paquets perdu
int start_index = 0  ; // premier indice dans le premier paquet que le récepteur doit interpréter
int stop_index = 0 ; // dernier indice dans le dernier paquet que le récepteur doit interpréter
int start_packet = 0  ; // id du premier paquet que le récepteur doit interpréter
int stop_packet = 0 ; // id du dernier paquet que le récepteur doit interpréter
bool first_iter = true ;
// How many NeoPixels are attached to the Arduino?
#define CHANNELS_PER_PIXEL 3 // RVB
#define PIX_PER_GROUP 1 // number of pixels together
#define PACKET_SIZE 60 // size of a packet received
#define DECOR_DMX_ADRESS  1 // adresse DMX du récepteur
#define PACKET_ID_MAX 9 // nombre de paquets maximal que peut envoyer l'émetteur (dépend de la taille des paquets)
#define PACKET_NBR (int)8 // nombre de paquets envoyés par l'emetteur

#define CHANNELS_NBR (NUMPIXELS*CHANNELS_PER_PIXEL/PIX_PER_GROUP) // on détermine le nombre de canaux nécessaires
#define LAST_DMX_ADRESS (DECOR_DMX_ADRESS+CHANNELS_NBR-1)

/****************************/
};




#endif // WirelessShow
