#include "WirelessShow.h"

//int packet_id_list[PACKET_AVAILABLE] = {1, 61, 121, 181, 241, 301, 361, 421, 481} ; // liste des premiers canaux de chaque paquet
int indice_packet = 1 ;

unsigned long package_rcv_delta_t = 0 ; // delta t entre deux rÃƒÂ©ceptions de packet
unsigned long iterations = 0;
unsigned long decimation = 100000;
long lastPeriod = -1;




//Fonctions receiver
// Cette fonction reconstitue la trame DMX afin de contrôler la bande LED selon l'adressage
void NodeShow::prepare_pixel_color1(int start_indice, int stop_indice, int packet_ID) {
  // Serial.println("prepare_pixel_color1 launched") ;
  //int pixel_offset = ((packet_ID-1)*PACKET_SIZE-DECOR_DMX_ADRESS)/CHANNELS_PER_PIXEL  ;
  int pixel_offset = ((packet_ID - 1) * PACKET_SIZE + 1 - DECOR_DMX_ADRESS) / 3 - 1  ;
  for (int i = start_indice; i < stop_indice  ; i += 3) { // parcours les éléments du tableau reçu
    // Serial.print("i : ") ;   Serial.println(i) ;
    if (radio.DATA[i] == 1 ) {
      radio.DATA[i] = 0 ;
    }
    if (radio.DATA[i + 1] == 1 ) {
      radio.DATA[i + 1] = 0 ;
    }
    if (radio.DATA[i + 2] == 1 ) {
      radio.DATA[i + 2] = 0 ;
    }
    pixels.setPixelColor((i + 2) / 3 + pixel_offset, pixels.Color(radio.DATA[i], radio.DATA[i + 1], radio.DATA[i + 2])); // change the color
#ifdef DEBUG
    //    Serial.print("Pixel:") ; Serial.print((i + 2) / 3 + pixel_offset) ; Serial.print(": ") ;
    //    Serial.print(radio.DATA[i]) ;
    //    Serial.print(" ") ;
    //    Serial.print(radio.DATA[i + 1]) ;
    //    Serial.print(" ") ;
    //    Serial.print(radio.DATA[i + 2]) ;
    //    Serial.println(" ") ;
#endif
  }
}

void NodeShow::stripLed_init(){
   //************* NEOPIXEL SETTINGS *********************************
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  pixels.begin(); // This initializes the NeoPixel library.
  //*****************************************************************
  black_strip() ;
}

void NodeShow::printDMX(){
  if (packet_id == 1){
    for (int i = 0;i<10;i++){
      Serial.print(theData.packet[i]); Serial.print(" ");
    }
    Serial.println();
  }
}

void NodeShow::black_strip(void) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 0, 0, 0);
    pixels.show(); // on met à jour les pîxels de la bande
  }
}

void NodeShow::find_index() {
  int packet_index = 1 ;

  while ( packet_index < PACKET_ID_MAX) { // on parcourt les packets pour trouver les indices de départ et d'arriver
    int channel_inf = (packet_index - 1) * PACKET_SIZE + 1 ;
    int channel_sup = packet_index * PACKET_SIZE ;
    if ( (channel_inf <= DECOR_DMX_ADRESS)   & (DECOR_DMX_ADRESS <= channel_sup)  ) {
      start_index = DECOR_DMX_ADRESS - ((packet_index - 1) * PACKET_SIZE) ;
      start_packet = packet_index ;
    }
    if (  (channel_inf <= LAST_DMX_ADRESS)   & (LAST_DMX_ADRESS <= channel_sup)  ) {
      stop_index = LAST_DMX_ADRESS - ((packet_index - 1) * PACKET_SIZE) ;
      stop_packet = packet_index ;
      break ;
    }
    packet_index++ ;
  }
}

void NodeShow::print_config(void) {
  Serial.print("Recepteur strip led ");

  Serial.println("\nNetwork : ") ;
  Serial.print("Node: ") ; Serial.println(NODEID) ;
  Serial.print("NETWORKID: ") ; Serial.println(NETWORKID) ;
  Serial.print("PACKET_SIZE: ") ; Serial.println(PACKET_SIZE) ;
  Serial.print("DECOR_DMX_ADRESS: ") ; Serial.println(DECOR_DMX_ADRESS) ;
  Serial.print("LAST_DMX_ADRESS: ") ; Serial.println(LAST_DMX_ADRESS) ;


  Serial.println("\nDecor parameters : ") ;
  Serial.print("NUMPIXELS: ") ; Serial.println(NUMPIXELS) ;
  Serial.print("PIX_PER_GROUP: ") ; Serial.println(PIX_PER_GROUP) ;
  Serial.print("CHANNELS_PER_PIXEL: ") ; Serial.print(CHANNELS_PER_PIXEL) ;  Serial.println("(RVB)") ;
  Serial.print("CHANNELS_NBR : ") ; Serial.println(CHANNELS_NBR) ;
  //  Serial.print("PIXELS_PER_PACKET : ") ; Serial.println(PIXELS_PER_PACKET) ;
  //  Serial.print("NUM_PACKET : ") ; Serial.println(NUM_PACKET) ;
  //  Serial.print("CHANNELS_REST : ") ; Serial.println(CHANNELS_REST) ;
  //  Serial.print("PIXELS_IN_LAST_PACKET : ") ; Serial.println(PIXELS_IN_LAST_PACKET) ;
  Serial.print("start_packet : ") ; Serial.println(start_packet) ;
  Serial.print("start_index : ") ; Serial.println(start_index) ;
  Serial.print("stop_packet : ") ; Serial.println(stop_packet) ;
  Serial.print("stop_index : ") ; Serial.println(stop_index) ; Serial.print("\n") ;
}

void NodeShow::wireless_init_rcv(void){
  //************************ RFM69 INIT ******************************
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower(); // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY); // set the ENCRYPTKEY
  radio.promiscuous(true);
  //*****************************************************************
  state = start_packet ; // initializes the state machine
}

bool NodeShow::_noDataSince_rcv() {
  package_rcv_delta_t = millis() - last_reception;
  debit = 1000/float(package_rcv_delta_t);
  if (package_rcv_delta_t > NO_DATA_SINCE) {
    digitalWrite(RECEPTION, LOW) ;
    trameCntOk = 0;
    broadcast_RSSI = 0;
    return false ;
  }
  else
    digitalWrite(RECEPTION, HIGH) ;
  return true ;
}

void NodeShow::printReception() {
static unsigned int  counter = 0;
static unsigned int print_decimation = 1000000;
if (counter>=print_decimation){
    //Serial.print("Broadcast RSSI: ");Serial.println(broadcast_RSSI);
    Serial.print("Qualité comm :") ; Serial.print(trameCntOk) ; Serial.println("/10") ;
    //Serial.print("Période :") ; Serial.println(package_rcv_delta_t) ;
    //Serial.print("Débit :") ; Serial.println(debit) ;



    Serial.print("Buff packet ID : ") ;
    for (int idx=0;idx<PACKET_NBR ;idx++){
      Serial.print(packetIdBuff[idx]);
    }
    Serial.println("") ;

    counter = 0;
  }
  else
    counter++;
}

void NodeShow::listenRadio(void){
  //check if something was received
  bPacketRcv = radio.receiveDone();
  _noDataSince_rcv() ; // display the com status with the LED

  if (bPacketRcv)
  {
    last_reception = millis() ;
    if (radio.DATALEN == sizeof(Payload) && radio.TARGETID == BROADCASTID)
    {
      theData = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      packet_id = theData.packetId;
      broadcast_RSSI = radio.readRSSI();
      checkCom();
    }
    radio.receiveDone(); //put back the radio in RX mode
  }
}

void NodeShow::checkCom(){
  static int packetCounter = 0;
  static int trameCnt = 0;
  static int trameCntOkTmp = 0;
  bool paquet_perdu = false; // True si un paquet n'a pas été reçu, false sinon
  #define TRAMESNBRMAX 10

  if (trameCnt>=TRAMESNBRMAX){
    trameCntOk = trameCntOkTmp;
    trameCntOkTmp = 0;
    trameCnt = 0;

  }

  if (first_iter == false){
    if (packet_id <last_packet_id){ // une nouvelle trame arrive
      paquet_perdu = false;
      trameCnt++;
      packetCounter = 0;
    }

    if (paquet_perdu ==false && packet_id>1 && packet_id<=PACKET_NBR && last_packet_id!=(packet_id-1)){ // si un paquet est perdu
      paquet_perdu = true;
    }

    if (packet_id==PACKET_NBR && !paquet_perdu){ // si à la fin de la trame aucun paquet n'a été perdu
      trameCntOkTmp++;
    }

  }
  last_packet_id = packet_id;

  if (packetCounter<10){
      packetIdBuff[packetCounter] = packet_id;
      packetCounter++;
  }
  else
      packetCounter = 0;
}

void NodeShow::traitement() {
  digitalWrite(6, 1);
  //check if something was received
  if (bPacketRcv)
  {
    if (packet_id == PACKET_NBR ) {
      //digitalWrite(LED2, HIGH) ;
    }
    else {
      //digitalWrite(LED2, LOW) ;
    }
  }

  if ((packet_id == state) ) { // check if the packet received is the one we are waiting for

    //Serial.print("packet ") ; Serial.print(packet_id) ; Serial.println(" received") ;
    // start_pixel = (packet_id - 1) * PIXELS_PER_PACKET ;


    if ((state == stop_packet) ) { // si le paquet reçu est le dernier paquet exigé
      digitalWrite(LED1, LOW) ;
      prepare_pixel_color1(1, stop_index, packet_id) ;
      pixels.show(); // on met à jour les pîxels de la bande
      state = start_packet ; // on retourne à l'état initial : attendre le premier paquet
      //Serial.print("Wait for packet ") ; Serial.print(state) ; Serial.println("...") ;
      //Serial.println("Strip updated!") ;

    }

    else if (state == start_packet) { // si le paquet reçu est le premier paquet exigé
      digitalWrite(LED1, HIGH) ;
      prepare_pixel_color1(start_index, PACKET_SIZE, packet_id) ;
      state++ ; // on attend le paquet suivant
      //Serial.print("Wait for packet ") ; Serial.print(state) ; Serial.println("...") ;

    }


    else {
      prepare_pixel_color1(1, PACKET_SIZE, packet_id) ;
      state++ ; // on attend le paquet suivant
      //Serial.print("Wait for packet ") ; Serial.print(state) ; Serial.println("...") ;
    }
  }
  if (packet_id == 6 ) {

    //pixels.show(); // on met à jour les pîxels de la bande
    state = start_packet ; // on retourne à l'état initial : attendre le premier paquet

  }
  // Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU // voir si nécessaire
  if (first_iter == true)
    first_iter = false;
}


/**************************************/

/********* Fonctions transmetteur *****/

void NodeShow::wireless_init(void){
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower(); // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
}

void NodeShow::sendPackets(uint8_t  *trame){
  uint8_t radiopacket[PACKET_SIZE_PLUS_ID] ;

  for (int i = 1 ; i <=PACKET_NBR ; i++){
      build_packet(i, trame, radiopacket); // construction et envoi du paquet i
      radio.send(NODERECEIVE, (const void*)radiopacket, strlen(radiopacket), false) ; // envoi du paquet de donnÃ©es
    }
}

/*
 * Construit un paquet de plusieurs canaux DMX selon le packet_id donnÃ©es en paramÃ¨tre.
 * @param int packet_id : l'identifiant du paquet Ã  envoyer
 * @return None.
 */
void NodeShow::build_packet(int packet_id, uint8_t  *trame , uint8_t  *packet ) {
  packet[0] = packet_id ; // L'identifiant du paquet est le premier byte du paquet Ã  envoyer
  int channel_offset = (packet_id - 1) * PACKET_SIZE ; // calcul de l'offset du canal DMX selon le packet_id

  for (int i = 1 ; i < (PACKET_SIZE + 1) ; i++) { // construction du paquet Ã  envoyer avec les canaux DMX
    packet[i] = trame[i + channel_offset] ;
    delete_zeros(packet, i) ;
  }
}

/*
 * delete_zeros regarde si la valeur du canal donnÃ©e en paramÃ¨tre est nulle. Si oui elle la chnage en 1. Cette function est utilisÃ©e pour permettre l'envoi des donnÃ©es via
 * le module RFM69. Le module ne prends pas les valeurs nulles.
 * @param int i : le canal concernÃ©
 * @return None.
 */
void NodeShow::delete_zeros(uint8_t *packet, int i){
   if (packet[i] == 0 )
      packet[i] = 1 ; // rejet des zeros sur les canaux DMX pour Ã©viter les erreurs de transmission sur la liason RFM69
}

void NodeShow::_noDataSince() {
  package_rcv_delta_t = millis() - last_reception;
  if (package_rcv_delta_t > NO_DATA_SINCE) {
    digitalWrite(RFM69_COM_LED, LOW) ;
  }
  else
    digitalWrite(RFM69_COM_LED, HIGH) ;
}

void NodeShow::checkRFMReception(){
  if (radio.receiveDone())
  {
    last_reception = millis() ;
    radio.receiveDone(); //put back the radio in RX mode
  }
  _noDataSince();

  if (iterations>=decimation){
    iterations=0;
    //Serial.println("Hello");
    //Serial.print('SENDERID: ');Serial.println(radio.SENDERID, DEC);
    //Serial.print(" [RX_RSSI:");Serial.print(radio.readRSSI());Serial.println("]");
    //Serial.print("PÃƒÂ©riode rÃƒÂ©ception paquet :") ; Serial.print(package_rcv_delta_t) ; Serial.println(" ms") ;
  }
  else {
    //Serial.println(iterations);
    iterations= iterations + 1;
  }
}

void NodeShow::stripAuto(){
    fullRed();
}

void NodeShow::fullRed() {
    for(uint16_t i=0; i<pixels.numPixels(); i++) {
        pixels.setPixelColor(i, pixels.Color(255,0,0 ) );
    }
    pixels.show();
}

