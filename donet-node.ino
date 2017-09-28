#include <SPI.h>
#include "RF24.h"

RF24 radio(14, 10);

struct __attribute__((packed)) packet {
  uint8_t s_addr;
  uint8_t d_addr;
  uint8_t counter;
  uint8_t command;
  uint8_t data[13];
};

byte address[] = "DoNet";

#define CMD_PING    0x01
#define CMD_PWM_ONE 0x02
#define CMD_PWM_THREE 0x03
#define CMD_UART_SEND 0x04
#define CMD_UART_RECV 0x05

#define CMD_PING_ACK      0xE1
#define CMD_PWM_ONE_ACK   0xE2
#define CMD_PWM_THREE_ACK 0xE3
#define CMD_UART_SEND_ACK 0xE4
#define CMD_UART_RECV_ACK 0xE5


#define ERR_OK 0x00
#define ERR_TIMEOUT 0x01
#define ERR_OTHER 0xEE


struct packet packetIn;
struct packet packetOut;

#define ADDRESS 0x02

#define MAX_TRIES 10 
#define TIMEOUT 200000 //In us

#define DEBUG_SERIAL 1

#ifdef DEBUG_SERIAL
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#define DPRINT(...) Serial.print(__VA_ARGS__)
#else
#define DPRINTLN(...) do {} while(0)
#define DPRINT(...) do {} while(0)
#endif

void setup() {
  Serial.begin(115200);
  radio.begin();

  pinMode(3,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(5,OUTPUT);
  
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);

  // Start the radio listening for data
  radio.startListening();
}


uint8_t send_packet(){
    uint8_t tries;
    tries = 0;

    radio.stopListening();
    
    while (tries < MAX_TRIES){
      if (radio.write( &packetOut, sizeof(struct packet) )){
        break;
      }
      tries++;  
    }
    radio.startListening(); 
    return tries;
}
void reply_ping(){
  packetOut.data[0] = 0xAD;
  reply_packet();
}

void reply_packet(){
  packetOut.s_addr = ADDRESS;
  packetOut.d_addr = packetIn.s_addr;
  packetOut.command = packetIn.command+0xE0;
  uint8_t tries;
  tries = send_packet();
  if (tries == MAX_TRIES){
    DPRINTLN("Send failed");  
  }
}


void setPwmOne(){
  analogWrite(5,packetIn.data[0]);
}

void setPwmThree(){
  analogWrite(3,packetIn.data[0]);
  analogWrite(6,packetIn.data[1]);
  analogWrite(9,packetIn.data[2]);
}

void loop() {
  while (!radio.available()){}
  radio.read(&packetIn,sizeof(struct packet));
  if (packetIn.d_addr != ADDRESS){
    return;  
  }
  switch (packetIn.command){
    case CMD_PING:
      DPRINTLN(packetIn.data[0],HEX);
      reply_ping();
      break;
    case CMD_PWM_ONE:
      DPRINTLN("One");
      setPwmOne();
      reply_packet();
      break;
    case CMD_PWM_THREE:
      DPRINTLN("Three");
      setPwmThree();
      reply_packet();
      break;
  }

}
