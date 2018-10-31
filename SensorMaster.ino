#include <SoftwareSerial.h>
#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"

#define MAX_BRIGHTNESS 255

#if defined(ARDUINO_AVR_UNO)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
#else
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
#endif
int32_t n_ir_buffer_length; //data length
int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t pulse; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

const unsigned long PULSEPOLL_PERIOD = 9000;
const unsigned long RECONN_PERIOD = 10000;

const String RECONN= "RECON!";  //Request recon asap
const String PULSE = "PULSE:";  //Pulse marker
const String OTHER = "OTHER:";  //Other marker (not used)
const String HAND1 = "HELLO?#"; //start handshake
const String HAND2 = "HELLO!#"; //respond handshake
const String END   = "#";       //end marker
const String GETPULSE = "PULSE?#"; //request (initial pulse)

bool RECONNECT = false; //State flag for reconnection


int lastPulse = 80;
int lastStatus;
int defaultPulse = 80;
int lowPulse;
int highPulse;

unsigned long startMillis;
unsigned long currentMillis;

struct packet {
  String type;
  String value;
};

SoftwareSerial BTSerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600); //38400
  masterInit();
  startMillis = millis();
}

void loop(){
  currentMillis=millis();
  if (currentMillis - startMillis >= PULSEPOLL_PERIOD){
     getPulse();
     sendPulse();
     startMillis = millis();
  }

  /*
  if (BTSerial.available()){
    digitalWrite(13,HIGH);
    Serial.write(BTSerial.read());
  }
  digitalWrite(13,LOW);
  if (Serial.available())
    BTSerial.write(Serial.read());
    */
}

struct packet readPacket(){
  struct packet newPacket;
  char c;
  int counter = 0;
  String packetType;
  String serialBuffer;
  Serial.println("READING FROM SERIAL");
  while(BTSerial.available() > 0) {
    c = BTSerial.read(); 
    Serial.println(c);
    if (c=='#'){
      break;
    }
    counter++;
    if (counter<7){
      packetType += c;
    } else {
      serialBuffer += c;
    }
  }
  newPacket.type = packetType;
  newPacket.value = serialBuffer;
  
  Serial.println("RECEIVED STRING: ");
  Serial.println(packetType);
  Serial.println(serialBuffer);
  Serial.println((newPacket.type + newPacket.value));
  Serial.println("-----------");
  

  if (packetType.equals(RECONN)){
    RECONNECT = masterInit();
    while(RECONNECT){
      RECONNECT = masterInit();
    }
  }
  
  return newPacket;
}

int getPulse(){
  uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i;
  float f_temp;
  
  un_brightness=0;
  un_min=0x3FFFF;
  un_max=0;
  
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while(1)
  {
    i=0;
    un_min=0x3FFFF;
    un_max=0;

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for(i=25;i<100;i++)
    {
      aun_red_buffer[i-25]=aun_red_buffer[i];
      aun_ir_buffer[i-25]=aun_ir_buffer[i];

      //update the signal min and max
      if(un_min>aun_red_buffer[i])
        un_min=aun_red_buffer[i];
      if(un_max<aun_red_buffer[i])
        un_max=aun_red_buffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for(i=75;i<100;i++)
    {
      un_prev_data=aun_red_buffer[i-1];
      while(digitalRead(10)==1);
      digitalWrite(9, !digitalRead(9));
      maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

      //calculate the brightness of the LED
      if(aun_red_buffer[i]>un_prev_data)
      {
        f_temp=aun_red_buffer[i]-un_prev_data;
        f_temp/=(un_max-un_min);
        f_temp*=MAX_BRIGHTNESS;
        f_temp=un_brightness-f_temp;
        if(f_temp<0)
          un_brightness=0;
        else
          un_brightness=(int)f_temp;
      }
      else
      {
        f_temp=un_prev_data-aun_red_buffer[i];
        f_temp/=(un_max-un_min);
        f_temp*=MAX_BRIGHTNESS;
        un_brightness+=(int)f_temp;
        if(un_brightness>MAX_BRIGHTNESS)
          un_brightness=MAX_BRIGHTNESS;
      }


#if defined(ARDUINO_AVR_FLORA8)
      LED.setPixelColor(0, un_brightness/BRIGHTNESS_DIVISOR, 0, 0);
      LED.show();
#endif


    }  
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &pulse, &ch_hr_valid); 
  }
  lastPulse = 80;
  lowPulse = lastPulse-25;
  highPulse = lastPulse+25;
  return lastPulse;
}

void sendPulse(){
  String pulse = PULSE;
  pulse += String(lastPulse);
  pulse += END;
  Serial.println("SENDING PULSE = ");
  Serial.println(pulse);
  BTSerial.print(pulse);
}

bool masterInit(){
  struct packet msg;
  Serial.println("Waiting 2s, then initing handshake");
  delay(2000);
  BTSerial.print(HAND1);    //1.
  Serial.println("SENT HANDSHAKE, waiting answer");
  startMillis=millis();
  while(BTSerial.available()<7){
    if(millis() - startMillis >= RECONN_PERIOD){
      Serial.println("Resending first handshake");
        BTSerial.print(HAND1);
    }
  }
  msg = readPacket();
  while (!msg.type.equals("HELLO!")){ //4.
    Serial.println("Slave didn't reply properly, sending again");
    BTSerial.print(HAND1); 
    delay(1000);
    msg = readPacket();
  }
  Serial.println("RECEIVED HANDSHAKE BACK, next waiting for pulse request");
  while(BTSerial.available()<7) ;
  msg = readPacket(); //6. //7 Receive pulse request 
  defaultPulse = getPulse();
  String pulse = String(defaultPulse);
  pulse = PULSE + pulse + END;
  BTSerial.print(pulse);   // 7. and send pulse
  Serial.println("Sent initial pulse to slave ");
  Serial.println(lastPulse);
  lowPulse = defaultPulse-25;
  highPulse = defaultPulse+25;
  Serial.println("Your pulse threshold are: (min, max)");
  Serial.println(lowPulse);
  Serial.println(highPulse);
  Serial.println("MASTER INIT DONE");
  return false;
}

