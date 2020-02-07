/*
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Example that shows how to periodically read a P1 message from a
 * serial port and automatically print the result.
 * 
 * library:
 * 
 * https://github.com/matthijskooijman/arduino-dsmr.git
 * 
*/
#include "dsmr.h"
#include <util/crc16.h>
#include <SPI.h>
#include <LoRa.h>
//LED Stuf
#include <FastLED.h>
//ssd1306 stuf
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);



#define NUM_LEDS 4
#define BRIGHTNESS  100
#define DATA_PIN 3
#define CLOCK_PIN 13
CRGB leds[NUM_LEDS];

//timer Stuf
#define BYTE unsigned char
#define USHORT unsigned short
#define DUUR   120000    // 2 x 60 x 1000 milliseconden
uint32_t timer;          // 32 bits timer

//MQTT Stuf
bool initLora = true;
String to_loraGW      = "";
String ean            = "";
String sendCongestion = "";
char recievedCongestion;
int recievedMessage = 0;
int Congestion = 0;
long message_count = 0;

/**
 * Define the data we're interested in, as well as the datastructure to
 * hold the parsed data. This list shows all supported fields, remove
 * any fields you are not using from the below list to make the parsing
 * and printing code smaller.
 * Each template argument below results in a field of the same name.
 */
using MyData = ParsedData<
  /* String */ identification,
  /* String */ p1_version,
  /* String */ timestamp,
  /* String */ equipment_id,
  /* FixedValue */ energy_delivered_tariff1,
  /* FixedValue */ energy_delivered_tariff2,
  /* FixedValue */ energy_returned_tariff1,
  /* FixedValue */ energy_returned_tariff2,
  /* String */ electricity_tariff,
  /* FixedValue */ power_delivered,
  /* FixedValue */ power_returned,
  /* FixedValue */ electricity_threshold,
  /* uint8_t */ electricity_switch_position,
  /* uint32_t */ electricity_failures,
  /* uint32_t */ electricity_long_failures,
  /* String */ electricity_failure_log,
  /* uint32_t */ electricity_sags_l1,
  /* uint32_t */ electricity_sags_l2,
  /* uint32_t */ electricity_sags_l3,
  /* uint32_t */ electricity_swells_l1,
  /* uint32_t */ electricity_swells_l2,
  /* uint32_t */ electricity_swells_l3,
  /* String */ message_short,
  /* String */ message_long,
  /* FixedValue */ voltage_l1,
  /* FixedValue */ voltage_l2,
  /* FixedValue */ voltage_l3,
  /* FixedValue */ current_l1,
  /* FixedValue */ current_l2,
  /* FixedValue */ current_l3,
  /* FixedValue */ power_delivered_l1,
  /* FixedValue */ power_delivered_l2,
  /* FixedValue */ power_delivered_l3,
  /* FixedValue */ power_returned_l1,
  /* FixedValue */ power_returned_l2,
  /* FixedValue */ power_returned_l3,
  /* uint16_t */ gas_device_type,
  /* String */ gas_equipment_id,
  /* uint8_t */ gas_valve_position,
  /* TimestampedFixedValue */ gas_delivered,
  /* uint16_t */ thermal_device_type,
  /* String */ thermal_equipment_id,
  /* uint8_t */ thermal_valve_position,
  /* TimestampedFixedValue */ thermal_delivered,
  /* uint16_t */ water_device_type,
  /* String */ water_equipment_id,
  /* uint8_t */ water_valve_position,
  /* TimestampedFixedValue */ water_delivered,
  /* uint16_t */ slave_device_type,
  /* String */ slave_equipment_id,
  /* uint8_t */ slave_valve_position,
  /* TimestampedFixedValue */ slave_delivered
>;

/**
 * This illustrates looping over all parsed fields using the
 * ParsedData::applyEach method.
 *
 * When passed an instance of this Printer object, applyEach will loop
 * over each field and call Printer::apply, passing a reference to each
 * field in turn. This passes the actual field object, not the field
 * value, so each call to Printer::apply will have a differently typed
 * parameter.
 *
 * For this reason, Printer::apply is a template, resulting in one
 * distinct apply method for each field used. This allows looking up
 * things like Item::name, which is different for every field type,
 * without having to resort to virtual method calls (which result in
 * extra storage usage). The tradeoff is here that there is more code
 * generated (but due to compiler inlining, it's pretty much the same as
 * if you just manually printed all field names and values (with no
 * cost at all if you don't use the Printer).
 */
struct Printer {
  template<typename Item>
  void apply(Item &i) {
    if (i.present()) {
      Serial.print(Item::name);
      Serial.print(F(": "));
      Serial.print(i.val());
      Serial.print(Item::unit());
      Serial.println();
    }
  }
};



// Set up to read from the second serial port, and use D2 as the request
// pin. On boards with only one (USB) serial port, you can also use
// SoftwareSerial.
#ifdef ARDUINO_ARCH_ESP32
// Create Serial1 connected to UART 1
HardwareSerial Serial1(1);
#endif
P1Reader reader(&Serial1, 2);

unsigned long last;

void setup() {
  // timer setup
  delay(5);
  //timer = millis();
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  #ifdef VCC_ENABLE
  // This is needed on Pinoccio Scout boards to enable the 3V3 pin.
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  #endif

  // start a read right away
  reader.enable(true);
  last = millis();

  if(initLora){
    Serial.println("Start Lora");
    if (!LoRa.begin(868000000)) { // Match to Gateway frequency setting
      Serial.println("Starting LoRa failed!");
      while (1);
    }
 
    LoRa.setSyncWord(0x34); // Match to Gateway sync setting
    LoRa.onReceive(onReceive); 
    LoRa_rxMode();
  }

  //Led Stuf
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );
  groen();
  
  //SSD1306 Stuf
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  
}

void resetNaTimer(){
  
  if (timer != 0) {
     //Serial.print("millis timer millis-Timer");Serial.print(millis());Serial.print("   ");Serial.print(timer);Serial.print("    ");Serial.println(millis()-timer);
     // kijk of de timer verlopen is
     if ((millis() - timer) > DUUR ) {
        Serial.println("Timer is verlopen....");
        // timer is verlopen dus doe je ding
        Congestion = 0;
        recievedMessage = 0;
        timer = 0;
        groen();
     }
  }
}


void LoRa_sendMessage(String message) {
  Serial.print("Send to Lora: "); Serial.println(message);
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  LoRa_rxMode();                        // set rx mode
}

void loop () {
  
  resetNaTimer();
  
  
  // Allow the reader to check the serial buffer regularly
  reader.loop();

  // Every 1 sec, fire off a one-off reading
  unsigned long now = millis();
  if (now - last > 5000) {
    reader.enable(true);
    
    last = now;
  }

  if (reader.available()) {
    MyData data;
    
    String err;
    String new_rawData;
    String rawdata = reader.raw();

    if (reader.parse(&data, &err)) {

      ean = decodeHEX(data.equipment_id);
      int len = ean.length();
      ean = ean.substring(len - 9, len);
      
      Serial.print("ean           ");Serial.println(ean); 

      //String recieved_congestie = "EAN00000012345678;;;;;;";
      String recieved_congestie = decodeHEX(data.message_long);
      Serial.print("recieved_congestie      ");Serial.println(recieved_congestie); 
      Serial.print("recieved_congestie  len ");Serial.println(recieved_congestie.length()); 

      if (recievedMessage != 1){
        if ( recieved_congestie.length() == 24){
          groen();
        } else {
          oranje();
        }
      }

      
      String spanning_l1 = (String)round(data.voltage_l1);
      String spanning_l2 = (String)round(data.voltage_l2);
      String spanning_l3 = (String)round(data.voltage_l3);
      String stroom_afname_l1 = (String)round(data.current_l1);
      String stroom_afname_l2 = (String)round(data.current_l2);
      String stroom_afname_l3 = (String)round(data.current_l3);
      String stroom_levering_l1 = (String)round(data.power_returned_l1/data.voltage_l1);
      String stroom_levering_l2 = (String)round(data.power_returned_l2/data.voltage_l2);
      String stroom_levering_l3 = (String)round(data.power_returned_l3/data.voltage_l3);


      /*   Change free field with recieved congestion signal   */
      new_rawData = rawdata;
      Serial.print("recievedMessage ");
      Serial.println(recievedMessage);
      
      
      
      if (recievedMessage == 1){
        Serial.println("a. start");
        new_rawData = generate_new_p1(String(rawdata),String(sendCongestion));
        //recievedMessage = 0;
      }
      
      new_rawData += "\r";
      Serial.print("/");
      Serial2.print("/");
      Serial.print(new_rawData);
      Serial2.print(new_rawData);
      
      char copy_rawdata[new_rawData.length()];
      new_rawData.toCharArray(copy_rawdata, new_rawData.length());
      
      Serial.print("!");  Serial.println(crc16(copy_rawdata, new_rawData.length()),HEX);
      Serial2.print("!"); Serial2.println(crc16(copy_rawdata, new_rawData.length()),HEX);   


      /*    send values to lora gateway   */
      if(initLora){
        to_loraGW = "<" + ean          + ">;"                     +
                    spanning_l1        + ";" +spanning_l2         + ";" +spanning_l3          + ";" +
                    stroom_afname_l1   + ";" +stroom_afname_l2    + ";" +stroom_afname_l3     + ";" +  
                    stroom_levering_l1 + ";" + stroom_levering_l2 + ";" + stroom_levering_l3  + ";" + Congestion            ;                  

        LoRa_sendMessage(to_loraGW);
      
      }

      
    } else {
      // Parser error, print error
      Serial.println(err);
    }
    

      
      testscrolltext(floatAlignRigiht(message_count),ean);
      message_count++;
      if(message_count == 9999999)
        message_count=1;
    
  }
}


String generate_new_p1( String frawdata, String sendCongestion){
   String fnew_Data = frawdata;
   String old_congestion = "";
   String recieved_ean = "";
   
   fnew_Data.replace('\r','<');
   fnew_Data.replace('\n','>');
   
   //fnew_Data.replace("3b3b3b3b3b3b","3b32353b3b3b3b3b");
 
   int start_index = fnew_Data.indexOf("0-0:96.13.0(") + 46;
   int end_index   =  fnew_Data.indexOf(')', start_index + 1 ) ;

   Serial.print("start_index    end_index  ");Serial.print(start_index);Serial.print("   ");Serial.println(end_index);
   //Serial.print("fnew_Data   ");Serial.println(fnew_Data);

 

   

   old_congestion = fnew_Data.substring(start_index,end_index);
   Serial.print("old_congestion ");Serial.println(old_congestion);
   
   recieved_ean = sendCongestion.substring(0,17) ;
   Serial.print("recieved_ean ");Serial.println(recieved_ean);
   
   Serial.print("sendCongestion ");Serial.println(sendCongestion);
   
   Serial.print("sendCongestion.substring(17) ");Serial.println(sendCongestion.substring(17));
  
   Serial.print("compare ");Serial.println("EAN000000" + ean);
   Serial.print("recieved_ean ");Serial.println(recieved_ean);
   
     
   if (recieved_ean.equals("EAN000000" + ean) ){
     Serial.println("ean is gelijk ");
     
     Congestion = 1;
     fnew_Data.replace(old_congestion,decodeSTR(sendCongestion.substring(17)));

   } else {
     Serial.println("ean is NIET gelijk ");
   }

   fnew_Data.replace('<','\r');
   fnew_Data.replace('>','\n');

   return fnew_Data ;  
}

char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}


String generate_new_p1_new( String frawdata, String sendCongestion){
   
   char fnewdata[1024];
   char frawdatatmp[1024];
   char fsendCongestiontmp[1024];
   String recieved_ean;

   Serial.println("z. start");

   strcpy(frawdatatmp, string2char(frawdata));
   Serial.print("0. frawdatatmp [");Serial.print(frawdatatmp);Serial.println("]");
   strcpy(fsendCongestiontmp, string2char(sendCongestion));
   Serial.print("01. fsendCongestiontmp [");Serial.print(fsendCongestiontmp);Serial.println("]");
   /*
   int len = frawdata.length() + 1;
   Serial.print("a. len = [");Serial.print(len);Serial.println("]");
   if(len >= sizeof(frawdatatmp)-1) len = sizeof(frawdatatmp) - 1;
   Serial.print("b. len = [");Serial.print(len);Serial.println("]");
   frawdata.toCharArray(frawdatatmp, len);
   Serial.print("0. frawdatatmp [");Serial.print(frawdatatmp);Serial.println("]");
   
   len = sendCongestion.length() + 1;
   if(len >= sizeof(fsendCongestiontmp)-1) len = sizeof(fsendCongestiontmp) - 1;
   sendCongestion.toCharArray(fsendCongestiontmp, len);
   Serial.print("01. fsendCongestiontmp [");Serial.print(fsendCongestiontmp);Serial.println("]");
   */
   
   strcpy(fnewdata, "");
   char *pfindstartstr;
   pfindstartstr = strstr(frawdatatmp,"0-0:96.13.0(");
   if(pfindstartstr != (char*)NULL)
   {
     pfindstartstr += 12;
     Serial.print("1. pfindstartstr [");Serial.print(pfindstartstr);Serial.println("]");
     memcpy(fnewdata, frawdatatmp, (pfindstartstr-frawdatatmp+1));
     //Serial.println("ean is gelijk ");
     char *pfindendstr;
     pfindendstr = strchr(pfindstartstr,')');
     if(pfindendstr != (char*)NULL)
     {
      Serial.print("2. pfindendstr [");Serial.print(pfindendstr);Serial.println("]");
      strcpy(fnewdata+(pfindstartstr-frawdatatmp-1), fsendCongestiontmp);
      Serial.print("3. fnewdata [");Serial.print(fnewdata);Serial.println("]");
      strcat(fnewdata, pfindendstr); 
      Serial.print("4. fnewdata [");Serial.print(fnewdata);Serial.println("]");
     }
   }
   
   Serial.print("sendCongestion ");Serial.println(sendCongestion);

   
   if (recieved_ean.equals("EAN000000" + ean) ){
     Serial.println("ean is gelijk ");
     
     Congestion = 1;
     

   } else {
     Serial.println("ean is NIET gelijk ");
   }

   //return fnewdata ;  
   
   return frawdata ;  
}

void onReceive(int packetSize) {
  // received a packet
  //Serial.print("Received packet '");

  String phrase = "";
  // read packet
  for (int i = 0; i < packetSize; i++) {
    recievedCongestion = (char)LoRa.read();
    //Serial.print(recievedCongestion);
    phrase = String(phrase + recievedCongestion);
  }

  
  if( phrase[0] != '<'){

    
  
    Serial.print("phrase : "); Serial.println(phrase);
  
    String recieved_ean = "EAN000000" + phrase.substring(0,8);
    Serial.print("recieved_ean : "); Serial.println(recieved_ean);
  
    phrase.replace( phrase.substring(0,8) , recieved_ean );
    Serial.print("changed phrase : "); Serial.println(phrase);
  
    sendCongestion = phrase;
    Serial.print("sendCongestion : "); Serial.println(sendCongestion);
    Serial.println(sendCongestion.length());

    
    if (recieved_ean.equals(String("EAN000000" + ean)) ){
       if( sendCongestion.length() > 23){
         Serial.println("Congestie WEL voor mij ontvangen");
         rood();
         recievedMessage = 1;
         // Start the timer
         timer = millis();
       } else if(sendCongestion.length() == 23) {
           Serial.println("Einde Congestie vanuit DMS");
           groen();
           recievedMessage = 0;
           timer = 0;
           Congestion = 0;
       }


    } else {
      Serial.println("Congestie NIET voor mij ontvangen");
    }

    //print RSSI of packet
    //Serial.print("' with RSSI ");
    //Serial.println(LoRa.packetRssi());
    
  } else {
    Serial.println("Bericht van andere node ontvangen");
  }
}

void LoRa_rxMode(){
  //LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  //LoRa.disableInvertIQ();               // normal mode
}


String decodeHEX(String ean_hex){

  String  temp;
  String ean_bytes = ean_hex;
  char c;
  int stringlen = ean_bytes.length() +1 ;
  char str[stringlen];
  ean_bytes.toCharArray(str, stringlen);

  char hex[5] = {0};
  hex[0] = '0';
  hex[1] = 'X';

  for (int i = 0; i < sizeof(str) ; i += 2) {
    hex[2] = str[i];
    hex[3] = str[i+1];
    int h = strtol(hex, NULL, 16);
    c = toascii(h);
    temp = String(temp + c);
  }
  return temp;
}

String decodeSTR(String ean){
  String  temp;
  String ean_bytes = ean;
  int stringlen = ean_bytes.length() +1 ;
  char ean_char_array[stringlen];
  ean_bytes.toCharArray(ean_char_array, stringlen);

  int x=0;
  for (x=0; x < stringlen -1 ; x++){
    if (x == stringlen){
        ean_char_array[x] = '\0';
    } else {
      char c = ean_char_array[x];
      uint8_t number = ( c );
      char Buffer[3];
      snprintf(Buffer, sizeof(Buffer), "%02X", number);
      temp = String(temp + Buffer);
    }
  }
  return temp;
}

USHORT crc16(const BYTE *data_p, int length)
{
  int pos;
  int i;
  USHORT crc = 0x0;

  for (pos = 0; pos < length; pos++)
  {
    crc ^= (USHORT) data_p[pos];

    for (i = 0; i < 8; i++)
    {
      if ((crc & 0x0001) == 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}


//LED Stuf
void rood(){
    leds[0] = CRGB::Blue;
    leds[1] = CRGB::Blue;
    leds[2] = CRGB::Blue;
    leds[3] = CRGB::Blue;
    FastLED.show();
}

void groen(){
    leds[0] = CRGB::Green;
    leds[1] = CRGB::Green;
    leds[2] = CRGB::Green;
    leds[3] = CRGB::Green;
    FastLED.show();
}

void oranje(){
    leds[0] = CRGB::Purple;
    leds[1] = CRGB::Purple;
    leds[2] = CRGB::Purple;
    leds[3] = CRGB::Purple;
    FastLED.show();
}


//SSD1306 Stuf
void testscrolltext(String message, String EAN) {
  
  display.clearDisplay();
  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,5);
  display.println(message);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10,25);
  display.println(EAN);

  display.display();
}

String floatAlignRigiht ( long num )
{
  int    space  = 0;
  String spaces = "";
 
  //how large is this number?
  //always assumes value has 2 digits after the decimal point.
  if ( num < 10 ) space = 6; //10=2chars. 12+2=14, etc
  else if ( num > 9 && 100 > num ) space = 5;
  else if ( num >= 100 && 1000 > num) space = 4;
  else if ( num >= 1000 && 10000 > num) space = 3;
  else if ( num >= 10000 && 100000 > num) space = 2;
  else if ( num >= 100000 && 1000000 > num) space = 1;
  else if ( num >= 1000000 && 10000000 > num) space = 0;
 
  //add the correct amount of spaces infront of our value
  for ( uint8_t s=0; s<space; s++ ) spaces += F(" ");
 
  //return value (or your code here)
  return spaces + String ( num );
}
