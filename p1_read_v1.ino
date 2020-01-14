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

bool initLora = true;



#define BYTE unsigned char
#define USHORT unsigned short

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


String to_loraGW = "";
String ean = "";
String sendCongestion = "87654321;23;45;56;8;9;0";

char recievedCongestion;

int recievedMessage = 0;
int Congestion = 0;



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
  // Allow the reader to check the serial buffer regularly
  reader.loop();

  // Every 1 sec, fire off a one-off reading
  unsigned long now = millis();
  if (now - last > 1000) {
    reader.enable(true);
    last = now;
  }

  if (reader.available()) {
    MyData data;
    
    String err;
    String new_rawData;
    String rawdata = reader.raw();

    if (reader.parse(&data, &err)) {

      ean = data.equipment_id.toInt();
      //Serial.print("ean");Serial.println(ean); 
      String spanning_l1 = (String)round(data.voltage_l1);
      String spanning_l2 = (String)round(data.voltage_l2);
      String spanning_l3 = (String)round(data.voltage_l3);
      String stroom_afname_l1 = (String)round(data.current_l1);
      String stroom_afname_l2 = (String)round(data.current_l2);
      String stroom_afname_l3 = (String)round(data.current_l3);
      String stroom_levering_l1 = (String)round(data.power_returned_l1/data.voltage_l1);
      String stroom_levering_l2 = (String)round(data.power_returned_l2/data.voltage_l2);
      String stroom_levering_l3 = (String)round(data.power_returned_l3/data.voltage_l3);

      /*    send values to lora gateway   */
      if(initLora){
        to_loraGW = "<" + ean          + ">;"                     +
                    spanning_l1        + ";" +spanning_l2         + ";" +spanning_l3          + ";" +
                    stroom_afname_l1   + ";" +stroom_afname_l2    + ";" +stroom_afname_l3     + ";" +  
                    stroom_levering_l1 + ";" + stroom_levering_l2 + ";" + stroom_levering_l3  + ";" + Congestion            ;                  

        LoRa_sendMessage(to_loraGW);
      
      }

      /*   Change free field with recieved congestion signal   */
      new_rawData = rawdata;
      if (recievedMessage == 1){
        new_rawData = generate_new_p1(rawdata, sendCongestion);
        recievedMessage = 0;
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
      
    } else {
      // Parser error, print error
      Serial.println(err);
    }
  }
}


String generate_new_p1( String frawdata, String sendCongestion){
   String fnew_Data = frawdata;
   String old_congestion = "";
   
   int start_index = fnew_Data.indexOf("0-0:96.13.0(") + 12;
   int end_index   =  fnew_Data.indexOf(')', start_index + 1 ) ;

   old_congestion = fnew_Data.substring(start_index,end_index);
   fnew_Data.replace(old_congestion,sendCongestion);

   //Serial.print("sendCongestion ");Serial.println(sendCongestion);
   //Serial.print("old_congestion ");Serial.println(old_congestion);

   return fnew_Data ;  
}

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received packet '");
  
  
  // read packet
  for (int i = 0; i < packetSize; i++) {
    recievedCongestion = (char)LoRa.read();
    Serial.print(recievedCongestion);
  }
  
  //Serial.print("recievedCongestion : "); Serial.println(recievedCongestion);

  sendCongestion = ean + ";25;25;25;0;0;0";
  Serial.print("sendCongestion : "); Serial.println(sendCongestion);
  
  // print RSSI of packet
  //Serial.print("' with RSSI ");
  //Serial.println(LoRa.packetRssi());

  recievedMessage = 1;
}

void LoRa_rxMode(){
  //LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  //LoRa.disableInvertIQ();               // normal mode
}
