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

uint16_t calcCRC(char* str)
{
  uint16_t crc=0; // starting value as you like, must be the same before each calculation
  for (int i=0;i<strlen(str);i++) // for each character in the string
  {
    crc= _crc16_update (crc, str[i]); // update the crc value
  }
  return crc;
}

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
String ean = "87654321";

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
    String rawdata = reader.raw();
 
    if (reader.parse(&data, &err)) {
            
      to_loraGW = ean + ";" +
                 (String)round(data.voltage_l1) + ";"+(String)round(data.voltage_l3) + ";"+(String)round(data.voltage_l3) + ";" +
                 (String)round(data.current_l1) + ";"+(String)round(data.current_l2) + ";"+(String)round(data.current_l3) + ";"  +  
                 (String)round(data.power_returned_l1/data.voltage_l1) + ";" +
                 (String)round(data.power_returned_l2/data.voltage_l2) + ";" +
                 (String)round(data.power_returned_l3/data.voltage_l3) ;                  
      
      if(initLora){
        LoRa_sendMessage(to_loraGW);
      }
      rawdata += to_loraGW;
      rawdata += "\r\n";
      
      
      Serial.print("/");
      Serial2.print("/");
      Serial.print(rawdata);
      Serial2.print(rawdata);
      
      char copy_rawdata[rawdata.length()];
      rawdata.toCharArray(copy_rawdata, rawdata.length());

      //Serial.println(calcCRC(copy_rawdata),HEX);
      //Serial2.println(calcCRC(copy_rawdata),HEX);

      Serial.print("!");  Serial.println(crc16(copy_rawdata, rawdata.length()),HEX);
      Serial2.print("!"); Serial2.println(crc16(copy_rawdata, rawdata.length()),HEX);
      
      
    } else {
      // Parser error, print error
      Serial.println(err);
    }
  }
}


void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received packet '");
  char message;
  
  // read packet
  for (int i = 0; i < packetSize; i++) {
    message = (char)LoRa.read();
    Serial.print(message);
  }

  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}

void LoRa_rxMode(){
  //LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  //LoRa.disableInvertIQ();               // normal mode
}


/*
/DSMR5 P1 Emulator

1-3:0.2.8(50)
0-0:1.0.0(191218164359W)
0-0:96.1.1(0000000000000000000000000000000123)
1-0:1.8.1(026009.500*kWh)
1-0:1.8.2(026009.500*kWh)
1-0:2.8.1(000000.000*kWh)
1-0:2.8.2(000000.000*kWh)
0-0:96.14.0(0001)
1-0:1.7.0(52019.000*kW)
1-0:2.7.0(00.000*kW)
0-0:96.7.21(8)
0-0:96.7.9(2)
1-0:99.97.0(2)
1-0:32.32.0(00001)
1-0:52.32.0(00001)
1-0:72.32.0(00002)
1-0:32.36.0(00000)
1-0:52.36.0(00000)
1-0:72.36.0(00000)
0-0:96.13.0(7b22486561646572223a7b22416374696f6e223a22454247446972656374436f6e74726f6c222c2256616c696446726f6d223a2231393130313031323030303053222c2256616c6964546f223a2231393130313031323135303053227d2c22426f6479223a7b22506572696f6473223a5b7b225374617274223a2231393130313031313539303053222c224d78507772223a3230307d2c7b225374617274223a2231393130313031323031303053222c224d78507772223a3131302c224d785068617365496d62616c223a3230302c224d7850777246616c6c62636b223a3630302c22536574506f696e7446616c6c62636b223a38302c2252656672657368496e74657276223a32307d2c7b225374617274223a2231393130313031323031333053222c224c6f6164526564756374223a38307d2c7b225374617274223a2231393130313031323031343053222c224d78507772223a313030302c224d7850777246616c6c62636b223a3830307d2c7b225374617274223a2231393130313031323033303053222c224d785068617365496d62616c223a3430302c224d78507772223a3135307d2c7b225374617274223a2231393130313031323033343053222c224c6f6164526564756374223a39397d2c7b225374617274223a2231393130313031323035303053222c224c6f6164526564756374223a3130307d2c7b225374617274223a2231393130313031323036303053222c2252656672657368496e74657276223a36302c224d7850777246616c6c62636b223a3131307d5d7d7d)
1-0:32.7.0(226.0*V)
1-0:52.7.0(233.0*V)
1-0:72.7.0(233.0*V)
1-0:31.7.0(105*A)
1-0:51.7.0(089*A)
1-0:71.7.0(032*A)
1-0:21.7.0(23730.000*kW)
1-0:41.7.0(20737.000*kW)
1-0:61.7.0(7552.000*kW)
1-0:22.7.0(00.000*kW)
1-0:42.7.0(00.000*kW)
1-0:62.7.0(00.000*kW)
0-1:24.1.0(003)
0-1:96.1.0(3232323241424344313233343536373839)
0-1:24.2.1(181106140010W)(01785.123*m3)
!BBCA


identification: SMR5 P1 Emulator
p1_version: 50
timestamp: 191218164550W
equipment_id: 0000000000000000000000000000000123
energy_delivered_tariff1: 15572.00kWh
energy_delivered_tariff2: 15572.00kWh
energy_returned_tariff1: 0.00kWh
energy_returned_tariff2: 0.00kWh
electricity_tariff: 0001
power_delivered: 31144.00kW
power_returned: 0.00kW
electricity_failures: 8
electricity_long_failures: 2
electricity_failure_log: (2)
electricity_sags_l1: 1
electricity_sags_l2: 1
electricity_sags_l3: 2
electricity_swells_l1: 0
electricity_swells_l2: 0
electricity_swells_l3: 0
message_long: 7b22486561646572223a7b22416374696f6e223a22454247446972656374436f6e74726f6c222c2256616c696446726f6d223a2231393130313031323030303053222c2256616c6964546f223a2231393130313031323135303053227d2c22426f6479223a7b22506572696f6473223a5b7b225374617274223a2231393130313031313539303053222c224d78507772223a3230307d2c7b225374617274223a2231393130313031323031303053222c224d78507772223a3131302c224d785068617365496d62616c223a3230302c224d7850777246616c6c62636b223a3630302c22536574506f696e7446616c6c62636b223a38302c2252656672657368496e74657276223a32307d2c7b225374617274223a2231393130313031323031333053222c224c6f6164526564756374223a38307d2c7b225374617274223a2231393130313031323031343053222c224d78507772223a313030302c224d7850777246616c6c62636b223a3830307d2c7b225374617274223a2231393130313031323033303053222c224d785068617365496d62616c223a3430302c224d78507772223a3135307d2c7b225374617274223a2231393130313031323033343053222c224c6f6164526564756374223a39397d2c7b225374617274223a2231393130313031323035303053222c224c6f6164526564756374223a3130307d2c7b225374617274223a2231393130313031323036303053222c2252656672657368496e74657276223a36302c224d7850777246616c6c62636b223a3131307d5d7d7d
voltage_l1: 239.00V
voltage_l2: 223.00V
voltage_l3: 223.00V
current_l1: 44A
current_l2: 24A
current_l3: 67A
power_delivered_l1: 10516.00kW
power_delivered_l2: 5352.00kW
power_delivered_l3: 15276.00kW
power_returned_l1: 0.00kW
power_returned_l2: 0.00kW
power_returned_l3: 0.00kW
gas_device_type: 3
gas_equipment_id: 3232323241424344313233343536373839
gas_delivered: 1785.12m3
12345678;239;223;223;44;24;67;0;0;0

*/
