#include <util/crc16.h>
#include <SPI.h>
#include <LoRa.h>

int DEBUG=0;
int ledPin = 12;
int ledPin2 = 7;

String send_p1plus_congestie = "";
String recieved_p1_congestie = " ";

//timer Stuf
#define BYTE unsigned char
#define USHORT unsigned short
#define LOCAL_CONGESTIE_DUUR   60000    // 1 x 60 x 1000 milliseconden
uint32_t timer;          // 32 bits timer

//Lora Stuf
bool initLora = true;
String values_to_lora_gateway[2]      = {};
int sendLoraCounter = 0;

//String send_p1plus_congestie = "";
char recieved_lna_congestie;
int recieved_lna_Message = 0;
String er_is_congestion = "0";
long totaal_aantal_berichten_van_p1 = 0;

String c;

String ean;
String U1;
String U2;
String U3;
String I1;
String I2;
String I3;
String T1;
String T2;
String T3;
String TarifAfnameH;
String TarifAfnameL;
String TarifTerugH;
String TarifTerugL;

char buf[1000];
int start_index, end_index = 0;   

long previousMillis = 0; 
long interval = 10000;  

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '/': // Ignore CR
                break;
            case '!': // Return on new-line       
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}
 

void startLora(){
  
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
  if(DEBUG) {
    Serial.print("Value send to Lora Gateway: "); Serial.println(message);
  }
  
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  LoRa_rxMode();                        // set rx mode

  
}


void LoRa_rxMode(){
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
}

void onReceive(int packetSize) {
  // received a packet
  //Serial.print("Received packet '");

  String phrase = "";
  // read packet
  for (int i = 0; i < packetSize; i++) {
    recieved_lna_congestie = (char)LoRa.read();
    //Serial.print(recieved_lna_congestie);
    phrase = String(phrase + recieved_lna_congestie);
  }
  
  if( phrase[0] != '<'){

    if(DEBUG) {
      Serial.print("phrase : "); Serial.println(phrase);
    }
  
    String recieved_ean = "EAN000000" + phrase.substring(0,8);
    if(DEBUG) {
      Serial.print("recieved_ean : "); Serial.println(recieved_ean);
    }
  
    phrase.replace( phrase.substring(0,8) , recieved_ean );
    if(DEBUG) {
      Serial.print("changed phrase : "); Serial.println(phrase);
    }
  
    send_p1plus_congestie = phrase;
    if(DEBUG) {
      Serial.print("send_p1plus_congestie : "); Serial.println(send_p1plus_congestie);
      Serial.println(send_p1plus_congestie.length());
    }
    
    if (recieved_ean.equals(String("EAN000000" + ean)) ){
       if( send_p1plus_congestie.length() > 23){
         if(DEBUG) {
          Serial.println("Congestie WEL voor mij ontvangen");
         }
         recieved_lna_Message = 1;
         // Start the timer
         timer = millis();
         
       } else if(send_p1plus_congestie.length() == 23) {
           if(DEBUG) {
            Serial.println("Einde Congestie vanuit DMS");
           }
           recieved_lna_Message = 0;
           timer = 0;
           er_is_congestion = "0";
       }

    } else {
      if(DEBUG) {
        Serial.println("Congestie NIET voor mij ontvangen");
      }
    }
  } else {
    if(DEBUG) {
      Serial.println("Bericht van andere node ontvangen");
    }
  }
}

void setup() {




  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin2, HIGH);
  delay(1000);
  digitalWrite(ledPin2, LOW);

  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  startLora();
  
  // For test
  //recieved_lna_Message = 1;

}


void loop() {

    resetNaTimer();

    if (readline(Serial1.read(), buf, 1000) > 0) {
        c += (String)buf;
    }

    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > interval) {
      if(DEBUG) {
        Serial.print("millis() - timer ");Serial.println(millis() - timer);
      }
      previousMillis = currentMillis;

      c = c.substring(6);
      start_index = c.indexOf("!");
      c = c.substring(0,start_index );
      //Serial.println(c);
 
      //values_to_lora_gateway = pars_data(c);
      pars_data(c);
      
      if(sendLoraCounter % 6 == 0)
        LoRa_sendMessage(values_to_lora_gateway[1]);
      else 
        LoRa_sendMessage(values_to_lora_gateway[0]);
      
      sendLoraCounter++;
      
      recieved_p1_congestie  = pars_congestie(c);
      if(DEBUG) {
        Serial.print("values_to_lora_gateway 1 =");Serial.println(values_to_lora_gateway[0]);
        Serial.print("values_to_lora_gateway 2 =");Serial.println(values_to_lora_gateway[1]);
        Serial.print("recieved_p1_congestie =");Serial.println(recieved_p1_congestie);
      }
      //Serial.print("recieved_lna_Message ");Serial.println(recieved_lna_Message);
      //Serial.print("timer ");Serial.println(timer);
      if (recieved_lna_Message == 1){
        c = generate_new_p1_str(c,send_p1plus_congestie);
      }

      c = "/" + c;
      c = c + "!";

      char copy_c[c.length()];
      c.toCharArray(copy_c, c.length());
     
      if(DEBUG) {
        Serial.println(c);
        Serial.println(crc16(copy_c, c.length()),HEX);
      }
      Serial2.println(c);
      Serial2.println(crc16(copy_c, c.length()),HEX);

      c = "";
      Serial.println("Bericht verstuurd naar P1+ ");

    }


}


String generate_new_p1_str( String frawdata, String send_p1plus_congestie){

   String old_congestion = "";
   String recieved_ean = "";
   
   int start_index = frawdata.indexOf("0-0:96.13.0(") + 46;
   int end_index   =  frawdata.indexOf(')', start_index + 1 ) ;

   //Serial.print("start_index    end_index  ");Serial.print(start_index);Serial.print("   ");Serial.println(end_index);

   old_congestion = frawdata.substring(start_index,end_index);
   //Serial.print("old_congestion ");Serial.println(old_congestion);
   
   recieved_ean = send_p1plus_congestie.substring(0,17) ;
   //Serial.print("recieved_ean ");Serial.println(recieved_ean);
   //Serial.print("send_p1plus_congestie ");Serial.println(send_p1plus_congestie);
   //Serial.print("send_p1plus_congestie.substring(17) ");Serial.println(send_p1plus_congestie.substring(17));
   //Serial.print("compare ");Serial.println("EAN000000" + ean);
   //Serial.print("recieved_ean ");Serial.println(recieved_ean);
   
     
   if (recieved_ean.equals("EAN000000" + ean) ){
     //Serial.println("ean is gelijk ");
     er_is_congestion = "1";
     digitalWrite(ledPin, HIGH);
         
     String send_p1plus_congestieLowerCase = decodeSTR(send_p1plus_congestie.substring(17));
     
     frawdata.replace(old_congestion,send_p1plus_congestieLowerCase);
     
   } else {
     if(DEBUG) {
      Serial.println("ean is NIET gelijk ");
     }
   }
   return frawdata ;  
}





void pars_data(String a){
   String val,TW1,TW2,TW3,congestie;
   String valarray[2] = {};
   int start_index, end_index = 0;   
   int tmpI1,tmpI2,tmpI3 = 0;
   int tmpU1,tmpU2,tmpU3 = 0;
   int tmpT1,tmpT2,tmpT3 = 0;
   
   //Serial.print("start_index    end_index  ");Serial.print(start_index);Serial.print("   ");Serial.println(end_index);
   
   start_index = a.indexOf("0-0:96.1.1(") + 11;
   end_index   =  a.indexOf(')', start_index + 1 ) ;
   ean = decodeHEX(a.substring(start_index,end_index));
   ean = ean.substring(9);

   start_index = a.indexOf("1-0:1.8.1(") + 10;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TarifAfnameH = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:1.8.2(") + 10;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TarifAfnameL = a.substring(start_index,end_index);
   
   start_index = a.indexOf("1-0:2.8.1(") + 10;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TarifTerugH = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:2.8.2(") + 10;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TarifTerugL = a.substring(start_index,end_index);
   
   
   //ean = "<" + ean + ">" 
   if(DEBUG) {
        Serial.print("ean ");Serial.println(ean);
   }


   start_index = a.indexOf("1-0:32.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   U1 = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:52.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   U2 = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:72.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   U3 = a.substring(start_index,end_index);

   if(DEBUG) {
      Serial.print("U1 ");Serial.println(U1);
      Serial.print("U2 ");Serial.println(U2);
      Serial.print("U3 ");Serial.println(U3);
   }

   start_index = a.indexOf("1-0:31.7.0(") + 11;
   end_index   =  a.indexOf('*', start_index + 1 ) ;
   I1 = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:51.7.0(") + 11;
   end_index   =  a.indexOf('*', start_index + 1 ) ;
   I2 = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:71.7.0(") + 11;
   end_index   =  a.indexOf('*', start_index + 1 ) ;
   I3 = a.substring(start_index,end_index);

   I1 = (String)I1.toInt();
   I2 = (String)I2.toInt();
   I3 = (String)I3.toInt();
   
   if(DEBUG) {
        Serial.print("I1 ");Serial.println(I1); 
        Serial.print("I2 ");Serial.println(I2); 
        Serial.print("I3 ");Serial.println(I3);
   }

   start_index = a.indexOf("1-0:21.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TW1 = a.substring(start_index,end_index);
 
   start_index = a.indexOf("1-0:41.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TW2 = a.substring(start_index,end_index);

   start_index = a.indexOf("1-0:61.7.0(") + 11;
   end_index   =  a.indexOf('.', start_index + 1 ) ;
   TW3 = a.substring(start_index,end_index);

   tmpT1 = round(TW1.toInt()/U1.toInt());
   tmpT2 = round(TW2.toInt()/U2.toInt());
   tmpT3 = round(TW3.toInt()/U3.toInt());
   T1 = (String)tmpT1;
   T2 = (String)tmpT2;
   T3 = (String)tmpT3;
   
   if(DEBUG) {
        Serial.print("T1 ");Serial.println(T1);
        Serial.print("T2 ");Serial.println(T2);
        Serial.print("T3 ");Serial.println(T3);
   }

   values_to_lora_gateway[0] = "<" + ean + ">;" + U1 + ";" + U2 + ";" + U3 + ";" + I1 + ";" + I2 + ";" + I3 + ";" + T1 + ";" + T2 + ";" + T3 + ";" + er_is_congestion            ;
   values_to_lora_gateway[1] = "<" + ean + ">;tarif;" + TarifAfnameH + ";" + TarifAfnameL + ";" + TarifTerugH + ";" + TarifTerugL            ;

   //return valarray;
}

String pars_congestie(String a){
   String val;
   int start_index, end_index = 0;   
   
   start_index = a.indexOf("0-0:96.13.0(") + 12;
   end_index   =  a.indexOf(')', start_index + 1 ) ;
   val = decodeHEX(a.substring(start_index,end_index));
   
   
   if(DEBUG) {
    Serial.print("val.length() ");Serial.println(val.length());
   }

   
   if(val.length() != 24) 
     digitalWrite(ledPin2, HIGH);
   else
     digitalWrite(ledPin2, LOW);

     
   return val;
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
  String temp2 = temp;
  temp2.toLowerCase();
  
  return temp2;
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


void resetNaTimer(){
  
  if (timer != 0) {
     // kijk of de timer verlopen is
     if ((millis() - timer) > LOCAL_CONGESTIE_DUUR ) {
        if(DEBUG) {
          Serial.println("Timer is verlopen....");
        }
        // timer is verlopen dus doe je ding
        er_is_congestion = "0";
        recieved_lna_Message = 0;
        timer = 0;
        digitalWrite(ledPin, LOW);

     }
  }
}
