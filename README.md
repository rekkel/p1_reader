## **Slimme meter P1+ DMSR 5.0**

**Arduino mega** 
Uitlezen p1 poort serial1 pin 19
Doorzetten naar serial2 pin 16

DMSR 5.0 bericht wordt verrijkt met congestie gegevens

Checksum wordt bepaalt tussen / (start bericht) en ! (einde bericht)

De metingen :

EAN;U Fase1;U Fase3;U Fase2;I Fase1;I Fase2;I Fase3;I Fase1 levering;I Fase2 levering;I Fase3 levering;

bv: `12345678;239;223;223;44;24;67;0;0;0`

wordt verstuurd naar de Lora gateway.

Met het ontvangen bericht vanaf Lora wordt het P1 signaal verrijkt en doorgestuurd naar de P1+ Serial2 poort.

Gebruikte DMSR library:
https://github.com/matthijskooijman/arduino-dsmr.git

