## **Slimme meter P1+ DMSR 5.0**

**Arduino mega** 
Uitlezen p1 poort serial1 pin 19
Doorzetten naar serial2 pin 16

DMSR 5.0 bericht wordt verrijkt met congestie gegevens

Checksum wordt bepaalt tussen / (start bericht) en ! (einde bericht)

De metingen :

*voltage_l1*
*voltage_l2*
*voltage_l3*
*current_l1*
*current_l2*
*current_l3*
*power_returned_l1 omgerekend naar current*
*power_returned_l2 omgerekend naar current*
*power_returned_l3 omgerekend naar current*

EAN;U Fase1;U Fase3;U Fase2;I Fase1;I Fase2;I Fase3;I Fase1 levering;I Fase2 levering;I Fase3 levering;
bv: `12345678;239;223;223;44;24;67;0;0;0`

worden verstuurd naar de Lora gateway.

Het ontvangen bericht vanaf Lora wordt doorgestuurd naar e P1+ Serial2 poort.

Gebruikte DMSR library:
https://github.com/matthijskooijman/arduino-dsmr.git

