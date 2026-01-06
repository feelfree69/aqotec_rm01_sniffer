# Sniffer für Aqotec RM01 Fernwärme-Station

Aqotec FW-Stationen können von einer Heizzentrale abgefragt werden und "verraten" so ihre aktuellen Betriebsdaten über eine serielle Schnittstelle (2400Bd,8N1).

Dieses Programm decodiert die Daten, die die FW-Station an die Zentrale liefert, um sie auch lokal in einem Smarthome-System verarbeiten zu können.

## Doku

Welche Doku? Just read the code...

## Hardware-Anschluss

Habe ich Über einen BC557 als Treiber, eine IR-LED als Sender und einen USB IR Optokopf potentialfrei gemacht. 

Der 10-polige Pfostenstecker für das Flachbandkabel hat folgende Belegung (von vorne draufgeschaut):

```
10      9       8       7       6
GND                           FssTx

+5V          MbusRx           FssRx
1       2       3       4       5
```

Das urpünglich Projekt unterstützt nur das Mithören an FssTx. Der Trigger für das Versenden der Daten kommt über FssRx von der Zentrale.

Updates: 
- Mit der Implementierung in fss_with_tx.cpp wird auch selbst gepollt, falls es die Heizzentrale nicht tut.
- `tasmota_meter_definition.tas` ist eine alternative Implementierung für tasmota sml, das eigetnlich hauptsächlich für Stromzähler verwendet wird. War bei mir zu Versuchszwecken kurz mit einem ESP32-S3 und einem Image von 
https://github.com/ottelo9/tasmota-sml-images installiert-
- `esphome_config.yaml`ist eine alternative Implementierung für esphome. Dort sind auch noch ein paar DS1820B 1-wire-Temperatursensoren eingebunden. Das benutze ich aktuell wegen der guten Integration in HomeAssistant.
