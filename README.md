# Sniffer für Aqotec RM01 Fernwärme-Station

Aqotec FW-Stationen können von einer Heizzentrale abgefragt werden und "verraten" so ihre aktuellen Betriebsdaten über eine serielle Schnittstelle (2400Bd,8N1).

Dieses Programm decodiert die Daten, die die FW-Station an die Zentrale liefert, um sie auch lokal in einem Smarthome-System verarbeiten zu können.

## Doku

Welche Doku? Just read the code...

## Hardware-Anschluss

Habe ich Über einen BC547 als Treiber, eine IR-LED als Sender und einen USB IR Optokopf potentialfrei gemacht. 

Der 10-polige Pfostenstecker für das Flachbandkabel hat folgende Belegung (von vorne draufgeschaut):

```
10      9       8       7       6
GND                           FssTx

+5V          MbusRx           FssRx
1       2       3       4       5
```

Dieses Projekt unterstützt nur das Mithören an FssTx . Der Trigger für das Versenden der Daten kommt über FssRx von der Zentrale.