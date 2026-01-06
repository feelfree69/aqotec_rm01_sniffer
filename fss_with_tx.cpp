#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

/* serial port */
#include <fcntl.h>
#include <sys/ioctl.h>
#include "termios.h"

using namespace std;

static int fd;
string device = "/dev/ttyUSB0";

static const char* dataFile;
ofstream of;

int set_interface_attribs (int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                cerr << "error" << errno << " from tcgetattr" << endl;
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;			// read blocks for at least 1 characters
        tty.c_cc[VTIME] = 50;           // 5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                cerr << "error" << errno << "from tcsetattr" << endl;
                return -1;
        }
        return 0;
}


int openDevice(string device)
{
	fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		cout << "open (" << device << ") returned " << strerror(errno);
		return -1;
	}

	set_interface_attribs (B2400, 0);

	return fd;
};

class temperatureT
{
    public:
    temperatureT(uint8_t* pt, int precision=10) 
    {
		if (*(pt+1) & 0x80)
		{
			*pt ^= 0xFF;
			(*pt+1) ^= 0xFF;
			m_t = -(*pt + *(pt+1) * 256);
		}
		else
		{
			m_t = *pt + *(pt+1) * 256; 
		}		
        m_precision=precision;
    };
    int m_t;
    int m_precision;

    friend ostream& operator<<(ostream& os, const temperatureT& t)
    {
		int floor = t.m_t/t.m_precision;
		int ceil = t.m_t%t.m_precision;
		if (t.m_precision == 100) 
		{ 
			ceil = (ceil+5)/10; // limit precision to 0.1°C
			if (ceil == 10) 
			{ 
				ceil = 0; 
    			floor -= 1;
			}
		}
        os << setfill(' ') << setw(2) << dec << floor << "." << setw(1) << ceil << " °C";
        return os;
    }
};

class onOffFlagT
{
    public:
    onOffFlagT(int flag) 
    {
        m_flag = flag; 
    };
    int m_flag;

    friend ostream& operator<<(ostream& os, const onOffFlagT& flag)
    {
		if (flag.m_flag)
			os << dec << flag.m_flag;
		else
			os << "0";
	    return os;
    }
};

void resetComPorts(void)
{
	close(fd);
	fd = openDevice(device);
	
	// ugly hack: also bring vzlogger back to life
	// system("/usr/bin/systemctl restart vzlogger");
}

void pollForData(void)
{
	uint8_t pollData[] = { 0x80, 0x00, 0x50, 0x00, 0x29, 0x4a, 0x01, 0x01, 0x64, 0x41};
	int sent = write(fd, &pollData, sizeof(pollData));
	cout << "Sent " << sent << " bytes for polling" << std::endl;
}

int readNBytes(uint8_t* buf, int N)
{

	int totalBytesRead = 0;
	int timeoutCount = 0;
	do {
		int bytesRead = read(fd, &buf[totalBytesRead], N-totalBytesRead);
		if (bytesRead == 0) 
		{
		    // Workaround for "usb_serial_generic_read_bulk_callback - urb stopped: -32" problem
			// Without closing/re-opening the device, the port will send no data anymore
			if (++timeoutCount > 360/15)
			{
				cout << "No data for 6 minutes; re-opening COM-port..."<< endl;
				resetComPorts();
				totalBytesRead = 0;
				timeoutCount = 0;
				pollForData();
			}
			continue;
		}
		else if (bytesRead < 0)
		{
			cout << "readNBytes returned " << bytesRead << ", re-opening COM-port..." << endl;
			resetComPorts();
			bytesRead = 0;
			totalBytesRead = 0;
			timeoutCount = 0;
			continue;
		}
		totalBytesRead += bytesRead;
	} while (totalBytesRead < N);

	return totalBytesRead;
}

int readUntilTimeout(uint8_t* buf, int N)
{
	int totalBytesRead = 0;
	do {
		int bytesRead = read(fd, &buf[totalBytesRead],  N-totalBytesRead);
		if (bytesRead <= 0) 
		{
			break;
		}
		totalBytesRead += bytesRead;
	} while (totalBytesRead < N);

	return totalBytesRead;
}

void printRawData(uint8_t* data, int len)
{
    for (int k=0; k<len; k++)
    {
        cout << " " << setfill('0') << setw(2) << hex << (int) data[k];
        if (((k+1) % 8) == 0) cout << " ";
        if (((k+1) % 64) == 0) cout << endl;
	}
}

bool checkRawData(uint8_t* data, int len)
{
    uint8_t checksum_l = 0;
    uint8_t checksum_h = 0;

    for (int k=0; k<len-2; k++)
    {
     	checksum_l ^= data[k];
	}

 	checksum_h = checksum_l;
    checksum_h ^= data[len-2];
    checksum_h ^= data[len-1];

	if ((checksum_l == data[len-2]) && (checksum_h == data[len-1]))
	{
		cout << " Checksum OK!" << endl << endl;
		return true;
	}
	else
	{
		cout << " INVALID CHECKSUM: " << hex << setw(2) << setfill('0') << (int) checksum_l << " " << setw(2) << setfill('0') << (int) checksum_h << endl << endl;
		return false;
	}
}

void checkAndPrintOperatingDataChanges(uint8_t* data)
{
	static bool firstStart = true;
	static uint8_t oldData[279];
    static bool changes[279];

	if (!firstStart)
	{
		cout << endl << "Aenderungen seit Start: ";
		for (int i=0; i<279; i++)
		{
			if (oldData[i] != data[i]) 
			{
				changes[i] = true;
			}

			if (changes[i])
			{
				cout << i << " ";
			}
		}
	}

	cout << endl;

	memcpy(oldData, data, 279);
    firstStart = false;
}

void outputOperatingData(uint8_t* data)
{
    temperatureT at(&data[6]);
    temperatureT priRLs(&data[8]);
    temperatureT secVL(&data[10]);
    temperatureT secRL(&data[12]);
    temperatureT secVLsoll(&data[14]);
	temperatureT priRLMax(&data[16]);
	onOffFlagT   boilerLadung = data[18];
	temperatureT fbhVL(&data[38]);
	temperatureT fbhVLsoll(&data[40]);
	onOffFlagT   fbhFlag = data[48];
	temperatureT boiler(&data[136]);
	temperatureT boilerLadeSoll(&data[140]);
	temperatureT zirk(&data[144]);
    temperatureT at_smooth(&data[158]);
	temperatureT zirkSoll(&data[164]);
	onOffFlagT   zirkFlag = data[166];

	uint16_t uhrzeit = *((uint16_t*) (&data[175]));
	uint8_t wd = data[177];
	uint8_t dd = data[178];
	uint8_t mm = data[179];
	uint8_t yy = data[180];

	uint32_t totalVol   = *((uint32_t*) (&data[197]));
	uint32_t leistung   = *((uint32_t*) (&data[201]));
 	uint32_t zaehler    = *((uint32_t*) (&data[205])); // just a guess, 0 anyway
	uint32_t serialNo   = *((uint32_t*) (&data[209]));
	uint32_t durchfluss = *((uint32_t*) (&data[213]));
	temperatureT priRL(&data[225], 100);
	temperatureT priVL(&data[227], 100);
	temperatureT spreizung(&data[229], 100);
	
	static const string WeekDay[8] = { "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag", "Sonntag" };
	if ((wd >= 1) && (wd <=7)) cout << WeekDay[wd-1] << ", ";
	cout << dec << setw(2) << setfill('0') << (int) dd << "." << setw(2) << setfill('0') << (int) mm << "." << setw(2) << setfill('0') << (int) yy;
	cout << ", " << setw(2) << setfill('0') << uhrzeit/100 << ":"  << setw(2) << setfill('0') << uhrzeit%100 << " Uhr" << endl;

	cout << " Außen:  " << at        << ", gefiltert:   " << at_smooth << endl;
	cout << " Pri-VL: " << priVL     << ", Pri-RL:      " << priRL          << ", Pri-RL(st):   " << priRLs << ", Pri-RL-max: " << priRLMax << endl;
	cout << " Sek-VL: " << secVL     << ", Sek-VL-Soll: " << secVLsoll      << ", Sek-RL:       " << secRL << endl;
	cout << " FBH-VL: " << fbhVL     << ", FBH-VL-Soll: " << fbhVLsoll      << ", FBH-Status:   " << fbhFlag << endl;
	cout << " Boiler: " << boiler    << ", Lade-Soll:   " << boilerLadeSoll << ", Lade-Status:  " << boilerLadung << endl;
	cout << " Zirku:  " << zirk      << ", Zirk-Soll:   " << zirkSoll       << ", Zirk-Status:  " << zirkFlag << endl;
	cout << " Spreiz: " << spreizung << ", Durchfluss: " << setfill(' ') << setw(3) << durchfluss << "  lph, Leistung:     " << leistung << " W" << endl;
	cout << " TotalVolume: " << totalVol << " l, SerialNo: " << serialNo << endl;

	if (dataFile)
	{
		of.open(dataFile);
		of << endl;
		of << "Datum " << dec << setw(2) << setfill('0') << (int) dd << "." << setw(2) << setfill('0') << (int) mm << "." << setw(2) << setfill('0') << (int) yy << endl;
		of << "Uhrzeit " << setw(2) << setfill('0') << uhrzeit/100 << ":"  << setw(2) << setfill('0') << uhrzeit%100 << " Uhr" << endl;
		of << "AT " << at << endl;
		of << "PriVL " << priVL << endl;			// vom Zähler
		of << "PriRL " << priRL << endl;			// vom Zähler
		of << "Spreizung " << spreizung << endl;	// vom Zähler
		of << "Leistung " << leistung << endl;		// vom Zähler
		of << "Durchfluss " << durchfluss << endl;	// vom Zähler
		of << "SekVL " << secVL << endl;
		of << "SekRL "  << secRL << endl;
		of << "SekVLsoll "  << secVLsoll << endl;
		of << "FBHVL " << fbhVL << endl;
		of << "FBHVLsoll " << fbhVLsoll << endl;
		of << "FBHstatus " << fbhFlag << endl;
		of << "Boiler " << boiler << endl;
		of << "LadeSoll " << boilerLadeSoll << endl;
		of << "Boilerladung " << boilerLadung << endl;
		of << "Zirkulation " << zirk << endl;
		of << "ZirkStatus " << zirkFlag << endl;
		of.close();
	}
};

int main(int argc, char** argv) 
{
	uint8_t data[2048];

    if (argc >= 2)
    {
		device = argv[1];
    }

	if (argc >= 3)
	{
		dataFile = argv[2];
	}

	fd = openDevice(device);
	if (fd < 0)
	{
        cerr << "Error opening device " << device << endl;
		return -1;
	}

	auto now = std::chrono::system_clock::now(); 
  	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  	
	cout << endl << argv[0] << " successfully started at " << std::ctime(&now_time) << endl;

	do 
	{
 		auto start = std::chrono::system_clock::now(); 
	
		cout << endl << "Warte auf Daten ... " << flush;
		int bytesRead = readNBytes(data, 4);

	    auto end = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsed = end - start;
		
		cout << "erhalten nach " << elapsed.count() << " s" << endl;

	    // Check for the start pattern
    	const uint8_t pattern[] = {0x41, 0x00, 0x29};
		if ((data[0] != pattern[0]) || (data[1] != pattern[1]) || (data[2] != pattern[2]))
		{
			// no match
			cout << "Pattern not found, draining buffer..." << endl;
			bytesRead += readUntilTimeout(&data[bytesRead], 2000);
			printRawData(data, bytesRead);
		}
		else
		{
           	cout << endl;

       		int dataType = data[3];
       		switch (dataType)
       		{
            	case 0x4A:
				{
					cout << "Betriebsdaten" << endl;
                	cout << "=============" << endl;

					int expectedBytes = 275;
					bytesRead += readUntilTimeout(&data[bytesRead], expectedBytes);
					printRawData(data, bytesRead);
					bool ok = checkRawData(data, bytesRead);
					// Checksum seems occasionally be corrupted by wrong data from meter. 
					// We don't care, since we read the metering data from mbus/vzlogger directly anyways
					if (bytesRead == expectedBytes+4)
					{
						outputOperatingData(data);
						checkAndPrintOperatingDataChanges(data);
					} 
				}
                break;

            	case 0x56:
				{
                	cout << "Parameterliste" << endl;
                	cout << "==============" << endl;

					int expectedBytes = 784;
					bytesRead += readUntilTimeout(&data[bytesRead], expectedBytes);
					printRawData(data, bytesRead);
					(void) checkRawData(data, bytesRead);
				}
                break;

            	case 0x42:
				{
                	cout << "HK-Bezeichnung" << endl;
                	cout << "==============" << endl;

					int expectedBytes = 32;
					bytesRead += readUntilTimeout(&data[bytesRead], expectedBytes);
					printRawData(data, bytesRead);
					bool ok = checkRawData(data, bytesRead);
					if (ok)
					{
						string hkName((const char *) &data[9]);
						cout << "HK-Bezeichung: \"" << hkName << "\"" << endl;
					}
				}
                break;

            	default:
				{
                	cout << "UNBEKANNTER DATENTYP: 0x" << hex << dataType << endl;
                	cout << "==========================" << endl;
					bytesRead += readUntilTimeout(&data[bytesRead], 1000);
					printRawData(data, bytesRead);
					(void) checkRawData(data, bytesRead);
				}
                break;
            }
		}
		
    } while (true); 

    close(fd);

    return 0;
}
