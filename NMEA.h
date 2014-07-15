#ifndef _NMEA_H
#define _NMEA_H

#if (ARDUINO >= 100)
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#define NMEA_BUFSZ 128

class NMEA {
	private:
		Stream *_dev;
		char _buffer[NMEA_BUFSZ];
		uint8_t _bufptr;
		bool _frameStart;

		bool _ok;
		double _lat;
		double _long;
		char _latd;
		char _longd;
		double _bearingT;
		double _bearingM;
		double _speedN;
		double _speedK;
		double _mgvar;
		char _mgvard;
		bool _updated;
		uint8_t _satellites;
		double _hdop;
		double _altitude;
		char _altitude_units;
		double _height;
		char _height_units;
		
		uint8_t _time_h;
		uint8_t _time_m;
		uint8_t _time_s;
		uint8_t _date_d;
		uint8_t _date_m;
		uint8_t _date_y;
		
		void processMessage();
		void processGPRMC();
		void processGPVTG();
		void processGPGGA();
		
		char *comma(char *str = NULL);
		double pos2dec(char *pos);
		void triplet(char *, uint8_t &, uint8_t &, uint8_t &);

	public:
		NMEA();
		NMEA(Stream &dev);
		void begin();
		void process();

		double getLatitude();
		double getLongitude();
		double getBearing(bool mag = false);
		double getSpeed(bool knots = false);
		double getAltitude();
		char getAltitudeUnits();
		double getEllipsoidHeight();
		char getEllipsoidHeightUnits();
		uint8_t getSatellites();

		uint8_t getDay();
		uint8_t getMonth();
		uint16_t getYear();
		uint8_t getHour();
		uint8_t getMinute();
		uint8_t getSecond();

		bool isLocked();
		bool isUpdated();
};

#endif
