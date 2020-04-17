/*
 * Copyright (c) 2014, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

        void (*_updateCallback)();

        boolean _doUpdate;
        uint32_t _lastRx;
        int32_t _offset;
		
		uint8_t _time_h;
		uint8_t _time_m;
		uint8_t _time_s;
		uint8_t _date_d;
		uint8_t _date_m;
		uint8_t _date_y;

		uint8_t _off_time_h;
		uint8_t _off_time_m;
		uint8_t _off_time_s;
		uint8_t _off_date_d;
		uint8_t _off_date_m;
		uint8_t _off_date_y;
		
		void processMessage();
		void processGPRMC();
		void processGPVTG();
		void processGPGGA();
		
		char *comma(char *str = NULL);
		double pos2dec(char *pos);
		void triplet(char *, uint8_t &, uint8_t &, uint8_t &);

        void csWrite(uint8_t c, uint8_t &csa, uint8_t &csb);

        void setOffsetTime(int32_t offset);

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
        uint8_t getDow();

		bool isLocked();
		bool isUpdated();

        // uBLOX specific commands
        void enableEco();
        void enablePowerSave();
        void enableFullPower();

        // Event Handling
        void onUpdate(void (*func)()) { _updateCallback = func; }

        // Time conversion
        uint32_t getTimestamp();

        void setGMTOffset(int32_t o) { _offset = o; }

};

#endif
