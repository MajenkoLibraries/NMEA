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

#include <NMEA.h>

/*! 
 * This constructor defaults the NMEA parser to using the Serial object.
 */
NMEA::NMEA() {
	_dev = &Serial;
}

/*!
 *  This constructor allows you to pass a specific serial object to the parser.  It is
 *  your responsibility to ensure that the serial object is configured for
 *  38400 baud (or the baud of your GPS module) and has had "begin" called on it.
 */
NMEA::NMEA(Stream &dev) {
	_dev = &dev;
	_frameStart = false;
    _updateCallback = NULL;
    _time_m = 0;
    _time_h = 0;
    _time_s = 0;
    _date_y = 0;
    _date_m = 0;
    _date_d = 0;
}

/*!
 *  Pre-configures any required variables.  Calling this
 *  before any processing is done is required.
 */
void NMEA::begin() {
	_frameStart = false;
    _updateCallback = NULL;
    _time_m = 0;
    _time_h = 0;
    _time_s = 0;
    _date_y = 0;
    _date_m = 0;
    _date_d = 0;
}

/*!
 *  This function is the main heart of the NMEA processor.  It
 *  receives characters from the serial device, identifies the frame
 *  wrapper characters, and stores the frame data into a buffer.  When
 *  the frame is complete it calls the relevant decoder function to
 *  handle the data.
 *
 *  This function must be called frequently to process the data.
 */
void NMEA::process() {
	while (_dev->available()) {
        _lastRx = millis();
		char c = _dev->read();
		if (c == '$') {
			_bufptr = 0;
			_frameStart = true;
			continue;
		}
		if (!_frameStart) {
			continue;
		}
		if (c == '\n' || c == '\r') {
			_frameStart = false;
			processMessage();
			continue;
		}
		if (_bufptr < NMEA_BUFSZ) {
			_buffer[_bufptr++] = c;
			_buffer[_bufptr] = 0;
		}
	}

    if (_doUpdate) {
        if (millis() - _lastRx > 1) {
            _doUpdate = false;
            if (_updateCallback != NULL) {
                _updateCallback();
            }
            _updated = true;
        }
    }
}

void NMEA::processMessage() {

	// Trim the buffer at the checksum star if there is one.
	char *star = strchr(_buffer, '*');
	if (star != NULL) {
		*star = 0;
	}

	if (!strncmp(_buffer, "GPRMC", 5)) {
		processGPRMC();
        _doUpdate = true;
        setOffsetTime(_offset);
		return;
	}

	if (!strncmp(_buffer, "GPVTG", 5)) {
		processGPVTG();
        _doUpdate = true;
        setOffsetTime(_offset);
		return;
	}

	if (!strncmp(_buffer, "GPGGA", 5)) {
		processGPGGA();
        _doUpdate = true;
        setOffsetTime(_offset);
		return;
	}
}


void NMEA::processGPVTG() {

	char *tok;
	char *type;


	tok = comma(_buffer); // Discard GPVTG
    if (tok == NULL) {
        return;
    }
	while ((tok = comma())) {
        if (tok == NULL) {
            return;
        }
		type = comma();
		if (type == NULL) {
			return;
		}
		switch (type[0]) {
			case 'T': // True track made good
				_bearingT = strtod(tok, NULL);
				break;
			case 'M': // Magnetic track made good
				_bearingM = strtod(tok, NULL);
				break;
			case 'N': // Ground speed, knots
				_speedN = strtod(tok, NULL);
				break;
			case 'K': // Ground speed, kmph
				_speedK = strtod(tok, NULL);
				break;
		}
	}

}
// $GPGLL,5155.32591,N,00234.41370,W,194533.00,A,A*7E

void NMEA::processGPRMC() {
    // $GPRMC,194533.00,A,5155.32591,N,00234.41370,W,0.159,,160415,,,A*6D
	char *tok;
	tok = comma(_buffer);                       // GPRMC
    if (tok == NULL) return;
	tok = comma();  //                          // Time
    if (tok == NULL) return;
	triplet(tok, _time_h, _time_m, _time_s);

    _time_h %= 24;
    _time_m %= 60;
    _time_s %= 60;

	tok = comma();                              // Acquired
    if (tok == NULL) return;
	if (tok[0] == 'A') {
		_ok = true;
	} 
	else if (tok[0] == 'V') {
		_ok = false;
		return;
	}

	tok = comma();                              // Latitude
    if (tok == NULL) return;
	_lat = pos2dec(tok);
	tok = comma();                              // N/S
    if (tok == NULL) return;
	if (tok[0] == 'S') {
		_lat = -_lat;
	}
	_latd = tok[0];

	tok = comma();                              // Longitude
    if (tok == NULL) return;
	_long = pos2dec(tok);
	tok = comma();                              // EW
    if (tok == NULL) return;
	if (tok[0] == 'W') {
		_long = -_long;
	}
	_longd = tok[0];

	tok = comma();                              // Speed
    if (tok == NULL) return;
	_speedN = strtod(tok, NULL);

	tok = comma();                              // Bearing
    if (tok == NULL) return;
	_bearingT = strtod(tok, NULL);

	tok = comma();                              // Date
    if (tok == NULL) return;
	triplet(tok, _date_d, _date_m, _date_y);

    _date_d %= 32;
    _date_m %= 13;
    _date_y %= 100;

	tok = comma();                              // Pressure
    if (tok == NULL) return;
	_mgvar = strtod(tok, NULL);
	tok = comma();
    if (tok == NULL) return;
	_mgvard = tok[0];

}

/*!
 *  Returns the current latitude in degrees
 */
double NMEA::getLatitude() {
	return _lat;
}

/*!
 *  Returns the current longitude in degrees
 */
double NMEA::getLongitude() {
	return _long;
}

/*!
 *  Returns true if the receiver is locked on, false otherwise.
 */
bool NMEA::isLocked() {
	return _ok;
}

char *NMEA::comma(char *str) {
	static char *src = NULL;

	char  *p,  *ret = 0;

	if (str != NULL) {
		src = str;
	}
	
	if (src == NULL) {
		return NULL;
	}

	if (!*src) {
		return NULL;
	}

	p = src;
	while (*p && *p != ',') {
		p++;
	}
	if (*p == 0) {
		ret = src;
		src = NULL;
		return ret;	
	}
	*p = 0;
	ret = src;
	src = p+1;
	return ret;
}

double NMEA::pos2dec(char *pos) {
	char *dotpos = strchr(pos, '.');
	if (dotpos == NULL) {
		return 0.0;
	}

	uint8_t hisize = dotpos - pos;
	if (hisize <= 2) { // No degrees, just minutes
		double mins = strtod(pos, NULL);
		return mins / 60.0;
	}

	double mins = strtod(dotpos - 2, NULL);
	*(dotpos - 2) = 0;
	double deg = strtod(pos, NULL);
	return deg + (mins / 60.0);
}

/*!
 *  Returns true if the processor has received and processed a new
 *  valid message.  Resets the updated flag internally.
 */
bool NMEA::isUpdated() {
	bool u = _updated;
	_updated = false;
	return u;
}

/*!
 *  Returns the current calculated speed.  If true is passed as a parameter it returns
 *  the speed in Knots.  If false is passed, or no parameter is used, it returns the
 *  speed in kilometers per hour.
 */
double NMEA::getSpeed(bool knots) {
	if (knots) {
		return _speedN;
	} else {
		return _speedK;
	}
}

/*!
 *  Returns the current calculated bearing or heading.  If true is passed as a parameter
 *  it returns the bearing to magnetic north.  If false is passed, or no parameter is used,
 *  it returns the bearing to true north.
 */
double NMEA::getBearing(bool mag) {
	if (mag) {
		return _bearingM;
	} else {
		return _bearingT;
	}
}

void NMEA::triplet(char *data, uint8_t &a, uint8_t &b, uint8_t &c) {
	a = (data[0] - '0') * 10 + (data[1] - '0');
	b = (data[2] - '0') * 10 + (data[3] - '0');
	c = (data[4] - '0') * 10 + (data[5] - '0');
}



void NMEA::processGPGGA() {
	
//  $GPGGA,194533.00,5155.32591,N,00234.41370,W,1,10,1.24,63.1,M,48.6,M,,*73
//   GPGGA,200022.00,5155.322853,1,12,05,15,042,29,09,08,322,33,16,41,298,34,18,01,144,


	char *tok = comma(_buffer); // GPGGA
    if (tok == NULL) return;

	char *time = comma();       // Time
    if (time == NULL) return;
	triplet(time, _time_h, _time_m, _time_s);

    _time_h %= 24;
    _time_m %= 60;
    _time_s %= 60;

	char *lat = comma();        // Latitude
    if (lat == NULL) return;
	_lat = pos2dec(lat);
	char *latd = comma();       // N/S
    if (latd == NULL) return;
	_latd = latd[0];
	if (_latd == 'S') {
		_lat = -_lat;
	}

	char *longitude = comma();  // Longitude
    if (longitude == NULL) return;
	_long = pos2dec(longitude);
	char *longd = comma();      // W/E
    if (longd == NULL) return;
	_longd = longd[0];
	if (_longd == 'W') {
		_long = -_long;
	}

	char *fix = comma();
    if (fix == NULL) return;
	if (fix[0] == '0') {
		_ok = false;
	} else {
		_ok = true;
	}

	char *sats = comma();
    if (sats == NULL) return;
	_satellites = atoi(sats);

	char *hdop = comma();
    if (hdop == NULL) return;
	_hdop = strtod(hdop, NULL);

	char *alt = comma();
    if (alt == NULL) return;
	_altitude = strtod(alt, NULL);
	char *au = comma();
    if (au == NULL) return;

	_altitude_units = au[0];
	
	char *height = comma();
    if (height == NULL) return;
	_height = strtod(height, NULL);
	char *hu = comma();
    if (hu == NULL) return;
	_height_units = hu[0];

}

/*!
 *  Returns the current height above sea level.  The units are not defined, but
 *  can be obtained with the getAltutudeUnits() function.
 */
double NMEA::getAltitude() {
	return _altitude;
}

/*!
 *  Returns the units used for the height above sea level.  Usually 'M' for meters.
 */
char NMEA::getAltitudeUnits() {
	return _altitude_units;
}

/*!
 *  Returns the height above the WGS84 Ellipsoid.  The units are not defined, but
 *  can be obtained with the getEllipsoidHeightUnits() function.
 *
 *  The WGS84 Ellipsoid is a mathematical approximation of the shape of the earth
 *  as a smooth oblate spheroid.
 *
 *  For more information see http://en.wikipedia.org/wiki/World_Geodetic_System
 */
double NMEA::getEllipsoidHeight() {
	return _height;
}

/*!
 *  Returns the units used for the height above the WGS84 Ellipsoid.  Usually 'M' for meters.
 */
char NMEA::getEllipsoidHeightUnits() {
	return _height_units;
}

/*!
 *  Returns the number of currently locked satellites.
 */
uint8_t NMEA::getSatellites() {
	return _satellites;
}

/*!
 *  Returns the current day of the month (1 ... 31).
 */
uint8_t NMEA::getDay() {
	return _off_date_d;
}

/*!
 *  Returns the current month number (1 ... 12).
 */
uint8_t NMEA::getMonth() {
	return _off_date_m;
}

/*!
 *  Returns the current year (2000 ... 2099).
 */
uint16_t NMEA::getYear() {
	return _off_date_y + 2000;
}

/*!
 *  Returns the current hour of the day (0 ... 23).
 */
uint8_t NMEA::getHour() {
	return _off_time_h;
}

/*!
 *  Returns the current minutes (0 ... 59).
 */
uint8_t NMEA::getMinute() {
	return _off_time_m;
}

/*!
 *  Returns the current seconds (0 ... 59).
 */
uint8_t NMEA::getSecond() {
	return _off_time_s;
}

/*!
 *  Calculates the day of the week (0 ... 6, 0 being Sunday) from the current
 *  date values.
 */
uint8_t NMEA::getDow() {
   static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    int y = _off_date_y + 2000;
    int m = _off_date_m;
    int d = _off_date_d;

    y -= m < 3;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

void NMEA::csWrite(uint8_t c, uint8_t &cka, uint8_t &ckb) {
    cka += c;
    ckb += cka;
    _dev->write(c);
}

static int _ytab[2][13] = {
    {-1, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
    {-1, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

#define isleap(y) (((y) % 4) == 0 && ((y) % 100) != 0 || ((y) % 400) == 0)


/*!
 *  Convert the time to a UNIX timestamp (seconds since 01/01/70).
 */

uint32_t NMEA::getTimestamp() {
    uint32_t ayear = _date_y + 2000;
    uint32_t amonth = _date_m;
    uint32_t tv_sec = 0;

    if (_date_m > 12) return 0;
    if (_date_m == 0) return 0;
    if (_date_d > 31) return 0;
    if (_date_d == 0) return 0;
    if (_time_h > 23) return 0;
    if (_time_m > 59) return 0;
    if (_time_s > 59) return 0;

    if (isleap(ayear) && amonth > 2)
        ++tv_sec;

    for (--ayear; ayear >= 1970; --ayear)
        tv_sec += isleap(ayear) ? 366 : 365;

    while (--amonth)
        tv_sec += _ytab[0][amonth];

    tv_sec += _date_d - 1;
    tv_sec = 24 * tv_sec + _time_h;
    tv_sec = 60 * tv_sec + _time_m;
    tv_sec = 60 * tv_sec + _time_s;
    return tv_sec;
}

void NMEA::setOffsetTime(int32_t offset) {
    uint32_t secs = getTimestamp();
    secs += (offset * 3600);

    uint32_t year = 1970;

    uint32_t dayclock = secs % 86400;
    uint32_t dayno = secs / 86400;

    _off_time_s = dayclock % 60;
    _off_time_m = (dayclock % 3600) / 60;
    _off_time_h = dayclock / 3600;

    while (dayno >= (isleap(year) ? 366 : 365)) {
        dayno -= (isleap(year) ? 366 : 365);
        year++;
    }
    _off_date_y = year - 2000;
    _off_date_m = 1;
    while (dayno >= _ytab[isleap(year)][_off_date_m]) {
        dayno -= _ytab[isleap(year)][_off_date_m];
        _off_date_m++;
    }
    _off_date_d = dayno + 1;
}

/*! \name uBLOX NEO-6
 *  These functions are specifically for working with the uBLOX NEO-6 series of GPS
 *  modules, such as the one on the Sparkfun GPS shield.
 */

/**@{*/

/*! Enable ECO power mode.  This is a half-way house between full power and power saving
 *  mode.  It uses more power during acquisition but saves power during idle time.
 */
void NMEA::enableEco() {
    uint8_t cka = 0, ckb = 0;
    _dev->write(0xB5);
    _dev->write(0x62);
    csWrite(0x06, cka, ckb);
    csWrite(0x11, cka, ckb);
    csWrite(0x02, cka, ckb);
    csWrite(0x00, cka, ckb);
    csWrite(0x08, cka, ckb);
    csWrite(0x04, cka, ckb);
    _dev->write(cka);
    _dev->write(ckb);
}

/*! Power save mode uses the minimum power.  Everything takes longer though.
 */
void NMEA::enablePowerSave() {
    uint8_t cka = 0, ckb = 0;
    _dev->write(0xB5);
    _dev->write(0x62);
    csWrite(0x06, cka, ckb);
    csWrite(0x11, cka, ckb);
    csWrite(0x02, cka, ckb);
    csWrite(0x00, cka, ckb);
    csWrite(0x08, cka, ckb);
    csWrite(0x01, cka, ckb);
    _dev->write(cka);
    _dev->write(ckb);
}

/*! Full power is what it says - it uses the maximum power all the time.  Everything works
 *  much faster; satellites are found more reliably, etc.
 */
void NMEA::enableFullPower() {
    uint8_t cka = 0, ckb = 0;
    _dev->write(0xB5);
    _dev->write(0x62);
    csWrite(0x06, cka, ckb);
    csWrite(0x11, cka, ckb);
    csWrite(0x02, cka, ckb);
    csWrite(0x00, cka, ckb);
    csWrite(0x08, cka, ckb);
    csWrite(0x00, cka, ckb);
    _dev->write(cka);
    _dev->write(ckb);
}

/**@}*/
