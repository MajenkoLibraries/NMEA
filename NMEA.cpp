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

NMEA::NMEA() {
	_dev = &Serial;
}

NMEA::NMEA(Stream &dev) {
	_dev = &dev;
}

void NMEA::begin() {
	_frameStart = false;
}

void NMEA::process() {
	while (_dev->available()) {
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
}

void NMEA::processMessage() {

	// Trim the buffer at the checksum star if there is one.
	char *star = strchr(_buffer, '*');
	if (star != NULL) {
		*star = 0;
	}
	
	if (!strncmp(_buffer, "GPRMC", 5)) {
		processGPRMC();
		return;
	}

	if (!strncmp(_buffer, "GPVTG", 5)) {
		processGPVTG();
		return;
	}

	if (!strncmp(_buffer, "GPGGA", 5)) {
		processGPGGA();
		return;
	}
}

void NMEA::processGPVTG() {

	char *tok;
	char *type;

	_updated = true;

	tok = comma(_buffer); // Discard GPVTG
	while ((tok = comma())) {
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

void NMEA::processGPRMC() {
	char *tok;
	_updated = true;
	tok = comma(_buffer);
	tok = comma();
	triplet(tok, _time_h, _time_m, _time_s);
	
	tok = comma();
	if (tok[0] == 'A') {
		_ok = true;
	} 
	else if (tok[0] == 'V') {
		_ok = false;
		return;
	}

	tok = comma();
	_lat = pos2dec(tok);
	tok = comma();
	if (tok[0] == 'S') {
		_lat = -_lat;
	}
	_latd = tok[0];

	tok = comma();
	_long = pos2dec(tok);
	tok = comma();
	if (tok[0] == 'E') {
		_long = -_long;
	}
	_longd = tok[0];

	tok = comma();
	_speedN = strtod(tok, NULL);

	tok = comma();
	_bearingT = strtod(tok, NULL);

	tok = comma();
	triplet(tok, _date_d, _date_m, _date_y);

	tok = comma();
	_mgvar = strtod(tok, NULL);
	tok = comma();
	_mgvard = tok[0];

}

double NMEA::getLatitude() {
	return _lat;
}

double NMEA::getLongitude() {
	return _long;
}

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

bool NMEA::isUpdated() {
	bool u = _updated;
	_updated = false;
	return u;
}

double NMEA::getSpeed(bool knots) {
	if (knots) {
		return _speedN;
	} else {
		return _speedK;
	}
}

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
	_updated = true;
		
	
	comma(_buffer); // Discard GPGGA

	char *time = comma();
	triplet(time, _time_h, _time_m, _time_s);

	char *lat = comma();
	_lat = pos2dec(lat);
	char *latd = comma();
	_latd = latd[0];
	if (_latd == 'S') {
		_lat = -_lat;
	}

	char *longitude = comma();
	_long = pos2dec(longitude);
	char *longd = comma();
	_longd = longd[0];
	if (_longd == 'E') {
		_long = -_long;
	}

	char *fix = comma();
	if (fix[0] == '0') {
		_ok = false;
	} else {
		_ok = true;
	}

	char *sats = comma();
	_satellites = atoi(sats);

	char *hdop = comma();
	_hdop = strtod(hdop, NULL);

	char *alt = comma();
	_altitude = strtod(alt, NULL);
	char *au = comma();
	_altitude_units = au[0];
	
	char *height = comma();
	_height = strtod(height, NULL);
	char *hu = comma();
	_height_units = hu[0];
}

double NMEA::getAltitude() {
	return _altitude;
}
char NMEA::getAltitudeUnits() {
	return _altitude_units;
}

double NMEA::getEllipsoidHeight() {
	return _height;
}

char NMEA::getEllipsoidHeightUnits() {
	return _height_units;
}

uint8_t NMEA::getSatellites() {
	return _satellites;
}


uint8_t NMEA::getDay() {
	return _date_d;
}

uint8_t NMEA::getMonth() {
	return _date_m;
}

uint16_t NMEA::getYear() {
	return _date_y + 2000;
}

uint8_t NMEA::getHour() {
	return _time_h;
}

uint8_t NMEA::getMinute() {
	return _time_m;
}

uint8_t NMEA::getSecond() {
	return _time_s;
}
