/*
 * NMEA.h
 *
 *  Created on: 26-Feb-2022
 *      Author: arunr
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "stdint.h"

typedef struct {
	int hour;
	int min;
	int sec;
}TIME;

typedef struct {
	float latitude;
	float latF;
	uint8_t latDeg;
	float latMin;
	char NS;
	float longitude;
	float longF;
	uint8_t longDeg;
	float longMin;
	char EW;
}LOCATION;

typedef struct {
	float altitude;
	char unit;
}ALTITUDE;

typedef struct {
	int Day;
	int Mon;
	int Yr;
}DATE;

typedef struct {
	LOCATION lcation;
	TIME tim;
	int isfixValid;
	ALTITUDE alt;
	int numofsat;
}GGASTRUCT;

typedef struct {
	DATE date;
	float speed;
	float course;
	int isValid;
}RMCSTRUCT;

typedef struct {
	GGASTRUCT ggastruct;
	RMCSTRUCT rmcstruct;
}GPSSTRUCT;

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga);

int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc);

#endif /* INC_NMEA_H_ */
