#ifndef RXDATA_H
#define RXDATA_H

typedef unsigned char BYTE;
typedef unsigned DWORD;

struct RxData
{
	bool who, rtr, ext;
	BYTE header, command;
	DWORD id, timestamp;
	BYTE len, data[255];
	BYTE txflags, rxflags;
} ;

#endif

