/*
 * ccsbcs.h
 *
 *  Created on: Dec 17, 2012
 *      Author: Jure
 */

#ifndef CCSBCS_H_
#define CCSBCS_H_

CHAR ff_convert (	//* Converted character, Returns zero on error
	WCHAR	src,	//* Character code to be converted
	UINT	dir		//* 0: Unicode to OEMCP, 1: OEMCP to Unicode
);

WCHAR ff_wtoupper (	//* Upper converted character
	WCHAR chr		//* Input character
);

#endif /* CCSBCS_H_ */
