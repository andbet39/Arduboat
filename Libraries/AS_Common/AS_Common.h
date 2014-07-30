//
// File			AS_Common.h
// Class library header
//
// Details		<#details#>
//
// Project	 	ardusail
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author		Andrea Terzani
// 				Andrea Terzani
//
// Date			12/07/14 21:34
// Version		<#version#>
//
// Copyright	© Andrea Terzani, 2014
// License	    <#license#>
//
// See			ReadMe.txt for references
//



#ifndef AS_Common_h
#define AS_Common_h
// used to pack structures
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define PACKED
#else
#define PACKED __attribute__((__packed__))
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>


struct PACKED Location {
    
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};


#endif
