//
// File			AS_Math.h
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
// Date			12/07/14 21:49
// Version		<#version#>
//
// Copyright	© Andrea Terzani, 2014
// License	    <#license#>
//
// See			ReadMe.txt for references
//



#ifndef AS_Math_h

#define AS_Math_h

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __AVR__
# include <AP_Math_AVR_Compat.h>
#endif

//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

//GPS Specific double precision conversions
//The precision here does matter when using the wsg* functions for converting
//between LLH and ECEF coordinates. Test code in examlpes/location/location.pde
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define DEG_TO_RAD_DOUBLE 0.0174532925199432954743716805978692718781530857086181640625  // equals to (M_PI / 180.0)
#define RAD_TO_DEG_DOUBLE 57.29577951308232286464772187173366546630859375               // equals to (180.0 / M_PI)
#endif




float toRadians(float deg);

// radians -> degrees
float toDegree(float rad);


// longitude_scale - returns the scaler to compensate for shrinking longitude as you move north or south from the equator
// Note: this does not include the scaling to convert longitude/latitude points to meters or centimeters
float                   longitude_scale(const struct Location &loc);

// return distance in meters between two locations
float                   get_distance(const struct Location &loc1, const struct Location &loc2);

// return distance in centimeters between two locations
uint32_t                get_distance_cm(const struct Location &loc1, const struct Location &loc2);

// return bearing in centi-degrees between two locations
int32_t    get_bearing_cd(const struct Location &loc1, const struct Location &loc2);






/*
// square
float sq(float v);
*/
// sqrt of sum of squares
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);



#endif
