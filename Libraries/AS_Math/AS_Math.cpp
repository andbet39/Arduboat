//
// AS_Math.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		ardusail
//
// Created by 	Andrea Terzani, 12/07/14 21:49
// 				Andrea Terzani
//
// Copyright 	© Andrea Terzani, 2014
// License     <#license#>
//
// See 			AS_Math.h and ReadMe.txt for references
//


// Library header
#include "AS_Math.h"

// square
float sq(float v) {
	return v*v;
}

// 2D vector length
float pythagorous2(float a, float b) {
	return sqrt(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrt(sq(a)+sq(b)+sq(c));
}

// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


// degrees -> radians
float toRadians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
float toDegree(float rad) {
	return rad * RAD_TO_DEG;
}

