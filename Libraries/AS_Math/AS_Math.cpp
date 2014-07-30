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




// degrees -> radians
float toRadians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
float toDegree(float rad) {
	return rad * RAD_TO_DEG;
}

