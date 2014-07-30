//
// File			AS_Menu_.h
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
// Date			12/07/14 15:49
// Version		<#version#>
//
// Copyright	© Andrea Terzani, 2014
// License	    <#license#>
//
// See			ReadMe.txt for references
//

#include <FastSerial.h>
#include "Arduino.h"


#ifndef AS_Menu_h
#define AS_Menu_h


#define MENU_COMMANDLINE_MAX    32
#define MENU_ARGS_MAX           3


class AS_Menu{
  
public:

    
    struct arg {
        const char *str;                        ///< string form of the argument
        long i;                                         ///< integer form of the argument (if a number)
        float f;                                        ///< floating point form of the argument (if a number)
    };

    
    
  AS_Menu();
  void run();
 
private:
 
    static char _inbuf[MENU_COMMANDLINE_MAX];
    static arg              _argv[MENU_ARGS_MAX + 1];
    

};

#endif
