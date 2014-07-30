//
// AS_Menu_.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		ardusail
//
// Created by 	Andrea Terzani, 12/07/14 15:49
// 				Andrea Terzani
//
// Copyright 	© Andrea Terzani, 2014
// License     <#license#>
//
// See 			AS_Menu .h and ReadMe.txt for references
//


#include "AS_Menu.h"
int incomingByte = 0;   // for incoming serial data

char AS_Menu::_inbuf[MENU_COMMANDLINE_MAX];
AS_Menu::arg AS_Menu::_argv[MENU_ARGS_MAX + 1];


uint8_t len;




AS_Menu::AS_Menu() {
    len = 0;
    
}

void AS_Menu::run(void){

int c;

    if ( Serial.available()){
        for (;;) {
        
            // read the incoming byte:
            c = Serial.read();
        
            // carriage return -> process command
            if ('\r' == c) {
                _inbuf[len] = '\0';
                Serial.write('\r');
                Serial.write('\n');
                break;
            }
            // backspace
            if ('\b' == c) {
                if (len > 0) {
                    len--;
                    Serial.write('\b');
                    Serial.write(' ');
                    Serial.write('\b');
                    continue;
                }
            }
            // printable character
            if (isprint(c) && (len < (MENU_COMMANDLINE_MAX - 1))) {
                _inbuf[len++] = c;
                Serial.write((char)c);
                continue;
            }
            
            
        }
    
    
        }
    }



