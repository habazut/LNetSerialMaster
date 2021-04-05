/*
 *  (c) 2020, Chris Harlow. All rights reserved.
 *  (c) 2021, Harald Barth. All rights reserved.
 *
 *  GPLv3, for the full license text, see the file LICENSE
 *
 */
#ifndef StringFormatter_h
#define StringFormatter_h
#include <Arduino.h>
#include "FSH.h"
#if defined(ARDUINO_ARCH_SAMD)
   // Some processors use a gcc compiler that renames va_list!!!
  #include <cstdarg>  
#endif

class Diag {
  public:
  static bool ACK;
  static bool CMD;
  static bool WIFI;
  static bool WITHROTTLE;
  static bool ETHERNET;
  static bool LCN;
  
};

class StringFormatter
{
  public:
    static void send(Print * serial, const FSH* input...);
    static void send(Print & serial, const FSH* input...);
    
    static void printEscapes(Print * serial,char * input);
    static void printEscapes(Print * serial,const FSH* input);
    static void printEscape(Print * serial, char c);

    // DIAG support
    static Print * diagSerial;
    static void diag( const FSH* input...);
    static void command( const FSH* input...);
    static void printEscapes(char * input);
    static void printEscape( char c);

    private: 
    static void send2(Print * serial, const FSH* input,va_list args);
    static void printPadded(Print* stream, long value, byte width, bool formatLeft);

};
#endif
