#ifndef IEC_TYPES_H
#define IEC_TYPES_H

//  #include <limits.h>
//  #include <float.h>
#include <time.h>
#ifndef MSVC
#include <stdint.h>
#endif

/*********************/
/*  IEC Types defs   */
/*********************/

typedef unsigned char  IEC_BOOL;

typedef char    IEC_SINT;
typedef short   IEC_INT;
typedef int   IEC_DINT;
typedef long long   IEC_LINT;

typedef unsigned char    IEC_USINT;
typedef unsigned short   IEC_UINT;
typedef unsigned int   IEC_UDINT;
typedef unsigned long long   IEC_ULINT;

typedef unsigned char    IEC_BYTE;
typedef unsigned short   IEC_WORD;
typedef unsigned int   IEC_DWORD;
typedef unsigned long long   IEC_LWORD;

typedef float    IEC_REAL;
typedef double   IEC_LREAL;


#if !defined __timespec_defined && !defined __time_t_defined
# define __timespec_defined     1

struct timespec
{
    long int tv_sec;            /* Seconds.  */
    long int tv_nsec;           /* Nanoseconds.  */
};

#endif

typedef struct timespec IEC_TIME;
typedef struct timespec IEC_DATE;
typedef struct timespec IEC_DT;
typedef struct timespec IEC_TOD;

#ifndef STR_MAX_LEN
#define STR_MAX_LEN 126
#endif

#ifndef STR_LEN_TYPE
#define STR_LEN_TYPE int
#endif

typedef STR_LEN_TYPE __strlen_t;
typedef struct
{
    __strlen_t len;
    unsigned char body[STR_MAX_LEN];
} IEC_STRING;

#endif /*IEC_TYPES_H*/
