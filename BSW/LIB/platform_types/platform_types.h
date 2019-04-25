
#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H



/*~+:Macro Definitions*/
#define CPU_TYPE_8                                       8             /**< Indicates a 8 bit processor */
#define CPU_TYPE_16                                      16            /**< Indicates a 16 bit processor */
#define CPU_TYPE_32                                      32            /**< Indicates a 32 bit processor */
#define MSB_FIRST                                        0             /**< The most significant bit is the first bit of the bit sequence. */
#define LSB_FIRST                                        1             /**< The least significant bit is the first bit of the bit sequence. */
#define HIGH_BYTE_FIRST                                  0             /**< Within a uint16, the high byte is located before the low byte. */
#define LOW_BYTE_FIRST                                   1             /**< Within a uint16, the low byte is located before the high byte. */
#define CPU_TYPE                                         CPU_TYPE_32  
#define CPU_BIT_ORDER                                    LSB_FIRST    
#define CPU_BYTE_ORDER                                   LOW_BYTE_FIRST 

/** The standard AUTOSAR type boolean shall only be used in conjunction with the standard symbols TRUE and FALSE. 
For value assignments of variables of type boolean no arithmetic or logical operators (+, ++, -, --, *, /, \, <<, >>, !, ~) must be used. 
The only allowed form of assignment is:

boolean var;
...
var = TRUE;
var = FALSE; */
typedef unsigned char boolean;

/** Range decimal: 0..255
Range hex:       0..FF */
typedef unsigned char uint8;

/** Range decimal: 0..65535
Range hex:       0..FFFF */
typedef unsigned short uint16;

/** Range decimal: 0..4294967295
Range hex:       0..FFFF */
typedef unsigned long uint32;

/** Range decimal: -128..127
Range hex:       0..FF (0..7F = 0..127, 80..FF = -128..-1) */
typedef signed char sint8;

/** Range decimal: -32768..32767
Range hex:       0..FFFF (0..7FFF = 0..32767, 8000..FFFF = -32768..-1) */
typedef signed short sint16;

/** Range decimal: -2147483648..2147483647
Range hex:       0..FFFF (0..7FFF = 0..2147483647, 8000..FFFF = -2147483648..-1) */
typedef signed long sint32;

/** A 32bit float (IEEE_754) is defined as: 1bit-sign, 8bits-exponent, 23bits-mantissa.
(Precision) Range: +/- 1.1754943508222875079687365372222e-38..1.7014118346046923173168730371588e+38 */
typedef float float32;

/** A 64bit float (IEEE_754) is defined as: 1bit-sign, 11bits-exponent, 52bits-mantissa.
(Precision) Range: +/- 2.2250738585072013830902327173324e-308..8.9884656743115795386465259539451e+307 */
typedef double float64;

/** This type is implemented as unsigned 32 bit on this platform.
Range decimal: 0..4294967295
Range hex:       0..FFFF */
typedef unsigned long uint8_least;

/** This type is implemented as unsigned 32 bit on this platform.
Range decimal: 0..4294967295
Range hex:       0..FFFF */
typedef unsigned long uint16_least;

/** This type is implemented as unsigned 32 bit on this platform.
Range decimal: 0..4294967295
Range hex:       0..FFFF */
typedef unsigned long uint32_least;

/** Range decimal: -128..127
Range hex:       0..FF (0..7F = 0..127, 80..FF = -128..-1) */
typedef signed long sint8_least;

/** Range decimal: -32768..32767
Range hex:       0..FFFF (0..7FFF = 0..32767, 8000..FFFF = -32768..-1) */
typedef signed long sint16_least;

/** Range decimal: -2147483648..2147483647
Range hex:       0..FFFF (0..7FFF = 0..2147483647, 8000..FFFF = -2147483648..-1) */
typedef signed long sint32_least;

/** Standard definition for 64 bit unsigned int symbol. */
typedef unsigned long long uint64;

/** Standard definition for 64 bit signed int symbol. */
typedef signed long long sint64;

/** Standard definition of platform specific size type. */
typedef uint32 usize;

#if (!defined USIZE_C)

/** \brief Macro to define a constant of type usize */
#define USIZE_C(x) (x ## UL)
#endif


/* Macro Type Name: BOOLEAN_STATES */
/* ## Macro Type Declaration Start  */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
/* ## Macro Type Declaration End */



#endif
