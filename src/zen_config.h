#ifndef ZEN_CONFIG_H_
#define ZEN_CONFIG_H_


 /*
 * SU ID
 * SU ID = Toilet ID
 * 0x**** (4 digits, if unused leave as 0)
 */
#define SET_UID 0x0007

 /*
 * Refill length in ppr
 * MAX_FILM_COUNT = mm_ppr * actual_film_length_mm 
 * 257600 for 16m
 */
#define MAX_FILM 257600 

 /*
 * Safety factor
 * Cut-off when 2400 pulses of Refill left
 */
#define COUNT_CUTOFF 2400

 /*
 * Actual length of Refill in mm
 * 16100 mm for a 16.0m refill
 */
#define ACTUAL_FILM 16100


 /*
 * Flushable Refill Length
 * Need to be in float-> always use decimal place
 * 16.0 out of 16.1 for a 16.0m Refill
 */
#define PPRMM 16


 /*
 * Jam Detection RPM
 */
#define JAM_RPM_CUTOFF 36

 /*
 * After how many Jam detections should the motor stop (50ms readout)
 */
#define MAX_LOW_RPM_COUNT_JAMMED_FLUSH 3


 /*
 * Motor Parameters
 * Encoder ppr of motor
 * gear ratio of motor
 */
float encoderPPR = 16.0;
float gearBoxRatio = 175.0;

#endif /* ZEN_CONFIG_H_ */