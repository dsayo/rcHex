#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "trig.h"
#include <time.h>

/* angle: uint16_t[0 - 65534](10lsb)[0-1023] */
/* Numbers *1024, or << 10 int32_t[-2147483648, +2147483647] */

/* Sine Lookup Table (1/4 phase) */
static const int32_t sin_tbl[] = {
      0,    6,   13,   19,   25,   31,   38,   44, 
     50,   57,   63,   69,   75,   82,   88,   94, 
    100,  107,  113,  119,  125,  132,  138,  144, 
    150,  156,  163,  169,  175,  181,  187,  194, 
    200,  206,  212,  218,  224,  230,  237,  243, 
    249,  255,  261,  267,  273,  279,  285,  291, 
    297,  303,  309,  315,  321,  327,  333,  339, 
    345,  351,  357,  363,  369,  374,  380,  386, 
    392,  398,  403,  409,  415,  421,  426,  432, 
    438,  443,  449,  455,  460,  466,  472,  477, 
    483,  488,  494,  499,  505,  510,  516,  521, 
    526,  532,  537,  543,  548,  553,  558,  564, 
    569,  574,  579,  584,  590,  595,  600,  605, 
    610,  615,  620,  625,  630,  635,  640,  645, 
    650,  654,  659,  664,  669,  674,  678,  683, 
    688,  692,  697,  702,  706,  711,  715,  720, 
    724,  729,  733,  737,  742,  746,  750,  755, 
    759,  763,  767,  771,  775,  779,  784,  788, 
    792,  796,  799,  803,  807,  811,  815,  819, 
    822,  826,  830,  834,  837,  841,  844,  848, 
    851,  855,  858,  862,  865,  868,  872,  875, 
    878,  882,  885,  888,  891,  894,  897,  900, 
    903,  906,  909,  912,  915,  917,  920,  923, 
    926,  928,  931,  934,  936,  939,  941,  944, 
    946,  948,  951,  953,  955,  958,  960,  962, 
    964,  966,  968,  970,  972,  974,  976,  978, 
    980,  982,  983,  985,  987,  989,  990,  992, 
    993,  995,  996,  998,  999, 1000, 1002, 1003, 
   1004, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 
   1013, 1014, 1015, 1016, 1016, 1017, 1018, 1018, 
   1019, 1020, 1020, 1021, 1021, 1022, 1022, 1022, 
   1023, 1023, 1023, 1024, 1024, 1024, 1024, 1024
};

/* Maps degrees to scaled angle [0,1023].
 */
uint16_t scaled_angle(float angle_deg)
{
   float ranged_deg = angle_deg;

   /* Range degrees to [0,360) */
   if (ranged_deg <= 0)
   {
      while (ranged_deg < 0)
      {  
         ranged_deg += 360;
      }
   }
   else
   {
      while (ranged_deg > 360)
      {
         ranged_deg -= 360;
      }
   }
   
   printf("%f\n", ranged_deg);
   return ranged_deg * (float)MAX_ANGLE / (float)360;
}

/* Input: scaled angle */
int32_t _sin(uint16_t a)
{
   uint16_t angle;

   angle = a & (MAX_ANGLE - 1);  /* 10-bit angle */

   /*    Scaled      Degrees     Quadrant */
   /* [  0,  255] :: [  0,  90]         I */
   if (angle < (MAX_ANGLE / 4))
   {
      return sin_tbl[angle]; 
   }

   /* [256,  511] :: ( 90, 180]        II */
   if (angle < (MAX_ANGLE / 2))
   {
      return sin_tbl[(MAX_ANGLE / 2) - angle - 1];
   }

   /* [512,  767] :: (180, 270]       III */
   if (angle < (MAX_ANGLE * 3 / 4))
   {
      return -sin_tbl[angle - (MAX_ANGLE / 2)];
   }
   
   /* [768, 1023] :: (270, 360)        IV */
   return -sin_tbl[(MAX_ANGLE) - angle - 1];
}

/* Calls trig. identity _cos(a) = _sin(a + 90*).
 */
int32_t _cos(uint16_t a)
{
   uint16_t shift_angle;

   shift_angle = a + MAX_ANGLE / 4;
   
   return _sin(shift_angle);
}

/* Evaluates _sin(a)/_cos(a).
 */
int32_t _tan(uint16_t a)
{
   float sin_a = _sin(a) / (float)SCALAR;
   float cos_a = _cos(a) / (float)SCALAR;

   return SCALAR * sin_a / cos_a;
}

void _acos()
{

}

void _atan2()
{

}

int main(int argc, char *argv[])
{
   uint16_t s;
   clock_t start, end;
   double cpu_time_used;
   int i;

   start = clock();
   for (i = 0; i < 360; i++)
   {
      tan(i*M_PI/180);
   }

   end = clock();
   cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
   printf("%lf\n", cpu_time_used);

   start = clock();
   for (i = 0; i < 360; i++)
   {
      _tan(i);
   }
   end = clock();
   cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
   printf("%lf\n", cpu_time_used);
   
   s = scaled_angle(16.875);
   printf("scaled angle: %d\n", s);
   printf("_sin ans: %d\n", _sin(s));
   printf("_cos ans: %d\n", _cos(s));
   printf("_tan ans: %d\n", _tan(s));
   return 0;       
}
