#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char *argv[])
{
   FILE *file;
   int i, j;
   double deg;

   file = fopen("sin_table.txt", "w+");

   for (i = 0, j = 0; i < 256; i++, j++)
   {
      deg = (double)i * (double)90 / (double)256 * M_PI / (double)180;
      fprintf(file, "%4d, ", (int)round(sin(deg) * 1024));
      if (j == 7)
      {
         fprintf(file, "\n"); 
         j = -1;
      }
   }

   return 0;
}
