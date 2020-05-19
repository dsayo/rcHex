/*******************************************************************************
 * term.c
 *
 * Terminal code to test transmit/receive SBUS parsing.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 24 Jan 2020
 *******************************************************************************
 */
#include "sbus.h"
#include "term.h"
#include "main.h"

UART_HandleTypeDef *uart;

void term_write_char(uint8_t c){
   HAL_UART_Transmit(uart, &c, 1, 10);
}

void term_write_str(char *str){
   uint32_t i = 0;
   while(str[i]) {
      term_write_char(str[i]);
      i++;
   }
}

/* <Esc>[2J */
void term_clear(){
   term_write_char(ESC);
   term_write_str("[2J");
}

/* <Esc>[H */
void term_cursor_home(){
   term_write_char(ESC);
   term_write_str("[H");
}

void term_write_num(int32_t num){
   uint8_t digits[5];
   int8_t i = 0;
   int32_t n = num;
   int32_t rem;

   /* Clear buffer */
   for(i = 4; i >= 0; i--){
      digits[i] = '0';
   }
   i = 0;
   /* Write decimal values to buffer */
   while(n != 0){
      rem = n % 10;
      digits[i++] = rem + '0';
      n /= 10;
   }

   for (i = 4; i > 0; i--){
      if (digits[i] != '0'){
         break;
      }
   }
   for (i; i > 0; i--){
      term_write_char(digits[i]);
   }
   term_write_char(digits[0]);
}

/* <Esc>[{ROW};{COLUMN}f */
void term_cursor_move(uint8_t row, uint8_t col){

   term_write_char(ESC);
   term_write_char('[');
   term_write_num(row);
   term_write_char(';');
   term_write_num(col);
   term_write_char('H');
}

void term_draw_bar(uint16_t val){
   uint8_t num_chars;
   uint8_t i;
   num_chars = (val - DEFAULT_MIN)/50;
   for (i = 0; i < num_chars; i++){
      term_write_char(BAR_SYMBOL);
   }
   term_write_char(' ');
   term_write_num(val);
   term_write_str("                                                      ");
}

/* prints all characters what do not need to be updated */
void init_term(UART_HandleTypeDef *huart)
{
   uart = huart;
   term_clear();
   term_cursor_home();
   term_write_str("CH 1:\n");
   term_cursor_move(2, 1);
   term_write_str("CH 2:\n");
   term_cursor_move(3, 1);
   term_write_str("CH 3:\n");
   term_cursor_move(4, 1);
   term_write_str("CH 4:\n");
   term_cursor_move(5, 1);
   term_write_str("CH 5:\n");
   term_cursor_move(6, 1);
   term_write_str("CH 6:\n");
}


void print_channels(RXData data)
{
   term_cursor_move(1, 7);
   term_draw_bar(data.channels[0]);
   term_cursor_move(2, 7);
   term_draw_bar(data.channels[1]);
   term_cursor_move(3, 7);
   term_draw_bar(data.channels[2]);
   term_cursor_move(4, 7);
   term_draw_bar(data.channels[3]);
   term_cursor_move(5, 7);
   term_draw_bar(data.channels[4]);
   term_cursor_move(6, 7);
   term_draw_bar(data.channels[5]);
}

