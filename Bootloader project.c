
#include "Bootloader.h"
#include "stdarg.h" // needed by print function
#include "stdio.h" // needed by print function
#include "string.h" // needed by print function

// lOCATION OF RESET HANDLER FOR USER APP
#define USER_APP_LOCATION (0X8020000 + 4)

//prototype
static void jump_to_user_app(void);




void bootloader_main(void){

while(1);

}

// user application routine
static void jump_to_user_app(void){
void (*user_app_reset_handler)(void) = (void *)(*((uint32_t *)(USER_APP_LOCATION)));
user_app_reset_handler();
}


/*
Static void print(char *msg, .....){

char buff[250];
va_list args;
vsprintf(buff,msg,args);

for(int i= 0; i<strlen(buff);i++){
USART2->DR = buff[i];

while (!(USART2->SR & USART_SR_TXE)); // CHECK IF TRANSMISSION IS ENABLE

}

while (!(USART2->SR & USART_SR_TC)); // CHECK IF TRANSMISSION IS COMPLETED


}

print ("time:%d\r\n", HAL_GetTick());
*/


