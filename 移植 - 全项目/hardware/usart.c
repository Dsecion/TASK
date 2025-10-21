#include "usart.h"

#define    PA2  TX
#define    PA3  RX

#define RCC_CR          *(volatile uint32_t *)(RCC_BASE + 0x00)
#define RCC_PLLCFGR     *(volatile uint32_t *)(RCC_BASE + 0x04)
#define RCC_CFGR        *(volatile uint32_t *)(RCC_BASE + 0x08)

#define SYSTICK_BASE    0xE000E010U
#define SYSTICK_CTRL    *(volatile uint32_t *)(SYSTICK_BASE + 0x00)
#define SYSTICK_LOAD    *(volatile uint32_t *)(SYSTICK_BASE + 0x04)

#define RCC_BASE        0x40023800
#define RCC_APB1ENR     *(volatile uint32_t*)(RCC_BASE + 0x40)  
#define RCC_AHB1ENR     *(volatile uint32_t*)(RCC_BASE + 0x30)  


#define GPIOA_BASE      0x40020000
#define GPIOA_MODER     *(volatile uint32_t*)(GPIOA_BASE + 0x00)
#define GPIOA_AFRL      *(volatile uint32_t*)(GPIOA_BASE + 0x20)


#define USART2_BASE     0x40004400
#define USART2_SR       *(volatile uint32_t*)(USART2_BASE + 0x00) // ×´Ì¬¼Ä´æÆ÷
#define USART2_DR       *(volatile uint32_t*)(USART2_BASE + 0x04) // Êý¾Ý¼Ä´æÆ÷
#define USART2_BRR      *(volatile uint32_t*)(USART2_BASE + 0x08) // ²¨ÌØÂÊ¼Ä´æÆ÷
#define USART2_CR1      *(volatile uint32_t*)(USART2_BASE + 0x0C) // ¿ØÖÆ¼Ä´æÆ÷1

void USART2_Init(void) {
    // 1. ¿ªÆôÊ±ÖÓ
    RCC_AHB1ENR |= (1 << 0);      // Ê¹ÄÜGPIOAÊ±ÖÓ
    RCC_APB1ENR |= (1 << 17);      // Ê¹ÄÜUSART2Ê±ÖÓ

    // 2. ÅäÖÃGPIO¸´ÓÃÄ£Ê½£¨PA2=TX, PA3=RX£©
	GPIOA_MODER &= ~(0x03 << (2 * 2));  // Çå³ýPA2Ä£Ê½
    GPIOA_MODER |=  (0x02 << (2 * 2));  // ¸´ÓÃ¹¦ÄÜÄ£Ê½
    GPIOA_AFRL &= ~(0xF << (4 * 2));  // Çå³ýAFRL2
    GPIOA_AFRL |=  (0x7 << (4 * 2));  // AF7£¨USART2

    // 3. ÅäÖÃ²¨ÌØÂÊ£¨¼ÙÉèAPB1Ê±ÖÓÆµÂÊÎª42MHz£©
    USART2_BRR = 0x1117;

    // 4. ÆôÓÃUSART¹¦ÄÜ
    USART2_CR1 |= (1 << 13) |    // UE£¨Ê¹ÄÜUSART£©
                   (1 << 3);      // TE£¨Ê¹ÄÜ·¢ËÍ£©
}

// ×èÈûÊ½·¢ËÍµ¥¸ö×Ö·û
void USART2_SendChar(uint8_t ch) 
{
    while (!(USART2_SR & (1 << 7))); // µÈ´ýTXE±êÖ¾ÖÃÎ»
    USART2_DR = (ch & 0xFF);
}   

//·¢ËÍ×Ö·û´®
void USART2_SendStr(char *str)
{
	while(*str)
	{
		USART2_SendChar(*str++);
	}
}

void USART2_SendNumber(int32_t num, uint8_t base) {
    char buffer[32];  // »º³åÇø´óÐ¡×ã¹»´¦Àí32Î»¶þ½øÖÆÊý
    uint8_t i = 0;
    uint8_t is_negative = 0;

    // ²ÎÊýÐ£Ñé
   

    // ´¦Àí¸ºÊý£¨½öÏÞÊ®½øÖÆ£©
    if (num < 0 && base == 10) {
        is_negative = 1;
        num = -num;
    }

    // ´¦Àí0µÄÌØÊâÇé¿ö
    if (num == 0) {
        USART2_SendChar('0');
        return;
    }

    // Êý×Ö×ª×Ö·û£¨ÄæÐò´æ´¢£©
    while (num > 0) {
        uint32_t rem = num % base;
        buffer[i++] = (rem < 10) ? (rem + '0') : (rem - 10 + 'A');
        num /= base;
    }

    // Ìí¼Ó¸ººÅ
    if (is_negative) {
        buffer[i++] = '-';
    }

    // ÄæÐò·¢ËÍ×Ö·û
    while (i > 0) {
        USART2_SendChar(buffer[--i]);
    }
}

//·¢ËÍ×Ö½ÚÊý×é
void USART2_SendArray(uint8_t *array, uint16_t length)
{
	uint16_t i;
	for (i = 0; i < length; i++)
	{
		USART2_SendChar(array[i]);
	}
}

void CalculateChecksums(uint8_t *data, uint16_t length, uint8_t *sum_check, uint8_t *add_check)
{
	uint8_t sumcheck = 0;  // Ê¹Óƒuint8_tÈ·Â±Â£8Î»Â½Ø¶Ï
	uint8_t addcheck = 0;
	uint16_t i;
	
	for (i = 0; i < data[3]+4; i++)
	{
		sumcheck += data[i];  // SUM CHECK: Ã€Û¼ÓƒÂ¿Â¸Ã¶×–Â½Ú
		addcheck += sumcheck; // ADD CHECK: Ã€Û¼Ó³umcheckÂµÄ–Ð¼ä–µ
	}
	*sum_check = sumcheck;
	*add_check = addcheck;
}
