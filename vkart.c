/*
:set tabstop=8 softtabstop=0 expandtab shiftwidth=4 smarttab
 * Copyright (c) 2017-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== fatsdraw.c ========
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#if defined(__IAR_SYSTEMS_ICC__)
/*
 *  Prevent time() macro in time.h from being used instead of our
 *  generated time() function.
 */
#define _NO_DEFINITIONS_IN_HEADER_FILES 1
#endif

#include <time.h>

#if defined(__IAR_SYSTEMS_ICC__)
#undef _NO_DEFINITIONS_IN_HEADER_FILES
#endif

#include <third_party/fatfs/ff.h>

#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDFatFS.h>

//#include <ti/drivers/UART.h>
#include "printf/printf.h"
#include "colorful-printf/colorprint_header.h"

/* Driver configuration */
#include "ti_drivers_config.h"

/* Buffer size used for the file copy process */
#ifndef CPY_BUFF_SIZE
#define CPY_BUFF_SIZE       2048
#endif

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

#include <ti/devices/msp432p4xx/inc/msp.h>
/* Drive number used for FatFs */
#define DRIVE_NUM           0

char default_file[] = "DUMPFILE.BIN";
char default_burnfile[] = "BURNFILE.BIN";
char *file = default_file;
char *burnfile = default_burnfile;
const char inputfile[] = STR(DRIVE_NUM)":input.txt";
const char outputfile[] = STR(DRIVE_NUM)":output.txt";

#define RED_LED 1
#define GREEN_LED 2
#define BLUE_LED 4


unsigned char cpy_buff[CPY_BUFF_SIZE + 1];

FIL src;
FIL dst;

bool debug = 0;

/*
 *  ======== printDrive ========
 *  Function to print drive information such as the total disk space
 *  This function was created by referencing FatFs's API documentation
 *  http://elm-chan.org/fsw/ff/en/getfree.html
 *
 *  This function call may take a while to process, depending on the size of
 *  SD Card used.
 */
static void do_reset(void);
static void menu();
void do_sd2(void);
void do_sd(void);
void write_cfi(char *, void *, uint32_t);
int printf2(const char* format, ...);
// needed for printf()
void _putchar(char);
void _putchar2(char, void*);

static void uart_init(uint32_t speed) {
    P1->SEL1 &= 0xf3; //11110011  P1.2 P1.3
    P1->SEL0 |= 0x0c; //00001100  
    P3->SEL1 &= 0xf3; //          P3.2 P3.3
    P3->SEL0 |= 0x0c;
    


    EUSCI_A0->CTLW0 = 0x0081; // clk and reset state
    //EUSCI_A0->BRW = 26; // 3000000/115200 = 26
    //EUSCI_A0->BRW = 104; // 12000000/115200 = 104
    //EUSCI_A0->BRW = 208; // 24000000/115200 = 208
    EUSCI_A0->BRW = 24000000/speed;
    EUSCI_A0->MCTLW = 0x0000;
    EUSCI_A0->CTLW0 = 0x0081; // still in reset
    EUSCI_A0->CTLW0 = 0x0080; // out of reset
    EUSCI_A0->IFG &= ~1; // clear rx interrupt
    EUSCI_A0->IE |= 1;  // enable interrupt

    //__enable_irq(); // enable global interrupt
    //NVIC->ISER[0] = 1 << ((16) & 31);   // enable euscia in nvic module



/*    __disable_irq();
    EUSCI_A0->CTLW0 |= 1;
    EUSCI_A0->MCTLW = 0;
    EUSCI_A0->CTLW0 = 0x0081;
    EUSCI_A0->BRW = 24000000/speed;
    P1->SEL0 |= 0x0C;
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;
    EUSCI_A0->IE |= 0;
    NVIC_SetPriority(EUSCIA0_IRQn, 4);
    NVIC_EnableIRQ(EUSCIA0_IRQn);
    __enable_irq();
*/

    EUSCI_A2->CTLW0 = 0x0081;
    //EUSCI_A2->BRW = 26; // 3000000/115200 = 26
    //EUSCI_A2->BRW = 104; // 12000000/115200 = 104
    EUSCI_A2->BRW = 208; // 24000000/115200 = 208
    EUSCI_A2->MCTLW = 0x0000;
    EUSCI_A2->IE = 0;
    EUSCI_A2->CTLW0 = 0x0081;
    EUSCI_A2->CTLW0 = 0x0080;
}

static uint16_t uart_recv(void) {
    while(1) {
        if (EUSCI_A0->IFG & 1) break;
    }
    printf2("%c\r", EUSCI_A0->RXBUF & 0xff);
    return EUSCI_A0->RXBUF;
}

static void uart_send(uint16_t x) {
    while (1){
        if (EUSCI_A0->IFG & 2) break;
    }
    //printf2("TX %02x\r\n", EUSCI_A0->RXBUF & 0xff);
    EUSCI_A0->TXBUF = x;
}
static uint16_t uart_recv2(void) {
    while(1) {
        if (EUSCI_A2->IFG & 1) break;
    }
    return EUSCI_A2->RXBUF;
}

static void uart_send2(uint16_t x) {
    while (1){
        if (EUSCI_A2->IFG & 2) break;
    }
    EUSCI_A2->TXBUF = x;
}
void _putchar(char c) {
    uart_send(c);
}

void _putchar2(char c, void *p) {
    uart_send2(c);
}


static void led_on(uint8_t led){
    P2->OUT |= led;
}

static void led_off(uint8_t led){
    P2->OUT &= ~led;
}
static void led_toggle(uint8_t led){
    P2->OUT ^= led;
}
static void init_leds() {
    P2->SEL1 &= ~RED_LED;
    P2->SEL0 &= ~RED_LED;
    P2->DIR |= RED_LED;
    P2->SEL1 &= ~GREEN_LED;
    P2->SEL0 &= ~GREEN_LED;
    P2->DIR |= GREEN_LED;
    P2->SEL1 &= ~BLUE_LED;
    P2->SEL0 &= ~BLUE_LED;
    P2->DIR |= BLUE_LED;
    led_off(RED_LED|GREEN_LED|BLUE_LED);
}
void EUSCIA0_IRQHandler(void) {
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG));
        printf2("%02x ", EUSCI_A0->RXBUF);
        led_toggle(BLUE_LED);
    }
}
int printf2(const char* format, ...)
{
  va_list va;
  va_start(va, format);
  const int ret = fctprintf(_putchar2, NULL, format, va);
  va_end(va);
  return ret;
}

uint32_t read_hex(uint8_t len) {
    char x;
    uint8_t n = 0;
    uint32_t r = 0;

    while (n < len) {
        x = uart_recv();
        if (x == 0x1b) { return 0xffffffff; }
        if ((x >= '0' && x <= '9') || ( x >= 'A' && x <='F') || (x >= 'a' && x <= 'f') ) {
            n++;
            uart_send(x);
            r <<= 4;
            if (x < 0x40) { //0-9
               r |= (x-48);
            } else if (x < 0x50) { //A-F
                r |= (x-55);
            } else { //a-f
                r |= (x-87);
            }
        }
        //if (x == 0x7a && n>0) {
        //    n--;
        //}

        if (x == 0xd || x == 0x3) break;
    }
    return r;
}

char *entry83(char *label) {
    char x;
    char *filename = (char *)malloc(12 * sizeof(char));
    int n =0;
    if (label) printf(label);
    printf("____________\r");
    if (label) printf(label);
    
    for(n=0; n < 12; n++) {
        x = uart_recv();
        if (x == 0xd || x == 0x9) { break; }
        uart_send(x);
        filename[n] = x;
    }
    filename[n] = '\0';
    return(filename);
}

void port_status() {
    printf("PORT   4        7        9        6       5        10\r\n");
    printf("      [        ADDRESS         ] [     DATA      ]      E W\r\n");

    printf("INPUT %08b.%08b.%08b %08b.%08b %08b @%08x %04X %02x\r\n", 
        P4->IN, P7->IN, P9->IN, P6->IN, P5->IN, P10->IN,
        (P4->IN)<<16 | (P7->IN)<<8 | (P9->IN), (P6->IN)<<8 | (P5->IN), P10->IN);

    printf("->OUT %08b.%08b.%08b %08b.%08b %08b @%08x %04X %02x\r\n", 
        P4->OUT, P7->OUT, P9->OUT, P6->OUT, P5->OUT, P10->OUT,
        (P4->OUT)<<16 | (P7->OUT)<<8 | (P9->OUT), (P6->OUT)<<8 | (P5->OUT), P10->OUT);

    printf("->DIR %08b.%08b.%08b %08b.%08b %08b @%08x %04X %02x\r\n", 
        P4->DIR, P7->DIR, P9->DIR, P6->DIR, P5->DIR, P10->DIR,
        (P4->DIR)<<16 | (P7->DIR)<<8 | (P9->DIR), (P6->DIR)<<8 | (P5->DIR), P10->DIR);
}

#define PIN_CE 0x4
#define PIN_RW 0x1

void set_ce(bool state) {
    if (state) P10->OUT |= PIN_CE;
    else P10->OUT &= ~PIN_CE;
}
void set_rw(bool state) {
    if (state) P10->OUT |= PIN_RW;
    else P10->OUT &= ~PIN_RW;
}

void set_data_dir(bool state) {
    if (state) {
        PC->DIR = 0xffff;
    } else {
        PC->DIR = 0x0;
    }
}

void set_address(uint8_t hi, uint8_t mid, uint8_t lo) {
    P4->OUT= hi;
    P7->OUT= mid;
    P9->OUT= lo;
}

void set_data(uint16_t data) {
    //P5->OUT = lo;
    //P6->OUT = hi;
    PC->OUT = data;
}

uint16_t get_data(void) {
    //return P6->IN << 8 | P5->IN;
    return PC->IN;
}

void write_word(uint8_t hi, uint8_t mid, uint8_t lo, uint16_t word) {
    set_ce(true);
    set_rw(false);
    set_data_dir(true);
    set_address(hi, mid, lo);
    set_ce(false);
    set_data(word);
    set_ce(true);
}

uint16_t read_word(uint8_t hi, uint8_t mid, uint8_t lo) {
    uint16_t retval;
    set_data_dir(false);
    set_ce(true);
    set_rw(true);
    set_address(hi, mid, lo);
    set_ce(false);
    retval = get_data();
    //set_ce(true);
    return retval;
}
// NOTE:
// Read device code
// 21..........9876543210
//             Xxx0xx1111 manufacturer
//             Xxx0xx0001 device code
//             Xxx0xx0011 extended memory verify code
// aaaaaaaaaaxxXxx0xx0010 block protect status of addr block
//          10000 00000010
                        
uint16_t get_device_id() {
    write_word(0x0, 0x05, 0x55, 0xAA);
    write_word(0x0, 0x02, 0xAA, 0x55);
    write_word(0x0, 0x05, 0x55, 0x90);
    return read_word(0x0,0x0,0x1);
}
void dump_info() {
    write_word(0x0, 0x05, 0x55, 0xAA);
    write_word(0x0, 0x02, 0xAA, 0x55);
    write_word(0x0, 0x05, 0x55, 0x90);
    printf("Manufacturer ID: %04X\r\n", read_word(0x0,0x0,0x0));
    do_reset();
    write_word(0x0, 0x05, 0x55, 0xAA);
    write_word(0x0, 0x02, 0xAA, 0x55);
    write_word(0x0, 0x05, 0x55, 0x90);
    printf("Device ID: %04X\r\n", read_word(0x0,0x0,0x1));
    do_reset();
    write_word(0x0, 0x05, 0x55, 0xAA);
    write_word(0x0, 0x02, 0xAA, 0x55);
    write_word(0x0, 0x05, 0x55, 0x90);
    printf("Secured Silicon: %04X\r\n",read_word(0x0,0x0,0x3));
    do_reset();
    uint32_t sect;
    for(sect = 0 ; sect < 128; sect++) {
        write_word(0x0, 0x05, 0x55, 0xAA);
        write_word(0x0, 0x02, 0xAA, 0x55);
        write_word(0x0, 0x05, 0x55, 0x90);
        printf("SA%02x %04x ", sect, read_word((sect<<12)&0xff,(sect<<4)&0xff,0x2));
        if ((sect&7)==7) printf("\r\n");
        do_reset();
    }
}


uint32_t check_status(void) {
    uint32_t i;
    uint8_t byte1,byte2;
    for(i = 0; i < 0xffffff; i++) {

        byte1 = read_word(0,0,0) & 0xff;
        byte2 = read_word(0,0,0) & 0xff;
        if (byte1 == byte2) break; // since the toggle bit (bit6, or 2 in some other cases) in the status reg is toggling
                                   // at every read, it stops toggling when finished, so two
                                   // consequent fetch which are the same means the chip is now in read mode
        // note: should be better to check bit6 only, not the whole byte

    }
    return i; //number of cycles reported for the cmd execution
}

void erase_chip_29w() {
    write_word(0x00, 0x05, 0x55, 0x00aa);
    write_word(0x00, 0x02, 0xaa, 0x0055);
    write_word(0x00, 0x05, 0x55, 0x0080);
    write_word(0x00, 0x05, 0x55, 0x00aa);
    write_word(0x00, 0x02, 0xaa, 0x0055);
    write_word(0x00, 0x05, 0x55, 0x0010);
    sleep(80); // typical 80s
    printf("chip erased in %lu cycles", check_status());
}

void erase_block_29w(uint32_t address) {
    write_word(0x00, 0x05, 0x55,0xaa);
    write_word(0x00, 0x02, 0xaa,0x55);
    write_word(0x00, 0x05, 0x55,0x80);
    write_word(0x00, 0x05, 0x55,0xaa);
    write_word(0x00, 0x02, 0xaa,0x55);
    write_word((address>>16)&0xff,(address>>8)&0xff, address&0xff,0x30);
    usleep(100000); // typical time to erase block: 0.8s
    printf("block @%08x erased in %lu cycles", address, check_status());
}

void write_word_mx(uint8_t hi, uint8_t mid, uint8_t lo, uint16_t d1) {

    write_word(0x00,0x05,0x55,0x0AAA);
    write_word(0x00,0x02,0xAA,0x0555);
    write_word(0x00,0x05,0x55,0x00A0);
    write_word(hi,mid,lo, d1);
    usleep(10); // typical 10us
    //check_status();
}

void write_word_29w(uint8_t hi, uint8_t mid, uint8_t lo, uint16_t d1, uint16_t d2) {

    write_word(0x00,0x05,0x55,0x0050);
    write_word(hi,mid,lo, d1);
    write_word(hi,mid,lo+1, d2);
    usleep(10); // typical 10us
    //check_status();
}

void write_dword_29w2(uint32_t address, uint16_t *words, uint32_t len) {

    uint32_t addr;
    for(uint32_t i = 0; i < len; i+=2) {
        addr = address+i;

        write_word_29w((addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff,
                       words[i], words[i+1]);
    }
}
void program(uint32_t address, void *data, uint32_t len) {
    if (len != 512) {
        printf("Expected 0x200 bytes of data, got %8x\r\n", len);
        return;
    }
    printf("Write @%08x\r\n", address);
    for(uint16_t offset = 0; offset < 0x100; offset+=0x40) {

    }
}
void program_block(uint32_t address, void *data, uint32_t len) {
    if (len != 0x10000) {
        printf("Expected 0x10000 bytes of data, got 0x8x\r\n", len);
        return;
    }
    erase_block_29w(address);
    for(uint32_t addr = 0; addr < 0x8000; addr+= 0xff) {
        program(addr+address, (uint8_t *)data+addr, 512);
    }
}

void init_pins(void) {
    P10->SEL1 = 0x0;
    P10->SEL0 = 0x0;
    P5->SEL1 = 0x0;
    P5->SEL0 = 0x0;
    P6->SEL1 = 0x0;
    P6->SEL0 = 0x0;
    P4->SEL1 = 0x0;
    P4->SEL0 = 0x0;
    P7->SEL1 = 0x0;
    P7->SEL0 = 0x0;
    P9->SEL1 = 0x0;
    P9->SEL0 = 0x0;
    P10->DIR |= 0x5;
    set_data_dir(false);
    P4->DIR |=0xff;
    P7->DIR |=0xff;
    P9->DIR |=0xff;
}

static void do_reset() {
    write_word(0x0, 0x0, 0x0, 0x00f0);
}

char ascii(char s) {
  if(s < 0x20) return '.';
  if(s > 0x7E) return '.';
  return s;
}

void hexdump(void *d, uint32_t len, uint32_t base) {
  unsigned char *data;
  int i, off;
  data = (unsigned char*)d;
  for (off=0; off<len; off += 16) {
    printf("%08x ",base+off);
    for(i=0; i<16; i++)
      if((i+off)>=len) printf("   ");
      else printf("%02x ", data[off+i]);

    printf(" ");
    for(i=0; i<16; i++)
      if((i+off)>=len) printf(" ");
      else printf("%c", ascii(data[off+i]));
    printf("\r\n");
  }
}
void hexdump2(void *d, uint32_t len, uint32_t base) {
  unsigned char *data;
  int i, off;
  data = (unsigned char*)d;
  for (off=0; off<len; off += 16) {
    printf2("%08x ", base+off);
    for(i=0; i<16; i++)
      if((i+off)>=len) fctprintf(_putchar2, NULL, "   ");
      else printf2("%02x ", data[off+i]);

    printf2(" ");
    for(i=0; i<16; i++)
      if((i+off)>=len) printf2(" ");
      else printf2("%c",ascii(data[off+i]));
    printf2("\r\n");
  }
}

static void dump_menu() {
    printf("DUMP SubMenu\r\n");
    printf("<d> dump to [%s]\r\n", file);
    printf("<f> change filename\r\n");
    printf("<s> start block\r\n");
    printf("<e> end block\r\n");
    printf("<q> abort\r\n");
}
static void dump() {
    uint16_t block, start_block, end_block, page, i;
    unsigned int bytesWritten = 0;
    uint16_t data[64*256];

    SDFatFS_Handle sdfatfsHandle;
    FIL fil;

    start_block = 0;
    end_block = 0x3f;

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        printf("Error starting the SD card\n");
        return;
    } else {
        printf("mounted\r\n");
        FILINFO fno;

        uint8_t key = 0;
        dump_menu();
        while (key != 'd') {
            key = uart_recv();
            switch(key) {
                case 'f':
                    file = entry83("enter a filename: ");
                    dump_menu();
                    break;
                case 's':
                    printf("start block (0x00-0x3f): ");
                    start_block = read_hex(2);printf("\r\n");
                    printf("will dump from %02x to %02x\r\n", start_block, end_block);
                    dump_menu();
                    break;
                case 'e':
                    printf("end block (0x01-0x40): ");
                    end_block = read_hex(2);printf("\r\n");
                    printf("will dump from %02x to %02x\r\n", start_block, end_block);
                    dump_menu();
                    break;
                case 'q':
                    printf("abort\r\n");
                    goto dump_abort;
            }
        }
        if (FR_OK == f_stat(file, &fno)) {
            if (FR_OK == f_unlink(file)) {
                printf("file deleted\r\n");
            } else {
                printf("failed to delete file\r\n");
                return;
            }
        } else {
            printf("no file, will be created\r\n");
        }
        if (!f_open(&fil, file, FA_CREATE_NEW|FA_WRITE)) {

            init_pins();
            do_reset();
            led_on(BLUE_LED);
            progress_start(1, end_block-start_block, NULL);

            for(block = start_block; block <= end_block; block++) {
                //printf("\rDump block %03d/%03d", block, end_block);
                progress_update(1);
                for (short quarter = 0; quarter < 4; quarter++) { // 16KW per quarter
                    for(page = 0; page < 0x40; page++) {
                        for(i = 0; i<=0x100;i++) data[(page<<8)|i] = read_word(block, (quarter<<6)|page, i);
                    }
                    f_write(&fil, data, 1<<15, &bytesWritten);
                    if (bytesWritten != 1<<15) {
                        printf("write error, only 0x%04x bytes written\r\n");
                    }
                    led_toggle(GREEN_LED);
                }
            }
            f_sync(&fil);
            f_close(&fil);
        }
dump_abort:
        SDFatFS_close(sdfatfsHandle);
        printf("\r\nDONE\r\n");
        led_off(BLUE_LED);
    }
}

static void burn_menu() {
    printf("BURN SubMenu\r\n");
    printf("<1> single word\r\n");
    printf("<2> double word\r\n");
    printf("<g> burn\r\n");
    printf("<s> start block\r\n");
    printf("<e> end block\r\n");
    printf("<n> erase mode\r\n");
    printf("<q> abort\r\n");
}

static void burn() {
    uint16_t block, i;
    //uint32_t status;
    unsigned int bytesRead = 0;
    uint16_t data[0x2000];
    uint16_t device_id = 0;
    uint8_t write_mode = 1;
    uint8_t first_block = 0;
    uint8_t last_block = 0x80;
    bool erase = true;

    SDFatFS_Handle sdfatfsHandle;
    FIL fil;

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        printf("Error starting the SD card\n");
        return;
    } else {
        printf("sd mounted\r\n");
        FILINFO fno;
        if (FR_OK == f_stat(burnfile, &fno)) {
            printf("burnfile found\r\n");
        } else {
            printf("%s not found on sdcard\r\n", burnfile);
            return;
        }
        if (!f_open(&fil, burnfile, FA_READ)) {

            init_pins();
            do_reset();
            device_id = get_device_id();
            do_reset();
            if (device_id == 0x22cb || device_id == 0x22c9) {
                write_mode = 1;
                printf("MX29W640 detected, single word program supported\r\n");
            }
            if (device_id == 0x22fd || device_id == 0x22ed) {
                write_mode = 1;
                printf("ST/Numonyx M29W640 detected, double word program supported\r\n");
            }
            uint8_t key = 0;
            burn_menu();
            while (key != 'g') {
                key = uart_recv();
                switch(key) {
                    case '1':
                        printf("single word mode\r\n");
                        write_mode = 1;
                        break;
                    case '2':
                        printf("double word mode\r\n");
                        write_mode = 2;
                        break;
                    case 'f':
                        burnfile = entry83("enter a filename: ");
                        burn_menu();
                        break;
                    case 's':
                        printf("start block (0x00-0x7f): ");
                        first_block = read_hex(2);printf("\r\n");
                        printf("will burn blocks from %02x to %02x\r\n", first_block, last_block);
                        break;
                    case 'e':
                        printf("end block (0x01-0x80): ");
                        last_block = read_hex(2);printf("\r\n");
                        printf("will burn blocks from %02x to %02x\r\n", first_block, last_block);
                        break;
                    case 'n':
                        erase ^= 1;
                        printf("%serase\r\n", (erase?"":"no "));
                        break;
                    case 'q':
                        printf("abort\r\n");
                        goto abort;
                }
            }

            led_on(RED_LED);
            for(block = first_block; block < last_block; block++) { // 128 blocs of 64Kb aka 0x8000 words
                if (erase) {
                    printf("Erase block %08x\r\n", block<<15);
                    erase_block_29w(block<<15);
                }
                printf("Burn block %02x\r\n", block);
                            

                for (short quarter = 0; quarter < 4; quarter++) { // 8KW per quarter
                    f_read(&fil, data, 1<<14, &bytesRead); // so read 16Kb
                    if (bytesRead != 1<<14) {
                        printf("Read error, only %04x bytes read\r\n", bytesRead);
                    }
                    for (i = 0; i < 0x2000; i+=2 ) { // 8KW
                        uint16_t d1 = data[i];
                        uint16_t d2 = data[i+1];
                        uint32_t addr = block*4*0x2000 + quarter*0x2000 + i;
                        if (write_mode == 2) {
                            write_word_29w((addr>>16)&0xff, (addr>>8)&0xff, addr&0xff, d1, d2);
                        }
                        if (write_mode == 1) {
                            write_word_mx((addr>>16)&0xff, (addr>>8)&0xff, addr&0xff, d1);
                            write_word_mx((addr>>16)&0xff, (addr>>8)&0xff, (addr+1)&0xff, d2);
                        }
                    }
                    led_toggle(GREEN_LED);
                }
            }

abort:
            f_close(&fil);
        } else {
            printf("fopen %s failed\r\n", burnfile);
        }
        SDFatFS_close(sdfatfsHandle);
        printf("DONE\r\n");
        led_off(RED_LED);
    }
}

static void read_cfi2() {
    uint8_t i, j;
    uint16_t data[0x8*0x10];
    init_pins();
    do_reset();
    write_word(0x0, 0x0, 0x55, 0x0098);
    for(i=0x0; i< 0x08; i++) {
        for(j = 0x0; j < 0x10; j++) {
            data[i*0x10+j] = (uint16_t)read_word(0,0, i*0x10+j);
        }
        //printf("\r\n");
    }
    hexdump(data, 256, 0x0);
    //write_cfi("CFI.BIN", (void *)data, 128);
    printf("device id: %04x\r\n", get_device_id());
    do_reset();
}
static void read_cfi() {
    uint8_t i, j;
    uint8_t data[0x8*0x10];
    init_pins();
    do_reset();
    write_word(0x0, 0x0, 0x55, 0x0098);
    for(i=0x0; i< 0x08; i++) {
        for(j = 0x0; j < 0x10; j++) {
            data[i*0x10+j] = (uint8_t)read_word(0,0, i*0x10+j) & 0xff;
        }
        //fprint("\r\n");
    }
    hexdump(data, 128, 0x0);
    //write_cfi("CFI.BIN", (void *)data, 128);
    printf("Device id: %04x\r\n", get_device_id());
    do_reset();
}

#define CMDDATALEN 0x204
#define READ  0x00
#define WRITE 0x01
#define PEEK  0x02
#define POKE  0x03
#define SETUP 0x10
#define START 0x20
#define STOP  0x21
#define CALL  0x30
#define EXEC  0x31
#define NOK   0x7E
#define OK    0x7F

#define MONITOR 0x00
#define DUMP 0x80
#define DEBUGAPP 0xFF


#define DEBUGSTR 0xFF
void handle(uint8_t app, uint8_t verb, uint32_t len);
//! Transmit a header.
void txhead(uint8_t app, uint8_t verb, uint32_t len);
//! Transmit data.
void txdata(uint8_t app, uint8_t verb, uint32_t len);
//! Transmit a string.
void txstring(uint8_t app, uint8_t verb, const char *str);

//! Receive a long.
uint32_t rxlong();
//! Receive a word.
uint16_t rxword();

//! Transmit a long.
void txlong(uint32_t l);
//! Transmit a word.
void txword(uint16_t l);

//! Transmit a debug string.
void debugstr(const char *str);
//! brief Debug a hex word string.
void debughex(uint16_t v);


uint8_t cmddata[CMDDATALEN];
uint8_t silent = 0;

void txstring(uint8_t app, uint8_t verb, const char *str) {
    uint32_t len = strlen(str);
    txhead(app, verb, len);
    while (len--) {
        uart_send(*(str++));
    }
}
void debugstr(const char *str) {
    txstring(0xFF, 0xFF, str);
}
void debughex(uint16_t v) {
    char a[7];
    a[0]='0'; a[1]='x';
    a[2]=0xf&(v>>12);

    a[2]+=(a[2]>9)?('a'-10):'0';

    a[3]=0xf&(v>>8);
    a[3]+=(a[3]>9)?('a'-10):'0';

    a[4]=0xf&(v>>4);
    a[4]+=(a[4]>9)?('a'-10):'0';

    a[5]=0xf&(v>>0);
    a[5]+=(a[5]>9)?('a'-10):'0';

    a[6]=0;

  txstring(0xFF,0xFF,a);
}

void debugbytes(const uint8_t *bytes, uint32_t len){
  uint16_t i;
  txhead(0xFF,0xFE,len);
  for(i=0;i<len;i++)
    uart_send(bytes[i]);
}

void txhead(uint8_t app, uint8_t verb, uint32_t len) {
    uart_send(app);
    uart_send(verb);
    txword(len);
}
void txdata(uint8_t app, uint8_t verb, uint32_t len) {
    uint32_t i=0;
    if (silent) return;
    txhead(app, verb, len);
    for(i=0; i<len; i++) {
        uart_send(cmddata[i]);
    }
}
uint32_t rxlong() {
    uint32_t toret=0;
    toret=uart_recv();
    toret|=(((uint32_t)uart_recv())<<8);
    toret|=(((uint32_t)uart_recv())<<16);
    toret|=(((uint32_t)uart_recv())<<24);
    return toret;
}
uint16_t rxword() {
    uint16_t toret=0;
    toret=uart_recv();
    toret|=(((uint16_t)uart_recv())<<8);
    return toret;
}

void txlong(uint32_t l) {
    uart_send(l&0xff);
    l>>=8;
    uart_send(l&0xff);
    l>>=8;
    uart_send(l&0xff);
    l>>=8;
    uart_send(l&0xff);
    l>>=8;
}
void txword(uint16_t l) {
    uart_send(l&0xff);
    l>>=8;
    uart_send(l&0xff);
    l>>=8;
}

static uint8_t flash_buffer[512];

void dumphandle(uint8_t app, uint8_t verb, uint32_t len) {
    uint16_t word;
    uint8_t a1,a2,a3,blen;
    debugstr("ohai");
    led_on(RED_LED);
    printf2("In dumphandle\r\n");
    switch (verb) {
        default:
            debugstr("ERROR: Command unsupported.");
        case 0xCE: 
            led_on(GREEN_LED);
            txdata(app,verb, 1);
            break;
        case 0xCF:
            led_off(GREEN_LED);
            txdata(app,verb, 1);
            break;
        case 0xD0:      // set #CE
            set_ce(cmddata[0]);
            txdata(app, verb, 1);
            break;
        case 0xD1:      // set R/#W
            set_rw(cmddata[0]);
            txdata(app, verb, 1);
            break;
        case 0xD2:      // data dir - 1: output 0: input
            set_data_dir(cmddata[0]);
            txdata(app,verb,1);
            break;
        case 0xD3:      // set address
            set_address(cmddata[0], cmddata[1], cmddata[2]);
            txdata(app, verb, 1);
            break;
        case 0xD4:      // set data
            set_data((cmddata[0] << 8)| cmddata[1]);
            txdata(app,verb,1);
            break;
        case 0xD5:      // get data
            word = get_data();
            cmddata[0] = word >> 8;
            cmddata[1] = word & 0xff;
            txdata(app, verb, 2);
            break;
        case 0xD7:     // read word
            word = read_word(cmddata[0], cmddata[1], cmddata[2]);
            cmddata[0] = word >> 8;
            cmddata[1] = word & 0xff;
            txdata(app, verb, 2);
            break;
        case 0xD9:    // read words
            blen = cmddata[0];
            a1 = cmddata[1];
            a2 = cmddata[2];
            a3 = cmddata[3];
            if (blen * 2 > sizeof(flash_buffer)) {
                debugstr("Error: overflow in read_words");
                txdata(app, verb, 1);
                break;
            }
            for(int i  = 0; i < 128; i++) {
                word = read_word(a1, a2, a3+i);
                cmddata[i*2+1] = word >> 8;
                cmddata[i*2] = word & 0xff;
            }
            txdata(app, verb, 128*2);
            break;
    }
    
}
void monitorhandle(uint8_t app, uint8_t verb, uint32_t len) {
    printf2("in monitorhandle\r\n");
}
void auto_init() {
    printf2("in auto_init\r\n");
}

void handle(uint8_t app, uint8_t verb, uint32_t len) {
    printf2("in handle %02x %02x %08x\r\n", app, verb, len);
    switch(app) {
        case 0x81:
            dumphandle(app, verb, len);
            break;
        case MONITOR:
            monitorhandle(app, verb, len);
            break;
        default:
            txdata(app, NOK, 0);
            break;
    }
}

void auto_main(void) {
    volatile uint32_t i;
    uint8_t app, verb;
    uint32_t len;

    printf2("in auto_main\r\n");
    auto_init();
    //txstring(MONITOR, OK, "goodgood");
    
    printf2("starting auto_main while(1)\r\n");
    //while (1) {
        app=uart_recv();
        printf2("app = %02x\r\n", app);
        verb = uart_recv();
        len=rxword();
        if (len <= CMDDATALEN) {
            for(i=0; i<len;i++) {
                cmddata[i]=uart_recv();
            }
            handle(app, verb, len);
        } else {
            for(i=0; i<len;i++) {
                uart_recv();
            }
            txdata(MONITOR, NOK, 0);
        }
    //}
}

void display_page(uint32_t base) {
    uint16_t data[0xff];
    for(uint32_t a = 0; a < 0x100; a++) {
        // read ff words
        uint32_t addr = base + a;
        data[a] = read_word((addr >> 16)&0xff, (addr>>8)&0xff, addr&0xff);
    }
    printf("--------v-----------------------------------------------------------------\r\n");
    hexdump(data, 0x100, base);
}

void explorer() {
    uint32_t base_address = 0x8000;
    char key;

    display_page(base_address&0xffffff);
    printf("[q] quit [a] set address [n] next page [N] next block [p] prev page [P] prev block\r\n");
    while (1) {
        key = uart_recv();
        if ((key == 'n' || key == 0xd) && (base_address < 0xffff00)) base_address+=0x100;
        if (key == 'N' && (base_address < 0x7f8000)) base_address+=0x8000;
        if (key == 'p' && (base_address > 0xff)) base_address-=0x100;
        if (key == 'P' && (base_address > 0x7fff)) base_address-=0x8000;
        if (key == 'a') base_address=read_hex(6);
        if (key == 'q') break;
        printf("\r");
        display_page(base_address&0xffffff);
        printf("[q] quit [a] set address [n] next page [N] next block [p] prev page [P] prev block\r\n");
    }
    menu();
}

void sneek_a(void) {
    uint32_t a;

    while (a != 0xffffffff) {
        printf("Enter XXXXXX address: ");
        a = read_hex(6); printf("\r\n");
        printf("%04x\r\n", read_word((a >> 16)&0xff, (a>>8)&0xff, a&0xff));

    }
}

void read_a_spot(void) {
    uint32_t a;
    uint16_t w;
    while (1) {
        printf("Enter XXXXXX address: ");
        a = read_hex(6); printf("\r\n");
        w = read_word((a >> 16)&0xff, (a>>8)&0xff, a&0xff);
        port_status();
        printf("AT %08x -> %04x\r\n", a, w);
    }
}

static void check_content(char *name, uint32_t addr, uint32_t len) {
    FIL fp;
    uint8_t *data;
    uint32_t dataread;
    SDFatFS_Handle sdfatfsHandle;

    if (len == 0) {
        printf("Enter addr: "); addr = read_hex(6);
        printf("Enter len: "); len = read_hex(2);
        printf("\r\n");
    }
    data = malloc(len);

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);

    if (!f_open(&fp,name, FA_READ)) {
        if (!f_lseek(&fp, addr)) {
            if (!f_read(&fp, data, len, (UINT*)&dataread)) {
                hexdump(data, len,addr);
            }
        }
        f_close(&fp);
    }
    free(data);
    SDFatFS_close(sdfatfsHandle);
}

#define NORMAL "0"
#define INVERT "7"
#define CYAN "36"
#define ESC 0x1b
static char normal[] = { 0x1b, 0x5b, 0x30, 0x6d, 0 };
static char cyan[] = { 0x1b, 0x5b, 0x33, 0x36, 0x6d, 0 };
static char red[] = { 0x1b, 0x5b, 0x33, 0x31, 0x6d, 0 };
static char green[] = { 0x1b, 0x5b, 0x33, 0x32, 0x6d, 0 };
static char invert[] = { 0x1b, 0x5b, 0x37, 0x6d, 0 };

static void menu() {
    //printf("%sV.Kart Menu%s\r\n", invert, normal);
    printf_color(1, "[lr]V[/lr][lg].[/lg][lb]K[/lb][lc]a[/lc][lm]r[/lm][ly]t[/ly] Menu\r\n");
    printf("<c> CFI\r\n");
    printf("<a> sneek at an address\r\n");
    printf("<w> peek flash address\r\n");
    printf("<e> Explore flash\r\n");
    printf("<F> Flash Erase\r\n");
    printf("<d> Dump to [%s%s%s]\r\n", green, file, normal);
    printf("<b> Burn from [%s%s%s]\r\n", red, burnfile, normal);
    printf("<f> change filenames submenu\r\n");
    printf("<D> toggle debug mode\r\n");
    printf("<p> show ports status\r\n");
    printf("<i> read an hex value\r\n");
    printf("<t> test various things\r\n");
    printf("<l> DIR list sdcard\r\n");
    printf("<S> check flash status\r\n");
    printf("<x> check content in the saved file\r\n");
    printf("<X> reset flash\r\n");
    printf("<q> exit\r\n");
    printf(debug?"DBG":"");
}

void printDrive(const char *driveNumber, FATFS **fatfs)
{
    FRESULT fresult;
    DWORD   freeClusterCount;
    DWORD   totalSectorCount;
    DWORD   freeSectorCount;

    printf("Reading disk information...");

    fresult = f_getfree(driveNumber, &freeClusterCount, fatfs);
    if (fresult) {
        printf("Error getting the free cluster count from the FatFs object (%08x)", fresult);
        while (1);
    }
    else {
        printf("done\r\n");

        /* Get total sectors and free sectors */
        totalSectorCount = ((*fatfs)->n_fatent - 2) * (*fatfs)->csize;
        freeSectorCount  = freeClusterCount * (*fatfs)->csize;

        printf("Total Disk size: %10lu KiB\r\n", totalSectorCount / 2);
        printf("Free Disk space: %10lu KiB\r\n", freeSectorCount / 2);
        printf("\r\n");
    }
}
enum SEGS { A_HI, A_MID, A_LO, D_HI, D_LO };
void bit_twidling_menu() {
    uint8_t *seg, high, mid, low, data_hi, data_low, cmd=0;
    while (cmd != 'q') {
        printf("%08b %08b %08b  %08b %08b\r\n", high, mid, low, data_hi, data_low);
        //printf("%s ", 
        cmd = uart_recv();
        switch(cmd) {
            case '1':
                *seg ^= 1; break;
            case '2':
                *seg ^= 1<<1; break;
            case '3':
                *seg ^= 1<<2; break;
            case '4':
                *seg ^= 1<<3; break;
            case '5':
                *seg ^= 1<<4; break;
            case '6':
                *seg ^= 1<<5; break;
            case '7':
                *seg ^= 1<<6; break;
            case '8':
                *seg ^= 1<<7; break;
        
            case 'a':
                seg = &high; break;
            case 's':
                seg = &mid; break;
            case 'd':
                seg = &low; break;

            case 'f':
                seg = &data_hi; break;
            case 'g':
                seg = &data_low; break;
        }
    }
    printf("Back to main menu\r\n");
}

/*
 *  ======== mainThread ========
 *  Thread to perform a file copy
 *
 *  Thread tries to open an existing file inputfile[]. If the file doesn't
 *  exist, create one and write some known content into it.
 *  The contents of the inputfile[] are then copied to an output file
 *  outputfile[]. Once completed, the contents of the output file are
 *  printed onto the system console (stdout).
 */
void *mainThread(void *arg0)
{
    uint8_t app,verb, len;


    if (0) { //???
    // select 48MHz external oscillator on PJ.2/PJ.3
    PJ->SEL0 |= 0x0C;
    PJ->SEL1 &= ~0x0C;
    CS->KEY = 0x695A;
    CS->CTL0 = CS_CTL0_DCORSEL_5;
    CS->CTL1 = CS_CTL1_SELA__REFOCLK | CS_CTL1_SELS__DCOCLK | CS_CTL1_SELM__DCOCLK;
    CS->KEY = 0;
    }

    GPIO_init();
    SDFatFS_init();

    uart_init(115200);
    init_pins();
    init_leds();

    //char status;
    uint8_t cmd=0;
    menu();
    //printf2("SECOND CONSOLE\r\n");
    while (cmd != 'q') {
        cmd = uart_recv();

        switch(cmd) {
        case '1':
            printf("switching to 115200\r\n");
            uart_init(115200);
            menu();
            break;
        case '2':
            printf("switching to 230400\r\n");
            uart_init(230400);
            menu();
            break;
        case '3':
            printf("switching to 460800\r\n");
            uart_init(460800);
            menu();
            break;
        case '4':
            printf("switching to 921600\r\n");
            uart_init(921600);
            menu();
            break;
        case 'c':
            read_cfi();
            break;
        case 'C':
            read_cfi2();
            break;
        case 'b':
            burn();
            break;
        case 'A':
            uart_send(0x41);
            cmd = uart_recv();
            if (cmd == 'A') auto_main();
            break;
        case 'a':
            sneek_a();
            break;
        case 'd':
            dump();
            break;
        /*case '=':
            write_word_29w(0x0, 0x0, 0x0, 0xf00f, 0x0ff0);
            write_word_mx(0x0,0x0,0x10, 0xabcd);
            printf("written\r\n");
            break;*/
        case 'D':
            debug ^= 1;
            printf("%s",debug?"debug on\r\n":"debug off\r\n");
            break;
        case 'e':
            explorer();
            break;
        case 'f':
            printf("  <d> change dump filename\r\n");
            printf("  <b> change burn filename\r\n");
            char kk = uart_recv();
            if (kk == 'd') file = entry83("enter a filename: ");
            if (kk == 'b') burnfile = entry83("enter a filename: ");
            menu();
            break;
        case 'p':
            port_status();
            break;
        case 'i':
            printf("%08x\r\n", read_hex(4));
            break;
        case 'T':
            printf_color(1, "\nHere is your rainbow: ");
            printf_color(1, "[r]R[/r][g]a[/g][b]i[/b][c]n[/c][m]b[/m][y]o[/y][w]w[/w] ");
            printf_color(1, "[lr]R[/lr][lg]a[/lg][lb]i[/lb][lc]n[/lc][lm]b[/lm][ly]o[/ly][lw]w[/lw] ");
            break;
        case 't':
            printf("TEST\r\n");



            printf("%08x\r\n", get_fattime());
            P2->DIR |= 2;
            P2->OUT |= 2;
            sleep(1);
            P2->OUT &= ~2;
            sleep(1);
            printf("%08x\r\n", get_fattime()); // 4A210000
                                  //
            break;                // 0100 1010 0010 0001 0000 0000 0000 0000
        case 'l':                 // |||| |||\ ||/  \||| /
                                  // 0100101  0001  0001
                                  //   2017   Jan     1
            do_sd2();
            break;
        case 'S':
            dump_info();
            break;
        case 'x':
            check_content("DUMP.BIN", 0x0, 0);
            break;
        case 'X':
            do_reset();
            printf("Reset Done\r\n");
            break;
        case 'w':
            read_a_spot();
            break;
        case 'F':
            printf("Erase full chip, should take 80s\r\n");
            erase_chip_29w();
            break;
        case '?':
            menu();
            break;
        case '_':
            printf("RW false\r\n");
            set_rw(false);
            break;
        case '+':
            printf("RW true\r\n");
            set_rw(true);
            break;
        case '-':
            printf("CE false\r\n");
            set_ce(false);
            break;
        case '=':
            printf("CE true\r\n");
            set_ce(true);
            break;
        case '*':
            bit_twidling_menu();
            break;
        case 0x81:
            printf2("81 received\r\n");
            app = 0x81;
            verb = uart_recv();
            len = uart_recv();
            handle(app, verb, len);
        }
        cmd = 0;
    }
    printf("END\r\n");
    return NULL;
}

FRESULT scan_files(char *path) {
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;
    printf("scan path %s\r\n", path);
    res = f_opendir(&dir, path);
    printf("%d\r\n", res);
    if (res == FR_OK) {
        for(;;){
            //printf("reading\r\n");
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_DIR){
                printf("subdir\r\n");
                i =strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
                //printf("file\r\n");
                FIL fp;
                int sz;
                if (!f_open(&fp,fno.fname, FA_READ)) {
                    sz = f_size(&fp);
                    f_close(&fp);
                } else { 
                    sz = -1;
                }

                printf("%08x (%10lu) %s/%s\r\n", sz, sz, path, fno.fname);
            }
        }
        f_closedir(&dir);
        printf("End\r\n");
    }
    return res;
}

void do_sd2(void) {


    SDFatFS_Handle sdfatfsHandle;
    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        printf("Error starting the SD card\n");
        return;
    } else {
        printf("mounted\r\n");
    }
    printDrive(STR(DRIVE_NUM), &(dst.obj.fs));
    scan_files("0:/");
    SDFatFS_close(sdfatfsHandle);
}

void write_cfi(char *filename, void *data, uint32_t len) {
    //unsigned int bytesRead = 0;
    unsigned int bytesWritten = 0;

    //unsigned int totalBytesCopied = 0;

    SDFatFS_Handle sdfatfsHandle;


    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        printf("Error starting the SD card\n");

        while (1);
    } else {
        printf("mounted\r\n");
    }
    printDrive(STR(DRIVE_NUM), &(dst.obj.fs));
    //scan_files("0:/");

    if (FR_OK == f_open(&src, filename, FA_CREATE_NEW|FA_READ|FA_WRITE)) {
        f_write(&src, data, len, &bytesWritten);
        f_sync(&src);
        f_close(&src);
    }


    SDFatFS_close(sdfatfsHandle);


}

/*
 *  ======== fatfs_getFatTime ========
 */
int32_t fatfs_getFatTime(void)
{
    /*
     *  FatFs uses this API to get the current time in FatTime format.  User's
     *  must implement this function based on their system's timekeeping
     *  mechanism.  See FatFs documentation for details on FatTime format.
     */
    /* Jan 1 2017 00:00:00 */
    return (0x4A210000);
}
