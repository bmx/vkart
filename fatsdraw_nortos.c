/*
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

char file[] = "DUMPFILE.BIN";
char burnfile[] = "BURNFILE.BIN";
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
void do_sd2(void);
void do_sd(void);
void write_cfi(char *, void *, uint32_t);

static void uart_init(void) {
    PA->SEL1_L &= 0xf3;
    PA->SEL0_L |= 0x0c;

    EUSCI_A0->CTLW0 = 0x0081;
    //EUSCI_A0->BRW = 26; // 3000000/115200 = 26
    //EUSCI_A0->BRW = 104; // 12000000/115200 = 104
    EUSCI_A0->BRW = 208; // 24000000/115200 = 208
    EUSCI_A0->MCTLW = 0x0000;
    EUSCI_A0->IE = 0;
    EUSCI_A0->CTLW0 = 0x0081;
    EUSCI_A0->CTLW0 = 0x0080;
}

static uint16_t uart_recv(void) {
    while(1) {
        if (EUSCI_A0->IFG & 1) break;
    }
    return EUSCI_A0->RXBUF;
}

static void uart_send(uint16_t x) {
    while (1){
        if (EUSCI_A0->IFG & 2) break;
    }
    EUSCI_A0->TXBUF = x;
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
void print(char *str) {
    uint32_t i = 0;
    while (str[i] != 0) {
        uart_send(str[i++]);
    }
}

void bitout(uint32_t b, uint8_t len) {
    for(int i = len-1; i>=0; i--) if (b & 1 << i) print("1"); else print("0");
}

#define bit8(x)      bitout(x, 8)
#define bit16(x)     bitout(x, 16)
#define bit32(x)     bitout(x, 32)

void hexout(uint32_t d, uint8_t len, bool cr) {
    uint32_t rb, rc;
    rb = len;
    while (1){
        rb-=4;
        rc=(d>>rb)&0xf;
        if(rc>9) rc+=0x37; else rc+=0x30;
        uart_send(rc);
        if(rb==0) break;
    }
    if (cr) {
        uart_send(0x0d);
        uart_send(0x0a);
    } else
        uart_send(0x20);
}

#define hex8(x)    hexout(x, 8, true)
#define hex16(x)    hexout(x, 16, true)
#define hex32(x)    hexout(x, 32, true)
#define hex8s(x)    hexout(x, 8, false)
#define hex16s(x)    hexout(x, 16, false)
#define hex32s(x)    hexout(x, 32, false)
/*void hex8(uint32_t x) { hexout(x, 8, true); }
void hex16(uint32_t x) { hexout(x, 16, true); }
void hex32(uint32_t x) { hexout(x, 32, true); }
void hex8s(uint32_t x) { hexout(x, 8, false); }
void hex16s(uint32_t x) { hexout(x, 16, false); }
void hex32s(uint32_t x) { hexout(x, 32, false); }*/

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
    //  0  1  2  3  4  5  6  7  8  9 10 11
    // [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]     m=0, n=0
    //  a                                       n=1

    char x,*file = malloc(12);
    int n =0;
    if (label) print(label);
    while (n < 12) {
        x = uart_recv();
        uart_send(x);
        if (x == 0xd || x == 0x9) break;
        if (n==7) file[++n]='.';
        file[n]=x;
        n++;
    }
    file[n]='\0';
    print("\r\n");
    return(file);
}

void port_status() {
    print("PORT  ");
    print("4 ("); hex16s(0); print(")");
    print("7 ("); hex16s(0); print(")");
    print("9 ("); hex16s(0); print(")");
    print("6 ("); hex16s(0); print(")");
    print("5 ("); hex16s(0); print(")");
    print("10("); hex16s(0); print(")\r\n");
    print("      [        ADDRESS         ] [     DATA      ]      E W\r\n");

    print("INPUT ");
    bit8(P4->IN); print("."); bit8(P7->IN); print("."); bit8(P9->IN); print(" ");
    bit8(P6->IN); print("."); bit8(P4->IN); print(" "); bit8(P10->IN);
    print(" @"); hex32s((P4->IN)<<16 | (P7->IN)<<8 | (P9->IN));
    print(" "); hex16s((P6->IN)<<8 | (P5->IN));
    print(" "); hex8s(P10->IN); print("\r\n");

    print("->OUT ");
    bit8(P4->OUT); print("."); bit8(P7->OUT); print("."); bit8(P9->OUT); print(" ");
    bit8(P6->OUT); print("."); bit8(P5->OUT); print(" "); bit8(P10->OUT);
    print(" @"); hex32s((P4->OUT)<<16 | (P7->OUT)<<8 | (P9->OUT));
    print(" "); hex16s((P6->OUT)<<8 | (P5->OUT));
    print(" "); hex8s(P10->OUT); print("\r\n");

    print("->DIR ");
    bit8(P4->DIR); print("."); bit8(P7->DIR); print("."); bit8(P9->DIR); print(" ");
    bit8(P6->DIR); print("."); bit8(P5->DIR); print(" "); bit8(P10->DIR);
    print(" @"); hex32s((P4->DIR)<<16 | (P7->DIR)<<8 | (P9->DIR));
    print(" "); hex16s((P6->DIR)<<8 | (P5->DIR));
    print(" "); hex8s(P10->DIR); print("\r\n");

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
    //if(debug) { print("DBG: set_address("); hex8s(hi); hex8s(mid); hex8s(lo); print(")\r\n"); }
    P4->OUT= hi;
    P7->OUT= mid;
    P9->OUT= lo;
}

void set_data(uint16_t data) {
    //if(debug) { print("DBG: set_data("); hex8s(hi); hex8s(lo); print(")\r\n"); }
    //P5->OUT = lo;
    //P6->OUT = hi;
    PC->OUT = data;
}

uint16_t get_data(void) {
    //return P6->IN << 8 | P5->IN;
    //if (debug) {
    //    print("DBG: get_data() returns "); hex16(res);
    //}
    //return res;
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
    //if (debug) {
    //    print("DBG: read_word("); hex8s(hi); hex8s(mid); hex8s(lo); print(")\r\n");
    //}
    set_data_dir(false);
    set_ce(true);
    set_rw(true);
    set_address(hi, mid, lo);
    set_ce(false);
    retval = get_data();
    //set_ce(true);
    //if (debug) {
    //    print("DBG: read_word() returns "); hex16(retval);
    //}
    return retval;
}
// NOTE:
// Read device code
// 21..........9876543210
//             Xxx0xx1111 manufacturer
//             Xxx0xx0001 device code
//             Xxx0xx0011 extended memory verify code
// aaaaaaaaaaxxXxx0xx0010 block protect status of addr block

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
    print("chip erased in "); hex32(check_status());
}

void erase_block_29w(uint32_t address) {
    write_word(0x00, 0x05, 0x55,0xaa);
    write_word(0x00, 0x02, 0xaa,0x55);
    write_word(0x00, 0x05, 0x55,0x80);
    write_word(0x00, 0x05, 0x55,0xaa);
    write_word(0x00, 0x02, 0xaa,0x55);
    write_word((address>>16)&0xff,(address>>8)&0xff, address&0xff,0x30);
    usleep(100000); // typical time to erase block: 0.8s
    print("block @"); hex32s(address); print("erased in "); hex32(check_status());
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
        print("Expected 0x200 bytes of data, got "); hex32(len);
        return;
    }
    print("Write "); hex32(address);
    for(uint16_t offset = 0; offset < 0x100; offset+=0x40) {

    }
}
void program_block(uint32_t address, void *data, uint32_t len) {
    if (len != 0x10000) {
        print("Expected 0x10000 bytes of data, got "); hex32(len);
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

void hexdump(void *d, uint32_t len) {
  unsigned char *data;
  int i, off;
  data = (unsigned char*)d;
  for (off=0; off<=len; off += 16) {
    hex32s(off);
    for(i=0; i<16; i++)
      if((i+off)>=len) print("   ");
      else hex8s(data[off+i]);

    print(" ");
    for(i=0; i<16; i++)
      if((i+off)>=len) print(" ");
      else uart_send(ascii(data[off+i]));
    print("\r\n");
  }
}

static void dump() {
    uint16_t block, page, i;
    unsigned int bytesWritten = 0;
    uint16_t data[64*256];

    SDFatFS_Handle sdfatfsHandle;
    FIL fil;

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        print("Error starting the SD card\n");
        return;
    } else {
        print("mounted\r\n");
        FILINFO fno;
        if (FR_OK == f_stat(file, &fno)) {
            if (FR_OK == f_unlink(file)) {
                print("file deleted\r\n");
            } else {
                print("failed to delete file\r\n");
                return;
            }
        } else {
            print("no file, will be created\r\n");
        }
        if (!f_open(&fil, file, FA_CREATE_NEW|FA_WRITE)) {

            init_pins();
            do_reset();
            led_on(BLUE_LED);
            for(block = 0; block < 0x40; block++) {
                print("\rDump block "); hex8s(block);
                for (short quarter = 0; quarter < 4; quarter++) { // 16KW per quarter
                    for(page = 0; page < 0x40; page++) {
                        for(i = 0; i<=0x100;i++) data[(page<<8)|i] = read_word(block, (quarter<<6)|page, i);
                    }
                    f_write(&fil, data, 1<<15, &bytesWritten);
                    if (bytesWritten != 1<<15) {
                        print("write error, only "); hex16s(bytesWritten); print(" written\r\n");
                    }
                    led_toggle(GREEN_LED);
                }
            }
            f_sync(&fil);
            f_close(&fil);
        }
        SDFatFS_close(sdfatfsHandle);
        print("DONE\r\n");
        led_off(BLUE_LED);
    }
}


static void burn() {
    uint16_t block, i;
    uint32_t status;
    unsigned int bytesRead = 0;
    uint16_t data[0x2000];

    SDFatFS_Handle sdfatfsHandle;
    FIL fil;

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        print("Error starting the SD card\n");
        return;
    } else {
        print("sd mounted\r\n");
        FILINFO fno;
        if (FR_OK == f_stat(burnfile, &fno)) {
            print("burnfile found\r\n");
        } else {
            print(burnfile); print(" not found on sdcard\r\n");
            return;
        }
        if (!f_open(&fil, burnfile, FA_READ)) {

            init_pins();
            do_reset();
            led_on(RED_LED);
            for(block = 0; block < 0x80; block++) { // 128 blocs of 64Kb aka 0x8000 words
                print("Erase block "); hex32(block<<15);
                erase_block_29w(block<<15);
                print("Burn block "); hex8(block);


                for (short quarter = 0; quarter < 4; quarter++) { // 8KW per quarter
                    f_read(&fil, data, 1<<14, &bytesRead); // so read 16Kb
                    if (bytesRead != 1<<14) {
                        print("Read error, only "); hex16s(bytesRead); print("read\r\n");
                    }
                    for (i = 0; i < 0x2000; i+=2 ) { // 8KW
                        uint16_t d1 = data[i];
                        uint16_t d2 = data[i+1];
                        uint32_t addr = block*4*0x2000 + quarter*0x2000 + i;
                        write_word_29w((addr>>16)&0xff, (addr>>8)&0xff, addr&0xff, d1, d2);
                        //if (status != 0) {
                        //    print("write status: "); hex32(status);
                        //}
                        //if (addr >= 0x8000 && addr < 0x8100) {
                        //    print("A "); hex32s(addr); hex16s(d1); hex16(d2);
                        //}
                    }
                    /* for(page = 0; page < 0x40; page++) {
                        for(i = 0; i<=0x100;i+=2) {
                            print("P ");
                            hex32s(block*4*0x40*0x100 + quarter*0x40*0x100 + page*0x100 + i);
                            hex16s((page<<8)|i);
                            hex16s((page<<8)|(i+1));
                            hex16s(data[(page<<8)|i]);
                            hex16(data[(page<<8)|(i+1)]);
                            //data[(page<<8)|i] = read_word(block, (quarter<<6)|page, i);
                        }
                    }*/

                    led_toggle(GREEN_LED);
                }
            }

            f_close(&fil);
        } else {
            print("fopen failed\r\n");
        }
        SDFatFS_close(sdfatfsHandle);
        print("DONE\r\n");
        led_off(RED_LED);
    }
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
            hex8s(data[i*0x10+j]);
        }
        print("\r\n");
    }
    write_cfi("CFI.BIN", (void *)data, 0x8*0x10);
}

void sneek_a(void) {
    uint32_t a;

    while (a != 0xffffffff) {
        print("Entre XXXXXX address: ");
        a = read_hex(6); print("\r\n");
        hex16(read_word((a >> 16)&0xff, (a>>8)&0xff, a&0xff));

    }
}

void read_a_spot(void) {
    uint32_t a;
    uint16_t w;
    while (1) {
        print("Entre XXXXXX address: ");
        a = read_hex(6); print("\r\n");
        w = read_word((a >> 16)&0xff, (a>>8)&0xff, a&0xff);
        port_status();
        print("AT "); hex32s(a); print(" -> "); hex16(w);
    }
}

static void check_content(char *name, uint32_t addr, uint32_t len) {
    FIL fp;
    uint8_t *data;
    uint32_t dataread;
    SDFatFS_Handle sdfatfsHandle;

    if (len == 0) {
        print("Enter addr: "); addr = read_hex(6);
        print("Enter len: "); len = read_hex(2);
        print("\r\n");
    }
    data = malloc(len);

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);

    if (!f_open(&fp,name, FA_READ)) {
        if (!f_lseek(&fp, addr)) {
            if (!f_read(&fp, data, len, (UINT*)&dataread)) {
                hexdump(data, len);
            }
        }
        f_close(&fp);
    }
    free(data);
    SDFatFS_close(sdfatfsHandle);
}


static void menu() {
    print("Dumper Menu\r\n");
    print("<c> CFI\r\n");
    print("<a> sneek at an address\r\n");
    print("<w> peek flash address\r\n");
    print("<d> Dump to ["); print(file); print("]\r\n");
    print("<b> Burn from ["); print(burnfile); print("]\r\n");
    print("<B> blink\r\n");
    print("<D> toggle debug mode\r\n");
    print("<p> show ports status\r\n");
    print("<i> read an hex value\r\n");
    print("<t> test various things\r\n");
    print("<l> DIR list sdcard\r\n");
    print("<x> check content in the saved file\r\n");
    print("<X> reset flash\r\n");
    print("<q> quest\r\n");
    print(debug?"DBG":"");
}

void printDrive(const char *driveNumber, FATFS **fatfs)
{
    FRESULT fresult;
    DWORD   freeClusterCount;
    DWORD   totalSectorCount;
    DWORD   freeSectorCount;

    print("Reading disk information...");

    fresult = f_getfree(driveNumber, &freeClusterCount, fatfs);
    if (fresult) {
        print("Error getting the free cluster count from the FatFs object");
        while (1);
    }
    else {
        print("done\r\n");

        /* Get total sectors and free sectors */
        totalSectorCount = ((*fatfs)->n_fatent - 2) * (*fatfs)->csize;
        freeSectorCount  = freeClusterCount * (*fatfs)->csize;

        /* Print the free space (assuming 512 bytes/sector) */
        print("total: ");
        //print(STR(totalSectorCount/2));
        hex32(totalSectorCount/2);
        print(" free: ");
        //print(STR(freeSectorCount/2));
        hex32(freeSectorCount/2);
        //Display_printf(display, 0, 0,
        //    "Total Disk size: %10lu KiB\n Free Disk space: %10lu KiB\n",
        //    totalSectorCount / 2, freeSectorCount  / 2);
        print("\r\n");
    }
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
    uart_init();
    init_pins();
    init_leds();

    //char status;
    char cmd=0;
    menu();
    while (cmd != 'q') {
        cmd = uart_recv();

        switch(cmd) {
        case 'c':
            read_cfi();
            break;
        case 'b':
            burn();
            break;
        case 'a':
            sneek_a();
            break;
        case 'd':
            dump();
            break;
        case 'D':
            debug ^= 1;
            print(debug?"debug on\r\n":"debug off\r\n");
            break;
        case 'p':
            port_status();
            break;
        case 'i':
            hex32(read_hex(4));
            break;
        case 't':
            print("TEST\r\n");


            hex32(get_fattime());
            P2->DIR |= 2;
            P2->OUT |= 2;
            sleep(1);
            P2->OUT &= ~2;
            sleep(1);
            hex32(get_fattime()); // 4A210000
                                  //
            break;                // 0100 1010 0010 0001 0000 0000 0000 0000
        case 'l':                 // |||| |||\ ||/  \||| /
                                  // 0100101  0001  0001
                                  //   2017   Jan     1
            do_sd2();
            break;
        case 'x':
            check_content("DUMP.BIN", 0x0, 0);
            break;
        case 'X':
            do_reset();
            break;
        case 'w':
            read_a_spot();
            break;
        case 'F':
            print("Erase full chip, should take 80s\r\n");
            erase_chip_29w();
            break;
        case 'q':
            print("QUEST\r\n");
            print(entry83("enter a filename: "));
            break;
        case '?':
            menu();
            break;
        }
        cmd = 0;
    }
    print("END\r\n");
    return NULL;
}

FRESULT scan_files(char *path) {
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;
    print("scan path "); print(path); print("\r\n");
    res = f_opendir(&dir, path);
    hex8(res);
    if (res == FR_OK) {
        for(;;){
            //print("reading\r\n");
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fattrib & AM_DIR){
                print("subdir\r\n");
                i =strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
                //print("file\r\n");
                print(path); print("/"); print(fno.fname); print("\r\n");
            }
        }
        f_closedir(&dir);
    }
    return res;
}

void do_sd2(void) {


    SDFatFS_Handle sdfatfsHandle;
    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        print("Error starting the SD card\n");
        return;
    } else {
        print("mounted\r\n");
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
        print("Error starting the SD card\n");

        while (1);
    } else {
        print("mounted\r\n");
    }
    printDrive(STR(DRIVE_NUM), &(dst.obj.fs));
    //scan_files("0:/");

    if (FR_OK == f_open(&src, filename, FA_CREATE_NEW|FA_READ|FA_WRITE)) {
        f_write(&src, data, len, &bytesWritten);
        f_sync(&src);
        f_close(&src);
    }


    SDFatFS_close(sdfatfsHandle);



//    printDrive(STR(DRIVE_NUM), &(dst.obj.fs));

    /* Try to open the source file */
//    fresult = f_open(&src, inputfile, FA_READ);
//    if (fresult != FR_OK) {
//        Display_printf(display, 0, 0, "Creating a new file \"%s\"...",
//            inputfile);

        /* Open file for both reading and writing */
//        fresult = f_open(&src, inputfile, FA_CREATE_NEW|FA_READ|FA_WRITE);
//        if (fresult != FR_OK) {
//            Display_printf(display, 0, 0,
//                "Error: \"%s\" could not be created.\nPlease check the "
//                "Board.html if additional jumpers are necessary.\n",
//                inputfile);
//            Display_printf(display, 0, 0, "Aborting...\n");
//            while (1);
//        }
//
//        f_write(&src, textarray, strlen(textarray), &bytesWritten);
//        f_sync(&src);

        /* Reset the internal file pointer */
//        f_lseek(&src, 0);

//        Display_printf(display, 0, 0, "done\n");
//   }
//    else {
//        Display_printf(display, 0, 0, "Using existing copy of \"%s\"\n",
//            inputfile);
//    }

    /* Create a new file object for the file copy */
//    fresult = f_open(&dst, outputfile, FA_CREATE_ALWAYS|FA_WRITE);
//    if (fresult != FR_OK) {
//        Display_printf(display, 0, 0, "Error opening \"%s\"\n", outputfile);
//        Display_printf(display, 0, 0, "Aborting...\n");
//        while (1);
 //   }
//    else {
//        Display_printf(display, 0, 0, "Starting file copy\n");
//    }

    /*  Copy the contents from the src to the dst */
//    while (true) {
        /*  Read from source file */
//        fresult = f_read(&src, cpy_buff, CPY_BUFF_SIZE, &bytesRead);
//        if (fresult || bytesRead == 0) {
//            break; /* Error or EOF */
//        }

        /*  Write to dst file */
//        fresult = f_write(&dst, cpy_buff, bytesRead, &bytesWritten);
//        if (fresult || bytesWritten < bytesRead) {
//            Display_printf(display, 0, 0, "Disk Full\n");
//            break; /* Error or Disk Full */
//        }

        /*  Update the total number of bytes copied */
//        totalBytesCopied += bytesWritten;
//    }

//    f_sync(&dst);

    /* Get the filesize of the source file */
//    filesize = f_size(&src);

    /* Close both inputfile[] and outputfile[] */
//    f_close(&src);
//    f_close(&dst);

//    Display_printf(display, 0, 0,
//        "File \"%s\" (%u B) copied to \"%s\" (Wrote %u B)\n", inputfile,
//         filesize, outputfile, totalBytesCopied);

    /* Now output the outputfile[] contents onto the console */
//    fresult = f_open(&dst, outputfile, FA_READ);
//   if (fresult != FR_OK) {
//        Display_printf(display, 0, 0, "Error opening \"%s\"\n", outputfile);
//        Display_printf(display, 0, 0, "Aborting...\n");
//        while (1);
//    }

    /* Print file contents */
//    while (true) {
//        /* Read from output file */
//        fresult = f_read(&dst, cpy_buff, CPY_BUFF_SIZE, &bytesRead);
//        if (fresult || bytesRead == 0) {
//            break; /* Error or EOF */
//        }
//        cpy_buff[bytesRead] = '\0';
        /* Write output */
//        Display_printf(display, 0, 0, "%s", cpy_buff);
//    }

    /* Close the file */
//    f_close(&dst);

//    printDrive(STR(DRIVE_NUM), &(dst.obj.fs));

    /* Stopping the SDCard */
//    SDFatFS_close(sdfatfsHandle);

//    Display_printf(display, 0, 0, "Drive %u unmounted\n", DRIVE_NUM);


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
