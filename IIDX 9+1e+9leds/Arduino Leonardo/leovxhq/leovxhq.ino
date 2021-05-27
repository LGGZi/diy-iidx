/* 
  hajihi version for arduino leonardo
  edit by hajihi
  move pin:
  key1-7 => p3-9
  spc1-4 => p10-13
  encoder => p1-2(first one using interrupt, so using p0/1/2/3/7 only)
  (p0 always auto set to 1 because of TX output, idk how to turn it off, so i can't use it)
  key1-7(led) => p18-23,14
  p14 position(Among the three ICSP outputs in the row closer to the chip, the one farthest from the chip):
  +---------------------------------------+
  |Chip(32u4)                             |
  |                                       |
  |     RESET SCK(P15)  MISO(P14)  [ICSP] |
  |     GND   MOSI(P16) 5V                |
  +---------------------------------------+
  p18-23 is equal to A0-5
*/

/*
  Keyboard.h

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  Permission to use, copy, modify, and/or distribute this software for
  any purpose with or without fee is hereby granted, provided that the
  above copyright notice and this permission notice appear in all copies.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
  WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
  BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
  OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
  WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
  ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
  SOFTWARE.
 */

#include <stdint.h>
#include <Arduino.h>
#include "PluggableUSB.h"

#if defined(USBCON)

#define _USING_HID

// HID 'Driver'
// ------------
#define HID_GET_REPORT        0x01
#define HID_GET_IDLE          0x02
#define HID_GET_PROTOCOL      0x03
#define HID_SET_REPORT        0x09
#define HID_SET_IDLE          0x0A
#define HID_SET_PROTOCOL      0x0B

#define HID_HID_DESCRIPTOR_TYPE         0x21
#define HID_REPORT_DESCRIPTOR_TYPE      0x22
#define HID_PHYSICAL_DESCRIPTOR_TYPE    0x23

// HID subclass HID1.11 Page 8 4.2 Subclass
#define HID_SUBCLASS_NONE 0
#define HID_SUBCLASS_BOOT_INTERFACE 1

// HID Keyboard/Mouse bios compatible protocols HID1.11 Page 9 4.3 Protocols
#define HID_PROTOCOL_NONE 0
#define HID_PROTOCOL_KEYBOARD 1
#define HID_PROTOCOL_MOUSE 2

// Normal or bios protocol (Keyboard/Mouse) HID1.11 Page 54 7.2.5 Get_Protocol Request
// "protocol" variable is used for this purpose.
#define HID_BOOT_PROTOCOL  0
#define HID_REPORT_PROTOCOL 1

// HID Request Type HID1.11 Page 51 7.2.1 Get_Report Request
#define HID_REPORT_TYPE_INPUT   1
#define HID_REPORT_TYPE_OUTPUT  2
#define HID_REPORT_TYPE_FEATURE 3

typedef struct
{
  uint8_t len;      // 9
  uint8_t dtype;    // 0x21
  uint8_t addr;
  uint8_t versionL; // 0x101
  uint8_t versionH; // 0x101
  uint8_t country;
  uint8_t desctype; // 0x22 report
  uint8_t descLenL;
  uint8_t descLenH;
} HIDDescDescriptor;

typedef struct 
{
  InterfaceDescriptor hid;
  HIDDescDescriptor   desc;
  EndpointDescriptor  in;
} HIDDescriptor;

class HIDSubDescriptor {
public:
  HIDSubDescriptor *next = NULL;
  HIDSubDescriptor(const void *d, const uint16_t l) : data(d), length(l) { }

  const void* data;
  const uint16_t length;
};

class HID_ : public PluggableUSBModule
{
public:
  HID_(void);
  int begin(void);
  int SendReport(uint8_t id, const void* data, int len);
  void AppendDescriptor(HIDSubDescriptor* node);

protected:
  // Implementation of the PluggableUSBModule
  int getInterface(uint8_t* interfaceCount);
  int getDescriptor(USBSetup& setup);
  bool setup(USBSetup& setup);
  uint8_t getShortName(char* name);

private:
  uint8_t epType[1];

  HIDSubDescriptor* rootNode;
  uint16_t descriptorSize;

  uint8_t protocol;
  uint8_t idle;
};

// Replacement for global singleton.
// This function prevents static-initialization-order-fiasco
// https://isocpp.org/wiki/faq/ctors#static-init-order-on-first-use
HID_& HID();

#define D_HIDREPORT(length) { 9, 0x21, 0x01, 0x01, 0, 1, 0x22, lowByte(length), highByte(length) }

#endif // USBCON

//  Low level key report: up to 6 keys and shift, ctrl etc at once
extern volatile uint16_t iivx_led;

typedef struct iivxReport     // Pretty self explanitory. Simple state to store all the joystick parameters
{
  uint16_t  buttons;
  uint8_t   xAxis;
  uint8_t   yAxis;
  uint8_t   zAxis;
} iivxReport_t;

class iivx_
{
public:
  iivx_();
  void setState(iivxReport_t *report);
  inline uint16_t readState(){ return iivx_led; };
};

extern iivx_ iivx;

iivxReport_t report; 

#define REPORT_DELAY 2000
// Number of microseconds between HID reports
// 2000 = 500hz

#define ENC_L_A 1
#define ENC_L_B 2
//active using ENC_L_A with interrupt, so ENC_L_A only avilable in pin 0, 1, 2, 3, 7
//#define ENC_L_B_ADDR 3
//#define ENCODER_SENSITIVITY (double) 2.34375
#define ENCODER_SENSITIVITY (double) 8
//The smaller the more sensitive
#define ENCODER_PORT PIND
//PIND is arduino register (0-7 pins) RO, set each bit from each pin, eg:01001110 (pin 1,2,3,6 = HIGH). using bit move right and bit AND to get single bit if necessary
// encoder sensitivity = number of positions per rotation (600) / number of positions for HID report (256)
/*
 * connect encoders
 * TURNTABLE to pin 0 and 1
 */

int tmp;
//uint8_t buttonCount = 9;
uint8_t buttonCount = 11;
uint8_t lightMode = 0;
// 0 = reactive lighting, 1 = HID lighting
//uint8_t ledPins[] = {2,3,4,5,6,7,8,9,10};
//uint8_t buttonPins[] = {11,12,13,18,19,20,21,22,23};
uint8_t ledPins[] = {18,19,20,21,22,23,14,0,0,0,0};
uint8_t buttonPins[] = {3,4,5,6,7,8,9,10,11,12,13};
//uint8_t sysPin = 11;
uint8_t reactiveLightPin = 21;
uint8_t hidLightPin = 22;
uint8_t sysInputPins[] = {13,18,19,20};
int32_t encL=0;
/* current pin layout
 *  pins 18 - 23 = A0 - A5
 *  pins 2 to 10 = LED 1 to 7
 *    connect pin to + termnial of LED
 *    connect ground to resistor and then - terminal of LED
 *  pins 11 and 12 = START and EFFECT
 *  pins 13, A0 to A5 = Button input 1 to 7
 *    connect button pin to ground to trigger button press
 *  pins 11 = system pin
 *    connect system pin to ground with a small button
 *      press together with other buttons to change lighting scheme
 *    system button + button 1 = reactive lighting
 *    system button + button 3 = HID lighting
 */
 

void doEncL(){
  //if((ENCODER_PORT >> ENC_L_B_ADDR)&1){
  // not recommand
  if (digitalRead(ENC_L_B) != HIGH) {
    encL++;
  } else {
    encL--;
  }
}

void lights(uint16_t lightDesc){
  for(int i=0;i<buttonCount;i++){
     if((lightDesc>>i)&1){
         digitalWrite(ledPins[i],HIGH);
     } else {
         digitalWrite(ledPins[i],LOW);
     }
  }
}

void setup() {
  delay(1000);
  // Setup I/O for pins
  for(int i=0;i<buttonCount;i++){
    pinMode(buttonPins[i],INPUT_PULLUP);
    pinMode(ledPins[i],OUTPUT);
  }
  //pinMode(sysPin,INPUT_PULLUP);
  //setup interrupts
  pinMode(ENC_L_A,INPUT_PULLUP);
  pinMode(ENC_L_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), doEncL, RISING);
}

void loop() {
  // Read buttons
  for(int i=0;i<buttonCount;i++){
    if(digitalRead(buttonPins[i])!=HIGH){
      report.buttons |= (uint16_t)1 << i;
    } else {
      delay(2);
      if(digitalRead(buttonPins[i])!=HIGH){
        report.buttons |= (uint16_t)1 << i;
      } else {
        report.buttons &= ~((uint16_t)1 << i);
      }
    }
  }
  // Read Encoders
  report.xAxis = (uint8_t)((int32_t)(encL / ENCODER_SENSITIVITY) % 256);
  // Light LEDs
  if(lightMode==0){
    lights(report.buttons);
  } else {
    lights(iivx_led);
  }
  // Detect Syspin Entries
  //if(digitalRead(buttonPins[0])!=HIGH){
  //  report.buttons = 0;
  //  for(int i=0;i<4;i++){
  //    if(digitalRead(sysInputPins[i])!=HIGH){
  //      report.buttons |= (uint16_t)1 << (i+buttonCount);
  //    } else {
   //     report.buttons &= ~((uint16_t)1 << (i+buttonCount));
 //     }
  //  }
 //   if(digitalRead(reactiveLightPin)!=HIGH){
  //    lightMode = 0;
 //   } else if (digitalRead(hidLightPin)!=HIGH){
 //     lightMode = 1;
 //   }
 // }
  // Send report and delay
  iivx.setState(&report);
  delayMicroseconds(REPORT_DELAY);
}
