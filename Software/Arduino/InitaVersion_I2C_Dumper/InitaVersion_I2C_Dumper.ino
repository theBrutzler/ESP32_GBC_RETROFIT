/*
  GBCartRead
  Version: 1.8 Rev 1.2

  Author: Alex from insideGadgets (https://www.insidegadgets.com)
  github: (https://github.com/insidegadgets/GBCartRead)
  Created: 18/03/2011
  Modified: 26/05/2016

  GBCartRead is an Arduino based Gameboy Cartridge Reader which uses a C program or python
  script to interface with the Arduino. GBCartRead allows you to dump your ROM,
  save the RAM and write to the RAM.
  Works with Arduino Duemilanove, Uno and Nano.
  Will work for other Arduinos too with VCC = +5V,
  but requires wiring changes!!
  Speed increase thanks to Frode vdM.(fvdm1992@yahoo.no) and David R.

  UPDATE: This code was changed to be compatible for Arduino NANO and a custom PCB!!

  Version: inital test version
  Author: WodoWiesel modify theBrutzler
  github: (https://github.com/wodowiesel/GB-Dumper)
  Last Modified: 30. October 2024
*/

// for REVISION 0.6 of the GB-Dumper Board
// compatible with Cartridges Classic (Original) + Color + Pocket + Micro :
// Class/Type A (grey+colors), Class B Dual (black), Type C (transparent/clear)
// NOT compatible with Class D (Advance GBA) -> 3.3V Logic
// cartridge internal coin battery Li-cell normally +3V CR2025, sometimes 2032, CR1616 GBA
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include <Adafruit_AW9523.h>

Adafruit_AW9523 aw_1;
Adafruit_AW9523 aw_2;

int baud = 9600;  // rate for serial connection

// Edit these in pindelcarations.h too if needed!
// Digital Pins-> D-Numbers
#define GB_A0 0
#define GB_A1 1
#define GB_A2 2
#define GB_A3 3
#define GB_A4 4
#define GB_A5 5
#define GB_A6 6
#define GB_A7 7
#define GB_A8 8
#define GB_A9 9
#define GB_A10 10
#define GB_A11 11
#define GB_A12 12
#define GB_A13 13
#define GB_A14 14
#define GB_A15 15
#define GB_D0 0
#define GB_D1 1
#define GB_D2 2
#define GB_D3 3
#define GB_D4 4
#define GB_D5 5
#define GB_D6 6
#define GB_D7 7
#define GB_CLK 8
#define GB_WR 9
#define GB_RD 10
#define GB_CS 11
#define GB_RES 12
#define GB_VIN 13
#define GB_VOLT 14

// https://gbdev.io/pandocs/Power_Up_Sequence.html
uint8_t nintendoLogo[] = { 0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
                           0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
                           0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E };

char gameTitle[17];
//char gameTitle[16];
uint16_t cartridgeType = 0;
uint16_t romSize = 0;
uint16_t romBanks = 0;
uint16_t ramSize = 0;
uint16_t ramBanks = 0;
uint16_t ramEndAddress = 0;

void setup() {

  Serial.begin(baud);  // for 4/16 MHz clock
  delay(1000);
  Wire.begin(2, 42);

  //Wire.begin(2, 1);

  if (!aw_1.begin(0x58)) {
    Serial.println("AW9523_1 not found? Check wiring!");
    while (1) delay(10);  // halt forever
  }

  //Serial.println("AW9523_1 found!");

  for (int address = 0; address <= 15; address++) {
    aw_1.pinMode(address, OUTPUT);
    aw_1.digitalWrite(address, LOW);
  }

  if (!aw_2.begin(0x5A)) {
    Serial.println("AW9523_2 not found? Check wiring!");
    while (1) delay(10);  // halt forever
  }

  //Serial.println("AW9523_2 found!");
  for (int address = 0; address <= 7; address++) {
    aw_2.pinMode(address, INPUT);
  }
  for (int address = 8; address <= 15; address++) {
    aw_2.pinMode(address, OUTPUT);
    aw_2.digitalWrite(address, LOW);
  }
  aw_2.pinMode(GB_VIN, INPUT);
  aw_2.digitalWrite(GB_VOLT, HIGH);

  rd_wr_mreq_off();

  //Serial.println("\nInit done \n");
}

void loop() {

  // Decode input
  char readInput[10];
  uint8_t readCount = 0;

  // Wait for serial input
  while (Serial.available() <= 0) {
    delay(200);
  }
  while (Serial.available() > 0) {
    char c = Serial.read();
    readInput[readCount] = c;
    readCount++;
  }
  //readInput[readCount] = '\0';
  readInput[16] = '\0';
  // Cartridge Header
  if (strstr(readInput, "HEADER")) {
    rd_wr_mreq_reset();

    gameTitle[16] = '\0';

    // Read cartridge title and check for non-printable text
    for (uint16_t romAddress = 0x0134; romAddress <= 0x143; romAddress++) {
      char headerChar = (char) read_byte(romAddress);
      if ((headerChar >= 0x30 && headerChar <= 0x57) || // 0-9
          (headerChar >= 0x41 && headerChar <= 0x5A) || // A-Z
          (headerChar >= 0x61 && headerChar <= 0x7A)) { // a-z
          gameTitle[(romAddress - 0x0134)] = headerChar;
          //Serial.println(headerChar);
        /*
         * MULICARD MBCs & EMS etc https://gbdev.gg8.se/wiki/articles/Memory_Bank_Controllers
          A header matching any of the following is detected as EMS mapper:
          Header name is "EMSMENU", NUL-padded
          Header name is "GB16M", NUL-padded
          Cartridge type ($0147) = $1B and region ($014A) = $E1
        */
      }

      cartridgeType = read_byte(0x0147); // MBC type should be specified in the byte at 0147h of the ROM
      romSize = read_byte(0x0148);
      ramSize = read_byte(0x0149);

      // ROM banks
      romBanks = 2; // Default 32K
      if (romSize >= 1) { // Calculate rom size
        romBanks = 2 << romSize;
      }

      // RAM banks
      ramBanks = 0; // Default 0K RAM
      if (cartridgeType == 6) {
        ramBanks = 1;
      }
      if (ramSize == 2) {
        ramBanks = 1;
      }
      if (ramSize == 3) {
        ramBanks = 4;
      }
      if (ramSize == 4) {
        ramBanks = 16;
      }
      if (ramSize == 5) {
        ramBanks = 8;
      }

      // RAM end address
      if (cartridgeType == 6) {
        ramEndAddress = 0xA1FF;  // MBC2 512 bytes (nibbles)
      }
      if (ramSize == 1) {
        ramEndAddress = 0xA7FF;  // 2K RAM
      }
      if (ramSize > 1) {
        ramEndAddress = 0xBFFF;  // 8K RAM
      }
   
    }
 
    // Nintendo Logo Check
    uint8_t logoCheck = 1;
    uint8_t x = 0;
    for (uint16_t romAddress = 0x0104; romAddress <= 0x133; romAddress++) {
      if (nintendoLogo[x] != read_byte(romAddress)) {
        logoCheck = 0;
        break;
      }
      x++;
    }
    
      Serial.println(gameTitle);
      Serial.println(cartridgeType);
      Serial.println(romSize);
      Serial.println(ramSize);
      Serial.println(logoCheck);
    }
  

    // Dump ROM
   else if (strstr(readInput, "READROM")) {
      rd_wr_mreq_reset();
      uint16_t romAddress = 0;

      // Read number of banks and switch banks
      for (uint16_t bank = 1; bank < romBanks; bank++) {
        if (cartridgeType >= 5) { // MBC2 and above
          write_byte(0x2100, bank); // Set ROM bank
        }
        else { // MBC1
          write_byte(0x6000, 0); // Set ROM Mode
          write_byte(0x4000, bank >> 5); // Set bits 5 & 6 (01100000) of ROM bank
          write_byte(0x2000, bank & 0x1F); // Set bits 0 & 4 (00011111) of ROM bank
        }
        if (bank > 1) {
          romAddress = 0x4000;
        }

        // Read up to 7FFF per bank
        while (romAddress <= 0x7FFF) {
          uint8_t readData[64];
          for (uint8_t i = 0; i < 64; i++) {
            readData[i] = read_byte(romAddress + i);
          }

          Serial.write(readData, 64); // Send the 64 byte chunk
          romAddress += 64;
        }
        Serial.println(" ");
        Serial.print("BANK_NUM: ");
        Serial.print(bank);
        Serial.println(" BANK_SWITCH");
      }
   }
  // Read RAM
  else if (strstr(readInput, "READRAM")) {
    rd_wr_mreq_reset();

    // MBC2 Fix (unknown why this fixes reading the ram, maybe has to read ROM before RAM?)
    read_byte(0x0134);

    // if cartridge have RAM test
    if (ramEndAddress > 0) {
      if (cartridgeType <= 4) {  // MBC1
        write_byte(0x6000, 1);   // Set RAM Mode
      }

      // Initialise MBC
      write_byte(0x0000, 0x0A);

      // Switch RAM banks
      for (uint8_t bank = 0; bank < ramBanks; bank++) {
        write_byte(0x4000, bank);

        // Read RAM
        for (uint16_t ramAddress = 0xA000; ramAddress <= ramEndAddress; ramAddress += 64) {
          uint8_t readData[64];
          for (uint8_t i = 0; i < 64; i++) {
            readData[i] = read_byte(ramAddress + i);
          }

          //Serial.println(readData,HEX);
        Serial.write(readData, 64);  // Send the 64 byte chunk
        }
      }

      // Disable RAM
      write_byte(0x0000, 0x00);
    }
  }

  // Write RAM
  else if (strstr(readInput, "WRITERAM")) {
    rd_wr_mreq_reset();

    // MBC2 Fix (unknown why this fixes it, maybe has to read ROM before RAM?)
    read_byte(0x0134);

    // Does cartridge have RAM
    if (ramEndAddress > 0) {
      if (cartridgeType <= 4) {  // MBC1
        write_byte(0x6000, 1);   // Set RAM Mode
      }

      // Initialise MBC
      write_byte(0x0000, 0x0A);

      // Switch RAM banks
      for (uint8_t bank = 0; bank < ramBanks; bank++) {
        write_byte(0x4000, bank);

        // Write RAM
        for (uint16_t ramAddress = 0xA000; ramAddress <= ramEndAddress; ramAddress++) {
          // Wait for serial input
          while (Serial.available() <= 0)
            ;

          // Read input
          uint8_t readValue = (uint8_t)Serial.read();

          // Write to RAM
          aw_2.digitalWrite(GB_CS, LOW);
          write_byte(ramAddress, readValue);
          aw_2.digitalWrite(GB_CS, HIGH);
        }
      }

      /// dump ram special
      cs2_dumpsave();

      // Disable RAM
      write_byte(0x0000, 0x00);

      // Flush any serial data that wasn't processed
      Serial.flush();
    }
  }

  rd_wr_mreq_off();
  ///Serial.println("Loop done \n");
}
//--- End loop!

// func definitions
uint8_t read_byte(uint16_t address) {
  shiftout_address(address);  // Shift out address
  aw_2.digitalWrite(GB_CS, LOW);
  aw_2.digitalWrite(GB_RD, LOW);
  aw_2.digitalWrite(GB_CLK, HIGH);
  uint16_t save = aw_2.inputGPIO();
  //Serial.println(save,HEX);
  save &= 0x00FF;
  uint8_t bval = (uint8_t)save;  // Read data
  aw_2.digitalWrite(GB_RD, HIGH);
  aw_2.digitalWrite(GB_CS, HIGH);
  aw_2.digitalWrite(GB_CLK, LOW);

  return bval;
}

void write_byte(uint16_t address, uint8_t data) {  
  
  aw_2.pinMode(GB_D0, OUTPUT);
  aw_2.pinMode(GB_D1, OUTPUT);
  aw_2.pinMode(GB_D2, OUTPUT);
  aw_2.pinMode(GB_D3, OUTPUT);
  aw_2.pinMode(GB_D4, OUTPUT);
  aw_2.pinMode(GB_D5, OUTPUT);
  aw_2.pinMode(GB_D6, OUTPUT);
  aw_2.pinMode(GB_D7, OUTPUT);
  shiftout_address(address);  // Shift out address

  uint16_t save = aw_2.inputGPIO();
  save &= 0xFF00;
  save |= data;
  aw_2.outputGPIO(save);
  aw_2.digitalWrite(GB_CLK, HIGH);

  // Pulse WR
  aw_2.digitalWrite(GB_WR, LOW);
  aw_2.digitalWrite(GB_WR, HIGH);

  // Set pins as inputs
  aw_2.digitalWrite(GB_CLK, LOW);
  aw_2.pinMode(GB_D0, INPUT);
  aw_2.pinMode(GB_D1, INPUT);
  aw_2.pinMode(GB_D2, INPUT);
  aw_2.pinMode(GB_D3, INPUT);
  aw_2.pinMode(GB_D4, INPUT);
  aw_2.pinMode(GB_D5, INPUT);
  aw_2.pinMode(GB_D6, INPUT);
  aw_2.pinMode(GB_D7, INPUT);
}

// Use the shift registers to shift out the address
void shiftout_address(uint16_t shiftAddress) {
  latchaddress(shiftAddress);
}

// Turn /RD, /WR and MREQ to high so they are deselected (reset state)
void rd_wr_mreq_reset(void) {

  aw_2.digitalWrite(GB_RD, HIGH);  // /RD off
  aw_2.digitalWrite(GB_WR, HIGH);  /// WR off
  //aw_2.digitalWrite(GB_CS, HIGH);  // MREQ off
  aw_2.digitalWrite(GB_CLK, HIGH);
  aw_2.digitalWrite(GB_CLK, LOW);

}

// Turn /RD, /WR and MREQ off as no power should be applied to GB Cart
void rd_wr_mreq_off(void) {
  aw_2.digitalWrite(GB_RD, LOW);  // /RD on
  aw_2.digitalWrite(GB_WR, LOW);  // /WR on
  //aw_2.digitalWrite(GB_CS, LOW);  // /CS
  aw_2.digitalWrite(GB_CLK, HIGH);
  aw_2.digitalWrite(GB_CLK, LOW);
}

// --------------experimental!!! if ready it will be built-in to the upper code!!!
// joaopauloap suggestions
unsigned int readDataBus() {
  Serial.println("CS2 read databus ");
  uint16_t save = aw_2.inputGPIO();
  save &= 0x00FF;
  unsigned int s = (uint8_t)save;  // Read data
  return (s);
}

void latchaddress(unsigned int addr) {
  aw_1.outputGPIO(addr);
}

void cs2_dumpsave() {  // digital active-low
  Serial.println("Dumping GBA Save ");
  //digitalWrite(mreqPin, HIGH);  // CS Disable ROM
  aw_2.digitalWrite(GB_RES,LOW);
  for (unsigned long addr = 0; addr < 0x1FFF; addr++) {
    latchaddress(addr);
    aw_2.digitalWrite(GB_RD, LOW);
    aw_2.digitalWrite(GB_RD, HIGH);
    aw_2.digitalWrite(GB_CLK, HIGH);
    int d = readDataBus();
    aw_2.digitalWrite(GB_CLK, LOW);
    Serial.write(d);
  }

  aw_2.digitalWrite(GB_RES,HIGH);
  Serial.println("CS2 dump done");
  // may ned adaption in C/Py code on pc app!
  // -> not sure what data is extly tranfered ? need more info !!
}
// end  suggestion

/*
    The problem is that the GB is using a base 2 frequency clock crystal,
    not a base 10 one. Ie, 1MiHz = 1024*1024 Hz = 1048576 Hz,
    where "normally" you'd use 1 MHz = 1000000 Hz. But that's easy to overcome with new frequency tables.
    https://gbdev.gg8.se/forums/viewtopic.php?id=10
    https://dhole.github.io/post/gameboy_cartridge_emu_2/ only for STM32
  */


// sd card via spi
void sd_card_action(void) {
  Serial.println("sd_card_action check \n");
  //add later with thebrutzler
}

//EOF
