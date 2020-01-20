
void FRAM_init() {
  digitalWrite(FRAM_CS, HIGH);
  if (fram.begin()) {
    debugln(F("Found SPI FRAM"));
  } else {
    debugln(F("No SPI FRAM found ... check your connections\r\n"));
    while (1);
  }
  digitalWrite(FRAM_CS, LOW);
}

void logAdd(uint32_t d, uint32_t t, float lat, float lng) { 
  uint16_t count = readCountRobust();

  if (FRAM_START_ADDR + 16 + count*16 > FRAM_LAST_ADDR)
    return;
  
  debug(F("Logging line: "));
  debugln(count);

  writeCountRobust(count + 1);

  uint32AsBytes.lval = d;
  fram.writeEnable(true);
  fram.write(FRAM_START_ADDR + 0 + count*16, uint32AsBytes.bytes, 4);
  fram.writeEnable(false);
  
  uint32AsBytes.lval = t;
  fram.writeEnable(true);
  fram.write(FRAM_START_ADDR + 4 + count*16, uint32AsBytes.bytes, 4);
  fram.writeEnable(false);

  floatAsBytes.fval = lat;
  fram.writeEnable(true);
  fram.write(FRAM_START_ADDR + 8 + count*16, floatAsBytes.bytes, 4);
  fram.writeEnable(false);

  floatAsBytes.fval = lng;
  fram.writeEnable(true);
  fram.write(FRAM_START_ADDR + 12 + count*16, floatAsBytes.bytes, 4);
  fram.writeEnable(false);
}

void logDump(unsigned int max_count) {
  uint16_t count = readCountRobust();

  debug(F("Dumping Log... "));
  debug(count);
  debug(" lines logged, showing max ");
  debugln(max_count);
  
  for (unsigned long i = 0; i < count && i < max_count; i++) {
    if (FRAM_START_ADDR + 16 + i*16 > FRAM_LAST_ADDR)
      break;
    
    uint32AsBytes.bytes[0] = fram.read8(FRAM_START_ADDR + 0 + i*16);
    uint32AsBytes.bytes[1] = fram.read8(FRAM_START_ADDR + 1 + i*16);
    uint32AsBytes.bytes[2] = fram.read8(FRAM_START_ADDR + 2 + i*16);
    uint32AsBytes.bytes[3] = fram.read8(FRAM_START_ADDR + 3 + i*16);
    debug(uint32AsBytes.lval);
    debug(" ");

    uint32AsBytes.bytes[0] = fram.read8(FRAM_START_ADDR + 4 + i*16);
    uint32AsBytes.bytes[1] = fram.read8(FRAM_START_ADDR + 5 + i*16);
    uint32AsBytes.bytes[2] = fram.read8(FRAM_START_ADDR + 6 + i*16);
    uint32AsBytes.bytes[3] = fram.read8(FRAM_START_ADDR + 7 + i*16);
    debug(uint32AsBytes.lval);
    debug(" ");
    
    floatAsBytes.bytes[0] = fram.read8(FRAM_START_ADDR + 8 + i*16);
    floatAsBytes.bytes[1] = fram.read8(FRAM_START_ADDR + 9 + i*16);
    floatAsBytes.bytes[2] = fram.read8(FRAM_START_ADDR + 10 + i*16);
    floatAsBytes.bytes[3] = fram.read8(FRAM_START_ADDR + 11 + i*16);
    debug2(floatAsBytes.fval, 6);
    debug(" ");
    
    floatAsBytes.bytes[0] = fram.read8(FRAM_START_ADDR + 12 + i*16);
    floatAsBytes.bytes[1] = fram.read8(FRAM_START_ADDR + 13 + i*16);
    floatAsBytes.bytes[2] = fram.read8(FRAM_START_ADDR + 14 + i*16);
    floatAsBytes.bytes[3] = fram.read8(FRAM_START_ADDR + 15 + i*16);      
    debug2(floatAsBytes.fval, 6);
    debugln();
  }
}

void logReset() {
  debugln(F("Resetting Log..."));
  writeCountRobust(0);
}

uint16_t readCount(unsigned char addr) {
  uint16AsBytes.bytes[0] = fram.read8(addr + 0);
  uint16AsBytes.bytes[1] = fram.read8(addr + 1);
  return uint16AsBytes.ival;
}

uint16_t readCountRobust() {
  uint16_t count[4];
  count[0] = readCount(0);
  count[1] = readCount(2);
  count[2] = readCount(4);
  count[3] = readCount(6);
  
  if (!(count[0] == count[1] && count[1] == count[2] && count[2] == count[3])) {
    debugln(F(">> FRAM POSITION CORRUPTED, FIXING..."));
    
    for (char i = 0; i < 4; i++) {
      for (char j = i+1; j < 4; j++) {
        if (count[i] == count[j]) {
          writeCountRobust(count[i]);
          debugln(F(">> FIXED"));
          return count[i];
        }
      }
    }
    
    debugln(F(">> PROBLEM FIXING"));
    return 1000;
  }
  
  return count[0];
}

void writeCount(uint16_t count, unsigned char addr) {
  uint16AsBytes.ival = count;
  fram.writeEnable(true);
  fram.write(addr, uint16AsBytes.bytes, 2);
  fram.writeEnable(false);
}

void writeCountRobust(uint16_t count) {
  writeCount(count, 0);
  writeCount(count, 2);
  writeCount(count, 4);
  writeCount(count, 6);
}

