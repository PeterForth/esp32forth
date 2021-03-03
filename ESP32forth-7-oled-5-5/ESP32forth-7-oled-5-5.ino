#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ******************************************************************************
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// **********************************************************************

typedef intptr_t cell_t;
typedef uintptr_t ucell_t;

#define Y(op, code) X(#op, id ## op, code)
#define NIP (--sp)
#define NIPn(n) (sp -= (n))
#define DROP (tos = *sp--)
#define DROPn(n) (NIPn(n-1), DROP)
#define DUP (*++sp = tos)
#define PUSH DUP; tos = (cell_t)
#define COMMA(n) *g_sys.heap++ = (n)
#define DOIMMEDIATE() (*g_sys.current)[-1] |= IMMEDIATE
#define UNSMUDGE() (*g_sys.current)[-1] &= ~SMUDGE
#define DOES(ip) **g_sys.current = (cell_t) ADDR_DODOES; (*g_sys.current)[1] = (cell_t) ip
#define PARK DUP; *++rp = (cell_t) sp; *++rp = (cell_t) ip

#ifndef SSMOD_FUNC
# if __SIZEOF_POINTER__ == 8
typedef __int128_t dcell_t;
# elif __SIZEOF_POINTER__ == 4 || defined(_M_IX86)
typedef int64_t dcell_t;
# else
#  error "unsupported cell size"
# endif
# define SSMOD_FUNC dcell_t d = (dcell_t) *sp * (dcell_t) sp[-1]; \
                    --sp; cell_t a = (cell_t) (d < 0 ? ~(~d / tos) : d / tos); \
                    *sp = (cell_t) (d - ((dcell_t) a) * tos); tos = a
#endif

#define OPCODE_LIST \
  X("0=", ZEQUAL, tos = !tos ? -1 : 0) \
  X("0<", ZLESS, tos = (tos|0) < 0 ? -1 : 0) \
  X("+", PLUS, tos += *sp--) \
  X("U/MOD", USMOD, w = *sp; *sp = (ucell_t) w % (ucell_t) tos; \
                    tos = (ucell_t) w / (ucell_t) tos) \
  X("*/MOD", SSMOD, SSMOD_FUNC) \
  Y(AND, tos &= *sp--) \
  Y(OR, tos |= *sp--) \
  Y(XOR, tos ^= *sp--) \
  Y(DUP, DUP) \
  Y(SWAP, w = tos; tos = *sp; *sp = w) \
  Y(OVER, DUP; tos = sp[-1]) \
  Y(DROP, DROP) \
  X("@", AT, tos = *(cell_t *) tos) \
  X("L@", LAT, tos = *(int32_t *) tos) \
  X("C@", CAT, tos = *(uint8_t *) tos) \
  X("!", STORE, *(cell_t *) tos = *sp--; DROP) \
  X("L!", LSTORE, *(int32_t *) tos = *sp--; DROP) \
  X("C!", CSTORE, *(uint8_t *) tos = *sp--; DROP) \
  X("SP@", SPAT, DUP; tos = (cell_t) sp) \
  X("SP!", SPSTORE, sp = (cell_t *) tos; DROP) \
  X("RP@", RPAT, DUP; tos = (cell_t) rp) \
  X("RP!", RPSTORE, rp = (cell_t *) tos; DROP) \
  X(">R", TOR, *++rp = tos; DROP) \
  X("R>", FROMR, DUP; tos = *rp; --rp) \
  X("R@", RAT, DUP; tos = *rp) \
  Y(EXECUTE, w = tos; DROP; JMPW) \
  Y(BRANCH, ip = (cell_t *) *ip) \
  Y(0BRANCH, if (!tos) ip = (cell_t *) *ip; else ++ip; DROP) \
  Y(DONEXT, *rp = *rp - 1; if (~*rp) ip = (cell_t *) *ip; else (--rp, ++ip)) \
  Y(DOLIT, DUP; tos = *ip++) \
  Y(ALITERAL, COMMA(g_sys.DOLIT_XT); COMMA(tos); DROP) \
  Y(CELL, DUP; tos = sizeof(cell_t)) \
  Y(FIND, tos = find((const char *) *sp, tos); --sp) \
  Y(PARSE, DUP; tos = parse(tos, sp)) \
  X("S>NUMBER?", CONVERT, tos = convert((const char *) *sp, tos, sp); \
                          if (!tos) --sp) \
  Y(CREATE, DUP; DUP; tos = parse(32, sp); \
            create((const char *) *sp, tos, 0, ADDR_DOCREATE); \
            COMMA(0); DROPn(2)) \
  X("DOES>", DOES, DOES(ip); ip = (cell_t *) *rp; --rp) \
  Y(IMMEDIATE, DOIMMEDIATE()) \
  X("'SYS", SYS, DUP; tos = (cell_t) &g_sys) \
  Y(YIELD, PARK; return rp) \
  X(":", COLON, DUP; DUP; tos = parse(32, sp); \
                create((const char *) *sp, tos, SMUDGE, ADDR_DOCOLON); \
                g_sys.state = -1; --sp; DROP) \
  Y(EVALUATE1, DUP; sp = evaluate1(sp); w = *sp--; DROP; if (w) JMPW) \
  Y(EXIT, ip = (cell_t *) *rp--) \
  X(";", SEMICOLON, UNSMUDGE(); COMMA(g_sys.DOEXIT_XT); g_sys.state = 0) \


#define SET tos = (cell_t)

#define n0 tos
#define n1 (*sp)
#define n2 sp[-1]
#define n3 sp[-2]
#define n4 sp[-3]
#define n5 sp[-4]
#define n6 sp[-5]
#define n7 sp[-6]
#define n8 sp[-7]
#define n9 sp[-8]
#define n10 sp[-9]

#define a0 ((void *) tos)
#define a1 (*(void **) &n1)
#define a2 (*(void **) &n2)
#define a3 (*(void **) &n3)
#define a4 (*(void **) &n4)

#define b0 ((uint8_t *) tos)
#define b1 (*(uint8_t **) &n1)
#define b2 (*(uint8_t **) &n2)
#define b3 (*(uint8_t **) &n3)
#define b4 (*(uint8_t **) &n4)

#define c0 ((char *) tos)
#define c1 (*(char **) &n1)
#define c2 (*(char **) &n2)
#define c3 (*(char **) &n3)
#define c4 (*(char **) &n4)



// For now, default on several options.
#define ENABLE_SPIFFS_SUPPORT
#define ENABLE_WIFI_SUPPORT
#define ENABLE_MDNS_SUPPORT
#define ENABLE_WEBSERVER_SUPPORT
#define ENABLE_SDCARD_SUPPORT
#define ENABLE_I2C_SUPPORT

// For now assume only boards with PSRAM (ESP32-CAM)
// will want SerialBluetooth (very large) and camera support.
// Other boards can support these if they're set to a larger
// parition size. But it's unclear the best way to configure this.
#ifdef BOARD_HAS_PSRAM
# define ENABLE_CAMERA_SUPPORT
# define ENABLE_SERIAL_BLUETOOTH_SUPPORT
#endif

#ifdef ENABLE_WEBSERVER_SUPPORT
# include "WebServer.h"
#endif

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#if defined(ESP32)
# define HEAP_SIZE (100 * 1024)
# define STACK_SIZE 512
#elif defined(ESP8266)
# define HEAP_SIZE (40 * 1024)
# define STACK_SIZE 512
#else
# define HEAP_SIZE 2 * 1024
# define STACK_SIZE 32
#endif

#define PLATFORM_OPCODE_LIST \
  /* Memory Allocation */ \
  Y(MALLOC, SET malloc(n0)) \
  Y(SYSFREE, free(a0); DROP) \
  Y(REALLOC, SET realloc(a1, n0); NIP) \
  Y(heap_caps_malloc, SET heap_caps_malloc(n1, n0); NIP) \
  Y(heap_caps_free, heap_caps_free(a0); DROP) \
  Y(heap_caps_realloc, \
      tos = (cell_t) heap_caps_realloc(a2, n1, n0); NIPn(2)) \
  /* Serial */ \
  X("Serial.begin", SERIAL_BEGIN, Serial.begin(tos); DROP) \
  X("Serial.end", SERIAL_END, Serial.end()) \
  X("Serial.available", SERIAL_AVAILABLE, PUSH Serial.available()) \
  X("Serial.readBytes", SERIAL_READ_BYTES, n0 = Serial.readBytes(b1, n0); NIP) \
  X("Serial.write", SERIAL_WRITE, n0 = Serial.write(b1, n0); NIP) \
  X("Serial.flush", SERIAL_FLUSH, Serial.flush()) \
  /* Pins and PWM */ \
  Y(pinMode, pinMode(n1, n0); DROPn(2)) \
  Y(digitalWrite, digitalWrite(n1, n0); DROPn(2)) \
  Y(digitalRead, n0 = digitalRead(n0)) \
  Y(analogRead, n0 = analogRead(n0)) \
  Y(ledcSetup, \
      n0 = (cell_t) (1000000 * ledcSetup(n2, n1 / 1000.0, n0)); NIPn(2)) \
  Y(ledcAttachPin, ledcAttachPin(n1, n0); DROPn(2)) \
  Y(ledcDetachPin, ledcDetachPin(n0); DROP) \
  Y(ledcRead, n0 = ledcRead(n0)) \
  Y(ledcReadFreq, n0 = (cell_t) (1000000 * ledcReadFreq(n0))) \
  Y(ledcWrite, ledcWrite(n1, n0); DROPn(2)) \
  Y(ledcWriteTone, \
      n0 = (cell_t) (1000000 * ledcWriteTone(n1, n0 / 1000.0)); NIP) \
  Y(ledcWriteNote, \
      tos = (cell_t) (1000000 * ledcWriteNote(n2, (note_t) n1, n0)); NIPn(2)) \
  /* General System */ \
  Y(MS, delay(n0); DROP) \
  Y(TERMINATE, exit(n0)) \
  /* File words */ \
  X("R/O", R_O, PUSH O_RDONLY) \
  X("R/W", R_W, PUSH O_RDWR) \
  X("W/O", W_O, PUSH O_WRONLY) \
  Y(BIN, ) \
  X("CLOSE-FILE", CLOSE_FILE, tos = close(tos); tos = tos ? errno : 0) \
  X("FLUSH-FILE", FLUSH_FILE, fsync(tos); /* fsync has no impl and returns ENOSYS :-( */ tos = 0) \
  X("OPEN-FILE", OPEN_FILE, cell_t mode = n0; DROP; cell_t len = n0; DROP; \
    memcpy(filename, a0, len); filename[len] = 0; \
    n0 = open(filename, mode, 0777); PUSH n0 < 0 ? errno : 0) \
  X("CREATE-FILE", CREATE_FILE, cell_t mode = n0; DROP; cell_t len = n0; DROP; \
    memcpy(filename, a0, len); filename[len] = 0; \
    n0 = open(filename, mode | O_CREAT | O_TRUNC); PUSH n0 < 0 ? errno : 0) \
  X("DELETE-FILE", DELETE_FILE, cell_t len = n0; DROP; \
    memcpy(filename, a0, len); filename[len] = 0; \
    n0 = unlink(filename); n0 = n0 ? errno : 0) \
  X("WRITE-FILE", WRITE_FILE, cell_t fd = n0; DROP; cell_t len = n0; DROP; \
    n0 = write(fd, a0, len); n0 = n0 != len ? errno : 0) \
  X("READ-FILE", READ_FILE, cell_t fd = n0; DROP; cell_t len = n0; DROP; \
    n0 = read(fd, a0, len); PUSH n0 < 0 ? errno : 0) \
  X("FILE-POSITION", FILE_POSITION, \
    n0 = (cell_t) lseek(n0, 0, SEEK_CUR); PUSH n0 < 0 ? errno : 0) \
  X("REPOSITION-FILE", REPOSITION_FILE, cell_t fd = n0; DROP; \
    n0 = (cell_t) lseek(fd, tos, SEEK_SET); n0 = n0 < 0 ? errno : 0) \
  X("RESIZE-FILE", RESIZE_FILE, cell_t fd = n0; DROP; n0 = ResizeFile(fd, tos)) \
  X("FILE-SIZE", FILE_SIZE, struct stat st; w = fstat(n0, &st); \
    n0 = (cell_t) st.st_size; PUSH w < 0 ? errno : 0) \
  OPTIONAL_SPIFFS_SUPPORT \
  OPTIONAL_WIFI_SUPPORT \
  OPTIONAL_MDNS_SUPPORT \
  OPTIONAL_WEBSERVER_SUPPORT \
  OPTIONAL_SDCARD_SUPPORT \
  OPTIONAL_I2C_SUPPORT \
  OPTIONAL_SERIAL_BLUETOOTH_SUPPORT \
  OPTIONAL_CAMERA_SUPPORT \
  Y(OledInit, forthdisplay())  \  
  Y(OledHOME, display.setCursor(0,0); DROP) \
  Y(OledCLS, display.clearDisplay()) \
  Y(OledTextc, display.setTextColor(n0); DROP) \
  Y(OledPrintln, display.println(c0); DROP) \
  Y(OledNumln, display.println(n0); DROP) \
  Y(OledNum, display.print(n0); DROP) \
  Y(OledDisplay, display.display()) \
  Y(OledPrint, display.write(c0); DROP) \
  Y(OledInvert, display.invertDisplay(n0 ); DROP) \
  Y(OledTextsize, display.setTextSize(n0 ); DROP) \  
  Y(OledSetCursor, display.setCursor(n1,n0); DROPn(2)) \
  Y(OledPixel, display.drawPixel(n2, n1, n0 ); DROPn(2)) \  
  Y(OledDrawL, display.drawLine(n4,n3,n2,n1,n0 ); DROPn(4)) \ 
  Y(OledCirc, display.drawCircle(n3,n2, n1, n0 ); DROPn(3)) \
  Y(OledCircF, display.fillCircle(n3,n2, n1, n0 ); DROPn(3)) \
  Y(OledRect,  display.drawRect(n4,n3,n2, n1, n0 ); DROPn(4)) \   
  Y(OledRectF, display.fillRect(n4,n3,n2, n1, n0 ); DROPn(3)) \  
  Y(OledRectR,  display.drawRoundRect(n5,n4,n3,n2, n1, n0 ); DROPn(5)) \  
  Y(OledRectRF,  display.fillRoundRect(n5,n4,n3,n2, n1, n0 ); DROPn(5)) \ 
  /*  // Normal 1:1 pixel scale   //     
  // n2= white 1 black 0 
  // Y(OLEDcirc, display.drawCircle(n3,n2, n1, n0 ); DROP) \  
  // display.drawCircle(64, 32, 10, WHITE);
  // n2= white 1 black 0 
  Y(OLEDdrawL, display.drawLine(n4,n3,n2, n1, n0 ); DROP) \   
  // n2= white 1 black 0 
  Y(OLEDinvert, display.invertDisplay(n0 ); DROP) \   
  // n= yes 1 no 0 
  Y(OLEDrect,  display.drawRect(n4,n3,n2, n1, n0 ); DROP) \     
  // display.drawLine(n4,n3,n2, n1, n0 )) 
  Y(OLEDcircle, display.drawCircle(n3,n2, n1,n0); DROP) \    
  // display.drawCircle(64, 32, 10, WHITE);
  */ 

#ifndef ENABLE_SPIFFS_SUPPORT
// Provide a default failing SPIFFS.begin
# define OPTIONAL_SPIFFS_SUPPORT \
  X("SPIFFS.begin", SPIFFS_BEGIN, NIPn(2); n0 = 0)
#else
# include "SPIFFS.h"
# define OPTIONAL_SPIFFS_SUPPORT \
  /* SPIFFS */ \
  X("SPIFFS.begin", SPIFFS_BEGIN, \
      tos = SPIFFS.begin(n2, c1, n0); NIPn(2)) \
  X("SPIFFS.end", SPIFFS_END, SPIFFS.end()) \
  X("SPIFFS.format", SPIFFS_FORMAT, PUSH SPIFFS.format()) \
  X("SPIFFS.totalBytes", SPIFFS_TOTAL_BYTES, PUSH SPIFFS.totalBytes()) \
  X("SPIFFS.usedBytes", SPIFFS_USED_BYTES, PUSH SPIFFS.usedBytes())
#endif

#ifndef ENABLE_CAMERA_SUPPORT
# define OPTIONAL_CAMERA_SUPPORT
#else
# include "esp_camera.h"
# define OPTIONAL_CAMERA_SUPPORT \
  /* Camera */ \
  Y(esp_camera_init, n0 = esp_camera_init((camera_config_t *) a0)) \
  Y(esp_camera_deinit, PUSH esp_camera_deinit()) \
  Y(esp_camera_fb_get, PUSH esp_camera_fb_get()) \
  Y(esp_camera_db_return, esp_camera_fb_return((camera_fb_t *) a0); DROP) \
  Y(esp_camera_sensor_get, PUSH esp_camera_sensor_get())
#endif

#ifndef ENABLE_SDCARD_SUPPORT
# define OPTIONAL_SDCARD_SUPPORT
#else
# include "SD_MMC.h"
# define OPTIONAL_SDCARD_SUPPORT \
  /* SD_MMC */ \
  X("SD_MMC.begin", SD_MMC_BEGIN, tos = SD_MMC.begin(c1, n0); NIP) \
  X("SD_MMC.end", SD_MMC_END, SD_MMC.end()) \
  X("SD_MMC.cardType", SD_MMC_CARD_TYPE, PUSH SD_MMC.cardType()) \
  X("SD_MMC.totalBytes", SD_MMC_TOTAL_BYTES, PUSH SD_MMC.totalBytes()) \
  X("SD_MMC.usedBytes", SD_MMC_USED_BYTES, PUSH SD_MMC.usedBytes())
#endif

#ifndef ENABLE_I2C_SUPPORT
# define OPTIONAL_I2C_SUPPORT
#else
# include <Wire.h>
# define OPTIONAL_I2C_SUPPORT \
  /* Wire */ \
  X("Wire.begin", WIRE_BEGIN, n0 = Wire.begin(n1, n0); NIP) \
  X("Wire.setClock", WIRE_SET_CLOCK, Wire.setClock(n0); DROP) \
  X("Wire.getClock", WIRE_GET_CLOCK, PUSH Wire.getClock()) \
  X("Wire.setTimeout", WIRE_SET_TIMEOUT, Wire.setTimeout(n0); DROP) \
  X("Wire.getTimeout", WIRE_GET_TIMEOUT, PUSH Wire.getTimeout()) \
  X("Wire.lastError", WIRE_LAST_ERROR, PUSH Wire.lastError()) \
  X("Wire.getErrorText", WIRE_GET_ERROR_TEXT, PUSH Wire.getErrorText(n0)) \
  X("Wire.beginTransmission", WIRE_BEGIN_TRANSMISSION, Wire.beginTransmission(n0); DROP) \
  X("Wire.endTransmission", WIRE_END_TRANSMISSION, PUSH Wire.endTransmission(n0)) \
  X("Wire.requestFrom", WIRE_REQUEST_FROM, n0 = Wire.requestFrom(n2, n1, n0); NIPn(2)) \
  X("Wire.writeTransmission", WIRE_WRITE_TRANSMISSION, \
      n0 = Wire.writeTransmission(n3, b2, n1, n0); NIPn(3)) \
  X("Wire.readTransmission", WIRE_READ_TRANSMISSION, \
      n0 = Wire.readTransmission(n4, b3, n2, n1, (uint32_t *) a0); NIPn(4)) \
  X("Wire.write", WIRE_WRITE, n0 = Wire.write(b1, n0); NIP) \
  X("Wire.available", WIRE_AVAILABLE, PUSH Wire.available()) \
  X("Wire.read", WIRE_READ, PUSH Wire.read()) \
  X("Wire.peek", WIRE_PEEK, PUSH Wire.peek()) \
  X("Wire.busy", WIRE_BUSY, PUSH Wire.busy()) \
  X("Wire.flush", WIRE_FLUSH, Wire.flush())
#endif

#ifndef ENABLE_SERIAL_BLUETOOTH_SUPPORT
# define OPTIONAL_SERIAL_BLUETOOTH_SUPPORT
#else
# include "esp_bt_device.h"
# include "BluetoothSerial.h"
# define bt0 ((BluetoothSerial *) a0)
# define OPTIONAL_SERIAL_BLUETOOTH_SUPPORT \
  /* SerialBT */ \
  X("SerialBT.new", SERIALBT_NEW, PUSH new BluetoothSerial()) \
  X("SerialBT.delete", SERIALBT_DELETE, delete bt0; DROP) \
  X("SerialBT.begin", SERIALBT_BEGIN, n0 = bt0->begin(c2, n1); NIPn(2)) \
  X("SerialBT.end", SERIALBT_END, bt0->end(); DROP) \
  X("SerialBT.available", SERIALBT_AVAILABLE, n0 = bt0->available()) \
  X("SerialBT.readBytes", SERIALBT_READ_BYTES, n0 = bt0->readBytes(b2, n1); NIPn(2)) \
  X("SerialBT.write", SERIALBT_WRITE, n0 = bt0->write(b2, n1); NIPn(2)) \
  X("SerialBT.flush", SERIALBT_FLUSH, bt0->flush(); DROP) \
  X("SerialBT.hasClient", SERIALBT_HAS_CLIENT, n0 = bt0->hasClient()) \
  X("SerialBT.enableSSP", SERIALBT_ENABLE_SSP, bt0->enableSSP(); DROP) \
  X("SerialBT.setPin", SERIALBT_SET_PIN, n0 = bt0->setPin(c1); NIP) \
  X("SerialBT.unpairDevice", SERIALBT_UNPAIR_DEVICE, \
      n0 = bt0->unpairDevice(b1); NIP) \
  X("SerialBT.connect", SERIALBT_CONNECT, n0 = bt0->connect(c1); NIP) \
  X("SerialBT.connectAddr", SERIALBT_CONNECT_ADDR, n0 = bt0->connect(b1); NIP) \
  X("SerialBT.disconnect", SERIALBT_DISCONNECT, n0 = bt0->disconnect()) \
  X("SerialBT.connected", SERIALBT_CONNECTED, n0 = bt0->connected(n1); NIP) \
  X("SerialBT.isReady", SERIALBT_IS_READY, n0 = bt0->isReady(n2, n1); NIPn(2)) \
  /* Bluetooth */ \
  Y(esp_bt_dev_get_address, PUSH esp_bt_dev_get_address())
#endif

#ifndef ENABLE_WIFI_SUPPORT
# define OPTIONAL_WIFI_SUPPORT
#else
# include <WiFi.h>
# include <WiFiClient.h>

static IPAddress ToIP(cell_t ip) {
  return IPAddress(ip & 0xff, ((ip >> 8) & 0xff), ((ip >> 16) & 0xff), ((ip >> 24) & 0xff));
}

static cell_t FromIP(IPAddress ip) {
  cell_t ret = 0;
  ret = (ret << 8) | ip[3];
  ret = (ret << 8) | ip[2];
  ret = (ret << 8) | ip[1];
  ret = (ret << 8) | ip[0];
  return ret;
}

# define OPTIONAL_WIFI_SUPPORT \
  /* WiFi */ \
  X("WiFi.config", WIFI_CONFIG, \
      WiFi.config(ToIP(n3), ToIP(n2), ToIP(n1), ToIP(n0)); DROPn(4)) \
  X("WiFi.begin", WIFI_BEGIN, WiFi.begin(c1, c0); DROPn(2)) \
  X("WiFi.disconnect", WIFI_DISCONNECT, WiFi.disconnect()) \
  X("WiFi.status", WIFI_STATUS, PUSH WiFi.status()) \
  X("WiFi.macAddress", WIFI_MAC_ADDRESS, WiFi.macAddress(b0); DROP) \
  X("WiFi.localIP", WIFI_LOCAL_IPS, PUSH FromIP(WiFi.localIP())) \
  X("WiFi.mode", WIFI_MODE, WiFi.mode((wifi_mode_t) n0); DROP) \
  X("WiFi.setTxPower", WIFI_SET_TX_POWER, WiFi.setTxPower((wifi_power_t) n0); DROP) \
  X("WiFi.getTxPower", WIFI_GET_TX_POWER, PUSH WiFi.getTxPower())
#endif

#ifndef ENABLE_MDNS_SUPPORT
# define OPTIONAL_MDNS_SUPPORT
#else
# include <ESPmDNS.h>
# define OPTIONAL_MDNS_SUPPORT \
  /* mDNS */ \
  X("MDNS.begin", MDNS_BEGIN, n0 = MDNS.begin(c0))
#endif

#ifndef ENABLE_WEBSERVER_SUPPORT
# define OPTIONAL_WEBSERVER_SUPPORT
#else
# include <WebServer.h>
# define ws0 ((WebServer *) a0)
# define OPTIONAL_WEBSERVER_SUPPORT \
  /* WebServer */ \
  X("WebServer.new", WEBSERVER_NEW, PUSH new WebServer(tos)) \
  X("WebServer.delete", WEBSERVER_DELETE, delete ws0; DROP) \
  X("WebServer.begin", WEBSERVER_BEGIN, ws0->begin(n1); DROPn(2)) \
  X("WebServer.stop", WEBSERVER_STOP, ws0->stop(); DROP) \
  X("WebServer.on", WEBSERVER_ON, InvokeWebServerOn(ws0, c2, n1); DROPn(3)) \
  X("WebServer.hasArg", WEBSERVER_HAS_ARG, n0 = ws0->hasArg(c1); DROP) \
  X("WebServer.arg", WEBSERVER_ARG, \
      string_value = ws0->arg(c1); \
      c1 = &string_value[0]; n0 = string_value.length()) \
  X("WebServer.argi", WEBSERVER_ARGI, \
      string_value = ws0->arg(n1); \
      c1 = &string_value[0]; n0 = string_value.length()) \
  X("WebServer.argName", WEBSERVER_ARG_NAME, \
      string_value = ws0->argName(n1); \
      c1 = &string_value[0]; n0 = string_value.length()) \
  X("WebServer.args", WEBSERVER_ARGS, n0 = ws0->args()) \
  X("WebServer.setContentLength", WEBSERVER_SET_CONTENT_LENGTH, \
      ws0->setContentLength(n1); DROPn(2)) \
  X("WebServer.sendHeader", WEBSERVER_SEND_HEADER, \
      ws0->sendHeader(c3, c2, n1); DROPn(4)) \
  X("WebServer.send", WEBSERVER_SEND, ws0->send(n3, c2, c1); DROPn(4)) \
  X("WebServer.sendContent", WEBSERVER_SEND_CONTENT, \
      ws0->sendContent(c1); DROPn(2)) \
  X("WebServer.method", WEBSERVER_METHOD, n0 = ws0->method()) \
  X("WebServer.handleClient", WEBSERVER_HANDLE_CLIENT, ws0->handleClient(); DROP)
#endif

static char filename[PATH_MAX];
static String string_value;

#define PRINT_ERRORS 0

#define CELL_LEN(n) (((n) + sizeof(cell_t) - 1) / sizeof(cell_t))
#define FIND(name) find(name, sizeof(name) - 1)
#define LOWER(ch) ((ch) & 0x5F)
#define IMMEDIATE 1
#define SMUDGE 2
#define VOCABULARY_DEPTH 16

#if PRINT_ERRORS
#include <unistd.h>
#endif

static struct {
  const char *tib;
  cell_t ntib, tin, state, base;
  cell_t *heap, **current, ***context, notfound;
  int argc;
  char **argv;
  cell_t *rp;  // spot to park main thread
  cell_t DOLIT_XT, DOEXIT_XT, YIELD_XT;
} g_sys;

static cell_t convert(const char *pos, cell_t n, cell_t *ret) {
  *ret = 0;
  cell_t negate = 0;
  cell_t base = g_sys.base;
  if (!n) { return 0; }
  if (pos[0] == '-') { negate = -1; ++pos; --n; }
  if (pos[0] == '$') { base = 16; ++pos; --n; }
  for (; n; --n) {
    uintptr_t d = pos[0] - '0';
    if (d > 9) {
      d = LOWER(d) - 7;
      if (d < 10) { return 0; }
    }
    if (d >= base) { return 0; }
    *ret = *ret * base + d;
    ++pos;
  }
  if (negate) { *ret = -*ret; }
  return -1;
}

static cell_t same(const char *a, const char *b, cell_t len) {
  for (;len && LOWER(*a) == LOWER(*b); --len, ++a, ++b);
  return len;
}

static cell_t find(const char *name, cell_t len) {
  for (cell_t ***voc = g_sys.context; *voc; ++voc) {
    cell_t *pos = **voc;
    cell_t clen = CELL_LEN(len);
    while (pos) {
      if (!(pos[-1] & SMUDGE) && len == pos[-3] &&
          same(name, (const char *) &pos[-3 - clen], len) == 0) {
        return (cell_t) pos;
      }
      pos = (cell_t *) pos[-2];  // Follow link
    }
  }
  return 0;
}

static void create(const char *name, cell_t length, cell_t flags, void *op) {
  char *pos = (char *) g_sys.heap;
  for (cell_t n = length; n; --n) { *pos++ = *name++; }  // name
  g_sys.heap += CELL_LEN(length);
  *g_sys.heap++ = length;  // length
  *g_sys.heap++ = (cell_t) *g_sys.current;  // link
  *g_sys.heap++ = flags;  // flags
  *g_sys.current = g_sys.heap;
  *g_sys.heap++ = (cell_t) op;  // code
}

static int match(char sep, char ch) {
  return sep == ch || (sep == ' ' && (ch == '\t' || ch == '\n' || ch == '\r'));
}

static cell_t parse(cell_t sep, cell_t *ret) {
  while (g_sys.tin < g_sys.ntib &&
         match(sep, g_sys.tib[g_sys.tin])) { ++g_sys.tin; }
  *ret = (cell_t) (g_sys.tib + g_sys.tin);
  while (g_sys.tin < g_sys.ntib &&
         !match(sep, g_sys.tib[g_sys.tin])) { ++g_sys.tin; }
  cell_t len = g_sys.tin - (*ret - (cell_t) g_sys.tib);
  if (g_sys.tin < g_sys.ntib) { ++g_sys.tin; }
  return len;
}

static cell_t *evaluate1(cell_t *sp) {
  cell_t call = 0;
  cell_t name;
  cell_t len = parse(' ', &name);
  if (len == 0) { *++sp = 0; return sp; }  // ignore empty
  cell_t xt = find((const char *) name, len);
  if (xt) {
    if (g_sys.state && !(((cell_t *) xt)[-1] & IMMEDIATE)) {
      *g_sys.heap++ = xt;
    } else {
      call = xt;
    }
  } else {
    cell_t n;
    cell_t ok = convert((const char *) name, len, &n);
    if (ok) {
      if (g_sys.state) {
        *g_sys.heap++ = g_sys.DOLIT_XT;
        *g_sys.heap++ = n;
      } else {
        *++sp = n;
      }
    } else {
#if PRINT_ERRORS
      write(2, (void *) name, len);
      write(2, "\n", 1);
#endif
      *++sp = name;
      *++sp = len;
      *++sp = -1;
      call = g_sys.notfound;
    }
  }
  *++sp = call;
  return sp;
}

static cell_t *forth_run(cell_t *initrp);

static void forth_init(int argc, char *argv[], void *heap,
                         const char *src, cell_t src_len) {
  g_sys.heap = (cell_t *) heap + 4;  // Leave a little room.
  cell_t *sp = g_sys.heap + 1; g_sys.heap += STACK_SIZE;
  cell_t *rp = g_sys.heap + 1; g_sys.heap += STACK_SIZE;

  // FORTH vocabulary
  *g_sys.heap++ = 0; cell_t *forth = g_sys.heap;
  *g_sys.heap++ = 0;  *g_sys.heap++ = 0;  *g_sys.heap++ = 0;
  // Vocabulary stack
  g_sys.current = (cell_t **) forth;
  g_sys.context = (cell_t ***) g_sys.heap;
  *g_sys.heap++ = (cell_t) forth;
  for (int i = 0; i < VOCABULARY_DEPTH; ++i) { *g_sys.heap++ = 0; }

  forth_run(0);
  (*g_sys.current)[-1] = IMMEDIATE;  // Make last word ; IMMEDIATE
  g_sys.DOLIT_XT = FIND("DOLIT");
  g_sys.DOEXIT_XT = FIND("EXIT");
  g_sys.YIELD_XT = FIND("YIELD");
  g_sys.notfound = FIND("DROP");
  cell_t *start = g_sys.heap;
  *g_sys.heap++ = FIND("EVALUATE1");
  *g_sys.heap++ = FIND("BRANCH");
  *g_sys.heap++ = (cell_t) start;
  g_sys.argc = argc;
  g_sys.argv = argv;
  g_sys.base = 10;
  g_sys.tib = src;
  g_sys.ntib = src_len;
  *++rp = (cell_t) sp;
  *++rp = (cell_t) start;
  g_sys.rp = rp;
}

#define JMPW goto **(void **) w
#define NEXT w = *ip++; JMPW
#define ADDR_DOCOLON && OP_DOCOLON
#define ADDR_DOCREATE && OP_DOCREATE
#define ADDR_DODOES && OP_DODOES

static cell_t *forth_run(cell_t *init_rp) {
  if (!init_rp) {
#define X(name, op, code) create(name, sizeof(name) - 1, name[0] == ';', && OP_ ## op);
    PLATFORM_OPCODE_LIST
    OPCODE_LIST
#undef X
    return 0;
  }
  register cell_t *ip, *rp, *sp, tos, w;
  rp = init_rp;  ip = (cell_t *) *rp--;  sp = (cell_t *) *rp--;
  DROP; NEXT;
#define X(name, op, code) OP_ ## op: { code; } NEXT;
  PLATFORM_OPCODE_LIST
  OPCODE_LIST
#undef X
  OP_DOCOLON: ++rp; *rp = (cell_t) ip; ip = (cell_t *) (w + sizeof(cell_t)); NEXT;
  OP_DOCREATE: DUP; tos = w + sizeof(cell_t) * 2; NEXT;
  OP_DODOES: DUP; tos = w + sizeof(cell_t) * 2;
             ++rp; *rp = (cell_t) ip; ip = (cell_t *) *(cell_t *) (w + sizeof(cell_t)); NEXT;
}

 
const char boot[] =
": (   41 parse drop drop ; immediate\n"
"\n"
"( Useful Basic Compound Words )\n"
": 2drop ( n n -- ) drop drop ;\n"
": 2dup ( a b -- a b a b ) over over ;\n"
": nip ( a b -- b ) swap drop ;\n"
": rdrop ( r: n n -- ) r> r> drop >r ;\n"
": */ ( n n n -- n ) */mod nip ;\n"
": * ( n n -- n ) 1 */ ;\n"
": /mod ( n n -- n n ) 1 swap */mod ;\n"
": / ( n n -- n ) /mod nip ;\n"
": mod ( n n -- n ) /mod drop ;\n"
": invert ( n -- ~n ) -1 xor ;\n"
": negate ( n -- -n ) invert 1 + ;\n"
": - ( n n -- n ) negate + ;\n"
": rot ( a b c -- c a b ) >r swap r> swap ;\n"
": -rot ( a b c -- b c a ) swap >r swap r> ;\n"
": < ( a b -- a<b ) - 0< ;\n"
": > ( a b -- a>b ) swap - 0< ;\n"
": = ( a b -- a!=b ) - 0= ;\n"
": <> ( a b -- a!=b ) = 0= ;\n"
": bl 32 ;   : nl 10 ;\n"
": 1+ 1 + ;   : 1- 1 - ;\n"
": 2* 2 * ;   : 2/ 2 / ;\n"
": 4* 4 * ;   : 4/ 4 / ;\n"
": +! ( n a -- ) swap over @ + swap ! ;\n"
"\n"
"( Cells )\n"
": cell+ ( n -- n ) cell + ;\n"
": cells ( n -- n ) cell * ;\n"
": cell/ ( n -- n ) cell / ;\n"
"\n"
"( System Variables )\n"
": 'tib ( -- a ) 'sys 0 cells + ;\n"
": #tib ( -- a ) 'sys 1 cells + ;\n"
": >in ( -- a ) 'sys 2 cells + ;\n"
": state ( -- a ) 'sys 3 cells + ;\n"
": base ( -- a ) 'sys 4 cells + ;\n"
": 'heap ( -- a ) 'sys 5 cells + ;\n"
": current ( -- a ) 'sys 6 cells + ;\n"
": 'context ( -- a ) 'sys 7 cells + ;  : context 'context @ ;\n"
": 'notfound ( -- a ) 'sys 8 cells + ;\n"
"\n"
"( Dictionary )\n"
": here ( -- a ) 'heap @ ;\n"
": allot ( n -- ) 'heap +! ;\n"
": aligned ( a -- a ) cell 1 - dup >r + r> invert and ;\n"
": align   here aligned here - allot ;\n"
": , ( n --  ) here ! cell allot ;\n"
": c, ( ch -- ) here c! 1 allot ;\n"
"\n"
"( Compilation State )\n"
": [ 0 state ! ; immediate\n"
": ] -1 state ! ; immediate\n"
"\n"
"( Quoting Words )\n"
": ' bl parse 2dup find dup >r -rot r> 0= 'notfound @ execute 2drop ;\n"
": ['] ' aliteral ; immediate\n"
": char bl parse drop c@ ;\n"
": [char] char aliteral ; immediate\n"
": literal aliteral ; immediate\n"
"\n"
"( Core Control Flow )\n"
": begin   here ; immediate\n"
": again   ['] branch , , ; immediate\n"
": until   ['] 0branch , , ; immediate\n"
": ahead   ['] branch , here 0 , ; immediate\n"
": then   here swap ! ; immediate\n"
": if   ['] 0branch , here 0 , ; immediate\n"
": else   ['] branch , here 0 , swap here swap ! ; immediate\n"
": while   ['] 0branch , here 0 , swap ; immediate\n"
": repeat   ['] branch , , here swap ! ; immediate\n"
": aft   drop ['] branch , here 0 , here swap ; immediate\n"
"\n"
"( Compound words requiring conditionals )\n"
": min 2dup < if drop else nip then ;\n"
": max 2dup < if nip else drop then ;\n"
": abs ( n -- +n ) dup 0< if negate then ;\n"
"\n"
"( Dictionary Format )\n"
": >name ( xt -- a n ) 3 cells - dup @ swap over aligned - swap ;\n"
": >link& ( xt -- a ) 2 cells - ;   : >link ( xt -- a ) >link& @ ;\n"
": >flags ( xt -- flags ) cell - ;\n"
": >body ( xt -- a ) dup @ [ ' >flags @ ] literal = 2 + cells + ;\n"
"\n"
"( Postpone - done here so we have ['] and IF )\n"
": immediate? ( xt -- f ) >flags @ 1 and 0= 0= ;\n"
": postpone ' dup immediate? if , else aliteral ['] , , then ; immediate\n"
"\n"
"( Constants and Variables )\n"
": constant ( n \"name\" -- ) create , does> @ ;\n"
": variable ( \"name\" -- ) create 0 , ;\n"
"\n"
"( Stack Convience )\n"
"sp@ constant sp0\n"
"rp@ constant rp0\n"
": depth ( -- n ) sp@ sp0 - cell/ ;\n"
"\n"
"( FOR..NEXT )\n"
": for   postpone >r postpone begin ; immediate\n"
": next   postpone donext , ; immediate\n"
"\n"
"( DO..LOOP )\n"
"variable leaving\n"
": leaving,   here leaving @ , leaving ! ;\n"
": leaving(   leaving @ 0 leaving ! ;\n"
": )leaving   leaving @ swap leaving !\n"
"             begin dup while dup @ swap here swap ! repeat drop ;\n"
": (do) ( n n -- .. ) swap r> -rot >r >r >r ;\n"
": do ( lim s -- ) leaving( postpone (do) here ; immediate\n"
": (?do) ( n n -- n n f .. ) 2dup = if 2drop 0 else -1 then ;\n"
": ?do ( lim s -- ) leaving( postpone (?do) postpone 0branch leaving,\n"
"                   postpone (do) here ; immediate\n"
": unloop   postpone rdrop postpone rdrop ; immediate\n"
": leave   postpone unloop postpone branch leaving, ; immediate\n"
": (+loop) ( n -- f .. ) dup 0< swap r> r> rot + dup r@ < -rot >r >r xor 0= ;\n"
": +loop ( n -- ) postpone (+loop) postpone until\n"
"                 postpone unloop )leaving ; immediate\n"
": loop   1 aliteral postpone +loop ; immediate\n"
": i ( -- n ) postpone r@ ; immediate\n"
": j ( -- n ) rp@ 3 cells - @ ;\n"
"\n"
"( Exceptions )\n"
"variable handler\n"
": catch ( xt -- n )\n"
"  sp@ >r handler @ >r rp@ handler ! execute r> handler ! r> drop 0 ;\n"
": throw ( n -- )\n"
"  dup if handler @ rp! r> handler ! r> swap >r sp! drop r> else drop then ;\n"
"' throw 'notfound !\n"
"\n"
"( Values )\n"
": value ( n -- ) create , does> @ ;\n"
": to ( n -- ) state @ if postpone ['] postpone >body postpone !\n"
"                      else ' >body ! then ; immediate\n"
"\n"
"( Deferred Words )\n"
": defer ( \"name\" -- ) create 0 , does> @ dup 0= throw execute ;\n"
": is ( xt \"name -- ) postpone to ; immediate\n"
"\n"
"( Defer I/O to platform specific )\n"
"defer type\n"
"defer key\n"
"defer bye\n"
": emit ( n -- ) >r rp@ 1 type rdrop ;\n"
": space bl emit ;   : cr nl emit ;\n"
"\n"
"( Numeric Output )\n"
"variable hld\n"
": pad ( -- a ) here 80 + ;\n"
": digit ( u -- c ) 9 over < 7 and + 48 + ;\n"
": extract ( n base -- n c ) u/mod swap digit ;\n"
": <# ( -- ) pad hld ! ;\n"
": hold ( c -- ) hld @ 1 - dup hld ! c! ;\n"
": # ( u -- u ) base @ extract hold ;\n"
": #s ( u -- 0 ) begin # dup while repeat ;\n"
": sign ( n -- ) 0< if 45 hold then ;\n"
": #> ( w -- b u ) drop hld @ pad over - ;\n"
": str ( n -- b u ) dup >r abs <# #s r> sign #> ;\n"
": hex ( -- ) 16 base ! ;   : octal ( -- ) 8 base ! ;\n"
": decimal ( -- ) 10 base ! ;   : binary ( -- ) 2 base ! ;\n"
": u. ( u -- ) <# #s #> type space ;\n"
": . ( w -- ) base @ 10 xor if u. exit then str type space ;\n"
": ? ( a -- ) @ . ;\n"
": n. ( n -- ) base @ swap decimal <# #s #> type base ! ;\n"
"\n"
"( Strings )\n"
": parse-quote ( -- a n ) [char] \" parse ;\n"
": $place ( a n -- ) for aft dup c@ c, 1+ then next drop 0 c, align ;\n"
": $@   r@ dup cell+ swap @ r> dup @ 1+ aligned + cell+ >r ;\n"
": s\"   parse-quote state @ if postpone $@ dup , $place\n"
"       else dup here swap >r >r $place r> r> then ; immediate\n"
": .\"   postpone s\" state @ if postpone type else type then ; immediate\n"
": z\"   postpone s\" state @ if postpone drop else drop then ; immediate\n"
": r\"   parse-quote state @ if swap aliteral aliteral then ; immediate\n"
": r|   [char] | parse state @ if swap aliteral aliteral then ; immediate\n"
": s>z ( a n -- z ) here >r $place r> ;\n"
": z>s ( z -- a n ) 0 over begin dup c@ while 1+ swap 1+ swap repeat drop ;\n"
"\n"
"( Fill, Move )\n"
": cmove ( a a n -- ) for aft >r dup c@ r@ c! 1+ r> 1+ then next 2drop ;\n"
": cmove> ( a a n -- ) for aft 2dup swap r@ + c@ swap r@ + c! then next 2drop ;\n"
": fill ( a a n -- ) swap for swap aft 2dup c! 1 + then next 2drop ;\n"
"\n"
"( Better Errors )\n"
": notfound ( a n n -- )\n"
"   if cr .\" ERROR: \" type .\"  NOT FOUND!\" cr -1 throw then ;\n"
"' notfound 'notfound !\n"
"\n"
"( Examine Dictionary )\n"
": see. ( xt -- ) >name type space ;\n"
": see-one ( xt -- xt+1 )\n"
"   dup @ dup ['] DOLIT = if drop cell+ dup @ . else see. then cell+ ;\n"
": exit= ( xt -- ) ['] exit = ;\n"
": see-loop   >body begin see-one dup @ exit= until ;\n"
": see   cr ['] : see.  ' dup see.  space see-loop drop  ['] ; see.  cr ;\n"
"75 value line-width\n"
": onlines ( n xt -- n xt )\n"
"   swap dup line-width > if drop 0 cr then over >name nip + 1+ swap ;\n"
": words   0 context @ @ begin dup while onlines dup see. >link repeat 2drop cr ;\n"
"\n"
"( Examine Memory )\n"
": dump ( a n -- )\n"
"   cr 0 do i 16 mod 0= if cr then dup i + c@ . loop drop cr ;\n"
": raw.s   depth 0 max for aft sp@ r@ cells - @ . then next ;\n"
"\n"
"( Input )\n"
"variable echo   -1 echo !\n"
": ?echo ( n -- ) echo @ if emit else drop then ;\n"
": ?echo-prompt   echo @ if >r >r raw.s r> r> .\" --> \" then ;\n"
": accept ( a n -- n ) ?echo-prompt 0 swap begin 2dup < while\n"
"     key\n"
"     dup nl = if ?echo drop nip exit then\n"
"     dup 8 = over 127 = or if\n"
"       drop over if rot 1- rot 1- rot 8 ?echo bl ?echo 8 ?echo then\n"
"     else\n"
"       dup ?echo\n"
"       >r rot r> over c! 1+ -rot swap 1+ swap\n"
"     then\n"
"   repeat drop nip ;\n"
"200 constant input-limit\n"
": tib ( -- a ) 'tib @ ;\n"
"create input-buffer   input-limit allot\n"
": tib-setup   input-buffer 'tib ! ;\n"
": refill   tib-setup tib input-limit accept #tib ! 0 >in ! -1 ;\n"
"\n"
"( REPL )\n"
": prompt   .\"  ok\" cr ;\n"
": evaluate-buffer   begin >in @ #tib @ < while evaluate1 repeat ;\n"
": evaluate ( a n -- ) 'tib @ >r #tib @ >r >in @ >r\n"
"                      #tib ! 'tib ! 0 >in ! evaluate-buffer\n"
"                      r> >in ! r> #tib ! r> 'tib ! ;\n"
": quit    begin ['] evaluate-buffer catch\n"
"          if 0 state ! sp0 sp! rp0 rp! .\" ERROR\" cr then\n"
"          prompt refill drop again ;\n"
": ok   .\" Forth\" cr prompt refill drop quit ;\n"
"( Implement Vocabularies )\n"
"variable last-vocabulary\n"
"current @ constant forth-wordlist\n"
": forth   forth-wordlist context ! ;\n"
": vocabulary ( \"name\" ) create 0 , current @ 2 cells + , current @ @ last-vocabulary !\n"
"                        does> cell+ context ! ;\n"
": definitions   context @ current ! ;\n"
": >name-length ( xt -- n ) dup 0= if exit then >name nip ;\n"
": vlist   0 context @ @ begin dup >name-length while onlines dup see. >link repeat 2drop cr ;\n"
"\n"
"( Make it easy to transfer words between vocabularies )\n"
": transfer-xt ( xt --  ) context @ begin 2dup @ <> while @ >link& repeat nip\n"
"                         dup @ swap dup @ >link swap ! current @ @ over >link& !   current @ ! ;\n"
": transfer ( \"name\" ) ' transfer-xt ;\n"
": ?transfer ( \"name\" ) bl parse find dup if transfer-xt else drop then ;\n"
": }transfer ;\n"
": transfer{ begin ' dup ['] }transfer = if drop exit then transfer-xt again ;\n"
"\n"
"( Watered down versions of these )\n"
": only   forth 0 context cell+ ! ;\n"
": voc-stack-end ( -- a ) context begin dup @ while cell+ repeat ;\n"
": also   context context cell+ voc-stack-end over - 2 cells + cmove> ;\n"
": sealed   0 last-vocabulary @ >body cell+ ! ;\n"
": voc. ( voc -- ) dup forth-wordlist = if .\" FORTH \" drop exit then 3 cells - see. ;\n"
": order   context begin dup @ while dup @ voc. cell+ repeat drop cr ;\n"
"\n"
"( Hide some words in an internals vocabulary )\n"
"vocabulary internals   internals definitions\n"
"transfer{\n"
"  transfer-xt voc-stack-end forth-wordlist voc.\n"
"  last-vocabulary\n"
"  branch 0branch donext dolit\n"
"  'context 'notfound notfound\n"
"  immediate? input-buffer ?echo ?echo-prompt\n"
"  evaluate1 evaluate-buffer\n"
"  'sys 'heap aliteral\n"
"  leaving( )leaving leaving leaving,\n"
"  (do) (?do) (+loop)\n"
"  parse-quote digit $@\n"
"  see. see-loop >name-length exit=\n"
"  see-one raw.s\n"
"  tib-setup input-limit\n"
"}transfer\n"
"forth definitions\n"
"\n"
"( Set up Basic I/O )\n"
"internals definitions\n"
": arduino-bye   0 terminate ;\n"
"' arduino-bye is bye\n"
": arduino-type ( a n -- ) Serial.write drop ;\n"
"' arduino-type is type\n"
": arduino-key ( -- n )\n"
"  begin Serial.available until 0 >r rp@ 1 Serial.readBytes drop r> ;\n"
"' arduino-key is key\n"
"forth definitions\n"
": key? ( -- n ) Serial.available ;\n"
"\n"
"( Map Arduino / ESP32 things to shorter names. )\n"
": pin ( n pin# -- ) swap digitalWrite ;\n"
": adc ( n -- n ) analogRead ;\n"
": duty ( n n -- ) 255 min 8191 255 */ ledcWrite ;\n"
": freq ( n n -- ) 1000 * 13 ledcSetup drop ;\n"
": tone ( n n -- ) 1000 * ledcWriteTone drop ;\n"
"\n"
"( Basic Ardiuno Constants )\n"
"0 constant LOW\n"
"1 constant HIGH\n"
"1 constant INPUT\n"
"2 constant OUTPUT\n"
"2 constant LED\n"
"\n"
"( Startup Setup )\n"
"-1 echo !\n"
"115200 Serial.begin\n"
"100 ms\n"
"-1 z\" /spiffs\" 10 SPIFFS.begin drop\n"
"led OUTPUT pinMode\n"
"high led pin\n"
"( Words with OS assist )\n"
": allocate ( n -- a ior ) malloc dup 0= ;\n"
": free ( a -- ior ) sysfree drop 0 ;\n"
": resize ( a n -- a ior ) realloc dup 0= ;\n"
"\n"
"( Migrate various words to separate vocabularies, and constants )\n"
"\n"
"vocabulary Wire   Wire definitions\n"
"transfer{\n"
"  Wire.begin Wire.setClock Wire.getClock\n"
"  Wire.setTimeout Wire.getTimeout\n"
"  Wire.lastError Wire.getErrorText\n"
"  Wire.beginTransmission Wire.endTransmission\n"
"  Wire.requestFrom Wire.writeTransmission\n"
"  Wire.readTransmission Wire.write\n"
"  Wire.available Wire.read\n"
"  Wire.peek Wire.busy Wire.flush\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary WebServer   WebServer definitions\n"
"transfer{\n"
"  WebServer.arg WebServer.argi WebServer.argName\n"
"  WebServer.new WebServer.delete\n"
"  WebServer.begin WebServer.stop\n"
"  WebServer.on WebServer.hasArg\n"
"  WebServer.sendHeader WebServer.send WebServer.sendContent\n"
"  WebServer.method WebServer.handleClient\n"
"  WebServer.args WebServer.setContentLength\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary WiFi   WiFi definitions\n"
"\n"
"transfer{\n"
"  WiFi.config\n"
"  WiFi.begin WiFi.disconnect\n"
"  WiFi.status\n"
"  WiFi.macAddress WiFi.localIP\n"
"  WiFi.mode\n"
"  WiFi.setTxPower WiFi.getTxPower\n"
"}transfer\n"
"\n"
"( WiFi Modes )\n"
"0 constant WIFI_MODE_NULL\n"
"1 constant WIFI_MODE_STA\n"
"2 constant WIFI_MODE_AP\n"
"3 constant WIFI_MODE_APSTA\n"
"\n"
"forth definitions\n"
"\n"
"vocabulary SD_MMC   SD_MMC definitions\n"
"transfer{\n"
"  SD_MMC.begin SD_MMC.end\n"
"  SD_MMC.cardType\n"
"  SD_MMC.totalBytes SD_MMC.usedBytes\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary SPIFFS   SPIFFS definitions\n"
"transfer{\n"
"  SPIFFS.begin SPIFFS.end\n"
"  SPIFFS.format\n"
"  SPIFFS.totalBytes SPIFFS.usedBytes\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary ledc  ledc definitions\n"
"transfer{\n"
"  ledcSetup ledcAttachPin ledcDetachPin\n"
"  ledcRead ledcReadFreq\n"
"  ledcWrite ledcWriteTone ledcWriteNote\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary Serial   Serial definitions\n"
"transfer{\n"
"  Serial.begin Serial.end\n"
"  Serial.available Serial.readBytes\n"
"  Serial.write Serial.flush\n"
"}transfer\n"
"forth definitions\n"
"\n"
"vocabulary bluetooth   bluetooth definitions\n"
"?transfer SerialBT.new\n"
"?transfer SerialBT.delete\n"
"?transfer SerialBT.begin\n"
"?transfer SerialBT.end\n"
"?transfer SerialBT.available\n"
"?transfer SerialBT.readBytes\n"
"?transfer SerialBT.write\n"
"?transfer SerialBT.flush\n"
"?transfer SerialBT.hasClient\n"
"?transfer SerialBT.enableSSP\n"
"?transfer SerialBT.setPin\n"
"?transfer SerialBT.unpairDevice\n"
"?transfer SerialBT.connect\n"
"?transfer SerialBT.connectAddr\n"
"?transfer SerialBT.disconnect\n"
"?transfer SerialBT.connected\n"
"?transfer SerialBT.isReady\n"
"?transfer esp_bt_dev_get_address\n"
"forth definitions\n"
"\n"
"internals definitions\n"
"transfer{\n"
"  malloc sysfree realloc\n"
"  heap_caps_malloc heap_caps_free heap_caps_realloc\n"
"}transfer\n"
"\n"
"( Heap Capabilities )\n"
"binary\n"
"0001 constant MALLOC_CAP_EXEC\n"
"0010 constant MALLOC_CAP_32BIT\n"
"0100 constant MALLOC_CAP_8BIT\n"
"1000 constant MALLOC_CAP_DMA\n"
": MALLOC_CAP_PID ( n -- ) 10000 over 11 ( 3 ) - for 2* next ;\n"
"000010000000000 constant MALLOC_CAP_SPIRAM\n"
"000100000000000 constant MALLOC_CAP_INTERNAL\n"
"001000000000000 constant MALLOC_CAP_DEFAULT\n"
"010000000000000 constant MALLOC_CAP_IRAM_8BIT\n"
"010000000000000 constant MALLOC_CAP_RETENTION\n"
"decimal\n"
"forth definitions\n"
"\n"
"( Including Files )\n"
": included ( a n -- )\n"
"   r/o open-file dup if nip throw else drop then\n"
"   dup file-size throw\n"
"   dup allocate throw\n"
"   swap 2dup >r >r\n"
"   rot dup >r read-file throw drop\n"
"   r> close-file throw\n"
"   r> r> over >r evaluate\n"
"   r> free throw ;\n"
": include ( \"name\" -- ) bl parse included ; \n"
": dump-file ( a n a n -- )\n"
"  w/o create-file if drop .\" failed create-file\" exit then\n"
"  >r r@ write-file if r> drop .\" failed write-file\" exit then\n"
"  r> close-file drop\n"
";\n"
"( Words built after boot )\n"
": assert ( f -- ) 0= throw ;\n"
"\n"
"internals definitions\n"
": mem= ( a a n -- f)\n"
"   for aft 2dup c@ swap c@ <> if 2drop rdrop 0 exit then 1+ swap 1+ then next 2drop -1 ;\n"
"forth definitions also internals\n"
": str= ( a n a n -- f) >r swap r@ <> if rdrop 2drop 0 exit then r> mem= ;\n"
": startswith? ( a n a n -- f ) >r swap r@ < if rdrop 2drop 0 exit then r> mem= ;\n"
": .s   .\" <\" depth n. .\" > \" raw.s cr ;\n"
"only forth definitions\n"
"\n"
": forget ( \"name\" ) ' dup >link current @ !  >name drop here - allot ;\n"
"( Cooperative Tasks )\n"
"\n"
"vocabulary tasks   tasks definitions\n"
"\n"
"variable task-list\n"
"\n"
"forth definitions tasks\n"
"\n"
": task ( xt rsz dsz \"name\" )\n"
"   create here >r 0 , 0 , 0 ,\n"
"   here cell+ r@ cell+ ! cells allot\n"
"   here r@ 2 cells + ! cells allot\n"
"   dup 0= if drop else >body r@ 2 cells + @ ! then rdrop ;\n"
"\n"
": start-task ( t -- )\n"
"   task-list @ if\n"
"     task-list @ @ over !\n"
"     task-list @ !\n"
"   else\n"
"     dup task-list !\n"
"     dup !\n"
"   then\n"
";\n"
"\n"
": pause\n"
"  rp@ task-list @ 2 cells + !\n"
"  sp@ task-list @ cell+ !\n"
"  task-list @ @ task-list !\n"
"  task-list @ cell+ @ sp!\n"
"  task-list @ 2 cells + @ rp!\n"
";\n"
"\n"
"tasks definitions\n"
"0 0 0 task main-task   main-task start-task\n"
"forth definitions\n"
"( Byte Stream / Ring Buffer )\n"
"\n"
"vocabulary streams   streams definitions\n"
"\n"
": stream ( n \"name\" ) create 1+ dup , 0 , 0 , allot align ;\n"
": >write ( st -- wr ) cell+ ;   : >read ( st -- rd ) 2 cells + ;\n"
": >offset ( n st -- a ) 3 cells + + ;\n"
": stream# ( sz -- n ) >r r@ >write @ r@ >read @ - r> @ mod ;\n"
": full? ( st -- f ) dup stream# swap @ 1- = ;\n"
": empty? ( st -- f ) stream# 0= ;\n"
": wait-write ( st -- ) begin dup full? while pause repeat drop ;\n"
": wait-read ( st -- ) begin dup empty? while pause repeat drop ;\n"
": ch>stream ( ch st -- )\n"
"   dup wait-write\n"
"   >r r@ >write @ r@ >offset c!\n"
"   r@ >write @ 1+ r@ @ mod r> >write ! ;\n"
": stream>ch ( st -- ch )\n"
"   dup wait-read\n"
"   >r r@ >read @ r@ >offset c@\n"
"   r@ >read @ 1+ r@ @ mod r> >read ! ;\n"
": >stream ( a n st -- )\n"
"   swap 0 do over c@ over ch>stream swap 1+ swap loop 2drop ;\n"
": stream> ( a n st -- )\n"
"   begin over 1 > over empty? 0= and while\n"
"   dup stream>ch >r rot dup r> swap c! 1+ rot 1- rot repeat 2drop 0 swap c! ;\n"
"\n"
"forth definitions\n"
"( Server Terminal )\n"
"\n"
"also streams also WebServer also WiFi\n"
"vocabulary web-interface   also web-interface definitions\n"
"\n"
": ip# dup 255 and n. [char] . emit 256 / ;\n"
": ip. ( n -- ) ip# ip# ip# 255 and . ;\n"
"\n"
"r|\n"
"<!html>\n"
"<head>\n"
"<title>esp32forth</title>\n"
"<style>\n"
"body {\n"
"  padding: 5px;\n"
"  background-color: #111;\n"
"  color: #2cf;\n"
"}\n"
"#prompt {\n"
"  width: 100%;\n"
"  padding: 5px;\n"
"  font-family: monospace;\n"
"  background-color: #ff8;\n"
"}\n"
"#output {\n"
"  width: 100%;\n"
"  height: 80%;\n"
"  resize: none;\n"
"}\n"
"</style>\n"
"</head>\n"
"<h2>ESP32forth_v7</h2>\n"
"<link rel=\"icon\" href=\"data:,\">\n"
"<body>\n"
"Upload File: <input id=\"filepick\" type=\"file\" name=\"files[]\"></input><br/>\n"
"<button onclick=\"ask('hex')\">hex</button>\n"
"<button onclick=\"ask('decimal')\">decimal</button>\n"
"<button onclick=\"ask('words')\">words</button>\n"
"<button onclick=\"ask('low led pin')\">LED OFF</button>\n"
"<button onclick=\"ask('high led pin')\">LED ON</button>\n"
"<button onclick=\"ask(' cls ')\">RUN</button>\n"
"<br/>\n"
"<textarea id=\"output\" readonly></textarea>\n"
"<input id=\"prompt\" type=\"prompt\"></input><br/>\n"
"<script>\n"
"var prompt = document.getElementById('prompt');\n"
"var filepick = document.getElementById('filepick');\n"
"var output = document.getElementById('output');\n"
"function httpPost(url, items, callback) {\n"
"  var fd = new FormData();\n"
"  for (k in items) {\n"
"    fd.append(k, items[k]);\n"
"  }\n"
"  var r = new XMLHttpRequest();\n"
"  r.onreadystatechange = function() {\n"
"    if (this.readyState == XMLHttpRequest.DONE) {\n"
"      if (this.status === 200) {\n"
"        callback(this.responseText);\n"
"      } else {\n"
"        callback(null);\n"
"      }\n"
"    }\n"
"  };\n"
"  r.open('POST', url);\n"
"  r.send(fd);\n"
"}\n"
"function ask(cmd, callback) {\n"
"  httpPost('/input',\n"
"           {cmd: cmd + '\\n'}, function(data) {\n"
"    if (data !== null) { output.value += data; }\n"
"    output.scrollTop = output.scrollHeight;  // Scroll to the bottom\n"
"    if (callback !== undefined) { callback(); }\n"
"  });\n"
"}\n"
"prompt.onkeyup = function(event) {\n"
"  if (event.keyCode === 13) {\n"
"    event.preventDefault();\n"
"    ask(prompt.value);\n"
"    prompt.value = '';\n"
"  }\n"
"};\n"
"filepick.onchange = function(event) {\n"
"  if (event.target.files.length > 0) {\n"
"    var reader = new FileReader();\n"
"    reader.onload = function(e) {\n"
"      var parts = e.target.result.split('\\n');\n"
"      function upload() {\n"
"        if (parts.length === 0) { filepick.value = ''; return; }\n"
"        ask(parts.shift(), upload);\n"
"      }\n"
"      upload();\n"
"    }\n"
"    reader.readAsText(event.target.files[0]);\n"
"  }\n"
"};\n"
"window.onload = function() {\n"
"  ask('');\n"
"  prompt.focus();\n"
"};\n"
"</script>\n"
"| s>z constant index-html\n"
"\n"
"variable webserver\n"
"20000 constant out-size\n"
"200 stream input-stream\n"
"out-size stream output-stream\n"
"create out-string out-size 1+ allot align\n"
"\n"
": handle-index\n"
"   index-html z>s nip webserver @ WebServer.setContentLength\n"
"   200 z\" text/html\" index-html webserver @ WebServer.send\n"
";\n"
"\n"
": handle-input\n"
"   z\" cmd\" webserver @ WebServer.hasArg if\n"
"     z\" cmd\" webserver @ WebServer.arg input-stream >stream pause\n"
"     out-string out-size output-stream stream>\n"
"     200 z\" text/plain\" out-string webserver @ WebServer.send\n"
"   else\n"
"     500 z\" text/plain\" z\" Missing Input\" webserver @ WebServer.send\n"
"   then\n"
";\n"
"\n"
": serve-type ( a n -- ) output-stream >stream ;\n"
": serve-key ( -- n ) input-stream stream>ch ;\n"
"\n"
": do-serve\n"
"   80 WebServer.new webserver !\n"
"   z\" /webui\" ['] handle-index webserver @ WebServer.on\n"
"   z\" /\" ['] handle-index webserver @ WebServer.on\n"
"   z\" /input\" ['] handle-input webserver @ WebServer.on\n"
"   webserver @ WebServer.begin\n"
"   begin\n"
"     webserver @ WebServer.handleClient\n"
"     1 ms\n"
"     yield\n"
"   again\n"
";\n"
"\n"
"' do-serve 1000 1000 task webserver-task\n"
"\n"
": serve\n"
"   ['] serve-type is type\n"
"   ['] serve-key is key\n"
"   webserver-task start-task\n"
";\n"
"\n"
": login ( z z -- )\n"
"   WIFI_MODE_STA Wifi.mode\n"
"   WiFi.begin begin WiFi.localIP 0= while 100 ms repeat WiFi.localIP ip. cr\n"
"   z\" forth\" MDNS.begin if .\" MDNS started\" else .\" MDNS failed\" then cr ;\n"
"\n"
"also forth definitions\n"
"\n"
": webui ( z z -- ) login serve ;\n"
"\n"
"only forth definitions\n"
"( Handling for OLED )\n"
"\n"
"vocabulary VocOled  also VocOled definitions\n"
"?transfer OledInit\n"
"?transfer OledHome\n"
"?transfer OledCls\n"
"?transfer OledTextc\n"  
"?transfer OledPrintln\n"  
// "?transfer OledNumln\n"  \
// "?transfer OledNum\n" \
// "?transfer OledDisplay\n"  \
// "?transfer OledPrint\n"  \
// "?transfer OledInvert\n"  \
// "  OledTextsize OledSetCursor OledPixel\n" \  
// "  OledDrawL OledCirc\n"  \
// "  OledCircF OledRect\n"  \
// "  OledRectR OledRectRF\n"  \
// "}transfer\n"  \
"\n"
"only forth definitions\n"
"( Handling for ESP32-CAM )\n"
"\n"
"vocabulary camera   camera definitions\n"
"\n"
"?transfer esp_camera_init\n"
"?transfer esp_camera_deinit\n"
"?transfer esp_camera_fb_get\n"
"?transfer esp_camera_fb_return\n"
"?transfer esp_camera_sensor_get\n"
"\n"
"0 constant PIXFORMAT_RGB565\n"
"1 constant PIXFORMAT_YUV422\n"
"2 constant PIXFORMAT_GRAYSCALE\n"
"3 constant PIXFORMAT_JPEG\n"
"4 constant PIXFORMAT_RGB888\n"
"5 constant PIXFORMAT_RAW\n"
"6 constant PIXFORMAT_RGB444\n"
"7 constant PIXFORMAT_RGB555\n"
"\n"
"5 constant FRAMESIZE_QVGA\n"
"8 constant FRAMESIZE_VGA\n"
"\n"
"( See https://github.com/espressif/esp32-camera/blob/master/driver/include/esp_camera.h )\n"
"( Settings for AI_THINKER )\n"
"create camera-config\n"
"  32 , ( pin_pwdn ) -1 , ( pin_reset ) 0 , ( pin_xclk )\n"
"  26 , ( pin_sscb_sda ) 27 , ( pin_sscb_scl )\n"
"  35 , 34 , 39 , 36 , 21 , 19 , 18 , 5 , ( pin_d7 - pin_d0 )\n"
"  25 , ( pin_vsync ) 23 , ( pin_href ) 22 , ( pin_pclk )\n"
"  20000000 , ( xclk_freq_hz )\n"
"  0 , ( ledc_timer ) 0 , ( ledc_channel )\n"
"  here\n"
"  PIXFORMAT_JPEG , ( pixel_format )\n"
"  FRAMESIZE_VGA , ( frame_size ) 12 , ( jpeg_quality 0-63 low good )\n"
"  here\n"
"  1 , ( fb_count )\n"
"constant camera-fb-count\n"
"constant camera-format\n"
"\n"
"forth definitions\n"
": cls ( a -- ) 30 0 do cr loop ;\n"
"( Block Files )\n"
"internals definitions\n"
": clobber-line ( a -- a' ) dup 63 bl fill 63 + nl over c! 1+ ;\n"
": clobber ( a -- ) 15 for clobber-line next drop ;\n"
"0 value block-dirty\n"
"create block-data 1024 allot\n"
"forth definitions internals\n"
"\n"
"-1 value block-fid   variable scr   -1 value block-id\n"
": open-blocks ( a n -- )\n"
"   block-fid 0< 0= if block-fid close-file throw -1 to block-fid then\n"
"   2dup r/w open-file if drop r/w create-file throw else nip nip then to block-fid ;\n"
": use ( \"name\" -- ) bl parse open-blocks ;\n"
"defer default-use\n"
"internals definitions\n"
": common-default-use s\" blocks.fb\" open-blocks ;\n"
"' common-default-use is default-use\n"
": use?!   block-fid 0< if default-use then ;\n"
": grow-blocks ( n -- ) 1024 * block-fid file-size throw max block-fid resize-file throw ;\n"
"forth definitions internals\n"
": save-buffers\n"
"   block-dirty if\n"
"     block-id grow-blocks block-id 1024 * block-fid reposition-file throw\n"
"     block-data 1024 block-fid write-file throw\n"
"     block-fid flush-file throw\n"
"     0 to block-dirty\n"
"   then ;\n"
": block ( n -- a ) use?! dup block-id = if drop block-data exit then\n"
"                   save-buffers dup grow-blocks\n"
"                   dup 1024 * block-fid reposition-file throw\n"
"                   block-data clobber\n"
"                   block-data 1024 block-fid read-file throw drop\n"
"                   to block-id block-data ;\n"
": buffer ( n -- a ) use?! dup block-id = if drop block-data exit then\n"
"                    save-buffers to block-id block-data ;\n"
": empty-buffers   -1 to block-id ;\n"
": update   -1 to block-dirty ;\n"
": flush   save-buffers empty-buffers ;\n"
"\n"
"( Loading )\n"
": load ( n -- ) block 1024 evaluate ;\n"
": thru ( a b -- ) over - 1+ for aft dup >r load r> 1+ then next drop ;\n"
"\n"
"( Utility )\n"
": copy ( from to -- )\n"
"   swap block pad 1024 cmove pad swap block 1024 cmove update ;\n"
"\n"
"( Peterforth addons )\n"
": oo (  -- ) OLEDdisplay ;\n"
"\n"
"( Editing )\n"
": list ( n -- ) scr ! .\" Block \" scr @ . cr scr @ block\n"
"   15 for dup 63 type [char] | emit space 15 r@ - . cr 64 + next drop ;\n"
"internals definitions\n"
": @line ( n -- ) 64 * scr @ block + ;\n"
": e' ( n -- ) @line clobber-line drop update ;\n"
"forth definitions internals\n"
"vocabulary editor   also editor definitions\n"
": l    scr @ list ;   : n    1 scr +! l ;  : p   -1 scr +! l ;\n"
": wipe   15 for r@ e' next l ;   : e   e' l ;\n"
": d ( n -- ) dup 1+ @line swap @line 15 @line over - cmove 15 e ;\n"
": r ( n \"line\" -- ) 0 parse 64 min rot dup e @line swap cmove l ;\n"
": a ( n \"line\" -- ) dup @line over 1+ @line 16 @line over - cmove> r ;\n"
"only forth definitions\n"
"internals definitions\n"
"( Change default block source on arduino )\n"
": arduino-default-use s\" /spiffs/blocks.fb\" open-blocks ;\n"
"' arduino-default-use is default-use\n"
"\n"
"( Check for autoexec.fs and run if present )\n"
": autoexec ( a n -- ) s\" /spiffs/autoexec.fs\" ['] included catch 2drop drop ;\n"
"' autoexec\n"
"forth definitions\n"
"execute\n"
"ok\n"
"\n";
// Work around lack of ftruncate
static cell_t ResizeFile(cell_t fd, cell_t size) {
  struct stat st;
  char buf[256];
  cell_t t = fstat(fd, &st);
  if (t < 0) { return errno; }
  if (size < st.st_size) {
    // TODO: Implement truncation
    return ENOSYS;
  }
  cell_t oldpos = lseek(fd, 0, SEEK_CUR);
  if (oldpos < 0) { return errno; }
  t = lseek(fd, 0, SEEK_END);
  if (t < 0) { return errno; }
  memset(buf, 0, sizeof(buf));
  while (st.st_size < size) {
    cell_t len = sizeof(buf);
    if (size - st.st_size < len) {
      len = size - st.st_size;
    }
    t = write(fd, buf, len);
    if (t != len) {
      return errno;
    }
    st.st_size += t;
  }
  t = lseek(fd, oldpos, SEEK_SET);
  if (t < 0) { return errno; }
  return 0;
}

#ifdef ENABLE_WEBSERVER_SUPPORT
static void InvokeWebServerOn(WebServer *ws, const char *url, cell_t xt) {
  ws->on(url, [xt]() {
    cell_t code[2];
    code[0] = xt;
    code[1] = g_sys.YIELD_XT;
    cell_t stack[16];
    cell_t rstack[16];
    cell_t *rp = rstack;
    *++rp = (cell_t) (stack + 1);
    *++rp = (cell_t) code;
    forth_run(rp);
  });
}
#endif

/******************************************************************************/
//-------------------------------------------------------------------------------------- 

void forthdisplay() {
   display.clearDisplay();
  // display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
   display.println(F("*Esp32forth*"));
                        // display.println(F(" WORDS "));
   display.display();
}

void oleddisplay1() {
 if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    }
   display.println(F("*Esp32forth*"));
   display.display();
   // testdrawchar();
}
 
/****************** ********* *********        */
/********************************** *********  */

void setup() {
  cell_t *heap = (cell_t *) malloc(HEAP_SIZE);
  forth_init(0, 0, heap, boot, sizeof(boot));
  oleddisplay1();
  forthdisplay();
}

void loop() {
  g_sys.rp = forth_run(g_sys.rp);
}
