// esp32 Forth, based on Version 6.3

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#ifdef esp32
#  include "freertos/FreeRTOS.h"
#  include "freertos/task.h"
#  include "esp_system.h"
#  include "driver/gpio.h"
#  include "driver/ledc.h"
#  include "esp_spi_flash.h"
#  include "esp_err.h"
#  include "esp_vfs_dev.h"
#  include "driver/uart.h"
#else
#  include <termios.h>
#  include <unistd.h>
#endif

#ifdef esp32
esp_err_t example_configure_stdin_stdout(void)
{
    // Initialize VFS & UART so we can use std::cout/cin
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install( (uart_port_t)CONFIG_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    return ESP_OK;
}
#else
static struct termios terminalOld;
static void RestoreTerminal(void) {
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalOld);
}
static void SetupTerminal(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  tcgetattr(STDIN_FILENO, &terminalOld);
  atexit(RestoreTerminal);
  struct termios t = terminalOld;
  t.c_lflag &= ~ECHO;
  t.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &t);
}
#endif

#define DEBUG_COREWORDS 0

typedef intptr_t cell_t;
typedef uintptr_t ucell_t;
#if __SIZEOF_POINTER__ == 8
typedef __int128_t dcell_t;
typedef __uint128_t udcell_t;
# define UPPER_MASK 0x5f5f5f5f5f5f5f5f
#elif __SIZEOF_POINTER__ == 4
typedef int64_t dcell_t;
typedef uint64_t udcell_t;
# define UPPER_MASK 0x5f5f5f5f
#else
# error "unsupported cell size"
#endif

#define CELL_BITS (sizeof(cell_t)*8)
#define PRIxCELL PRIxPTR
#define CELL_MASK (sizeof(cell_t)-1)

# define  FALSE 0
# define  TRUE  -1
# define  LOGICAL ? TRUE : FALSE
# define  LOWER(x,y) ((ucell_t)(x)<(ucell_t)(y))
# define  pop top = stack[(unsigned char)S--]
# define  push stack[(unsigned char)++S] = top; top =
# define  popR rack[(unsigned char)R--]
# define  pushR rack[(unsigned char)++R]

cell_t rack[256] = {0};
cell_t stack[256] = {0};
unsigned char R, S, bytecode ;
cell_t* Pointer ;
cell_t  P, IP, WP, top, links, len ;
uint8_t* cData ;
dcell_t d, n, m ;

int EXITT=0,BRAN=0,QBRAN=0,DONXT=0,DOTQP=0,STRQP=0,TOR=0,ABORQP=0;

//#include "rom_54.h" /* load dictionary */
cell_t data[16000] = {};
int IMEDD=0x80;
int COMPO=0x40;

static void HEADER_WITH_FLAGS(int flags, const char *name) {
  P=IP/sizeof(cell_t);
  int i;
  int len = strlen(name) & 0x1f;
  data[P++] = links;
  IP=P*sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, links);
  for (i=links/sizeof(cell_t);i<P;i++) {
    printf(" ");
    printf("%" PRIxCELL, data[i]);
  }
#endif
  links=IP;
  cData[IP++]=len | flags;
  for (i=0;i<len;i++) {
    cData[IP++] = name[i];
  }
  while (IP&CELL_MASK) {
    cData[IP++]=0;
  }
#if DEBUG_COREWORDS
  printf("\n");
  printf("%s", name);
  printf(" ");
  printf("%" PRIxCELL, IP);
#endif
}

static void HEADER(const char *name) {
  HEADER_WITH_FLAGS(0, name);
}

static void HEADER_IMMEDIATE(const char *name) {
  HEADER_WITH_FLAGS(IMEDD, name);
}

static int WithPadding(int sz) {
  return (sz + CELL_MASK) & ~CELL_MASK;
}

static void Comma(cell_t n) {
  assert(IP % sizeof(cell_t) == 0);
  P=IP/sizeof(cell_t);
  data[P++] = n;
  IP=P*sizeof(cell_t);
}

int COLON_OLD(int len, ... ) {
  int addr=IP;
  P=IP/sizeof(cell_t);
  data[P++]=6; // dolist
  va_list argList;
  va_start(argList, len);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
  printf(" ");
  printf("6");
#endif
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%" PRIxCELL, data[P-1]);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
}

int LABEL(int len, ... ) {
  int addr=IP;
  P=IP/sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
#endif
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
  }
void BEGIN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" BEGIN ");
#endif
  pushR=P;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
}
void AGAIN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" AGAIN ");
#endif
  data[P++]=BRAN;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void UNTIL(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" UNTIL ");
#endif
  data[P++]=QBRAN;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void WHILE(int len, ... ) {
  P=IP/sizeof(cell_t);
  int k;
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" WHILE ");
#endif
  data[P++]=QBRAN;
  data[P++]=0;
  k=popR;
  pushR=(P-1);
  pushR=k;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void REPEAT(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" REPEAT ");
#endif
  data[P++]=BRAN;
  data[P++]=popR*sizeof(cell_t);
  data[popR]=P*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void IF(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" IF ");
#endif
  data[P++]=QBRAN;
  pushR=P;
  data[P++]=0;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void ELSE(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ELSE ");
#endif
  data[P++]=BRAN;
  data[P++]=0;
  data[popR]=P*sizeof(cell_t);
  pushR=P-1;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void THEN(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" THEN ");
#endif
  data[popR]=P*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void FOR(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" FOR ");
#endif
  data[P++]=TOR;
  pushR=P;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void NEXT(int len, ... ) {
  P=IP/sizeof(cell_t);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" NEXT ");
#endif
  data[P++]=DONXT;
  data[P++]=popR*sizeof(cell_t);
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void AFT(int len, ... ) {
  P=IP/sizeof(cell_t);
  int k;
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" AFT ");
#endif
  data[P++]=BRAN;
  data[P++]=0;
  k=popR;
  (void)k;
  pushR=P;
  pushR=P-1;
  va_list argList;
  va_start(argList, len);
  for(; len;len--) {
    int j=va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", j);
#endif
  }
  IP=P*sizeof(cell_t);
  va_end(argList);
  }
void DOTQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=DOTQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}
void STRQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=STRQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}
void ABORQ(char seq[]) {
  P=IP/sizeof(cell_t);
  int i;
  int len=strlen(seq);
  data[P++]=ABORQP;
  IP=P*sizeof(cell_t);
  cData[IP++]=len;
  for (i=0;i<len;i++)
     {cData[IP++]=seq[i];}
  while (IP%sizeof(cell_t)) {cData[IP++]=0;}
#if DEBUG_COREWORDS
  printf("\n");
  printf("%" PRIxCELL, IP);
  printf(" ");
  printf("%s", seq);
#endif
}

void CheckSum() {
  int i;
  unsigned char sum=0;
  printf("\n");
  printf("%04" PRIxCELL " ",IP);
  for (i=0;i<32;i++) {
    sum += cData[IP];
    printf("%02x",cData[IP++]);
  }
  printf(" %02x",sum);
}
/******************************************************************************/
/* ledc                                                                       */
/******************************************************************************/
/* LEDC Software Fade */
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            5
int brightness = 255;    // how bright the LED is

/******************************************************************************/
/* PRIMITIVES                                                                 */
/******************************************************************************/

void next(void)
{ P = data[IP/sizeof(cell_t)];
  IP += sizeof(cell_t);
  WP = P+sizeof(cell_t);  }

static int duplexread(unsigned char* dst, int sz) {
  int len = 0;
  while (sz > 0) {
    int ch = fgetc(stdin);
    if (ch == 127 || ch == 8) {
      if (len > 0) {
        fputc(8, stdout);
        fputc(' ', stdout);
        fputc(8, stdout);
        --len;
      }
      continue;
    }
    if (ch == '\n') {
      fputc('\r', stdout);
    }
    fputc(ch, stdout);
    if (ch == '\n' || ch < 0) {
      break;
    }
    dst[len++] = ch;
  }
  return len;
}

void accep()
/* WiFiClient */
{
  len = duplexread(cData, top);
  top = len;
}
void qrx(void)
  { push fgetc(stdin);
    push -1; }

void txsto(void)
{  char c=top;
   fputc(c, stdout);
   pop;
}

void docon(void)
{  push data[WP/sizeof(cell_t)]; }

void dolit(void)
{   push data[IP/sizeof(cell_t)];
  IP += sizeof(cell_t);
  next(); }

void dolist(void)
{   rack[(unsigned char)++R] = IP;
  IP = WP;
  next(); }

void exitt(void)
{   IP = (cell_t) rack[(unsigned char)R--];
  next(); }

void execu(void)
{  P = top;
  WP = P + sizeof(cell_t);
  pop; }

void donext(void)
{   if(rack[(unsigned char)R]) {
    rack[(unsigned char)R] -= 1 ;
    IP = data[IP/sizeof(cell_t)];
  } else { IP += sizeof(cell_t);  R-- ;  }
  next(); }

void qbran(void)
{   if(top == 0) IP = data[IP/sizeof(cell_t)];
  else IP += sizeof(cell_t);  pop;
  next(); }

void bran(void)
{   IP = data[IP/sizeof(cell_t)];
  next(); }

void store(void)
{   data[top/sizeof(cell_t)] = stack[(unsigned char)S--];
  pop;  }

void at(void)
{   top = data[top/sizeof(cell_t)];  }

void cstor(void)
{   cData[top] = (unsigned char) stack[(unsigned char)S--];
  pop;  }

void cat(void)
{   top = (cell_t) cData[top];  }

void rpat(void) {}
void rpsto(void) {}

void rfrom(void)
{   push rack[(unsigned char)R--];  }

void rat(void)
{   push rack[(unsigned char)R];  }

void tor(void)
{   rack[(unsigned char)++R] = top;  pop;  }

void spat(void) {}
void spsto(void) {}

void drop(void)
{   pop;  }

void dup_(void)
{   stack[(unsigned char)++S] = top;  }

void swap(void)
{   WP = top;
  top = stack[(unsigned char)S];
  stack[(unsigned char)S] = WP;  }

void over(void)
{  push stack[(unsigned char)(S-1)];  }

void zless(void)
{   top = (top < 0) LOGICAL;  }

void andd(void)
{   top &= stack[(unsigned char)S--];  }

void orr(void)
{   top |= stack[(unsigned char)S--];  }

void xorr(void)
{   top ^= stack[(unsigned char)S--];  }

void uplus(void)
{   stack[(unsigned char)S] += top;
  top = LOWER(stack[(unsigned char)S], top);  }

void nop(void)
{   next(); }

void qdup(void)
{   if(top) stack[(unsigned char)++S] = top ;  }

void rot(void)
{   WP = stack[(unsigned char)(S-1)];
  stack[(unsigned char)(S-1)] = stack[(unsigned char)S];
  stack[(unsigned char)S] = top;
  top = WP;  }

void ddrop(void)
{   drop(); drop();  }

void ddup(void)
{   over(); over();  }

void plus(void)
{   top += stack[(unsigned char)S--];  }

void inver(void)
{   top = -top-1;  }

void negat(void)
{   top = 0 - top;  }

void dnega(void)
{   inver();
  tor();
  inver();
  push 1;
  uplus();
  rfrom();
  plus(); }

void subb(void)
{   top = stack[(unsigned char)S--] - top;  }

void abss(void)
{   if(top < 0)
    top = -top;  }

void great(void)
{   top = (stack[(unsigned char)S--] > top) LOGICAL;  }

void less(void)
{   top = (stack[(unsigned char)S--] < top) LOGICAL;  }

void equal(void)
{   top = (stack[(unsigned char)S--] == top) LOGICAL;  }

void uless(void)
{   top = LOWER(stack[(unsigned char)S], top) LOGICAL; S--;  }

void ummod(void)
{  d = (udcell_t)((ucell_t)top);
  m = (udcell_t)((ucell_t)stack[(unsigned char) S]);
  n = (udcell_t)((ucell_t)stack[(unsigned char) (S - 1)]);
  n += m << CELL_BITS;
  pop;
  if (d == 0) {
    top = 0;
    stack[S] = 0;
    return;
  }
  top = (ucell_t)(n / d);
  stack[(unsigned char) S] = (ucell_t)(n%d); }
void msmod(void)
{ d = (dcell_t)((cell_t)top);
  m = (dcell_t)((cell_t)stack[(unsigned char) S]);
  n = (dcell_t)((cell_t)stack[(unsigned char) S - 1]);
  n += m << CELL_BITS;
  pop;
  if (d == 0) {
    top = 0;
    stack[S] = 0;
    return;
  }
  top = (cell_t)(n / d);
  stack[(unsigned char) S] = (cell_t)(n%d); }
void slmod(void)
{ if (top != 0) {
    WP = stack[(unsigned char) S] / top;
    stack[(unsigned char) S] %= top;
    top = WP;
  } }
void mod(void)
{ top = (top) ? stack[(unsigned char) S--] % top : stack[(unsigned char) S--]; }
void slash(void)
{ top = (top) ? stack[(unsigned char) S--] / top : (S--, 0); }
void umsta(void)
{ d = (udcell_t)top;
  m = (udcell_t)stack[(unsigned char) S];
  m *= d;
  top = (ucell_t)(m >> CELL_BITS);
  stack[(unsigned char) S] = (ucell_t)m; }
void star(void)
{ top *= stack[(unsigned char) S--]; }
void mstar(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  m *= d;
  top = (cell_t)(m >> CELL_BITS);
  stack[(unsigned char) S] = (cell_t)m; }
void ssmod(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  n = (dcell_t)stack[(unsigned char) (S - 1)];
  n *= m;
  pop;
  top = (cell_t)(n / d);
  stack[(unsigned char) S] = (cell_t)(n%d); }
void stasl(void)
{ d = (dcell_t)top;
  m = (dcell_t)stack[(unsigned char) S];
  n = (dcell_t)stack[(unsigned char) (S - 1)];
  n *= m;
  pop; pop;
  top = (cell_t)(n / d); }

void pick(void)
{   top = stack[(unsigned char)(S-top)];  }

void pstor(void)
{   data[top/sizeof(cell_t)] += stack[(unsigned char)S--], pop;  }

void dstor(void)
{   data[(top/sizeof(cell_t))+1] = stack[(unsigned char)S--];
  data[top/sizeof(cell_t)] = stack[(unsigned char)S--];
  pop;  }

void dat(void)
{   push data[top/sizeof(cell_t)];
  top = data[(top/sizeof(cell_t))+1];  }

void count(void)
{   stack[(unsigned char)++S] = top + 1;
  top = cData[top]; }

void dovar(void)
{   push WP; }

void maxx(void)
{   if (top < stack[(unsigned char)S]) pop;
  else S--; }

void minn(void)
{   if (top < stack[(unsigned char)S]) S--;
  else pop; }

void audio(void)
{  WP=top; pop;
   // ledcWriteTone(WP,top);
   pop;
}

void sendPacket(void)
{}

void poke(void)
{   Pointer = (cell_t*)top; *Pointer = stack[(unsigned char)S--];
    pop;  }

void peeek(void)
{   Pointer = (cell_t*)top; top = *Pointer;  }

void adc(void) {
  //top= (cell_t) analogRead(top);
  top= (cell_t) 0;
}

static void setpin(int p, int level) {
#ifdef esp32
   gpio_pad_select_gpio(p);
   gpio_set_direction(p, GPIO_MODE_OUTPUT);
   gpio_set_level(p, top);
#endif
}

void pin(void)
{  WP=top; pop;
   //ledcAttachPin(top,WP);
   setpin(WP, top);
   pop;
}

void ms(void) {
  WP = top; pop;
#ifdef esp32
  vTaskDelay(WP / portTICK_PERIOD_MS);
#endif
}

void duty(void)
{  WP=top; pop;
   //ledcAnalogWrite(WP,top,255);
   pop;
}

void freq(void)
{  WP=top; pop;
   //ledcSetup(WP,top,13);
   pop;
}

void (*primitives[73])(void) = {
    /* case 0 */ nop,
    /* case 1 */ accep,
    /* case 2 */ qrx,
    /* case 3 */ txsto,
    /* case 4 */ docon,
    /* case 5 */ dolit,
    /* case 6 */ dolist,
    /* case 7 */ exitt,
    /* case 8 */ execu,
    /* case 9 */ donext,
    /* case 10 */ qbran,
    /* case 11 */ bran,
    /* case 12 */ store,
    /* case 13 */ at,
    /* case 14 */ cstor,
    /* case 15 */ cat,
    /* case 16 */ nop,
    /* case 17 */ nop,
    /* case 18 */ rfrom,
    /* case 19 */ rat,
    /* case 20 */ tor,
    /* case 21 */ nop,
    /* case 22 */ nop,
    /* case 23 */ drop,
    /* case 24 */ dup_,
    /* case 25 */ swap,
    /* case 26 */ over,
    /* case 27 */ zless,
    /* case 28 */ andd,
    /* case 29 */ orr,
    /* case 30 */ xorr,
    /* case 31 */ uplus,
    /* case 32 */ next,
    /* case 33 */ qdup,
    /* case 34 */ rot,
    /* case 35 */ ddrop,
    /* case 36 */ ddup,
    /* case 37 */ plus,
    /* case 38 */ inver,
    /* case 39 */ negat,
    /* case 40 */ dnega,
    /* case 41 */ subb,
    /* case 42 */ abss,
    /* case 43 */ equal,
    /* case 44 */ uless,
    /* case 45 */ less,
    /* case 46 */ ummod,
    /* case 47 */ msmod,
    /* case 48 */ slmod,
    /* case 49 */ mod,
    /* case 50 */ slash,
    /* case 51 */ umsta,
    /* case 52 */ star,
    /* case 53 */ mstar,
    /* case 54 */ ssmod,
    /* case 55 */ stasl,
    /* case 56 */ pick,
    /* case 57 */ pstor,
    /* case 58 */ dstor,
    /* case 59 */ dat,
    /* case 60 */ count,
    /* case 61 */ dovar,
    /* case 62 */ maxx,
    /* case 63 */ minn,
    /* case 64 */ audio,
    /* case 65 */ sendPacket,
    /* case 66 */ poke,
    /* case 67 */ peeek,
    /* case 68 */ adc,
    /* case 69 */ pin,
    /* case 70 */ duty,
    /* case 71 */ freq,
    /* case 72 */ ms };

int as_nop=0;
int as_accept=1;
int as_qrx=2;
int as_txsto=3;
int as_docon=4;
int as_dolit=5;
int as_dolist=6;
int as_exit=7;
int as_execu=8;
int as_donext=9;
int as_qbran=10;
int as_bran=11;
int as_store=12;
int as_at=13;
int as_cstor=14;
int as_cat=15;
int as_rpat=16;
int as_rpsto=17;
int as_rfrom=18;
int as_rat=19;
int as_tor=20;
int as_spat=21;
int as_spsto=22;
int as_drop=23;
int as_dup=24;
int as_swap=25;
int as_over=26;
int as_zless=27;
int as_andd=28;
int as_orr=29;
int as_xorr=30;
int as_uplus=31;
int as_next=32;
int as_qdup=33;
int as_rot=34;
int as_ddrop=35;
int as_ddup=36;
int as_plus=37;
int as_inver=38;
int as_negat=39;
int as_dnega=40;
int as_subb=41;
int as_abss=42;
int as_equal=43;
int as_uless=44;
int as_less=45;
int as_ummod=46;
int as_msmod=47;
int as_slmod=48;
int as_mod=49;
int as_slash=50;
int as_umsta=51;
int as_star=52;
int as_mstar=53;
int as_ssmod=54;
int as_stasl=55;
int as_pick=56;
int as_pstor=57;
int as_dstor=58;
int as_dat=59;
int as_count=60;
int as_dovar=61;
int as_max=62;
int as_min=63;
int as_tone=64;
int as_sendPacket=65;
int as_poke=66;
int as_peek=67;
int as_adc=68;
int as_pin=69;
int as_duty=70;
int as_freq=71;
int as_ms=72;

int CODE(const char *name, ... ) {
  HEADER(name);
  int addr=IP;
  int s;
  int len = 0;
  va_list argList;
  va_start(argList, name);
  do {
    s = va_arg(argList, int);
    cData[IP++] = s;
    ++len;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%x", s);
#endif
  } while(s != as_next);
  int total = WithPadding(len);
  int padding = total - len;
  for (; padding; --padding) {
    cData[IP++]=0;
  }
  va_end(argList);
  return addr;
}

int COLON_WITH_FLAGS(int flags, const char *name, ...) {
  HEADER_WITH_FLAGS(flags, name);
  int addr=IP;
  P=IP/sizeof(cell_t);
  data[P++] = as_dolist;
  va_list argList;
  va_start(argList, name);
#if DEBUG_COREWORDS
  printf("\n");
  printf("%x", addr);
  printf(" ");
  printf("6");
#endif
  int j;
  do {
    j = va_arg(argList, int);
    data[P++]=j;
#if DEBUG_COREWORDS
    printf(" ");
    printf("%" PRIxCELL, data[P-1]);
#endif
  } while (j != EXITT);
  IP=P*sizeof(cell_t);
  va_end(argList);
  return addr;
}

#define COLON(...) COLON_WITH_FLAGS(0, __VA_ARGS__)
#define COLON_IMMEDIATE(...) COLON_WITH_FLAGS(IMEDD, __VA_ARGS__)

static int CONSTANT(const char *name, cell_t n) {
  int ret = CODE(name, as_docon, as_next);
  Comma(n);
  return ret;
}

void evaluate()
{
  for(;;) {
    bytecode=(unsigned char)cData[P++];
    //printf("%d ", bytecode);
    if (bytecode) primitives[bytecode]();
    else break;
  }
}

static void run() {
  printf("AIBOT\n");
  // TODO: Find better way to start in decimal.
  strcpy((char*) cData, "decimal");
  len = strlen((char*) cData);
  for (;;) {
    data[0x66] = 0;                   // >IN
    data[0x67] = len;                 // #TIB
    data[0x68] = 0;                   // 'TIB
    P = 0x60 * sizeof(cell_t);        // EVAL
    WP = P + sizeof(cell_t);
    evaluate();
    len = duplexread(cData, 255);
  }
}

#ifdef esp32
void app_main(void) {
#else
int main(void) {
#endif
#ifdef esp32
  example_configure_stdin_stdout();
#else
  SetupTerminal();
#endif
#if 0
  printf("booting...\n");
#endif
  P = 0x60 * sizeof(cell_t);
  WP = P + sizeof(cell_t);
  IP = 0;
  S = 0;
  R = 0;
  top = 0;
  cData = (uint8_t *) data;

#if 0
// Setup timer and attach timer to a led pin
  ledcSetup(0, 100, LEDC_TIMER_13_BIT);
  ledcAttachPin(5, 0);
  ledcAnalogWrite(0, 250, brightness);
  pinMode(2,OUTPUT);
  digitalWrite(2, HIGH);   // turn the LED2 on
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);   // motor1 forward
  pinMode(17,OUTPUT);
  digitalWrite(17, LOW);   // motor1 backward
  pinMode(18,OUTPUT);
  digitalWrite(18, LOW);   // motor2 forward
  pinMode(19,OUTPUT);
  digitalWrite(19, LOW);   // motor2 bacward
#endif

  IP=128 * sizeof(cell_t);
  cell_t datap = 0x64 * sizeof(cell_t) - sizeof(cell_t);
  R=0;
  int HLD=CONSTANT("HLD", datap += sizeof(cell_t));
  int SPAN=CONSTANT("SPAN", datap += sizeof(cell_t));
  int INN=CONSTANT(">IN", datap += sizeof(cell_t));
  int NTIB=CONSTANT("#TIB", datap += sizeof(cell_t));
  int TTIB=CONSTANT("'TIB", datap += sizeof(cell_t));
  int BASE=CONSTANT("BASE", datap += sizeof(cell_t));
  int CNTXT=CONSTANT("CONTEXT", datap += sizeof(cell_t));
  int CP=CONSTANT("CP", datap += sizeof(cell_t));
  int LAST=CONSTANT("LAST", datap += sizeof(cell_t));
  int TEVAL=CONSTANT("'EVAL", datap += sizeof(cell_t));
  int TABRT=CONSTANT("'ABORT", datap += sizeof(cell_t));
  int TEMP=CONSTANT("tmp", datap += sizeof(cell_t));
  int Z=CONSTANT("Z", 0);
  int PPQN=CONSTANT("ppqn", datap += sizeof(cell_t));
  int CHANN=CONSTANT("channel", datap += sizeof(cell_t));

  int NOP=CODE("NOP", as_nop, as_next);
  int ACCEP=CODE("ACCEPT", as_accept, as_next);
  int QKEY=CODE("?KEY", as_qrx, as_next);
  int EMIT=CODE("EMIT", as_txsto, as_next);
  int DOLIT=CODE("DOLIT", as_dolit, as_next);
  int DOLST=CODE("DOLIST", as_dolist, as_next);
  EXITT=CODE("EXIT", as_exit, as_next);
  int EXECU=CODE("EXECUTE", as_execu, as_next);
  DONXT=CODE("DONEXT", as_donext, as_next);
  QBRAN=CODE("QBRANCH", as_qbran, as_next);
  BRAN=CODE("BRANCH", as_bran, as_next);
  int STORE=CODE("!", as_store, as_next);
  int AT=CODE("@", as_at, as_next);
  int CSTOR=CODE("C!", as_cstor, as_next);
  int CAT=CODE("C@", as_cat, as_next);
  int RFROM=CODE("R>", as_rfrom, as_next);
  int RAT=CODE("R@", as_rat, as_next);
  TOR=CODE(">R", as_tor, as_next);
  int DROP=CODE("DROP", as_drop, as_next);
  int DUPP=CODE("DUP", as_dup, as_next);
  int SWAP=CODE("SWAP", as_swap, as_next);
  int OVER=CODE("OVER", as_over, as_next);
  int ZLESS=CODE("0<", as_zless, as_next);
  int ANDD=CODE("AND", as_andd, as_next);
  int ORR=CODE("OR", as_orr, as_next);
  int XORR=CODE("XOR", as_xorr, as_next);
  int UPLUS=CODE("UM+", as_uplus, as_next);
  int QDUP=CODE("?DUP", as_qdup, as_next);
  int ROT=CODE("ROT", as_rot, as_next);
  int DDROP=CODE("2DROP", as_ddrop, as_next);
  int DDUP=CODE("2DUP", as_ddup, as_next);
  int PLUS=CODE("+", as_plus, as_next);
  int INVER=CODE("NOT", as_inver, as_next);
  int NEGAT=CODE("NEGATE", as_negat, as_next);
  int DNEGA=CODE("DNEGATE", as_dnega, as_next);
  int SUBBB=CODE("-", as_subb, as_next);
  int ABSS=CODE("ABS", as_abss, as_next);
  int EQUAL=CODE("=", as_equal, as_next);
  int ULESS=CODE("U<", as_uless, as_next);
  int LESS=CODE("<", as_less, as_next);
  int UMMOD=CODE("UM/MOD", as_ummod, as_next);
  int MSMOD=CODE("M/MOD", as_msmod, as_next);
  int SLMOD=CODE("/MOD", as_slmod, as_next);
  int MODD=CODE("MOD", as_mod, as_next);
  int SLASH=CODE("/", as_slash, as_next);
  int UMSTA=CODE("UM*", as_umsta, as_next);
  int STAR=CODE("*", as_star, as_next);
  int MSTAR=CODE("M*", as_mstar, as_next);
  int SSMOD=CODE("*/MOD", as_ssmod, as_next);
  int STASL=CODE("*/", as_stasl, as_next);
  int PICK=CODE("PICK", as_pick, as_next);
  int PSTOR=CODE("+!", as_pstor, as_next);
  int DSTOR=CODE("2!", as_dstor, as_next);
  int DAT=CODE("2@", as_dat, as_next);
  int COUNT=CODE("COUNT", as_count, as_next);
  int MAX=CODE("MAX", as_max, as_next);
  int MIN=CODE("MIN", as_min, as_next);
  int BLANK=CONSTANT("BL", 32);
  int CELL=CONSTANT("CELL", sizeof(cell_t));
  int CELLP=CODE("CELL+", as_docon, as_plus, as_next); Comma(sizeof(cell_t));
  int CELLM=CODE("CELL-", as_docon, as_subb, as_next); Comma(sizeof(cell_t));
  int CELLS=CODE("CELLS", as_docon, as_star, as_next); Comma(sizeof(cell_t));
  int CELLD=CODE("CELL/", as_docon, as_slash, as_next); Comma(sizeof(cell_t));
  int ONEP=CODE("1+", as_docon, as_plus, as_next); Comma(1);
  int ONEM=CODE("1-", as_docon, as_subb, as_next); Comma(1);
  int TWOP=CODE("2+", as_docon, as_plus, as_next); Comma(2);
  int TWOM=CODE("2-", as_docon, as_subb, as_next); Comma(2);
  int TWOST=CODE("2*", as_docon, as_star, as_next); Comma(2);
  int TWOS=CODE("2/", as_docon, as_slash, as_next); Comma(2);
  int SENDP=CODE("sendPacket", as_sendPacket, as_next);
  int POKE=CODE("POKE", as_poke, as_next);
  int PEEK=CODE("PEEK", as_peek, as_next);
  int ADC=CODE("ADC", as_adc, as_next);
  int PIN=CODE("PIN", as_pin, as_next);
  int TONE=CODE("TONE", as_tone,as_next);
  int DUTY=CODE("DUTY", as_duty, as_next);
  int FREQ=CODE("FREQ", as_freq, as_next);
  int MS=CODE("MS", as_ms, as_next);

  HEADER("KEY");
  int KEY=COLON_OLD(0);
  BEGIN(1,QKEY);
  UNTIL(1,EXITT);
  int WITHI=COLON("WITHIN", OVER,SUBBB,TOR,SUBBB,RFROM,ULESS,EXITT);
  HEADER(">CHAR");
  int TCHAR=COLON_OLD(8,DOLIT,0x7F,ANDD,DUPP,DOLIT,127,BLANK,WITHI);
  IF(3,DROP,DOLIT,'_');
  THEN(1,EXITT);
  HEADER("ALIGNED");
  int ALIGN=COLON_OLD(7,DOLIT,CELL_MASK,PLUS,
                  DOLIT,~CELL_MASK,ANDD,EXITT);
  int HERE=COLON("HERE", CP,AT,EXITT);
  int PAD=COLON("PAD", HERE,DOLIT,80,PLUS,EXITT);
  int TIB=COLON("TIB", TTIB,AT,EXITT);
  HEADER("@EXECUTE");
  int ATEXE=COLON_OLD(2,AT,QDUP);
  IF(1,EXECU);
  THEN(1,EXITT);
  HEADER("CMOVE");
  int CMOVEE=COLON_OLD(0);
  FOR(0);
  AFT(8,OVER,CAT,OVER,CSTOR,TOR,ONEP,RFROM,ONEP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  HEADER("MOVE");
  int MOVE=COLON_OLD(1,CELLD);
  FOR(0);
  AFT(8,OVER,AT,OVER,STORE,TOR,CELLP,RFROM,CELLP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  HEADER("FILL");
  int FILL=COLON_OLD(1,SWAP);
  FOR(1,SWAP);
  AFT(3,DDUP,CSTOR,ONEP);
  THEN(0);
  NEXT(2,DDROP,EXITT);
  int DIGIT=COLON("DIGIT", DOLIT,9,OVER,LESS,DOLIT,7,ANDD,PLUS,
                           DOLIT,'0',PLUS,EXITT);
  int EXTRC=COLON("EXTRACT", DOLIT,0,SWAP,UMMOD,SWAP,DIGIT,EXITT);
  int BDIGS=COLON("<#", PAD,HLD,STORE,EXITT);
  int HOLD=COLON("HOLD", HLD,AT,ONEM,DUPP,HLD,STORE,CSTOR,EXITT);
  int DIG=COLON("#", BASE,AT,EXTRC,HOLD,EXITT);
  HEADER("#S");
  int DIGS=COLON_OLD(0);
  BEGIN(2,DIG,DUPP);
  WHILE(0);
  REPEAT(1,EXITT);
  HEADER("SIGN");
  int SIGN=COLON_OLD(1,ZLESS);
  IF(3,DOLIT,'-',HOLD);
  THEN(1,EXITT);
  int EDIGS=COLON("#>", DROP,HLD,AT,PAD,OVER,SUBBB,EXITT);
  int STRR=COLON("str", DUPP,TOR,ABSS,BDIGS,DIGS,RFROM,SIGN,EDIGS,EXITT);
  int HEXX=COLON("HEX", DOLIT,16,BASE,STORE,EXITT);
  int DECIM=COLON("DECIMAL", DOLIT,10,BASE,STORE,EXITT);
  HEADER("wupper");
  int UPPER=COLON_OLD(1,DOLIT); Comma(UPPER_MASK); Comma(ANDD); Comma(EXITT);
  HEADER(">upper");
  int TOUPP=COLON_OLD(6,DUPP,DOLIT,'a',DOLIT,'{',WITHI);
  IF(3,DOLIT,0x5F,ANDD);
  THEN(1,EXITT);
  HEADER("DIGIT?");
  int DIGTQ=COLON_OLD(9,TOR,TOUPP,DOLIT,'0',SUBBB,DOLIT,9,OVER,LESS);
  IF(8,DOLIT,7,SUBBB,DUPP,DOLIT,10,LESS,ORR);
  THEN(4,DUPP,RFROM,ULESS,EXITT);
  HEADER("NUMBER?");
  int NUMBQ=COLON_OLD(12,BASE,AT,TOR,DOLIT,0,OVER,COUNT,OVER,CAT,DOLIT,'$',EQUAL);
  IF(5,HEXX,SWAP,ONEP,SWAP,ONEM);
  THEN(13,OVER,CAT,DOLIT,'-',EQUAL,TOR,SWAP,RAT,SUBBB,SWAP,RAT,PLUS,QDUP);
  IF(1,ONEM);
  FOR(6,DUPP,TOR,CAT,BASE,AT,DIGTQ);
  WHILE(7,SWAP,BASE,AT,STAR,PLUS,RFROM,ONEP);
  NEXT(2,DROP,RAT);
  IF(1,NEGAT);
  THEN(1,SWAP);
  ELSE(6,RFROM,RFROM,DDROP,DDROP,DOLIT,0);
  THEN(1,DUPP);
  THEN(6,RFROM,DDROP,RFROM,BASE,STORE,EXITT);
  int SPACE=COLON("SPACE", BLANK,EMIT,EXITT);
  HEADER("CHARS");
  int CHARS=COLON_OLD(4,SWAP,DOLIT,0,MAX);
  FOR(0);
  AFT(2,DUPP,EMIT);
  THEN(0);
  NEXT(2,DROP,EXITT);
  int SPACS=COLON("SPACES", BLANK,CHARS,EXITT);
  HEADER("TYPE");
  int TYPES=COLON_OLD(0);
  FOR(0);
  AFT(5,DUPP,CAT,TCHAR,EMIT,ONEP);
  THEN(0);
  NEXT(2,DROP,EXITT);
  int CR=COLON("CR", DOLIT,'\n',DOLIT,'\r',EMIT,EMIT,EXITT);
  int DOSTR=COLON("do$", RFROM,RAT,RFROM,COUNT,PLUS,ALIGN,TOR,SWAP,TOR,EXITT);
  int STRQP=COLON("$\"|", DOSTR,EXITT);
  DOTQP=COLON(".\"|", DOSTR,COUNT,TYPES,EXITT);
  int DOTR=COLON(".R", TOR,STRR,RFROM,OVER,SUBBB,SPACS,TYPES,EXITT);
  int UDOTR=COLON("U.R", TOR,BDIGS,DIGS,EDIGS,RFROM,OVER,SUBBB,SPACS,TYPES,EXITT);
  int UDOT=COLON("U.", BDIGS,DIGS,EDIGS,SPACE,TYPES,EXITT);
  HEADER(".");
  int DOT=COLON_OLD(5,BASE,AT,DOLIT,10,XORR);
  IF(3,UDOT,EXITT);
  THEN(4,STRR,SPACE,TYPES,EXITT);
  int QUEST=COLON("?", AT,DOT,EXITT);
  HEADER("(parse)");
  int PARS=COLON_OLD(5,TEMP,CSTOR,OVER,TOR,DUPP);
  IF(5,ONEM,TEMP,CAT,BLANK,EQUAL);
  IF(0);
  FOR(6,BLANK,OVER,CAT,SUBBB,ZLESS,INVER);
  WHILE(1,ONEP);
  NEXT(6,RFROM,DROP,DOLIT,0,DUPP,EXITT);
  THEN(1,RFROM);
  THEN(2,OVER,SWAP);
  FOR(9,TEMP,CAT,OVER,CAT,SUBBB,TEMP,CAT,BLANK,EQUAL);
  IF(1,ZLESS);
  THEN(0);
  WHILE(1,ONEP);
  NEXT(2,DUPP,TOR);
  ELSE(5,RFROM,DROP,DUPP,ONEP,TOR);
  THEN(6,OVER,SUBBB,RFROM,RFROM,SUBBB,EXITT);
  THEN(4,OVER,RFROM,SUBBB,EXITT);
  int PACKS=COLON("PACK$", DUPP,TOR,DDUP,PLUS,DOLIT,~CELL_MASK,ANDD,DOLIT,0,SWAP,STORE,DDUP,CSTOR,ONEP,SWAP,CMOVEE,RFROM,EXITT);
  int PARSE=COLON("PARSE", TOR,TIB,INN,AT,PLUS,NTIB,AT,INN,AT,SUBBB,RFROM,PARS,INN,PSTOR,EXITT);
  int TOKEN=COLON("TOKEN", BLANK,PARSE,DOLIT,0x1F,MIN,HERE,CELLP,PACKS,EXITT);
  int WORDD=COLON("WORD", PARSE,HERE,CELLP,PACKS,EXITT);
  int NAMET=COLON("NAME>", COUNT,DOLIT,0x1F,ANDD,PLUS,ALIGN,EXITT);
  HEADER("SAME?");
  int SAMEQ=COLON_OLD(4,DOLIT,0x1F,ANDD,CELLD);
  FOR(0);
  AFT(14,OVER,RAT,CELLS,PLUS,AT,UPPER,OVER,RAT,CELLS,PLUS,AT,UPPER,SUBBB,QDUP);
  IF(3,RFROM,DROP,EXITT);
  THEN(0);
  THEN(0);
  NEXT(3,DOLIT,0,EXITT);
  HEADER("find");
  int FIND=COLON_OLD(10,SWAP,DUPP,AT,TEMP,STORE,DUPP,AT,TOR,CELLP,SWAP);
  BEGIN(2,AT,DUPP);
  IF(9,DUPP,AT,DOLIT,~0xC0,ANDD,UPPER,RAT,UPPER,XORR);
  IF(3,CELLP,DOLIT,-1);
  ELSE(4,CELLP,TEMP,AT,SAMEQ);
  THEN(0);
  ELSE(6,RFROM,DROP,SWAP,CELLM,SWAP,EXITT);
  THEN(0);
  WHILE(2,CELLM,CELLM);
  REPEAT(9,RFROM,DROP,SWAP,DROP,CELLM,DUPP,NAMET,SWAP,EXITT);
  int NAMEQ=COLON("NAME?", CNTXT,FIND,EXITT);
  int EXPEC=COLON("EXPECT", ACCEP,SPAN,STORE,DROP,EXITT);
  int QUERY=COLON("QUERY", TIB,DOLIT,0x100,ACCEP,NTIB,STORE,DROP,DOLIT,0,INN,STORE,EXITT);
  int ABORT=COLON("ABORT", NOP,TABRT,ATEXE,EXITT);
  HEADER("abort\"");
  ABORQP=COLON_OLD(0);
  IF(4,DOSTR,COUNT,TYPES,ABORT);
  THEN(3,DOSTR,DROP,EXITT);
  HEADER("ERROR");
  int ERRORR=COLON_OLD(8,SPACE,COUNT,TYPES,DOLIT,'?',EMIT,CR,ABORT);
  HEADER("$INTERPRET");
  int INTER=COLON_OLD(2,NAMEQ,QDUP);
  IF(4,CAT,DOLIT,COMPO,ANDD);
  ABORQ(" compile only");
  int INTER0=LABEL(2,EXECU,EXITT);
  THEN(1,NUMBQ);
  IF(1,EXITT);
  THEN(1,ERRORR);
  int LBRAC=COLON_IMMEDIATE("[", DOLIT,INTER,TEVAL,STORE,EXITT);
  HEADER(".OK");
  int DOTOK=COLON_OLD(6,CR,DOLIT,INTER,TEVAL,AT,EQUAL);
  IF(14,TOR,TOR,TOR,DUPP,DOT,RFROM,DUPP,DOT,RFROM,DUPP,DOT,RFROM,DUPP,DOT);
  DOTQ(" ok>");
  THEN(1,EXITT);
  HEADER("EVAL");
  int EVAL=COLON_OLD(1,LBRAC);
  BEGIN(3,TOKEN,DUPP,AT);
  WHILE(2,TEVAL,ATEXE);
  REPEAT(4,DROP,DOTOK,NOP,EXITT);
  HEADER("QUIT");
  int QUITT=COLON_OLD(1,LBRAC);
  BEGIN(2,QUERY,EVAL);
  AGAIN(0);
  int LOAD=COLON("LOAD", NTIB,STORE,TTIB,STORE,DOLIT,0,INN,STORE,EVAL,EXITT);
  int COMMA=COLON(",", HERE,DUPP,CELLP,CP,STORE,STORE,EXITT);
  int LITER=COLON_IMMEDIATE("LITERAL", DOLIT,DOLIT,COMMA,COMMA,EXITT);
  int ALLOT=COLON("ALLOT", ALIGN,CP,PSTOR,EXITT);
  int STRCQ=COLON("$,\"", DOLIT,'"',WORDD,COUNT,PLUS,ALIGN,CP,STORE,EXITT);
  HEADER("?UNIQUE");
  int UNIQU=COLON_OLD(3,DUPP,NAMEQ,QDUP);
  IF(6,COUNT,DOLIT,0x1F,ANDD,SPACE,TYPES);
  DOTQ(" reDef");
  THEN(2,DROP,EXITT);
  HEADER("$,n");
  int SNAME=COLON_OLD(2,DUPP,AT);
  IF(14,UNIQU,DUPP,NAMET,CP,STORE,DUPP,LAST,STORE,CELLM,CNTXT,AT,SWAP,STORE,EXITT);
  THEN(1,ERRORR);
  HEADER("'");
  int TICK=COLON_OLD(2,TOKEN,NAMEQ);
  IF(1,EXITT);
  THEN(1,ERRORR);
  int BCOMP=COLON_IMMEDIATE("[COMPILE]", TICK,COMMA,EXITT);
  int COMPI=COLON("COMPILE", RFROM,DUPP,AT,COMMA,CELLP,TOR,EXITT);
  HEADER("$COMPILE");
  int SCOMP=COLON_OLD(2,NAMEQ,QDUP);
  IF(4,AT,DOLIT,IMEDD,ANDD);
  IF(1,EXECU);
  ELSE(1,COMMA);
  THEN(1,EXITT);
  THEN(1,NUMBQ);
  IF(2,LITER,EXITT);
  THEN(1,ERRORR);
  int OVERT=COLON("OVERT", LAST,AT,CNTXT,STORE,EXITT);
  int RBRAC=COLON("]", DOLIT,SCOMP,TEVAL,STORE,EXITT);
  int COLN=COLON(":", TOKEN,SNAME,RBRAC,DOLIT,as_dolist,COMMA,EXITT);
  HEADER_IMMEDIATE(";");
  int SEMIS=COLON_OLD(6, DOLIT,EXITT,COMMA,LBRAC,OVERT,EXITT);
  HEADER("dm+");
  int DMP=COLON_OLD(4,OVER,DOLIT,6,UDOTR);
  FOR(0);
  AFT(6,DUPP,AT,DOLIT,9,UDOTR,CELLP);
  THEN(0);
  NEXT(1,EXITT);
  HEADER("DUMP");
  int DUMP=COLON_OLD(10,BASE,AT,TOR,HEXX,DOLIT,0x1F,PLUS,DOLIT,0x20,SLASH);
  FOR(0);
  AFT(10,CR,DOLIT,8,DDUP,DMP,TOR,SPACE,CELLS,TYPES,RFROM);
  THEN(0);
  NEXT(5,DROP,RFROM,BASE,STORE,EXITT);
  HEADER(">NAME");
  int TNAME=COLON_OLD(1,CNTXT);
  BEGIN(2,AT,DUPP);
  WHILE(3,DDUP,NAMET,XORR);
  IF(1,ONEM);
  ELSE(3,SWAP,DROP,EXITT);
  THEN(0);
  REPEAT(3,SWAP,DROP,EXITT);
  int DOTID=COLON(".ID", COUNT,DOLIT,0x1F,ANDD,TYPES,SPACE,EXITT);
  HEADER("WORDS");
  int WORDS=COLON_OLD(6,CR,CNTXT,DOLIT,0,TEMP,STORE);
  BEGIN(2,AT,QDUP);
  WHILE(9,DUPP,SPACE,DOTID,CELLM,TEMP,AT,DOLIT,0x10,LESS);
  IF(4,DOLIT,1,TEMP,PSTOR);
  ELSE(5,CR,DOLIT,0,TEMP,STORE);
  THEN(0);
  REPEAT(1,EXITT);
  HEADER("FORGET");
  int FORGT=COLON_OLD(3,TOKEN,NAMEQ,QDUP);
  IF(12,CELLM,DUPP,CP,STORE,AT,DUPP,CNTXT,STORE,LAST,STORE,DROP,EXITT);
  THEN(1,ERRORR);
  HEADER("COLD");
  int COLD=COLON_OLD(1,CR);
  DOTQ("esp32forth V6.3, 2019 ");
  int DOTQ1=LABEL(2,CR,EXITT);
  HEADER("LINE");
  int LINE=COLON_OLD(2,DOLIT,0x7);
  FOR(6,DUPP,PEEK,DOLIT,0x9,UDOTR,CELLP);
  NEXT(1,EXITT);
  HEADER("PP");
  int PP=COLON_OLD(0);
  FOR(0);
  AFT(7,CR,DUPP,DOLIT,0x9,UDOTR,SPACE,LINE);
  THEN(0);
  NEXT(1,EXITT);
  int P0=COLON("P0", DOLIT,0x3FF44004,POKE,EXITT);
  int P0S=COLON("P0S", DOLIT,0x3FF44008,POKE,EXITT);
  int P0C=COLON("P0C", DOLIT,0x3FF4400C,POKE,EXITT);
  int P1=COLON("P1", DOLIT,0x3FF44010,POKE,EXITT);
  int P1S=COLON("P1S", DOLIT,0x3FF44014,POKE,EXITT);
  int P1C=COLON("P1C", DOLIT,0x3FF44018,POKE,EXITT);
  int P0EN=COLON("P0EN", DOLIT,0x3FF44020,POKE,EXITT);
  int P0ENS=COLON("P0ENS", DOLIT,0x3FF44024,POKE,EXITT);
  int P0ENC=COLON("P0ENC", DOLIT,0x3FF44028,POKE,EXITT);
  int P1EN=COLON("P1EN", DOLIT,0x3FF4402C,POKE,EXITT);
  int P1ENS=COLON("P1ENS", DOLIT,0x3FF44030,POKE,EXITT);
  int P1ENC=COLON("P1ENC", DOLIT,0x3FF44034,POKE,EXITT);
  int P0IN=COLON("P0IN", DOLIT,0x3FF4403C,PEEK,DOT,EXITT);
  int P1IN=COLON("P1IN", DOLIT,0x3FF44040,PEEK,DOT,EXITT);
  int PPP=COLON("PPP", DOLIT,0x3FF44000,DOLIT,3,PP,DROP,EXITT);
  HEADER("EMITT");
  int EMITT=COLON_OLD(2,DOLIT,0x3);
  FOR(8,DOLIT,0,DOLIT,0x100,MSMOD,SWAP,TCHAR,EMIT);
  NEXT(2,DROP,EXITT);
  HEADER("TYPEE");
  int TYPEE=COLON_OLD(3,SPACE,DOLIT,0x7);
  FOR(4,DUPP,PEEK,EMITT,CELLP);
  NEXT(2,DROP,EXITT);
  HEADER("PPPP");
  int PPPP=COLON_OLD(0);
  FOR(0);
  AFT(10,CR,DUPP,DUPP,DOLIT,0x9,UDOTR,SPACE,LINE,SWAP,TYPEE);
  THEN(0);
  NEXT(1,EXITT);
  int KKK=COLON("KKK", DOLIT,0x3FF59000,DOLIT,0x10,PP,DROP,EXITT);
  int THENN=COLON_IMMEDIATE("THEN", HERE,SWAP,STORE,EXITT);
  int FORR=COLON_IMMEDIATE("FOR", COMPI,TOR,HERE,EXITT);
  int BEGIN=COLON_IMMEDIATE("BEGIN", HERE,EXITT);
  int NEXT=COLON_IMMEDIATE("NEXT", COMPI,DONXT,COMMA,EXITT);
  int UNTIL=COLON_IMMEDIATE("UNTIL", COMPI,QBRAN,COMMA,EXITT);
  int AGAIN=COLON_IMMEDIATE("AGAIN", COMPI,BRAN,COMMA,EXITT);
  int IFF=COLON_IMMEDIATE("IF", COMPI,QBRAN,HERE,DOLIT,0,COMMA,EXITT);
  int AHEAD=COLON_IMMEDIATE("AHEAD", COMPI,BRAN,HERE,DOLIT,0,COMMA,EXITT);
  int REPEA=COLON_IMMEDIATE("REPEAT", AGAIN,THENN,EXITT);
  int AFT=COLON_IMMEDIATE("AFT", DROP,AHEAD,HERE,SWAP,EXITT);
  int ELSEE=COLON_IMMEDIATE("ELSE", AHEAD,SWAP,THENN,EXITT);
  int WHILEE=COLON_IMMEDIATE("WHILE", IFF,SWAP,EXITT);
  int ABRTQ=COLON_IMMEDIATE("ABORT\"", DOLIT,ABORQP,HERE,STORE,STRCQ,EXITT);
  int STRQ=COLON_IMMEDIATE("$\"", DOLIT,STRQP,HERE,STORE,STRCQ,EXITT);
  int DOTQQ=COLON_IMMEDIATE(".\"", DOLIT,DOTQP,HERE,STORE,STRCQ,EXITT);
  int CODE=COLON("CODE", TOKEN,SNAME,OVERT,HERE,ALIGN,CP,STORE,EXITT);
  int CREAT=COLON("CREATE", CODE,DOLIT,as_dovar + (as_next << 8),COMMA,EXITT);
  int VARIA=COLON("VARIABLE", CREAT,DOLIT,0,COMMA,EXITT);
  int CONST=COLON("CONSTANT", CODE,DOLIT,as_docon + (as_next << 8),COMMA,COMMA,EXITT);
  int DOTPR=COLON_IMMEDIATE(".(", DOLIT,')',PARSE,TYPES,EXITT);
  int BKSLA=COLON_IMMEDIATE("\\", DOLIT,'\n',WORDD,DROP,EXITT);
  int PAREN=COLON_IMMEDIATE("(", DOLIT,')',PARSE,DDROP,EXITT);
  int ONLY=COLON_IMMEDIATE("COMPILE-ONLY", DOLIT,COMPO,LAST,AT,PSTOR,EXITT);
  int IMMED=COLON("IMMEDIATE", DOLIT,IMEDD,LAST,AT,PSTOR,EXITT);
  int ENDD=IP;
#if DEBUG_COREWORDS
  printf("\n");
  printf("IP=");
  printf("%" PRIxCELL, IP);
  printf(" R-stack= ");
  printf("%" PRIxCELL, popR*sizeof(cell_t));
#endif
  IP = 0x60 * sizeof(cell_t);
  int USER=LABEL(18,
                 as_dolist,EVAL,0,0,
                 0,  // HLD
                 0,  // SPAN
                 0,  // >IN
                 0,  // #TIB
                 0,  // 'TIB
                 0x10,  // BASE
                 IMMED-WithPadding(10), // CONTEXT
                 ENDD,  // CP
                 IMMED-WithPadding(10), // LAST
                 INTER,  // 'EVAL
                 EVAL,  // 'ABORT
                 0,  // tmp
                 0,  // ppqn
                 0);  // channel

#if DEBUG_COREWORDS
  // dump dictionary
  IP=0;
  for (len=0;len<0x48 * sizeof(cell_t);len++) {
    CheckSum();
  }
  printf("\n");
#endif

  setpin(13, 0);
  run();
}

