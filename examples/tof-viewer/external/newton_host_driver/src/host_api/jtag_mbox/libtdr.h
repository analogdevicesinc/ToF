
#ifndef __TDR_INTERFACE_INC__
#define __TDR_INTERFACE_INC__

#define VERBOSE0      0
#define VERBOSE1      1
#define VERBOSE2      2
#define VERBOSE3      3
#define VERBOSE4      4

#ifdef __cplusplus
extern "C" {
#endif

//===================================
// Prototypes
//===================================
int openJtag(uint16_t vid, uint16_t pid, const char * vendor, uint32_t verbosity=VERBOSE0);
void addTap(uint32_t ir, const char * name);
void tmsReset();
void setFrequency(uint32_t freq);

int readIDCode();
int setTdrTap(uint32_t tap, uint32_t verbosity=VERBOSE0);
void wrJtagMbox0(uint32_t mbx0);
void wrJtagMbox1(uint32_t mbx1);
uint32_t rdJtagMbox0();
uint32_t rdJtagMbox1();
void rdSOCIdData(uint32_t *soc);
void rdSTS(uint32_t *status);
uint32_t rdSPStat();
uint32_t rdMemStat();
uint32_t rdWdtStat();
uint32_t rdCryptoErr();
void rdSS(uint32_t *ss);
void rdLSMStatus(uint32_t *status);
void enRetestChange();
void disRetestChange();
uint64_t rdSPRomBist();
void rdAEBBist(uint32_t *bist);
void setSPReset();
void setHSPReset();
void releaseReset();
void tapDisable(uint8_t tap);
void tapEnable(uint8_t tap);

#ifdef __cplusplus
};
#endif
#endif
