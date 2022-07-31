#pragma once

#include <functional>
#include <map>
#include <bitset>

#include "TMCBase.h"

class OutputStream;
class SPI;
class ConfigReader;
class Pin;
class GCode;

#define DTIME_400 0x00
#define DTIME_450 0x01
#define DTIME_650 0x02
#define DTIME_850 0x03

#define ISGAIN_5  	0x00
#define ISGAIN_10 	0x01
#define ISGAIN_20 	0x02
#define ISGAIN_40 	0x03

#define STEPS_1 	0x00
#define STEPS_2 	0x01
#define STEPS_4 	0x02
#define STEPS_8 	0x03
#define STEPS_16 	0x04
#define STEPS_32 	0x05
#define STEPS_64 	0x06
#define STEPS_128 	0x07
#define STEPS_256 	0x08

#define SIMPLTH_50 	0x00
#define SIMPLTH_100 0x01
#define SIMPLTH_200 0x02
#define SIMPLTH_300 0x03
#define SIMPLTH_400 0x04
#define SIMPLTH_600 0x05
#define SIMPLTH_800 0x06
#define SIMPLTH_1000 0x07

#define DECMOD_SLOW 	0x00
#define DECMOD_SLOWMIX  0x01
#define DECMOD_FAST 	0x02
#define DECMOD_MIXED 	0x03
#define DECMOD_SLOWAUTO 0x04
#define DECMOD_MIXAUTO 	0x05

#define VDIV_32		0x00
#define VDIV_16		0x01
#define VDIV_8		0x02
#define VDIV_4		0x03

#define SDCNT_1		0x00
#define SDCNT_2		0x01
#define SDCNT_4		0x02
#define SDCNT_8		0x03

#define IDRIVEP_50	0x00
#define IDRIVEP_100	0x01
#define IDRIVEP_150 0x02
#define IDRIVEP_200	0x03

#define IDRIVEN_100	0x00
#define IDRIVEN_200	0x01
#define IDRIVEN_300	0x02
#define IDRIVEN_400	0x03

#define TDRIVEP_250	0x00
#define TDRIVEP_500 0x01
#define TDRIVEP_1000 0x02
#define TDRIVEP_2000 0x03

#define TDRIVEN_250 0x00
#define TDRIVEN_500 0x01
#define TDRIVEN_1000 0x02
#define TDRIVEN_2000 0x03

#define OCPDEG_1	0x00
#define OCPDEG_2	0x01
#define OCPDEG_4	0x02
#define OCPDEG_8	0x03

#define OCPTH_250	0x00
#define OCPTH_500	0x01
#define OCPTH_750	0x02
#define OCPTH_1000	0x03

#define ON			0x01
#define OFF			0x00 

#define REGWRITE    0x0000
#define REGREAD     0x8000

// CTRL Register
struct CTRL_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int DTIME;		// bits 11-10
	unsigned int ISGAIN;	// bits 9-8
	unsigned int EXSTALL;	// bit 7
	unsigned int MODE;		// bits 6-3
	unsigned int RSTEP;		// bit 2
	unsigned int RDIR;		// bit 1
	unsigned int ENBL;		// bit 0
};

// TORQUE Register
struct TORQUE_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	unsigned int SIMPLTH;  	// bits 10-8
	unsigned int TORQUE;	// bits 7-0
};

// OFF Register
struct OFF_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	unsigned int PWMMODE;  	// bit 8
	unsigned int TOFF;		// bits 7-0
};

// BLANK Register
struct BLANK_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	unsigned int ABT;  		// bit 8
	unsigned int TBLANK;	// bits 7-0
};

// DECAY Register
struct DECAY_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	unsigned int DECMOD;  	// bits 10-8
	unsigned int TDECAY;	// bits 7-0
};

// STALL Register
struct STALL_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int VDIV;  	// bits 11-10
	unsigned int SDCNT;		// bits 9-8
	unsigned int SDTHR;		// bits 7-0
};

// DRIVE Register
struct DRIVE_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int IDRIVEP;  	// bits 11-10
	unsigned int IDRIVEN;	// bits 9-8
	unsigned int TDRIVEP;	// bits 7-6
	unsigned int TDRIVEN;	// bits 5-4
	unsigned int OCPDEG;	// bits 3-2
	unsigned int OCPTH;		// bits 1-0
};

// STATUS Register
struct STATUS_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */			// bits 11-8
	unsigned int STDLAT;  	// bit 7
	unsigned int STD;		// bit 6
	unsigned int UVLO;		// bit 5
	unsigned int BPDF;		// bit 4
	unsigned int APDF;		// bit 3
	unsigned int BOCP;		// bit 2
	unsigned int AOCP;		// bit 1
	unsigned int OTS;		// bit 0
};

class DRV8711 : public TMCBase
{
 public:
  DRV8711 (char designator);

  bool SavedStart; 
  bool ErrorFlag;
  
  virtual void init();
  virtual void setMicrosteps(int number_of_steps);
  virtual int getMicrosteps(void);
  virtual void setEnabled(bool enabled);
  virtual bool isEnabled();
  virtual void setCurrent(unsigned int current);
  virtual bool set_raw_register(OutputStream& stream, uint32_t reg, uint32_t val);
  virtual bool check_errors();

  virtual bool config(ConfigReader& cr, const char *actuator_name);
  virtual void dump_status(OutputStream& stream, bool readable= true);
  virtual bool set_options(const GCode& gcode);

private:
	// SPI sender
    unsigned int SPI_ReadWrite(unsigned int datagram);
    bool sendSPI(void *b, void *r);
    bool SPI_VerifiedWrite(unsigned int sendData);

    // one set of common settings
    static bool common_setup;

    // one instance of SPI is shared
    static uint8_t spi_channel;
    static SPI *spi;
    static Pin *reset_pin;

    Pin *spi_cs;
    std::string name;
    char designator;
	bool started;
    
	// used to debounce the errors
    std::bitset<8> error_detected;
    std::bitset<8> error_reported;
    static uint32_t max_current;
    //the driver status result
    unsigned long driver_status_result;

    struct CTRL_Register 	G_CTRL_REG;
    struct TORQUE_Register 	G_TORQUE_REG;
    struct OFF_Register 	G_OFF_REG;
    struct BLANK_Register	G_BLANK_REG;
    struct DECAY_Register 	G_DECAY_REG;
    struct STALL_Register 	G_STALL_REG;
    struct DRIVE_Register 	G_DRIVE_REG;
    struct STATUS_Register 	G_STATUS_REG;
  
  unsigned int resistor{75}; // current sense resitor value in milliohm

  void enable ();  
  void disable ();
  void get_status ();
  void clear_status ();
  void clear_error ();
  void set_defaults ();
  
  /* Dummies for Compatibility - Need to be removed/refactored */
  unsigned int getCurrent(void);
  void readStatus(int8_t read_value);
  
  void ReadAllRegisters () ;
  void ReadCTRLRegister();
  void ReadTORQUERegister();	
  void ReadOFFRegister();
  void ReadBLANKRegister();
  void ReadDECAYRegister();
  void ReadSTALLRegister();
  void ReadDRIVERegister();
  void ReadSTATUSRegister();
  
  void WriteAllRegisters () ;
  void WriteCTRLRegister();
  void WriteTORQUERegister();	
  void WriteOFFRegister();
  void WriteBLANKRegister();
  void WriteDECAYRegister();
  void WriteSTALLRegister();
  void WriteDRIVERegister();
  void WriteSTATUSRegister();
};