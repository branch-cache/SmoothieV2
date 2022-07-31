#ifdef DRIVER_TMC
#include "DRV8711.h"
#include "main.h"
#include "Consoles.h"
#include "OutputStream.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "Spi.h"
#include "Pin.h"
#include "ConfigReader.h"
#include "StringUtils.h"
#include "GCode.h"
#include "Lock.h"

#include <cmath>
#include <iostream>


// config keys
#define spi_cs_pin_key                  "spi_cs_pin"
#define resistor_key                    "sense_resistor"
#define max_current_key                 "max_current"
#define raw_register_key                "reg"
#define step_interpolation_key          "step_interpolation"
#define passive_fast_decay_key          "passive_fast_decay"
#define reset_pin_key                   "reset_pin"
#define spi_channel_key                 "spi_channel"

#ifdef BOARD_DEVEBOX
constexpr static int def_spi_channel= 0;
#elif BOARD_IKOSYBOT
constexpr static int def_spi_channel= 0;
#else
constexpr static int def_spi_channel= 1;
#endif

//statics common to all instances
SPI *DRV8711::spi = nullptr;
uint8_t DRV8711::spi_channel= def_spi_channel;
bool DRV8711::common_setup = false;
uint32_t DRV8711::max_current = 3000; // 3 amps
Pin *DRV8711::reset_pin = nullptr;

#ifdef BOARD_PRIME
// setup default SPI CS pin for Prime
static std::map<std::string, const char*> def_cs_pins = {
    {"alpha", "PJ13"},
    {"beta", "PG8"},
    {"gamma", "PG7"},
    {"delta", "PG6"}
};
#endif

#ifdef BOARD_IKOSYBOT
// setup default SPI CS pin for Prime
static std::map<std::string, const char*> def_cs_pins = {
    {"alpha", "PD7"},
    {"beta", "PD1"}
};
#endif

/*
 * Constructor
 */
DRV8711::DRV8711(char d) : designator(d)
{
    //we are not started yet
    started = false;

    error_reported.reset();
    error_detected.reset();

    // setting the default register values
	set_defaults();
}

bool DRV8711::config(ConfigReader& cr, const char *actuator_name)
{
    name = actuator_name;
	ConfigReader::sub_section_map_t ssm;
    if(!cr.get_sub_sections("drv8711", ssm)) {
        printf("ERROR:config_drv8711: no drv8711 section found\n");
        return false;
    }

    auto s = ssm.find(actuator_name);
    if(s == ssm.end()) {
        printf("ERROR:config_drv8711: %s - no drv8711 entry found\n", actuator_name);
        return false;
    }

    auto& mm = s->second; // map of drv8711 config values for this actuator
	const char *def_cs= "nc";
	#ifdef BOARD_IKOSYBOT
    {
        auto dcss= def_cs_pins.find(actuator_name);
        if(dcss != def_cs_pins.end()) {
            def_cs= dcss->second;
        }
    }
#endif

	// Note CS pins go low to select the chip, we do not invert the pin to avoid confusion here
    std::string cs_pin = cr.get_string(mm, spi_cs_pin_key, def_cs);
    spi_cs = new Pin(cs_pin.c_str(), Pin::AS_OUTPUT_ON); // set high on creation
    if(!spi_cs->connected()) {
        delete spi_cs;
        printf("ERROR:config_drv8711: %s - spi cs pin %s is invalid\n", actuator_name, cs_pin.c_str());
        return false;
    }
    spi_cs->set(true);
    printf("DEBUG:configure-drv8711: %s - spi cs pin: %s\n", actuator_name, spi_cs->to_string().c_str());

    // setup singleton spi instance
    if(spi == nullptr) {
        bool ok = false;
        spi = SPI::getInstance(spi_channel);
        if(spi != nullptr) {
            if(spi->init(8, 0, 500000)) { // 8bit, mode0, 500KHz
                ok = true;
            }
        }
        if(!ok) {
            printf("ERROR: DRV8711 failed to get SPI channel %d\n", spi_channel);
            return false;
        }
    }
    printf("DEBUG:configure-drc=v8711: %s - sense resistor: %d milliohms\n", actuator_name, resistor);

#if 0
    // if raw registers are defined set them 1,2,3 etc in hex
    std::string str = cr.get_string(mm, raw_register_key, "");
    if(!str.empty()) {
        std::vector<uint32_t> regs = stringutils::parse_number_list(str.c_str(), 16);
        if(regs.size() == 5) {
            uint32_t reg = 0;
            OutputStream os(&std::cout);
            for(auto i : regs) {
                // this just sets the local storage, it does not write to the chip
                set_raw_register(os, ++reg, i);
            }
        }
    }
#endif
    return true;
}

/*
 * configure the stepper driver
 * must be called after Vbb is applied
 */
void DRV8711::init()
{
    // set the saved values
 	set_defaults();
    ReadAllRegisters();
    started = true;
}

// Current is passed in as milliamps
void DRV8711::setCurrent(unsigned int current)
{
  return;
}

unsigned int DRV8711::getCurrent(void)
{
	double result = 0.0;
    return (unsigned int)round(result);
}

/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void DRV8711::setMicrosteps(int number_of_steps)
{

}

/*
 * returns the effective number of microsteps at the moment
 */
int DRV8711::getMicrosteps(void)
{
    return 128;
}

void DRV8711::setEnabled(bool enabled)
{
	// Set Enable
	G_CTRL_REG.ENBL = enabled;  //enable/disable motor
	WriteCTRLRegister();
}

bool DRV8711::isEnabled()
{
	return (G_CTRL_REG.ENBL == ON);
}

/*
 * reads a value from the DRV8711 status register. The value is not obtained directly but can then
 * be read by the various status routines.
 *
 */
void DRV8711::readStatus(int8_t read_value)
{
	ReadSTATUSRegister();
}

void DRV8711::set_defaults ()
{
 	// CTRL Register
	G_CTRL_REG.Address 	= 0x00;
	G_CTRL_REG.DTIME 	= DTIME_850; 	//Dead Time in ns
	G_CTRL_REG.ISGAIN   = ISGAIN_20;	//ISENSE amp gain 
	G_CTRL_REG.EXSTALL 	= OFF;			//External Stall Detect
	G_CTRL_REG.MODE     = STEPS_32;		//Microstepping Mode
	G_CTRL_REG.RSTEP 	= OFF;			//Step 
	G_CTRL_REG.RDIR 	= OFF;			//Reverse Direction Input
	G_CTRL_REG.ENBL 	= OFF;			//Enable Motor
	
	// TORQUE Register
	G_TORQUE_REG.Address = 0x01;
	G_TORQUE_REG.SIMPLTH = SIMPLTH_100;	//Back EMF Sample Threshold uS
	G_TORQUE_REG.TORQUE  = 128;			//Torque Value 0..255
        
	// OFF Register
	G_OFF_REG.Address 	= 0x02;
	G_OFF_REG.PWMMODE 	= OFF;			//PWM Direct Mode - OFF for Indexed Mode 
	G_OFF_REG.TOFF 		= 47;			//Off Time 0..255 in 500ns inc.

	// BLANK Register
	G_BLANK_REG.Address = 0x03;			
	G_BLANK_REG.ABT 	= OFF;			//Enable Adaptive Blanking Time
	G_BLANK_REG.TBLANK 	= 125;			//Blanking Time 0..255 in 20ns inc ( 1us min )

	// DECAY Register.
	G_DECAY_REG.Address = 0x04;
	G_DECAY_REG.DECMOD  = DECMOD_MIXAUTO;	//Decay Mode
	G_DECAY_REG.TDECAY 	= 15;				//Decay Time 0..255 * 500ns

	// STALL Register
	G_STALL_REG.Address = 0x05;
	G_STALL_REG.VDIV 	= VDIV_4;		//Back EMF Div factor
	G_STALL_REG.SDCNT 	= SDCNT_8;		//Stall Step Count 
	G_STALL_REG.SDTHR 	= 0 ;			//Stall Detection Threshold 0..255

	// DRIVE Register
	G_DRIVE_REG.Address = 0x06;
	G_DRIVE_REG.IDRIVEP = IDRIVEP_50;	//High Side Gate Current mA (Source)
	G_DRIVE_REG.IDRIVEN = IDRIVEN_100;	//Low Side Gate Current mA (Sink)
	G_DRIVE_REG.TDRIVEP = TDRIVEP_500;	//High Side Gate Drive Time ns 
	G_DRIVE_REG.TDRIVEN = TDRIVEN_500;	//Low Side Gate Drive Time ns 
	G_DRIVE_REG.OCPDEG  = OCPDEG_2;		//OCP Deglitch time uS  
	G_DRIVE_REG.OCPTH   = OCPTH_500;	//OCP Threshold mV

	// STATUS Register
	G_STATUS_REG.Address = 0x07;
	G_STATUS_REG.STDLAT  = OFF;			//Latched Stall Detect
	G_STATUS_REG.STD     = OFF;  		//Stall Detected 
	G_STATUS_REG.UVLO    = OFF;  		//UnderVoltage Lockout  
	G_STATUS_REG.BPDF    = OFF; 	 	//Channel B Predriver Fault 
	G_STATUS_REG.APDF    = OFF;			//Channel A Predriver Fault 
	G_STATUS_REG.BOCP    = OFF;			//Channel B OverCurrent 
	G_STATUS_REG.AOCP    = OFF;			//Channel A OverCurrent 
	G_STATUS_REG.OTS     = OFF;			//Over Temperature 
}

void DRV8711::clear_status ()
{
	G_STATUS_REG.STDLAT  = OFF;
	G_STATUS_REG.STD     = OFF;
	G_STATUS_REG.UVLO    = OFF;
	G_STATUS_REG.BPDF    = OFF;
	G_STATUS_REG.APDF    = OFF;
	G_STATUS_REG.BOCP    = OFF;
	G_STATUS_REG.AOCP    = OFF;
	G_STATUS_REG.OTS     = OFF;

	WriteSTATUSRegister();
}

void DRV8711::clear_error()
{
	ErrorFlag = false;
}

void DRV8711::ReadCTRLRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read CTRL Register
    sendData = REGREAD | (G_CTRL_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);
}

void DRV8711::ReadTORQUERegister()
{
	unsigned int sendData = 0;
    unsigned int readData = 0;

	// Read TORQUE Register
    sendData = REGREAD | (G_TORQUE_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);
}

void DRV8711::ReadOFFRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read OFF Register
    sendData = REGREAD | (G_OFF_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);
}

void DRV8711::ReadBLANKRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read BLANK Register
    sendData = REGREAD | (G_BLANK_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);
}

void DRV8711::ReadDECAYRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read DECAY Register
    sendData = REGREAD | (G_DECAY_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);
}

void DRV8711::ReadSTALLRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read STALL Register
    sendData = REGREAD | (G_STALL_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);
}

void DRV8711::ReadDRIVERegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read DRIVE Register
    sendData = REGREAD | (G_DRIVE_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);
}

void DRV8711::ReadSTATUSRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read STATUS Register
    sendData = REGREAD | (G_STATUS_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void DRV8711::ReadAllRegisters()
{
	ReadCTRLRegister();
	ReadTORQUERegister();	
	ReadOFFRegister();
	ReadBLANKRegister();
	ReadDECAYRegister();
	ReadSTALLRegister();
	ReadDRIVERegister();
	ReadSTATUSRegister();

	printf("DTIME:%x\tISGAIN:%x\tEXSTALL:%x\n",  G_CTRL_REG.DTIME, G_CTRL_REG.ISGAIN, G_CTRL_REG.EXSTALL);
	printf("MODE:%x\tRSTEP:%d\tDIR:%x\tENBL:%d\n\n",G_CTRL_REG.MODE, G_CTRL_REG.RSTEP, G_CTRL_REG.RDIR, G_CTRL_REG.ENBL);

	printf("SIMPLTH:%x\t,TORUE:%x\n\n", G_TORQUE_REG.SIMPLTH, G_TORQUE_REG.TORQUE);
 	printf("PWMMODE:%x\tTOFF:%x\n",G_OFF_REG.PWMMODE, G_OFF_REG.TOFF);

}

void DRV8711::WriteCTRLRegister()
{
    unsigned int sendData = 0;
    
	// Write CTRL Register
    sendData = REGWRITE | (G_CTRL_REG.Address << 12) | (G_CTRL_REG.DTIME << 10) | (G_CTRL_REG.ISGAIN << 8);
    sendData |= (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
	printf("Writing CTRL Reg\n");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to CTRL Register Failed\n");
	    ErrorFlag = true;
	};
}

void DRV8711::WriteTORQUERegister()
{
    unsigned int sendData = 0;
    
    // Write TORQUE Register
    sendData = REGWRITE | (G_TORQUE_REG.Address << 12) | (G_TORQUE_REG.SIMPLTH << 8);
    sendData |= G_TORQUE_REG.TORQUE;
	printf("Writing TORQUE Reg");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to TORQUE Register Failed");
        ErrorFlag = true;
	};
}	

void DRV8711::WriteOFFRegister()
{
    unsigned int sendData = 0;
    
    // Write OFF Register
    sendData = REGWRITE | (G_OFF_REG.Address << 12) | (G_OFF_REG.PWMMODE << 8);
    sendData |= G_OFF_REG.TOFF;
	printf("Writing OFF Reg");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to OFF Register Failed");
	    ErrorFlag = true;
	};
}

void DRV8711::WriteBLANKRegister()
{
    unsigned int sendData = 0;
    
    // Write BLANK Register
    sendData = REGWRITE | (G_BLANK_REG.Address << 12) | (G_BLANK_REG.ABT << 8);
    sendData |= G_BLANK_REG.TBLANK;
	printf("Writing BLANK Reg");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to BLANK Register Failed");
	    ErrorFlag = true;
	};
}

void DRV8711::WriteDECAYRegister()
{
    unsigned int sendData = 0;
    
    // Write DECAY Register
    sendData = REGWRITE | (G_DECAY_REG.Address << 12) | (G_DECAY_REG.DECMOD << 8);
    sendData |= G_DECAY_REG.TDECAY;
	printf("Writing DECAY Reg");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to DECAY Register Failed");
	    ErrorFlag = true;
	};
}

void DRV8711::WriteSTALLRegister()
{
    unsigned int sendData = 0;
    
    // Write STALL Register
    sendData = REGWRITE | (G_STALL_REG.Address << 12) | (G_STALL_REG.VDIV << 10) | (G_STALL_REG.SDCNT << 8);
    sendData |= G_STALL_REG.SDTHR;
	printf("Writing STALL Reg");
	if (!SPI_VerifiedWrite(sendData)){
	   printf("Write to STALL Register Failed");
	   ErrorFlag = true;
	};
}

void DRV8711::WriteDRIVERegister()
{
    unsigned int sendData = 0;
    
    // Write DRIVE Register
    sendData = REGWRITE | (G_DRIVE_REG.Address << 12) | (G_DRIVE_REG.IDRIVEP << 10) | (G_DRIVE_REG.IDRIVEN << 8);
    sendData |= (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
	printf("Writing DRIVE Reg");
	if (!SPI_VerifiedWrite(sendData)){
	    printf("Write to DRIVE Register Failed");
	    ErrorFlag = true;
	};
}

void DRV8711::WriteSTATUSRegister()
{
    unsigned int sendData = 0;
    
    // Write STATUS Register
    sendData = REGWRITE | (G_STATUS_REG.Address << 12) ;
    sendData |= (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
	printf("Writing STATUS Reg");
	SPI_ReadWrite(sendData);
}

void DRV8711::WriteAllRegisters()
{
	WriteCTRLRegister();
	WriteTORQUERegister();	
	WriteOFFRegister();
	WriteBLANKRegister();
	WriteDECAYRegister();
	WriteSTALLRegister();
	WriteDRIVERegister();
	WriteSTATUSRegister();
}

void DRV8711::dump_status(OutputStream& stream, bool readable)
{
    stream.printf("designator %c, actuator %s, Chip type DRV8711\n", designator, name.c_str());

}

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
// FIXME status registers not reading back correctly, check docs
bool DRV8711::set_raw_register(OutputStream& stream, uint32_t reg, uint32_t val)
{
    return true;
}


bool DRV8711::SPI_VerifiedWrite(unsigned int sendData)
{
    unsigned int readData = 0;
	int attempts = 0;
    const int maxtries = 10;	
	bool success = false;
	
	do {
	  attempts++;
	  // Write
      SPI_ReadWrite(sendData);
      // Readback
      readData = SPI_ReadWrite(REGREAD | sendData);
      // Compare
	  if ((readData << 4) == (sendData << 4)) {
		  success = true;
	    } else {
		    printf("SPI Write Error, attempt:%d\n",attempts);
		  //delayMicroseconds(attempts); // delay before trying again
		}
	} while ( (success == false) && (attempts < maxtries) ) ;
	return success;
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 * sends 20bits, the last 20 bits of the 24bits is taken as the command
 */
unsigned int DRV8711::SPI_ReadWrite(unsigned int datagram)
{
    uint8_t txbuf[] {(uint8_t)(datagram >>  8), (uint8_t)(datagram & 0xff)};
    uint8_t rxbuf[2] {0};
	uint32_t i_datagram = -1;
    // write/read the values
    if(sendSPI(txbuf, rxbuf)) {
        //store the datagram as status result
        i_datagram = ((rxbuf[0] << 8) | (rxbuf[1])) >> 4;
        driver_status_result = i_datagram;
    }else{
        printf("ERROR: SPI_ReadWrite failed\n");
    }
	return(i_datagram);
}

// Called by the drivers codes to send and receive SPI data to/from the chip
bool DRV8711::sendSPI(void *b, void *r)
{
    // lock the SPI bus for this transaction
    if(!spi->begin_transaction()) return false;
    spi_cs->set(false); // enable chip select
    bool stat= spi->write_read(b, r, 2);
    spi_cs->set(true); // disable chip select
    spi->end_transaction();
    return stat;
}

#define HAS(X) (gcode.has_arg(X))
#define GET(X) (gcode.get_int_arg(X))
bool DRV8711::set_options(const GCode& gcode)
{
    bool set = false;
    return set;
}

bool DRV8711::check_errors()
{
	return false;
}
#endif
