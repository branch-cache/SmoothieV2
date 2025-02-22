#ifdef DRIVER_TMC
#include "TMC2590.h"
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

//! return value for TMC2590.getOverTemperature() if there is a overtemperature situation in the TMC chip
/*!
 * This warning indicates that the TCM chip is too warm.
 * It is still working but some parameters may be inferior.
 * You should do something against it.
 */
#define TMC2590_OVERTEMPERATURE_PREWARING 1
//! return value for TMC2590.getOverTemperature() if there is a overtemperature shutdown in the TMC chip
/*!
 * This warning indicates that the TCM chip is too warm to operate and has shut down to prevent damage.
 * It will stop working until it cools down again.
 * If you encouter this situation you must do something against it. Like reducing the current or improving the PCB layout
 * and/or heat management.
 */
#define TMC2590_OVERTEMPERATURE_SHUTDOWN 2

//which values can be read out
/*!
 * Selects to readout the microstep position from the motor.
 *\sa readStatus()
 */
#define TMC2590_READOUT_POSITION 0
/*!
 * Selects to read out the StallGuard value of the motor.
 *\sa readStatus()
 */
#define TMC2590_READOUT_STALLGUARD 1
/*!
 * Selects to read out the current current setting (acc. to CoolStep) and the upper bits of the StallGuard value from the motor.
 *\sa readStatus(), setCurrent()
 */
#define TMC2590_READOUT_CURRENT 3
/*!
 * Selects to read out all the flags.
 */
#define TMC2590_READOUT_ALL_FLAGS 4

/*!
 * Define to set the minimum current for CoolStep operation to 1/2 of the selected CS minium.
 *\sa setCoolStepConfiguration()
 */
#define COOL_STEP_HALF_CS_LIMIT 0
/*!
 * Define to set the minimum current for CoolStep operation to 1/4 of the selected CS minium.
 *\sa setCoolStepConfiguration()
 */
#define COOL_STEP_QUARTDER_CS_LIMIT 1


//some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE 32

//TMC2590 register definitions
#define DRIVER_CONTROL_REGISTER            0x00000ul
#define CHOPPER_CONFIG_REGISTER            0x80000ul
#define COOL_STEP_REGISTER                 0xA0000ul
#define STALL_GUARD2_LOAD_MEASURE_REGISTER 0xC0000ul
#define DRIVER_CONFIG_REGISTER             0xE0000ul

#define REGISTER_BIT_PATTERN               0xFFFFFul

//definitions for the driver control register DRVCTL
#define MICROSTEPPING_PATTERN          0x000Ful
#define STEP_INTERPOLATION             0x0200ul
#define DOUBLE_EDGE_STEP               0x0100ul

//definitions for the driver config register DRVCONF
#define SLPH                           0xC000ul
#define SLPL                           0x3000ul
#define SLPHL2                         0x0800ul
#define DIS_S2G                        0x0400ul
#define TS2G                           0x0300ul
#define VSENSE                         0x0040ul
#define READ_MICROSTEP_POSITION        0x0000ul
#define READ_STALL_GUARD_READING       0x0010ul
#define READ_STALL_GUARD_AND_COOL_STEP 0x0020ul
#define READ_ALL_FLAGS                 0x0030ul
#define READ_SELECTION_PATTERN         0x0030ul
#define OTSENS                         0x0008ul
#define SHRTSENS                       0x0004ul
#define EN_PFD                         0x0002ul
#define EN_S2VS                        0x0001ul

//definitions for the chopper config register
#define CHOPPER_MODE_STANDARD          0x00000ul
#define CHOPPER_MODE_T_OFF_FAST_DECAY  0x04000ul
#define T_OFF_PATTERN                  0x0000ful
#define RANDOM_TOFF_TIME               0x02000ul
#define BLANK_TIMING_PATTERN           0x18000ul
#define BLANK_TIMING_SHIFT             15
#define HYSTERESIS_DECREMENT_PATTERN   0x01800ul
#define HYSTERESIS_DECREMENT_SHIFT     11
#define HYSTERESIS_LOW_VALUE_PATTERN   0x00780ul
#define HYSTERESIS_LOW_SHIFT           7
#define HYSTERESIS_START_VALUE_PATTERN 0x00070ul
#define HYSTERESIS_START_VALUE_SHIFT   4
#define T_OFF_TIMING_PATERN            0x0000Ful

//definitions for cool step register
#define MINIMUM_CURRENT_FOURTH          0x8000ul
#define CURRENT_DOWN_STEP_SPEED_PATTERN 0x6000ul
#define SE_MAX_PATTERN                  0x0F00ul
#define SE_CURRENT_STEP_WIDTH_PATTERN   0x0060ul
#define SE_MIN_PATTERN                  0x000Ful

//definitions for stall guard2 current register
#define STALL_GUARD_FILTER_ENABLED          0x10000ul
#define STALL_GUARD_THRESHHOLD_VALUE_PATTERN 0x07F00ul
#define CURRENT_SCALING_PATTERN             0x0001Ful
#define STALL_GUARD_CONFIG_PATTERN          0x07F00ul
#define STALL_GUARD_VALUE_PATTERN           0x07F00ul

//definitions for the input from the TCM2590
#define STATUS_STALL_GUARD_STATUS        0x00001ul
#define STATUS_OVER_TEMPERATURE_SHUTDOWN 0x00002ul
#define STATUS_OVER_TEMPERATURE_WARNING  0x00004ul
#define STATUS_SHORT_TO_GROUND_A         0x00008ul
#define STATUS_SHORT_TO_GROUND_B         0x00010ul
#define STATUS_OPEN_LOAD_A               0x00020ul
#define STATUS_OPEN_LOAD_B               0x00040ul
#define STATUS_STAND_STILL               0x00080ul
#define READOUT_VALUE_PATTERN            0xFFC00ul

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
#else
constexpr static int def_spi_channel= 1;
#endif

//statics common to all instances
SPI *TMC2590::spi = nullptr;
uint8_t TMC2590::spi_channel= def_spi_channel;
bool TMC2590::common_setup = false;
uint32_t TMC2590::max_current = 3000; // 3 amps
Pin *TMC2590::reset_pin = nullptr;

#ifdef BOARD_PRIME
// setup default SPI CS pin for Prime
static std::map<std::string, const char*> def_cs_pins = {
    {"alpha", "PJ13"},
    {"beta", "PG8"},
    {"gamma", "PG7"},
    {"delta", "PG6"}
};
#endif

/*
 * Constructor
 */
TMC2590::TMC2590(char d) : designator(d)
{
    //we are not started yet
    started = false;
    //by default cool step is not enabled
    cool_step_enabled = false;
    error_reported.reset();
    error_detected.reset();

    // setting the default register values
    driver_control_register_value = DRIVER_CONTROL_REGISTER;
    chopper_config_register_value = CHOPPER_CONFIG_REGISTER;
    cool_step_register_value = COOL_STEP_REGISTER;
    stall_guard2_current_register_value = STALL_GUARD2_LOAD_MEASURE_REGISTER;
    driver_configuration_register_value = DRIVER_CONFIG_REGISTER | READ_STALL_GUARD_READING;
}

bool TMC2590::config(ConfigReader& cr, const char *actuator_name)
{
    name = actuator_name;
    ConfigReader::sub_section_map_t ssm;
    if(!cr.get_sub_sections("tmc2590", ssm)) {
        printf("ERROR:config_tmc2590: no tmc2590 section found\n");
        return false;
    }

    auto s = ssm.find(actuator_name);
    if(s == ssm.end()) {
        printf("ERROR:config_tmc2590: %s - no tmc2590 entry found\n", actuator_name);
        return false;
    }

    auto& mm = s->second; // map of tmc2590 config values for this actuator
    const char *def_cs= "nc";
#ifdef BOARD_PRIME
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
        printf("ERROR:config_tmc2590: %s - spi cs pin %s is invalid\n", actuator_name, cs_pin.c_str());
        return false;
    }
    spi_cs->set(true);
    printf("DEBUG:configure-tmc2590: %s - spi cs pin: %s\n", actuator_name, spi_cs->to_string().c_str());

    // if this is the first instance then get any common settings
    if(!common_setup) {
        auto c = ssm.find("common");
        if(c != ssm.end()) {
            auto& cm = c->second; // map of common tmc2590 config values
            spi_channel = cr.get_int(cm, spi_channel_key, def_spi_channel);
            max_current = cr.get_int(cm, max_current_key, this->resistor == 75 ? 3100 : 4600);
            if(reset_pin == nullptr) {
                reset_pin= new Pin(cr.get_string(cm, reset_pin_key, "nc"), Pin::AS_OUTPUT_OFF); // sets low
                if(reset_pin->connected()) {
                    printf("DEBUG:configure-tmc2590: reset pin set to: %s\n", reset_pin->to_string().c_str());
                    reset_pin->set(true); // sets high turns on all drivers
                }else{
                    delete reset_pin;
                    reset_pin= nullptr;
                }
            }
        }
        common_setup = true;
    }

    // setup singleton spi instance
    if(spi == nullptr) {
        bool ok = false;
        spi = SPI::getInstance(spi_channel);
        if(spi != nullptr) {
            if(spi->init(8, 3, 500000)) { // 8bit, mode3, 500KHz
                ok = true;
            }
        }
        if(!ok) {
            printf("ERROR: TMC2590 failed to get SPI channel %d\n", spi_channel);
            return false;
        }
    }

    // setup default values
#if 0
    // setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
    //set to a conservative start value
    setConstantOffTimeChopper(7, 54, 13, 12, 1);
#else
    //void TMC2590::setSpreadCycleChopper( constant_off_time,  blank_time,  hysteresis_start,  hysteresis_end,  hysteresis_decrement);

    // openbuilds high torque nema23 3amps (2.8)
    //setSpreadCycleChopper(5, 36, 6, 0, 0);

    // for 1.5amp kysan @ 12v
    setSpreadCycleChopper(5, 54, 5, 0, 0);
    //setSpreadCycleChopper(4, 36, 4, 0, 0);

    // for 4amp Nema24 @ 12v
    //setSpreadCycleChopper(5, 54, 4, 0, 0);

    // set medium gate driver strength high= 3, low= 3 (0x0F000)
    driver_configuration_register_value |= (SLPH | SLPL);
    driver_configuration_register_value &= ~(SLPHL2);
#endif

    setEnabled(false);

    // set a nice microstepping value
    setMicrosteps(DEFAULT_MICROSTEPPING_VALUE);

    // set stallguard to a conservative value so it doesn't trigger immediately
    setStallGuardThreshold(10, 1);

    // get rest of instance specific configs
    this->resistor = cr.get_int(mm, resistor_key, 75); // in milliohms
    printf("DEBUG:configure-tmc2590: %s - sense resistor: %d milliohms\n", actuator_name, resistor);

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

    // the following override the raw register settings if set
    // see if we want step interpolation
    bool interpolation = cr.get_bool(mm, step_interpolation_key, false);
    if(interpolation) {
        setStepInterpolation(1);
        printf("DEBUG:configure-tmc2590: %s - step interpolation is on\n", actuator_name);
    }

    bool pfd = cr.get_bool(mm, passive_fast_decay_key, true);
    if(pfd) {
        setPassiveFastDecay(1);
        printf("DEBUG:configure-tmc2590: %s - passive fast decay is on\n", actuator_name);
    }

    return true;
}

/*
 * configure the stepper driver
 * must be called after Vbb is applied
 */
void TMC2590::init()
{
    // set the saved values
    send20bits(driver_control_register_value);
    send20bits(chopper_config_register_value);
    send20bits(cool_step_register_value);
    send20bits(stall_guard2_current_register_value);
    send20bits(driver_configuration_register_value);

    started = true;
}

// Current is passed in as milliamps
void TMC2590::setCurrent(unsigned int current)
{
    if(current > max_current) current = max_current;

    uint8_t current_scaling = 0;
    //calculate the current scaling from the max current setting (in mA)
    double mASetting = (double)current;
    double resistor_value = (double) this->resistor;
    // remove vesense flag
    this->driver_configuration_register_value &= ~(VSENSE);
    //this is derived from I=(cs+1)/32*(Vsense/Rsense)
    //leading to cs = 32*R*I/V (with V = 0,325V or 0,173V and I = 1000*current)
    //with Rsense=resistor_value
    //for vsense = 0,3250V (VSENSE not set)
    //or vsense = 0,173V (VSENSE set)
    current_scaling = (uint8_t)((((resistor_value/1000) * (mASetting/1000) * 32.0) / 0.325) - 0.5); //theoretically - 1.0 for better rounding it is 0.5

    //check if the current scaling is too low
    if (current_scaling < 16) {
        //set the vsense bit to get a use half the sense voltage (to support lower motor currents)
        this->driver_configuration_register_value |= VSENSE;
        //and recalculate the current setting
        current_scaling = (uint8_t)((((resistor_value/1000) * (mASetting/1000) * 32.0) / 0.173) - 0.5); //theoretically - 1.0 for better rounding it is 0.5
    }

    //do some sanity checks
    if (current_scaling > 31) {
        current_scaling = 31;

    } else if(current_scaling < 16) {
        printf("WARNING: tmc2590: %c - suboptimal current setting %d at %umA with sense resistor value %umilliOhms\n", designator, current_scaling, current, resistor);
    }

    //delete the old value
    stall_guard2_current_register_value &= ~(CURRENT_SCALING_PATTERN);
    //set the new current scaling
    stall_guard2_current_register_value |= current_scaling;
    //if started we directly send it to the motor
    if (started) {
        send20bits(driver_configuration_register_value);
        send20bits(stall_guard2_current_register_value);
    }
}

unsigned int TMC2590::getCurrent(void)
{
    //we calculate the current according to the datasheet to be on the safe side
    //this is not the fastest but the most accurate and illustrative way
    double result = (double)(stall_guard2_current_register_value & CURRENT_SCALING_PATTERN);
    double resistor_value = (double)this->resistor;
    double voltage = (driver_configuration_register_value & VSENSE) ? 0.173 : 0.325;
    // resistor_value is in milliohms so convert to ohms, result is mA
    result = ((result + 1.0F) / 32.0F) * (voltage / (resistor_value / 1000.0)) * 1000.0; // mA
    return (unsigned int)round(result);
}

void TMC2590::setStallGuardThreshold(int8_t stall_guard_threshold, int8_t stall_guard_filter_enabled)
{
    if (stall_guard_threshold < -64) {
        stall_guard_threshold = -64;
        //We just have 5 bits
    } else if (stall_guard_threshold > 63) {
        stall_guard_threshold = 63;
    }
    //add trim down to 7 bits
    stall_guard_threshold &= 0x7f;
    //delete old stall guard settings
    stall_guard2_current_register_value &= ~(STALL_GUARD_CONFIG_PATTERN);
    if (stall_guard_filter_enabled) {
        stall_guard2_current_register_value |= STALL_GUARD_FILTER_ENABLED;
    }
    //Set the new stall guard threshold
    stall_guard2_current_register_value |= (((unsigned long)stall_guard_threshold << 8) & STALL_GUARD_CONFIG_PATTERN);
    //if started we directly send it to the motor
    if (started) {
        send20bits(stall_guard2_current_register_value);
    }
}

int8_t TMC2590::getStallGuardThreshold(void)
{
    unsigned long stall_guard_threshold = stall_guard2_current_register_value & STALL_GUARD_VALUE_PATTERN;
    //shift it down to bit 0
    stall_guard_threshold >>= 8;
    //convert the value to an int to correctly handle the negative numbers
    int8_t result = stall_guard_threshold;
    //check if it is negative and fill it up with leading 1 for proper negative number representation
    if (result & (1 << 6)) {
        result |= 0xC0;
    }
    return result;
}

int8_t TMC2590::getStallGuardFilter(void)
{
    if (stall_guard2_current_register_value & STALL_GUARD_FILTER_ENABLED) {
        return 1;
    } else {
        return 0;
    }
}
/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void TMC2590::setMicrosteps(int number_of_steps)
{
    long setting_pattern = 8;
    //poor mans log
    if (number_of_steps >= 256) {
        setting_pattern = 0;
        microsteps = 256;
    } else if (number_of_steps >= 128) {
        setting_pattern = 1;
        microsteps = 128;
    } else if (number_of_steps >= 64) {
        setting_pattern = 2;
        microsteps = 64;
    } else if (number_of_steps >= 32) {
        setting_pattern = 3;
        microsteps = 32;
    } else if (number_of_steps >= 16) {
        setting_pattern = 4;
        microsteps = 16;
    } else if (number_of_steps >= 8) {
        setting_pattern = 5;
        microsteps = 8;
    } else if (number_of_steps >= 4) {
        setting_pattern = 6;
        microsteps = 4;
    } else if (number_of_steps >= 2) {
        setting_pattern = 7;
        microsteps = 2;
        //1 and 0 lead to full step
    } else if (number_of_steps <= 1) {
        setting_pattern = 8;
        microsteps = 1;
    }

    //delete the old value
    this->driver_control_register_value &= ~(MICROSTEPPING_PATTERN);
    //set the new value
    this->driver_control_register_value |= setting_pattern;

    //if started we directly send it to the motor
    if (started) {
        send20bits(driver_control_register_value);
    }
}

/*
 * returns the effective number of microsteps at the moment
 */
int TMC2590::getMicrosteps(void)
{
    return microsteps;
}

void TMC2590::setStepInterpolation(int8_t value)
{
    if (value) {
        driver_control_register_value |= STEP_INTERPOLATION;
    } else {
        driver_control_register_value &= ~(STEP_INTERPOLATION);
    }
    //if started we directly send it to the motor
    if (started) {
        send20bits(driver_control_register_value);
    }
}

bool TMC2590::getStepInterpolation()
{
    return (driver_control_register_value & STEP_INTERPOLATION) != 0;
}

void TMC2590::setDoubleEdge(int8_t value)
{
    if (value) {
        driver_control_register_value |= DOUBLE_EDGE_STEP;
    } else {
        driver_control_register_value &= ~(DOUBLE_EDGE_STEP);
    }
    //if started we directly send it to the motor
    if (started) {
        send20bits(driver_control_register_value);
    }
}

void TMC2590::setPassiveFastDecay(int8_t value)
{
    if (value) {
        driver_configuration_register_value |= EN_PFD;
    } else {
        driver_configuration_register_value &= ~(EN_PFD);
    }
    //if started we directly send it to the motor
    if (started) {
        send20bits(driver_configuration_register_value);
    }
}

void TMC2590::setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
{
  
}

/*
 * constant_off_time: The off time setting controls the minimum chopper frequency.
 * For most applications an off time within the range of 5μs to 20μs will fit.
 *      2...15: off time setting
 *
 * blank_time: Selects the comparator blank time. This time needs to safely cover the switching event and the
 * duration of the ringing on the sense resistor. For
 *      0: min. setting 3: max. setting
 *
 * hysteresis_start: Hysteresis start setting. Please remark, that this value is an offset to the hysteresis end value HEND.
 *      1...8
 *
 * hysteresis_end: Hysteresis end setting. Sets the hysteresis end value after a number of decrements. Decrement interval time is controlled by HDEC.
 * The sum HSTRT+HEND must be <16. At a current setting CS of max. 30 (amplitude reduced to 240), the sum is not limited.
 *      -3..-1: negative HEND 0: zero HEND 1...12: positive HEND
 *
 * hysteresis_decrement: Hysteresis decrement setting. This setting determines the slope of the hysteresis during on time and during fast decay time.
 *      0: fast decrement 3: very slow decrement
 */

void TMC2590::setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end, int8_t hysteresis_decrement)
{
    h_start = hysteresis_start;
    h_end = hysteresis_end;
    h_decrement = hysteresis_decrement;
    this->vblank_time = blank_time;

    //perform some sanity checks
    if (constant_off_time < 2) {
        constant_off_time = 2;
    } else if (constant_off_time > 15) {
        constant_off_time = 15;
    }
    //save the constant off time
    this->vconstant_off_time = constant_off_time;
    int8_t blank_value;
    //calculate the value acc to the clock cycles
    if (blank_time >= 54) {
        blank_value = 3;
    } else if (blank_time >= 36) {
        blank_value = 2;
    } else if (blank_time >= 24) {
        blank_value = 1;
    } else {
        blank_value = 0;
    }

    if (hysteresis_start < 1) {
        hysteresis_start = 1;
    } else if (hysteresis_start > 8) {
        hysteresis_start = 8;
    }
    hysteresis_start--;

    if (hysteresis_end < -3) {
        hysteresis_end = -3;
    } else if (hysteresis_end > 12) {
        hysteresis_end = 12;
    }
    //shift the hysteresis_end
    hysteresis_end += 3;

    // Fixme this should be %00: 16, %01: 32, %10: 48, %11: 64
    if (hysteresis_decrement < 0) {
        hysteresis_decrement = 0;
    } else if (hysteresis_decrement > 3) {
        hysteresis_decrement = 3;
    }

    //first of all delete all the values for this
    chopper_config_register_value &= ~(CHOPPER_MODE_T_OFF_FAST_DECAY | BLANK_TIMING_PATTERN | HYSTERESIS_DECREMENT_PATTERN | HYSTERESIS_LOW_VALUE_PATTERN | HYSTERESIS_START_VALUE_PATTERN | T_OFF_TIMING_PATERN);

    //set the blank timing value
    chopper_config_register_value |= ((unsigned long)blank_value) << BLANK_TIMING_SHIFT;
    //setting the constant off time
    chopper_config_register_value |= constant_off_time;
    //set the hysteresis_start
    chopper_config_register_value |= ((unsigned long)hysteresis_start) << HYSTERESIS_START_VALUE_SHIFT;
    //set the hysteresis end
    chopper_config_register_value |= ((unsigned long)hysteresis_end) << HYSTERESIS_LOW_SHIFT;
    //set the hystereis decrement
    chopper_config_register_value |= ((unsigned long)hysteresis_decrement) << HYSTERESIS_DECREMENT_SHIFT;

    //if started we directly send it to the motor
    if (started) {
        send20bits(chopper_config_register_value);
    }
}

/*
 * In a constant off time chopper scheme both coil choppers run freely, i.e. are not synchronized.
 * The frequency of each chopper mainly depends on the coil current and the position dependant motor coil inductivity, thus it depends on the microstep position.
 * With some motors a slightly audible beat can occur between the chopper frequencies, especially when they are near to each other. This typically occurs at a
 * few microstep positions within each quarter wave. This effect normally is not audible when compared to mechanical noise generated by ball bearings, etc.
 * Further factors which can cause a similar effect are a poor layout of sense resistor GND connection.
 * Hint: A common factor, which can cause motor noise, is a bad PCB layout causing coupling of both sense resistor voltages
 * (please refer to sense resistor layout hint in chapter 8.1).
 * In order to minimize the effect of a beat between both chopper frequencies, an internal random generator is provided.
 * It modulates the slow decay time setting when switched on by the RNDTF bit. The RNDTF feature further spreads the chopper spectrum,
 * reducing electromagnetic emission on single frequencies.
 */
void TMC2590::setRandomOffTime(int8_t value)
{
    if (value) {
        chopper_config_register_value |= RANDOM_TOFF_TIME;
    } else {
        chopper_config_register_value &= ~(RANDOM_TOFF_TIME);
    }
    //if started we directly send it to the motor
    if (started) {
        send20bits(chopper_config_register_value);
    }
}

void TMC2590::setCoolStepConfiguration(unsigned int lower_SG_threshold, unsigned int SG_hysteresis, uint8_t current_decrement_step_size,
                                       uint8_t current_increment_step_size, uint8_t lower_current_limit)
{
    //sanitize the input values
    if (lower_SG_threshold > 480) {
        lower_SG_threshold = 480;
    }
    //divide by 32
    lower_SG_threshold >>= 5;
    if (SG_hysteresis > 480) {
        SG_hysteresis = 480;
    }
    //divide by 32
    SG_hysteresis >>= 5;
    if (current_decrement_step_size > 3) {
        current_decrement_step_size = 3;
    }
    if (current_increment_step_size > 3) {
        current_increment_step_size = 3;
    }
    if (lower_current_limit > 1) {
        lower_current_limit = 1;
    }
    //store the lower level in order to enable/disable the cool step
    this->cool_step_lower_threshold = lower_SG_threshold;
    //if cool step is not enabled we delete the lower value to keep it disabled
    if (!this->cool_step_enabled) {
        lower_SG_threshold = 0;
    }
    //the good news is that we can start with a complete new cool step register value
    //and simply set the values in the register
    cool_step_register_value = ((unsigned long)lower_SG_threshold) | (((unsigned long)SG_hysteresis) << 8) | (((unsigned long)current_decrement_step_size) << 5)
                               | (((unsigned long)current_increment_step_size) << 13) | (((unsigned long)lower_current_limit) << 15)
                               //and of course we have to include the signature of the register
                               | COOL_STEP_REGISTER;

    if (started) {
        send20bits(cool_step_register_value);
    }
}

void TMC2590::setCoolStepEnabled(bool enabled)
{
    //simply delete the lower limit to disable the cool step
    cool_step_register_value &= ~SE_MIN_PATTERN;
    //and set it to the proper value if cool step is to be enabled
    if (enabled) {
        cool_step_register_value |= this->cool_step_lower_threshold;
    }
    //and save the enabled status
    this->cool_step_enabled = enabled;
    //save the register value
    if (started) {
        send20bits(cool_step_register_value);
    }
}

bool TMC2590::isCoolStepEnabled(void)
{
    return this->cool_step_enabled;
}

unsigned int TMC2590::getCoolStepLowerSgThreshold()
{
    //we return our internally stored value - in order to provide the correct setting even if cool step is not enabled
    return this->cool_step_lower_threshold << 5;
}

unsigned int TMC2590::getCoolStepUpperSgThreshold()
{
    return (uint8_t)((cool_step_register_value & SE_MAX_PATTERN) >> 8) << 5;
}

uint8_t TMC2590::getCoolStepCurrentIncrementSize()
{
    return (uint8_t)((cool_step_register_value & CURRENT_DOWN_STEP_SPEED_PATTERN) >> 13);
}

uint8_t TMC2590::getCoolStepNumberOfSGReadings()
{
    return (uint8_t)((cool_step_register_value & SE_CURRENT_STEP_WIDTH_PATTERN) >> 5);
}

uint8_t TMC2590::getCoolStepLowerCurrentLimit()
{
    return (uint8_t)((cool_step_register_value & MINIMUM_CURRENT_FOURTH) >> 15);
}

void TMC2590::setEnabled(bool enabled)
{
    //delete the t_off in the chopper config to get sure
    chopper_config_register_value &= ~(T_OFF_PATTERN);
    if (enabled) {
        // and set the t_off time
        chopper_config_register_value |= this->vconstant_off_time;
    }
    // if not enabled we don't have to do anything since we already delete t_off from the register

    if (started) {
        send20bits(chopper_config_register_value);
    }
}

bool TMC2590::isEnabled()
{
    if (chopper_config_register_value & T_OFF_PATTERN) {
        return true;
    } else {
        return false;
    }
}

/*
 * reads a value from the TMC2590 status register. The value is not obtained directly but can then
 * be read by the various status routines.
 *
 */
void TMC2590::readStatus(int8_t read_value)
{
    unsigned long old_driver_configuration_register_value = driver_configuration_register_value;
    //reset the readout configuration
    driver_configuration_register_value &= ~(READ_SELECTION_PATTERN);
    //this now equals TMC2590_READOUT_POSITION - so we just have to check the other two options
    if (read_value == TMC2590_READOUT_STALLGUARD) {
        driver_configuration_register_value |= READ_STALL_GUARD_READING;
    } else if (read_value == TMC2590_READOUT_CURRENT) {
        driver_configuration_register_value |= READ_STALL_GUARD_AND_COOL_STEP;
    } else if (read_value == TMC2590_READOUT_ALL_FLAGS) {
        driver_configuration_register_value |= READ_ALL_FLAGS;
    }

    // FIXME this needs to be atomic as a readStatus inbetween the two could change the results read back
    if (driver_configuration_register_value != old_driver_configuration_register_value) {
        // we need to write the value twice - one time for configuring, second time to get the value
        send20bits(driver_configuration_register_value);
    }
    //write the configuration to get the last status
    send20bits(driver_configuration_register_value);
}

//reads the stall guard setting from last status
//returns -1 if stallguard information is not present
int TMC2590::getCurrentStallGuardReading(void)
{
    //if we don't yet started there cannot be a stall guard value
    if (!started) {
        return -1;
    }
    //not time optimal, but solution optiomal:
    //first read out the stall guard value
    readStatus(TMC2590_READOUT_STALLGUARD);
    return getReadoutValue();
}

uint8_t TMC2590::getCurrentCSReading(void)
{
    //if we don't yet started there cannot be a stall guard value
    if (!started) {
        return 0;
    }

    //first read out the stall guard value
    readStatus(TMC2590_READOUT_CURRENT);
    return (getReadoutValue() & 0x1f);
}

unsigned int TMC2590::getCoolstepCurrent(void)
{
    float result = (float)getCurrentCSReading();
    float resistor_value = (float)this->resistor;
    float voltage = (driver_configuration_register_value & VSENSE) ? 0.173F : 0.325F;
    result = (result + 1.0F) / 32.0F * voltage / resistor_value * 1000.0F * 1000.0F;
    return (unsigned int)roundf(result);
}

/*
 return true if the stallguard threshold has been reached
*/
bool TMC2590::isStallGuardOverThreshold(void)
{
    if (!this->started) {
        return false;
    }
    return (driver_status_result & STATUS_STALL_GUARD_STATUS);
}

/*
 returns if there is any over temperature condition:
 OVER_TEMPERATURE_PREWARING if pre warning level has been reached
 OVER_TEMPERATURE_SHUTDOWN if the temperature is so hot that the driver is shut down
 Any of those levels are not too good.
*/
int8_t TMC2590::getOverTemperature(void)
{
    if (!this->started) {
        return 0;
    }
    if (driver_status_result & STATUS_OVER_TEMPERATURE_SHUTDOWN) {
        return TMC2590_OVERTEMPERATURE_SHUTDOWN;
    }
    if (driver_status_result & STATUS_OVER_TEMPERATURE_WARNING) {
        return TMC2590_OVERTEMPERATURE_PREWARING;
    }
    return 0;
}

bool TMC2590::isOverTemperature_SHUTDOWN(void)
{
    return getOverTemperature() == TMC2590_OVERTEMPERATURE_SHUTDOWN;
}

bool TMC2590::isOverTemperature_WARNING(void)
{
    return getOverTemperature() == TMC2590_OVERTEMPERATURE_PREWARING;
}

//is motor channel A shorted to ground
bool TMC2590::isShortToGroundA(void)
{
    if (!this->started) {
        return false;
    }
    return (driver_status_result & STATUS_SHORT_TO_GROUND_A);
}

//is motor channel B shorted to ground
bool TMC2590::isShortToGroundB(void)
{
    if (!this->started) {
        return false;
    }
    return (driver_status_result & STATUS_SHORT_TO_GROUND_B);
}

//is motor channel A connected
bool TMC2590::isOpenLoadA(void)
{
    // NOTE this will give false readings if not moving or going fast
    // should only test when going slow
    if (!this->started || this->isStandStill()) {
        return false;
    }
    return (driver_status_result & STATUS_OPEN_LOAD_A);
}

//is motor channel B connected
bool TMC2590::isOpenLoadB(void)
{
    if (!this->started || this->isStandStill()) {
        return false;
    }
    return (driver_status_result & STATUS_OPEN_LOAD_B);
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC2590::isStandStill(void)
{
    if (!this->started) {
        return false;
    }
    return (driver_status_result & STATUS_STAND_STILL);
}

//is chopper inactive since 2^20 clock cycles - defaults to ~0,08s
bool TMC2590::isStallGuardReached(void)
{
    if (!this->started) {
        return false;
    }
    return (driver_status_result & STATUS_STALL_GUARD_STATUS);
}

int TMC2590::getReadoutValue(void)
{
    return (int)(driver_status_result >> 10);
}

bool TMC2590::isCurrentScalingHalfed()
{
    if (this->driver_configuration_register_value & VSENSE) {
        return true;
    } else {
        return false;
    }
}

void TMC2590::dump_status(OutputStream& stream, bool readable)
{
    // always report errors
    error_reported.reset();
    error_detected.set();

    if (readable) {
        stream.printf("designator %c, actuator %s, Chip type TMC2590\n", designator, name.c_str());

        check_error_status_bits(stream);

        readStatus(TMC2590_READOUT_POSITION); // get the status bits
        if((driver_status_result & 0x00300) != 0) stream.printf("WARNING: Response read appears incorrect: %05lX\n", driver_status_result);

        if (this->isStallGuardReached()) {
            stream.printf("Stall Guard level reached\n");
        }

        if (this->isStandStill()) {
            stream.printf("Motor is standing still\n");
        }

        int value = getReadoutValue();
        stream.printf("Microstep position phase A: %d\n", value);

        value = getCurrentStallGuardReading();
        stream.printf("Stall Guard value: %d\n", value);

        stream.printf("Current setting: %dmA Peak - %f Amps RMS\n", getCurrent(), (getCurrent() * 0.707F) / 1000);
        stream.printf("Coolstep current: %dmA\n", getCoolstepCurrent());

        stream.printf("Microsteps: 1/%d\n", microsteps);
        stream.printf("Step interpolation is %s\n", getStepInterpolation() ? "on" : "off");
        stream.printf("Overtemperature sensitivity: %dC\n", (driver_configuration_register_value & OTSENS) ? 136 : 150);
        stream.printf("Short to ground sensitivity: %s\n", (driver_configuration_register_value & SHRTSENS) ? "high" : "low");
        stream.printf("Passive fast decay is %s\n", (driver_configuration_register_value & EN_PFD) ? "on" : "off");
        stream.printf("Short to VS protection is %s\n", (driver_configuration_register_value & EN_S2VS) ? "enabled" : "disabled");
        stream.printf("Short to GND protection is %s\n", (driver_configuration_register_value & DIS_S2G) ? "disabled" : "enabled");
        stream.printf("Short detection delay is ");
        switch((driver_configuration_register_value & TS2G) >> 8) {
            case 0: stream.printf("3.2us\n"); break;
            case 1: stream.printf("1.6us\n"); break;
            case 2: stream.printf("1.2us\n"); break;
            case 3: stream.printf("0.8us\n"); break;
        }
        uint8_t slope_high = ((driver_configuration_register_value & SLPH) >> 14) | ((driver_configuration_register_value & SLPHL2) >> 9);
        uint8_t slope_low = ((driver_configuration_register_value & SLPL) >> 12) | ((driver_configuration_register_value & SLPHL2) >> 9);
        stream.printf("Slope control - high: %d, low: %d\n", slope_high, slope_low);

        readStatus(TMC2590_READOUT_ALL_FLAGS);
        if((driver_status_result & 0x00300) != 0x00300) {
            stream.printf("WARNING: Read all flags appears incorrect: %05lX\n", driver_status_result);
        }else{
            value = driver_status_result;
            if(value & 0xFFC00) {
                stream.printf("Detailed Flags...\n");
                if(value & 0x80000) stream.printf("  Low voltage detected\n");
                if(value & 0x40000) stream.printf("  ENN enabled\n");
                if(value & 0x20000) stream.printf("  Short to high B\n");
                if(value & 0x10000) stream.printf("  Short to low B\n");
                if(value & 0x08000) stream.printf("  Short to high A\n");
                if(value & 0x04000) stream.printf("  Short to low A\n");
                if(value & 0x02000) stream.printf("  Overtemp 150\n");
                if(value & 0x01000) stream.printf("  Overtemp 136\n");
                if(value & 0x00800) stream.printf("  Overtemp 120\n");
                if(value & 0x00400) stream.printf("  Overtemp 100\n");
            }
        }

        stream.printf("Register dump:\n");
        stream.printf(" driver control register: %05lX(%ld)\n", driver_control_register_value, driver_control_register_value);
        stream.printf(" chopper config register: %05lX(%ld)\n", chopper_config_register_value, chopper_config_register_value);
        stream.printf(" cool step register: %05lX(%ld)\n", cool_step_register_value, cool_step_register_value);
        stream.printf(" stall guard2 current register: %05lX(%ld)\n", stall_guard2_current_register_value, stall_guard2_current_register_value);
        stream.printf(" driver configuration register: %05lX(%ld)\n", driver_configuration_register_value, driver_configuration_register_value);
        stream.printf(" %s.reg = %05lX,%05lX,%05lX,%05lX,%05lX\n", name.c_str(), driver_control_register_value, chopper_config_register_value, cool_step_register_value, stall_guard2_current_register_value, driver_configuration_register_value);

    } else {
        // This is the format the processing app uses for tuning TMX26X chips
        int n = designator < 'X' ? designator - 'A' + 3 : designator - 'X';
        bool moving = Robot::getInstance()->actuators[n]->is_moving();
        // dump out in the format that the processing script needs
        if (moving) {
            stream.printf("#sg%d,p%lu,k%u,r,", getCurrentStallGuardReading(), Robot::getInstance()->actuators[n]->get_current_step(), getCoolstepCurrent());
        } else {
            readStatus(TMC2590_READOUT_POSITION); // get the status bits
            stream.printf("#s,");
        }
        stream.printf("d%d,", Robot::getInstance()->actuators[n]->which_direction() ? -1 : 1);
        stream.printf("c%u,m%d,", getCurrent(), getMicrosteps());
        // stream.printf('S');
        // stream.printf(TMC2590Stepper.getSpeed(), DEC);
        stream.printf("t%d,f%d,", getStallGuardThreshold(), getStallGuardFilter());

        //print out the general cool step config
        if (isCoolStepEnabled()) stream.printf("Ke+,");
        else stream.printf("Ke-,");

        stream.printf("Kl%u,Ku%u,Kn%u,Ki%u,Km%u,",
                      getCoolStepLowerSgThreshold(), getCoolStepUpperSgThreshold(), getCoolStepNumberOfSGReadings(), getCoolStepCurrentIncrementSize(), getCoolStepLowerCurrentLimit());

        //detect the winding status
        if (isOpenLoadA()) {
            stream.printf("ao,");
        } else if(isShortToGroundA()) {
            stream.printf("ag,");
        } else {
            stream.printf("a-,");
        }
        //detect the winding status
        if (isOpenLoadB()) {
            stream.printf("bo,");
        } else if(isShortToGroundB()) {
            stream.printf("bg,");
        } else {
            stream.printf("b-,");
        }

        char temperature = getOverTemperature();
        if (temperature == 0) {
            stream.printf("x-,");
        } else if (temperature == TMC2590_OVERTEMPERATURE_PREWARING) {
            stream.printf("xw,");
        } else {
            stream.printf("xe,");
        }

        if (isEnabled()) {
            stream.printf("e1,");
        } else {
            stream.printf("e0,");
        }

        //write out the current chopper config
        stream.printf("Cm%d,", (chopper_config_register_value & CHOPPER_MODE_T_OFF_FAST_DECAY) != 0);
        stream.printf("Co%d,Cb%d,", vconstant_off_time, vblank_time);
        if ((chopper_config_register_value & CHOPPER_MODE_T_OFF_FAST_DECAY) == 0) {
            stream.printf("Cs%d,Ce%d,Cd%d,", h_start, h_end, h_decrement);
        }
        stream.printf("\n");
    }

    error_reported.reset();
    error_detected.reset();
}

// static test functions
const std::array<TMC2590::TestFun, 6> TMC2590::test_fncs {{
    std::mem_fn(&TMC2590::isOverTemperature_WARNING),
    std::mem_fn(&TMC2590::isOverTemperature_SHUTDOWN),
    std::mem_fn(&TMC2590::isShortToGroundA),
    std::mem_fn(&TMC2590::isShortToGroundB),
    std::mem_fn(&TMC2590::isOpenLoadA),
    std::mem_fn(&TMC2590::isOpenLoadB)
}};

// define tests here: id, fatal, message
const std::array<TMC2590::e_t, 6> TMC2590::tests {{
    {0, false, "Overtemperature Prewarning"},
    {1, true, "Overtemperature Shutdown"},
    {2, true, "SHORT to ground on channel A"},
    {3, true, "SHORT to ground on channel B"},
    // {4, false, "Channel A seems to be unconnected"}, // unreliable
    // {5, false, "Channel B seems to be unconnected"}
}};

// check error bits and report, only report once, and debounce the test
// can be called from timer task or command task
bool TMC2590::check_error_status_bits(OutputStream& stream)
{
    readStatus(TMC2590_READOUT_POSITION); // get the status bits
    // test the flags are ok
    if((driver_status_result & 0x00300) != 0){
        stream.printf("WARNING: Response read appears incorrect: %05lX\n", driver_status_result);
        return false;
    }

    bool error = false;
    for(auto& i : TMC2590::tests) {
        int n = std::get<0>(i);
        if(TMC2590::test_fncs[n](this)) {
            if(!error_detected.test(n)) {
                // debounce, needs to be set when checked two times in a row
                error_detected.set(n);
            } else {
                if(!error_reported.test(n)) {
                    // only reports error once until it has been cleared
                    stream.printf("WARNING: %c: %s\n", designator, std::get<2>(i));
                    error_reported.set(n);
                }
                if(!error && std::get<1>(i)) {
                    // fatal error and error not yet set
                    error = true;
                }
            }
        } else {
            error_reported.reset(n);
            error_detected.reset(n);
        }
    }

    // if(error) {
    //     stream.printf("%08X\n", driver_status_result);
    // }
    return error;
}

// called from the timer task
bool TMC2590::check_errors()
{
    std::ostringstream oss;
    OutputStream os(&oss);
    bool b = check_error_status_bits(os);
    if(!oss.str().empty()) {
        print_to_all_consoles(oss.str().c_str());
    }
    return b;
}

// sets a raw register to the value specified, for advanced settings
// register 255 writes them, 0 displays what registers are mapped to what
// FIXME status registers not reading back correctly, check docs
bool TMC2590::set_raw_register(OutputStream& stream, uint32_t reg, uint32_t val)
{
    switch(reg) {
        case 255:
            send20bits(driver_control_register_value);
            send20bits(chopper_config_register_value);
            send20bits(cool_step_register_value);
            send20bits(stall_guard2_current_register_value);
            send20bits(driver_configuration_register_value);
            stream.printf("Registers written\n");
            break;


        case 1: driver_control_register_value = val; stream.printf("driver control register set to %05lX\n", val); break;
        case 2: chopper_config_register_value = val; stream.printf("chopper config register set to %05lX\n", val); break;
        case 3: cool_step_register_value = val; stream.printf("cool step register set to %05lX\n", val); break;
        case 4: stall_guard2_current_register_value = val; stream.printf("stall guard2 current register set to %05lX\n", val); break;
        case 5: driver_configuration_register_value = val; stream.printf("driver configuration register set to %05lX\n", val); break;

        default:
            stream.printf("1: driver control register\n");
            stream.printf("2: chopper config register\n");
            stream.printf("3: cool step register\n");
            stream.printf("4: stall guard2 current register\n");
            stream.printf("5: driver configuration register\n");
            stream.printf("255: update all registers\n");
            return false;
    }
    return true;
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 * sends 20bits, the last 20 bits of the 24bits is taken as the command
 */
void TMC2590::send20bits(uint32_t datagram)
{
    uint8_t txbuf[] {(uint8_t)(datagram >> 16), (uint8_t)(datagram >>  8), (uint8_t)(datagram & 0xff)};
    uint8_t rxbuf[3] {0};

    // write/read the values
    if(sendSPI(txbuf, rxbuf)) {
        //store the datagram as status result
        uint32_t i_datagram = ((rxbuf[0] << 16) | (rxbuf[1] << 8) | (rxbuf[2])) >> 4;
        driver_status_result = i_datagram;
    }else{
        printf("ERROR: send20bits failed\n");
    }
}

// Called by the drivers codes to send and receive SPI data to/from the chip
bool TMC2590::sendSPI(void *b, void *r)
{
    // lock the SPI bus for this transaction
    if(!spi->begin_transaction()) return false;
    spi_cs->set(false); // enable chip select
    bool stat= spi->write_read(b, r, 3);
    spi_cs->set(true); // disable chip select
    spi->end_transaction();
    return stat;
}

#define HAS(X) (gcode.has_arg(X))
#define GET(X) (gcode.get_int_arg(X))
bool TMC2590::set_options(const GCode& gcode)
{
    bool set = false;
    if(HAS('O') || HAS('Q')) {
        // void TMC2590::setStallGuardThreshold(int8_t stall_guard_threshold, int8_t stall_guard_filter_enabled)
        int8_t o = HAS('O') ? GET('O') : getStallGuardThreshold();
        int8_t q = HAS('Q') ? GET('Q') : getStallGuardFilter();
        setStallGuardThreshold(o, q);
        set = true;
    }

    if(HAS('H') && HAS('I') && HAS('J') && HAS('K') && HAS('L')) {
        //void TMC2590::setCoolStepConfiguration(unsigned int lower_SG_threshold, unsigned int SG_hysteresis, uint8_t current_decrement_step_size, uint8_t current_increment_step_size, uint8_t lower_current_limit)
        setCoolStepConfiguration(GET('H'), GET('I'), GET('J'), GET('K'), GET('L'));
        set = true;
    }

    if(HAS('S')) {
        uint32_t s = GET('S');
        if(s == 0 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //void TMC2590::setConstantOffTimeChopper(int8_t constant_off_time, int8_t blank_time, int8_t fast_decay_time_setting, int8_t sine_wave_offset, uint8_t use_current_comparator)
            setConstantOffTimeChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 1 && HAS('U') && HAS('V') && HAS('W') && HAS('X') && HAS('Y')) {
            //void TMC2590::setSpreadCycleChopper(int8_t constant_off_time, int8_t blank_time, int8_t hysteresis_start, int8_t hysteresis_end, int8_t hysteresis_decrement);
            setSpreadCycleChopper(GET('U'), GET('V'), GET('W'), GET('X'), GET('Y'));
            set = true;

        } else if(s == 2 && HAS('Z')) {
            setRandomOffTime(GET('Z'));
            set = true;

        } else if(s == 3 && HAS('Z')) {
            setDoubleEdge(GET('Z'));
            set = true;

        } else if(s == 4 && HAS('Z')) {
            setStepInterpolation(GET('Z'));
            set = true;

        } else if(s == 5 && HAS('Z')) {
            setCoolStepEnabled(GET('Z') == 1);
            set = true;

        } else if(s == 6 && HAS('Z')) {
            setPassiveFastDecay(GET('Z') == 1);
            set = true;
        }
    }

    return set;
}
#endif
