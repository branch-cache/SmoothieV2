/*
TemperatureSwitch is an optional module that will automatically turn on or off a switch
based on a setpoint temperature. It is commonly used to turn on/off a cooling fan or water pump
to cool the hot end's cold zone. Specifically, it turns one of the small MOSFETs on or off.

Author: Michael Hackney, mhackney@eclecticangler.com
*/

#include "TemperatureSwitch.h"
#include "TemperatureControl.h"
#include "GCode.h"
#include "Dispatcher.h"
#include "OutputStream.h"
#include "SlowTicker.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#define hotend_key "hotend"
#define threshold_temp_key "threshold_temp"
#define type_key "type"
#define switch_key "switch"
#define heatup_poll_key "heatup_poll"
#define cooldown_poll_key "cooldown_poll"
#define trigger_key "trigger"
#define inverted_key "inverted"
#define arm_command_key "arm_mcode"
#define designator_key "designator"

// register this module for creation in main
REGISTER_MODULE(TemperatureSwitch, TemperatureSwitch::load)

TemperatureSwitch::TemperatureSwitch(const char *name) : Module("temperature switch", name)
{
    valid= false; // set to false until we have checked it in after_load()
}

TemperatureSwitch::~TemperatureSwitch()
{
}

bool TemperatureSwitch::load(ConfigReader& cr)
{
    printf("DEBUG: configure temperature switches\n");
    ConfigReader::sub_section_map_t ssmap;
    if(!cr.get_sub_sections("temperature switch", ssmap)) {
        printf("INFO: configure-temperature-switch: no temperature switch section found\n");
        return false;
    }

    int cnt = 0;
    for(auto& i : ssmap) {
        // foreach temp switch
        std::string name = i.first;
        auto& m = i.second;
        if(cr.get_bool(m, "enable", false)) {
            TemperatureSwitch *ts = new TemperatureSwitch(name.c_str());
            if(ts->configure(cr, m)) {
                ++cnt;
            } else {
                delete ts;
            }
        }
    }

    if(cnt == 0) return false;

    printf("INFO: %d temperature switch(es) configured and enabled\n", cnt);
    return true;
}

// Load module
bool TemperatureSwitch::configure(ConfigReader& cr, ConfigReader::section_map_t& m)
{
    // create a temperature control and load settings
    std::string s= cr.get_string(m, designator_key, "");
    if(s.empty()){
        printf("WARNING: configure-temperature-switch: requires a designator\n");
        return false;

    }else{
        this->designator= s[0];
    }

    // load settings from config file
    this->switch_name = cr.get_string(m, switch_key, "");
    if(this->switch_name.empty()) {
            // no switch specified so invalid entry
            printf("WARNING: configure-temperature-switch: no switch specified\n");
            return false;
    }

    // if we should turn the switch on or off when trigger is hit
    this->inverted = cr.get_bool(m, inverted_key, false);

    // if we should trigger when above and below, or when rising through, or when falling through the specified temp
    std::string trig = cr.get_string(m, trigger_key, "level");
    if(trig == "level") this->trigger= LEVEL;
    else if(trig == "rising") this->trigger= RISING;
    else if(trig == "falling") this->trigger= FALLING;
    else this->trigger= LEVEL;

    // the mcode used to arm the switch
    this->arm_mcode = cr.get_float(m, arm_command_key, 0);

    this->threshold_temp = cr.get_float(m, threshold_temp_key, 50.0f);

    // these are to tune the heatup and cooldown polling frequencies
    this->heatup_poll = cr.get_float(m, heatup_poll_key, 15);
    this->cooldown_poll = cr.get_float(m, cooldown_poll_key, 60);
    this->current_delay = this->heatup_poll;

    // set initial state
    this->current_state= NONE;
    this->second_counter = this->current_delay; // do test immediately on first second_tick
    // if not defined then always armed, otherwise start out disarmed
    this->armed= (this->arm_mcode == 0);

    // register gcodes and mcodes
    using std::placeholders::_1;
    using std::placeholders::_2;

    if(this->arm_mcode != 0) {
       Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, arm_mcode, std::bind(&TemperatureSwitch::handle_arm, this, _1, _2));
    }

    // we read the temperature in this timer. 1Hz is fast enough
    SlowTicker::getInstance()->attach(1, std::bind(&TemperatureSwitch::tick, this));

    // register a startup function that will be called after all modules have been loaded
    // (as this module relies on Switch module having been loaded)
    register_startup(std::bind(&TemperatureSwitch::after_load, this));

    return true;
}

void TemperatureSwitch::after_load()
{
    // all modules have been loaded so check that the switch exists here
    // get current switch state for the named switch
    Module *m = Module::lookup("switch", this->switch_name.c_str());
    if(m != nullptr) {
        // check it is an output switch
        if(m->request("is_output", nullptr)) {
            valid= true;
        }else{
            printf("ERROR: TemperatureSwitch: switch %s is not an output switch\n", this->switch_name.c_str());
        }
    }else{
        printf("ERROR: TemperatureSwitch: switch %s does not exist\n", this->switch_name.c_str());
    }
}

bool TemperatureSwitch::handle_arm(GCode& gcode, OutputStream& os)
{
    this->armed= (gcode.has_arg('S') && gcode.get_arg('S') != 0);
    os.printf("temperature switch %s\n", this->armed ? "armed" : "disarmed");
    return true;
}

// run in a slowtimer. This should be safe so long as the switch only sets pins
// if switches require commands to be executed we need to rethink this here and in switch
void TemperatureSwitch::tick()
{
    if(!valid) return; // wait until it is valid

    ++second_counter;

    if (second_counter < current_delay) return;

    second_counter = 0;
    float current_temp = this->get_highest_temperature();

    if (current_temp >= this->threshold_temp) {
        set_state(HIGH_TEMP);

    } else {
        set_state(LOW_TEMP);
   }
}

void TemperatureSwitch::set_state(STATE state)
{
    if(state == this->current_state) return; // state did not change

    // state has changed
    switch(this->trigger) {
        case LEVEL:
            // switch on or off depending on HIGH or LOW
            set_switch(state == HIGH_TEMP);
            break;

        case RISING:
            // switch on if rising edge
            if(this->current_state == LOW_TEMP && state == HIGH_TEMP) set_switch(true);
            break;

        case FALLING:
            // switch off if falling edge
            if(this->current_state == HIGH_TEMP && state == LOW_TEMP) set_switch(false);
            break;
    }

    this->current_delay = state == HIGH_TEMP ? this->cooldown_poll : this->heatup_poll;
    this->current_state= state;
}

// Get the highest temperature from the set of temperature controllers
float TemperatureSwitch::get_highest_temperature()
{
    float high_temp = 0.0;

    // TODO optimize by getting a list of valid temperature controls in config
    //   maybe then only get called in ctx if it is on.
    // scan all temperature controls with the specified designator
    std::vector<Module*> controllers = Module::lookup_group("temperature control");
    for(auto m : controllers) {
        TemperatureControl::pad_temperature_t temp;
        if(m->request("get_current_temperature", &temp)) {
            // check if this controller's temp is the highest and save it if so
            if (temp.designator[0] == this->designator && !isinf(temp.current_temperature) && temp.current_temperature > high_temp) {
                high_temp = temp.current_temperature;
            }
        }
    }

    return high_temp;
}

// Turn the switch on (true) or off (false)
void TemperatureSwitch::set_switch(bool switch_state)
{
    if(!this->armed) return; // do not actually switch anything if not armed

    if(this->arm_mcode != 0 && this->trigger != LEVEL) {
        // if edge triggered we only trigger once per arming, if level triggered we switch as long as we are armed
        this->armed= false;
    }

    if(this->inverted) switch_state= !switch_state; // turn switch on or off inverted

    // get current switch state for the named switch
    Module *m = Module::lookup("switch", this->switch_name.c_str());
    if(m != nullptr) {
        bool state;

        // get switch state
        m->request("state", &state);

        if(state != switch_state) {
            // set switch state
            m->request("set-state", &switch_state);
        }

    }
}
