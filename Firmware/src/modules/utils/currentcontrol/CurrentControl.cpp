#include "CurrentControl.h"
#include "ConfigReader.h"
#include "Pwm.h"
#include "GCode.h"
#include "OutputStream.h"
#include "Dispatcher.h"
#include "Robot.h"
#include "StepperMotor.h"

#include <string>
#include <map>
#include <tuple>

#define current_key "current"
#define control_key "control"
#define pin_key "pin"

// this puts the lookup table in FLASH
// and allows one lut to handle both mappings axis <=> name
static const struct name_lut_t {
    const char *name;
    char axis;
} name_lut[] = {
    {"alpha", 'X'},
    {"beta", 'Y'},
    {"gamma", 'Z'},
    {"delta", 'A'},
    {"epsilon", 'B'},
    {"zeta", 'C'}
};
static char lookup_name(const char *n)
{
    for(auto& i : name_lut) {
        if(strcmp(n, i.name) == 0) return i.axis;
    }
    return 0;
}
static const char* lookup_axis(char a)
{
    for(auto& i : name_lut) {
        if(a == i.axis) return i.name;
    }
    return 0;
}

REGISTER_MODULE(CurrentControl, CurrentControl::create)

bool CurrentControl::create(ConfigReader& cr)
{
    printf("DEBUG: configure current control\n");
    CurrentControl *current_control = new CurrentControl();
    if(!current_control->configure(cr)) {
        printf("INFO: No current controls configured\n");
        delete current_control;
        current_control = nullptr;
    }
    return true;
}

CurrentControl::CurrentControl() : Module("currentcontrol")
{
}

bool CurrentControl::configure(ConfigReader& cr)
{
    ConfigReader::sub_section_map_t ssmap;
    if(!cr.get_sub_sections("current control", ssmap)) {
        printf("INFO: configure-current-control: no current control section found\n");
        return false;
    }

    for(auto& i : ssmap) {
        // foreach channel
        std::string name = i.first;
        auto& m = i.second;

        // Get current settings in Amps
        float c = cr.get_float(m, current_key, -1);
        if(c <= 0) {
            printf("INFO: configure-current-control: %s - invalid current\n", name.c_str());
            continue;
        }

#ifdef BOARD_MINIALPHA
        // we have a PWM current control
        std::string pin = cr.get_string(m, pin_key, "nc");
        if(pin == "nc") continue;

        Pwm *pwm = new Pwm(pin.c_str());
        if(!pwm->is_valid()) {
            delete pwm;
            printf("INFO: configure-current-control: %s - pin %s in not a valid PWM pin\n", name.c_str(), pin.c_str());
            continue;
        }

        pins[name] = pwm;
        bool ok = set_current(name, c);

#elif defined(DRIVER_TMC)
        // SPI defined current control, we ask actuator to deal with it
        bool ok = set_current(name, c);
#else
        bool ok = false;
#endif
        if(ok) {
            printf("INFO: configure-current-control: %s set to %1.5f amps\n", name.c_str(), c);
        } else {
            printf("INFO: configure-current-control: Failed to set current for %s\n", name.c_str());
        }
    }

    if(currents.size() > 0) {
        // register gcodes and mcodes
        using std::placeholders::_1;
        using std::placeholders::_2;

        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 500, std::bind(&CurrentControl::handle_gcode, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 906, std::bind(&CurrentControl::handle_gcode, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 907, std::bind(&CurrentControl::handle_gcode, this, _1, _2));
        return true;
    }

    return false;
}

// current is passed in Amps
bool CurrentControl::set_current(const std::string& name, float current)
{
#ifdef BOARD_MINIALPHA
    char axis = lookup_name(name.c_str());
    if(axis == 0) return false;
    auto x = pins.find(name);
    if(x == pins.end()) {
        return false;
    }
    x->second->set(current_to_pwm(current));
    bool ok = true;

#elif defined(DRIVER_TMC)
    // ask Actuator to set its current
    char axis = lookup_name(name.c_str());
    if(axis == 0) return false;

    int n = axis < 'X' ? axis - 'A' + 3 : axis - 'X';
    if(n >= Robot::getInstance()->get_number_registered_motors()) return false;
    bool ok = Robot::getInstance()->actuators[n]->set_current(current);
#else
    char axis = lookup_name(name.c_str());
    if(axis == 0) return false;
    bool ok = false;
#endif

    if(ok) {
        currents[name] = current;
    }

    return ok;
}

bool CurrentControl::handle_gcode(GCode& gcode, OutputStream& os)
{
    if(gcode.get_code() == 906 || gcode.get_code() == 907) {
        if(gcode.has_no_args()) {
            for (auto i : currents) {
                os.printf("%s: %1.5f Amps\n", i.first.c_str(), i.second);
            }
            return true;
        }

        // set current in mA or Amps
        for (int i = 0; i < Robot::getInstance()->get_number_registered_motors(); i++) {
            char axis = i < 3 ? 'X' + i : 'A' + i - 3;
            if (gcode.has_arg(axis)) {
                float current = gcode.get_arg(axis);
                if(gcode.get_code() == 906) {
                    // convert to Amps
                    current /= 1000.0F;
                }
                const char *name = lookup_axis(axis);
                if(name == 0) {
                    os.printf("WARNING: could not find axis %c\n", axis);
                } else {
                    if(!set_current(name, current)) {
                        os.printf("axis %c is not configured for current control\n", axis);
                    }
                }
            }
        }
        return true;

    } else if(gcode.get_code() == 500) {
        std::string res;
        for (int i = 0; i < Robot::getInstance()->get_number_registered_motors(); i++) {
            char axis = i < 3 ? 'X' + i : 'A' + i - 3;
            const char *name = lookup_axis(axis);
            if(name != 0) {
                auto c = currents.find(name);
                if (c != currents.end()) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%c%1.5f ", axis, c->second);
                    res += buf;
                }
            }
        }
        if(!res.empty()) {
            os.printf(";Motor currents (amps):\nM907 ");
            os.printf("%s\n", res.c_str());
        }
        return true;
    }

    return false;
}

