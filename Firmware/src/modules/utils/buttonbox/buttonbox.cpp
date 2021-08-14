#include "buttonbox.h"

#include "ConfigReader.h"
#include "SlowTicker.h"
#include "Consoles.h"
#include "Pin.h"
#include "OutputStream.h"
#include "Conveyor.h"
#include "MessageQueue.h"

#include <cstring>

#define enable_key "enable"
#define pin_key "pin"
#define press_key "press"
#define release_key "release"

/*
    A button box is a device with many programmable buttons.
    They can be used to define macros or functions or set some state.
    eg
     buttons could be used to jog X and Y axis in each direction.
     A button could issue a command, gcode or mcode.
     A button could set some state used by a subsequent button (like toggle direction)

    TODO handle button matrix scanning
*/
REGISTER_MODULE(ButtonBox, ButtonBox::create)

bool ButtonBox::create(ConfigReader& cr)
{
    printf("DEBUG: configure ButtonBox\n");
    ButtonBox *bb = new ButtonBox();
    if(!bb->configure(cr)) {
        printf("INFO: No buttons enabled\n");
        delete bb;
    }
    return true;
}

ButtonBox::ButtonBox() : Module("ButtonBox")
{
}

bool ButtonBox::configure(ConfigReader& cr)
{
    ConfigReader::sub_section_map_t ssmap;
    if(!cr.get_sub_sections("button box", ssmap)) {
        printf("configure-buttonbox: no button box section found\n");
        return false;
    }

    int cnt = 0;
    for(auto& i : ssmap) {
        // foreach button
        std::string name = i.first;
        auto& m = i.second;
        if(!cr.get_bool(m, enable_key, false)) continue; // skip if not enabled
        Pin *b = new Pin(cr.get_string(m, pin_key, "nc"), Pin::AS_INPUT);
        if(!b->connected()) {
            printf("ERROR: configure-buttonbox: No, or illegal, button defined\n");
            delete b;
            continue;
        }

        std::string press = cr.get_string(m, press_key, "");
        if(press.empty()) {
            printf("ERROR: configure-buttonbox: No press defined\n");
            delete b;
            continue;
        }
        std::string release = cr.get_string(m, release_key, "");

        but_t bt {
            .but = b,
            .press_act = press,
            .release_act =  release,
            .state = false,
        };
        buttons.push_back(bt);

        printf("DEBUG: button %s, press: %s, release: %s\n", name.c_str(), press.c_str(), release.c_str());

        ++cnt;
    }

    printf("INFO: %d button(s) loaded\n", cnt);

    if(cnt > 0) {
        SlowTicker::getInstance()->attach(20, std::bind(&ButtonBox::button_tick, this));
        return true;
    }
    return false;
}

// Check the state of the buttons and act accordingly
// Note this is the RTOS timer task so don't do anything slow or blocking in here
void ButtonBox::button_tick()
{
    static OutputStream nullos; // null output stream
    for(auto& i : buttons) {
        const char *cmd = nullptr;
        if(!i.state && i.but->get()) {
            // pressed
            cmd = i.press_act.c_str();
            i.state = true;

        } else if(i.state && !i.but->get()) {
            // released
            cmd = i.release_act.c_str();
            i.state = false;
        }

        if(cmd != nullptr) {
            if(strcmp(cmd, "^Y") == 0) {
                if(Conveyor::getInstance()->get_continuous_mode()) {
                    Conveyor::getInstance()->set_continuous_mode(false);
                } else {
                    // set so $J will be ignored if sent too fast
                    nullos.set_stop_request(true);
                }

            } else {
                send_message_queue(cmd, &nullos, false);
            }
        }
    }
}
