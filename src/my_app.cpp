#include "MyClass.hpp"
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Core.hpp>
#include <MEL/Communications.hpp>
#include <MEL/Math.hpp>

using namespace meii;
using namespace mel;

ctrl_bool g_stop_flag(false);

bool my_handler(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC) {
        print("ctrlC pressed");
        g_stop_flag = true;
    }
    // ... check other Ctrl values as needed
    return 1;
}


int main() {

    register_ctrl_handler(my_handler);

    // make Q8 USB and configure
    Q8Usb q8;
	q8.open();
    q8.DO.set_enable_values(std::vector<Logic>(8, High));
    q8.DO.set_disable_values(std::vector<Logic>(8, High));
    q8.DO.set_expire_values(std::vector<Logic>(8, High));
    if (!q8.identify(7)) {
        LOG(Error) << "Incorrect DAQ";
        return 0;
    }
    q8.DO[0].set_value(Low);
    q8.DO[1].set_value(Low);
    // zero joint positions 0 and 1
    q8.encoder[0].zero();
    q8.encoder[1].zero();
    // create MahiExoII and bind Q8 channels to it
    std::vector<Amplifier> amplifiers;
    std::vector<double> amp_gains;
    for (uint32 i = 0; i < 2; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Low,
                q8.DO[i + 1],
                1.8,
                q8.AO[i + 1])
        );
    }
    for (uint32 i = 2; i < 5; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Low,
                q8.DO[i + 1],
                0.184,
                q8.AO[i + 1])
        );
    }
    MeiiConfiguration config(q8, q8.watchdog, q8.encoder[{1, 2, 3, 4, 5}], amplifiers);
    MahiExoII meii(config);  
    q8.enable();
    meii.enable();

    MelShare ms("testing");
    
    PdController pd0(100, 1.25);
    PdController pd1(35, 0.3);

    Waveform sinwave(Waveform::Type::Sin, seconds(4), 20*DEG2RAD);

    Timer timer(hertz(1000));
    Time t = Time::Zero;


    while(!g_stop_flag) {
        q8.update_input();

        double position0 = meii.get_joint(0).get_position();
        double position1 = meii.get_joint(1).get_position();
        double velocity0 = meii.get_joint(0).get_velocity();
        double velocity1 = meii.get_joint(1).get_velocity();

        double torque0 = pd0.calculate(sinwave.evaluate(t), position0, 0, velocity0);
        double torque1 = pd1.calculate(0, position1, 0, velocity1);

        meii.get_joint(0).set_torque(torque0);
        meii.get_joint(1).set_torque(torque1);
        meii.get_joint(2).set_torque(0.0);
        meii.get_joint(3).set_torque(0.0);
        meii.get_joint(4).set_torque(0.0);

        ms.write_data({position0,position1,velocity0, velocity1});


        q8.update_output();
        t = timer.wait();
    }

    q8.disable();


    return 0;
}
