#include <EposCAN.h>

EPOS::CAN can;
EPOS::Motor m1(1);
bool fwd = true;

void setup()
{
    //Serial.begin(115200);
    printf("CAN serial console\n");
    can.begin(true);
    m1.reset();
    m1.profile_position_relative(fwd? 32000: -32000);
}

void loop()
{
    if (m1.is_target_reached()) {
        fwd = !fwd;
        delay(5000);
        printf("rel move %d\n", (fwd? 32000: -32000));
        m1.profile_position_relative(fwd? 32000: -32000);
    }
}
