#include <cstdio>
#include <unistd.h>
#include <memory>

#include "Navio2/RCInput_Navio2.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"

using namespace Navio;

constexpr int INPUT_CHANNEL = 2;      // RC input channel to read (throttle)
constexpr int OUTPUT_CHANNEL = 0;     // PWM output channel to control motor
constexpr int PWM_FREQUENCY = 50;     // 50 Hz typical for servo/ESC
constexpr int FEED_US = 20000;        // 20 ms update interval (50 Hz)

std::unique_ptr<RCOutput_Navio2> get_rcout()
{
    // For Navio2 only here (simplified)
    return std::make_unique<RCOutput_Navio2>();
}

int main(int argc, char* argv[])
{
    // Check running as root
    if (getuid() != 0) {
        fprintf(stderr, "Please run as root: sudo %s\n", argv[0]);
        return 1;
    }

    // Initialize RC input
    RCInput_Navio2 rcin;
    rcin.initialize();

    // Initialize RC output (PWM)
    auto pwm = get_rcout();
    if (!pwm->initialize(OUTPUT_CHANNEL)) {
        fprintf(stderr, "Failed to initialize PWM output channel %d\n", OUTPUT_CHANNEL);
        return 1;
    }

    if (!pwm->enable(OUTPUT_CHANNEL)) {
        fprintf(stderr, "Failed to enable PWM output channel %d\n", OUTPUT_CHANNEL);
        return 1;
    }

    pwm->set_frequency(OUTPUT_CHANNEL, PWM_FREQUENCY);

    printf("Starting RC passthrough: Input CH%d -> Output CH%d\n", INPUT_CHANNEL, OUTPUT_CHANNEL);

    while (true) {
        int pwm_in = rcin.read(INPUT_CHANNEL);

        // Input validation
        if (pwm_in < 990 || pwm_in > 2050) {
            fprintf(stderr, "Invalid input PWM: %d us, skipping\n", pwm_in);
            usleep(FEED_US);
            continue;
        }

        // According to RCOutput_Navio2::set_duty_cycle(channel, period_in_ms)
        bool ok = pwm->set_duty_cycle(OUTPUT_CHANNEL, pwm_in);
        if (!ok) {
            fprintf(stderr, "Failed to set PWM output duty cycle\n");
        }

        printf("Output PWM: %d µs\n", pwm_in);

        usleep(FEED_US);
    }

    return 0;
}
