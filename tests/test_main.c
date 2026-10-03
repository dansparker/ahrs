#include "test.h"

int g_failures, g_checks;

static uint32_t rng_state = 1;

void rand_seed(uint32_t s) { rng_state = s ? s : 1; }

static float randu(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return ((float)(rng_state >> 8) + 0.5f) / 16777216.0f;
}

float randn(void) {
    const float u1 = randu(), u2 = randu();
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

void run_unit_tests(void);
void run_sim_tests(void);

int main(void) {
    run_unit_tests();
    run_sim_tests();
    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
