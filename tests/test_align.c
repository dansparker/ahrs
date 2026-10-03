/* Installation alignment: levelling and heading alignment with a tilted / turned board. */
#include <string.h>

#include "../core/ahrs.h"
#include "test.h"

static ahrs_t A;

/* aircraft at rest with attitude `ac` (rad), board turned by Rbs relative to the aircraft */
static void feed(ahrs_t* a, quat_t ac, const float Rbs[3][3], float seconds) {
    const float fn[3] = {0.0f, 0.0f, -GRAVITY};
    const float Bn[3] = {21.0f, 1.7f, 43.0f};
    float Rab[3][3], fb[3], fs[3], mb[3], ms[3];
    quat_to_dcm(ac, Rab);
    m3t_mul_v(Rab, fn, fb);
    m3t_mul_v(Rbs, fb, fs);
    m3t_mul_v(Rab, Bn, mb);
    m3t_mul_v(Rbs, mb, ms);
    const float gyro[3] = {0, 0, 0};
    const long n = (long)(seconds * 208.0f);
    for (long k = 0; k < n; ++k) {
        float acc[3], mag[3];
        for (int i = 0; i < 3; ++i) {
            acc[i] = fs[i] + 0.02f * randn();
            mag[i] = ms[i] + 0.2f * randn();
        }
        ahrs_imu(a, gyro, acc, 1.0f / 208.0f);
        if (k % 4 == 0) ahrs_mag(a, mag);
    }
}

static void board(float r, float p, float y, float R[3][3]) { quat_to_dcm(quat_from_euler(r * DEG2RAD, p * DEG2RAD, y * DEG2RAD), R); }

static void test_level(void) {
    ahrs_config_t cfg;
    ahrs_default_config(&cfg);
    ahrs_init(&A, &cfg);
    float Rbs[3][3];
    board(2.0f, -3.0f, 0.0f, Rbs);
    const quat_t level = quat_from_euler(0.0f, 0.0f, 40.0f * DEG2RAD);
    CHECK(ahrs_level_start(&A) == AHRS_ALIGN_NOT_READY);
    feed(&A, level, Rbs, 4.0f);
    ahrs_out_t o;
    ahrs_output(&A, &o);
    CHECK_NEAR(o.roll_deg, 2.0f, 0.2f); /* untrimmed: shows the board */
    CHECK_NEAR(o.pitch_deg, -3.0f, 0.2f);

    CHECK(ahrs_level_start(&A) == AHRS_ALIGN_OK);
    CHECK(ahrs_level_start(&A) == AHRS_ALIGN_BUSY);
    int st = -1;
    CHECK(ahrs_level_poll(&A, &st) == 0);
    feed(&A, level, Rbs, 2.5f);
    CHECK(ahrs_level_poll(&A, &st) == 1 && st == AHRS_ALIGN_OK);
    CHECK(ahrs_take_mount_changed(&A) == 1);
    CHECK_NEAR(A.mount_rpy[0], 2.0f, 0.05f);
    CHECK_NEAR(A.mount_rpy[1], -3.0f, 0.05f);
    feed(&A, level, Rbs, 4.0f); /* re-alignment */
    ahrs_output(&A, &o);
    CHECK(o.valid & AHRS_OUT_ATTITUDE);
    CHECK_NEAR(o.roll_deg, 0.0f, 0.1f);
    CHECK_NEAR(o.pitch_deg, 0.0f, 0.1f);
    /* after a power cycle with the stored offsets: aircraft pitched up 5 deg shows 5 deg */
    const float stored[3] = {A.mount_rpy[0], A.mount_rpy[1], A.mount_rpy[2]};
    ahrs_init(&A, &cfg);
    ahrs_set_mount(&A, stored);
    feed(&A, quat_from_euler(0.0f, 5.0f * DEG2RAD, 40.0f * DEG2RAD), Rbs, 4.0f);
    ahrs_output(&A, &o);
    CHECK_NEAR(o.pitch_deg, 5.0f, 0.15f);
    CHECK_NEAR(o.roll_deg, 0.0f, 0.15f);

    /* refused: too far off, not standing still */
    ahrs_init(&A, &cfg);
    board(15.0f, 0.0f, 0.0f, Rbs);
    feed(&A, level, Rbs, 4.0f);
    CHECK(ahrs_level_start(&A) == AHRS_ALIGN_OK);
    feed(&A, level, Rbs, 2.5f);
    CHECK(ahrs_level_poll(&A, &st) == 1 && st == AHRS_ALIGN_OUT_OF_RANGE);
    A.stationary = 0;
    CHECK(ahrs_level_start(&A) == AHRS_ALIGN_NOT_ON_GROUND);
}

static void test_heading_align(void) {
    ahrs_config_t cfg;
    ahrs_default_config(&cfg);
    ahrs_init(&A, &cfg);
    ahrs_declination(&A, atan2f(1.7f, 21.0f) * RAD2DEG); /* from the display */
    float Rbs[3][3];
    board(0.0f, 0.0f, 5.0f, Rbs); /* board turned 5 deg to the right */
    const quat_t ac = quat_from_euler(0.0f, 0.0f, 40.0f * DEG2RAD);
    const float mag_hdg = 40.0f - atan2f(1.7f, 21.0f) * RAD2DEG;
    feed(&A, ac, Rbs, 6.0f);
    ahrs_out_t o;
    ahrs_output(&A, &o);
    CHECK(o.valid & AHRS_OUT_HEADING);
    CHECK_NEAR(o.heading_mag_deg, mag_hdg + 5.0f, 0.5f);

    CHECK(ahrs_heading_align(&A, mag_hdg) == AHRS_ALIGN_OK);
    CHECK_NEAR(A.mount_rpy[2], 5.0f, 0.5f);
    feed(&A, ac, Rbs, 6.0f);
    ahrs_output(&A, &o);
    CHECK(o.valid & AHRS_OUT_HEADING);
    CHECK_NEAR(o.heading_mag_deg, mag_hdg, 0.5f);
    CHECK(ahrs_heading_align(&A, mag_hdg + 45.0f) == AHRS_ALIGN_OUT_OF_RANGE);

    const float zero[3] = {0, 0, 0};
    ahrs_set_mount(&A, zero);
    feed(&A, ac, Rbs, 6.0f);
    ahrs_output(&A, &o);
    CHECK_NEAR(o.heading_mag_deg, mag_hdg + 5.0f, 0.5f);
}

void run_align_tests(void) {
    RUN(test_level);
    RUN(test_heading_align);
}
