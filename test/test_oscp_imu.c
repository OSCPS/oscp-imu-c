/**
 * test_oscp_imu.c
 *
 * Test bench for oscp_imu.h / oscp_imu.c
 *
 * Framework: Unity v2.6.1
 *
 * Organized to mirror the library's transport split:
 *   SHARED       — struct layout, CRC, COBS, utilities
 *   TRANSPORT A  — byte-stream parser (RS-422 / UART)
 *   TRANSPORT B  — message decode (CAN-FD)
 *   EQUIVALENCE  — both transports must decode identical bytes identically
 *   COMMANDS     — encoding for both transports, enum validation, buffer sizing
 **/

/** Includes */

#include "unity.h"
#include "../oscp_imu.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Compile-time layout guards — these fail the BUILD, not the test run.
 *
 * _Static_assert is C11. The library targets C99, and MSVC's C compiler does not
 * enable it without /std:c11 (it parses as an unknown identifier: C2143). Clang
 * also warns -Wc11-extensions when it appears in C99 mode. So: use _Static_assert
 * where the compiler advertises C11, and fall back to the negative-array-size
 * trick everywhere else. Both fail at compile time; only the message differs.
 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define OSCP_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#  define OSCP_STATIC_ASSERT_CAT2(a, b) a##b
#  define OSCP_STATIC_ASSERT_CAT(a, b)  OSCP_STATIC_ASSERT_CAT2(a, b)
#  define OSCP_STATIC_ASSERT(cond, msg) \
       typedef char OSCP_STATIC_ASSERT_CAT(oscp_static_assert_line_, __LINE__)[(cond) ? 1 : -1]
#endif

OSCP_STATIC_ASSERT(sizeof(oscp_raw_t)     == 61, "oscp_raw_t must be 61 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_euler_t)   == 25, "oscp_euler_t must be 25 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_quat_t)    == 29, "oscp_quat_t must be 29 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_rot_mat_t) == 49, "oscp_rot_mat_t must be 49 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_gnss_t)    == 64, "oscp_gnss_t must be 64 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_debug_1_t) == 58, "oscp_debug_1_t must be 58 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_debug_2_t) == 58, "oscp_debug_2_t must be 58 bytes");
OSCP_STATIC_ASSERT(sizeof(oscp_startup_t) == 40, "oscp_startup_t must be 40 bytes");

/** Shared callback state */
static oscp_frame_t  g_last_frame;
static int           g_call_count;

static void frame_cb(const oscp_frame_t *frame, void *ctx) {
    (void)ctx;
    g_last_frame = *frame;
    g_call_count++;
}

/** setUp and tearDown */

void setUp(void) {
    memset(&g_last_frame, 0, sizeof(g_last_frame));
    g_call_count = 0;
}

void tearDown(void) {}

/* ==========================================================================
 * Byte-level writers
 * ========================================================================*/

static void write_u16(uint8_t *p, const uint16_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8u);
}

static void write_u32(uint8_t *p, const uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8u);
    p[2] = (uint8_t)(v >> 16u);
    p[3] = (uint8_t)(v >> 24u);
}

static void write_u64(uint8_t *p, const uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i));
}

static void write_f32(uint8_t *p, const float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    write_u32(p, u);
}

static void append_crc(uint8_t *buf, const size_t payload_len) {
    const uint16_t crc = oscp_crc16(buf, payload_len);
    buf[payload_len]     = (uint8_t)(crc & 0xFFu);
    buf[payload_len + 1] = (uint8_t)(crc >> 8u);
}

static uint8_t make_byte0(const oscp_frame_type_t type, const oscp_operating_mode_t mode, const uint8_t misalign) {
    return (uint8_t)((((uint8_t)misalign & 0x03u) << 6u) | (((uint8_t)mode & 0x07u) << 3u) | ((uint8_t)type & 0x07u));
}

/* ==========================================================================
 * Frame PAYLOAD builders (raw, CRC-terminated, pre-COBS).
 * These feed BOTH transports: the UART tests COBS-wrap them, the CAN-FD tests
 * hand them straight to oscp_frame_decode. Each returns the payload length.
 * ========================================================================*/

static size_t build_raw_payload(uint8_t *p, uint8_t counter, uint64_t ts_ms,
                                float gyro_x, float gyro_z, float accel_z,
                                float temp, uint8_t status) {
    memset(p, 0, 61);
    p[0] = make_byte0(OSCP_FRAME_RAW, OSCP_OP_MODE_MEDIUM, OSCP_MISALIGNMENT_CORR_ENABLED);
    p[1] = counter;
    write_u64(&p[2], ts_ms);
    write_f32(&p[10], gyro_x);
    write_f32(&p[14], 0.0f);
    write_f32(&p[18], gyro_z);
    write_f32(&p[22], 0.0f);
    write_f32(&p[26], 0.0f);
    write_f32(&p[30], accel_z);
    write_f32(&p[34], 0.0f);   /* incl_x */
    write_f32(&p[38], 0.0f);   /* incl_y */
    write_f32(&p[42], 0.0f);   /* mag_x  */
    write_f32(&p[46], 0.0f);   /* mag_y  */
    write_f32(&p[50], 0.0f);   /* mag_z  */
    write_f32(&p[54], temp);
    p[58] = status;
    append_crc(p, 59);
    return 61;
}

static size_t build_euler_payload(uint8_t *p, uint8_t counter, uint64_t ts_ms,
                                  float roll, float pitch, float yaw, uint8_t status) {
    memset(p, 0, 25);
    p[0] = make_byte0(OSCP_FRAME_EULER, OSCP_OP_MODE_LOW, 0);
    p[1] = counter;
    write_u64(&p[2], ts_ms);
    write_f32(&p[10], roll);
    write_f32(&p[14], pitch);
    write_f32(&p[18], yaw);
    p[22] = status;
    append_crc(p, 23);
    return 25;
}

static size_t build_quat_payload(uint8_t *p, uint8_t counter, uint64_t ts_ms,
                                 float w, float x, float y, float z, uint8_t status) {
    memset(p, 0, 29);
    p[0] = make_byte0(OSCP_FRAME_QUATERNION, OSCP_OP_MODE_MEDIUM, 0);
    p[1] = counter;
    write_u64(&p[2], ts_ms);
    write_f32(&p[10], w);
    write_f32(&p[14], x);
    write_f32(&p[18], y);
    write_f32(&p[22], z);
    p[26] = status;
    append_crc(p, 27);
    return 29;
}

/* Rotation matrix: rm[3][3] row-major at offset 10, 36 bytes. */
static size_t build_rot_mat_payload(uint8_t *p, uint8_t counter, uint64_t ts_ms,
                                    const float rm[3][3], uint8_t status) {
    memset(p, 0, 49);
    p[0] = make_byte0(OSCP_FRAME_ROT_MATRIX, OSCP_OP_MODE_MEDIUM, 0);
    p[1] = counter;
    write_u64(&p[2], ts_ms);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            write_f32(&p[10 + (r * 3 + c) * 4], rm[r][c]);
    p[46] = status;
    append_crc(p, 47);
    return 49;
}

static size_t build_gnss_payload(uint8_t *p, uint8_t counter, uint64_t ts_ms,
                                 uint8_t fix_type, uint8_t num_sats,
                                 float lon, float lat, int32_t height,
                                 int32_t vel_n, int32_t vel_e, int32_t vel_d,
                                 float pdop, uint8_t fix_ok, uint8_t status) {
    memset(p, 0, 64);
    p[0]  = make_byte0(OSCP_FRAME_GNSS, OSCP_OP_MODE_MEDIUM, 0);
    p[1]  = counter;
    write_u64(&p[2], ts_ms);
    p[10] = fix_type;
    p[11] = num_sats;
    write_f32(&p[12], lon);
    write_f32(&p[16], lat);
    write_u32(&p[20], (uint32_t)height);
    write_u32(&p[24], 1500u);          /* horizontal_accuracy */
    write_u32(&p[28], 2500u);          /* vertical_accuracy   */
    write_u32(&p[32], (uint32_t)vel_n);
    write_u32(&p[36], (uint32_t)vel_e);
    write_u32(&p[40], (uint32_t)vel_d);
    write_u32(&p[44], 300u);           /* speed_accuracy */
    write_f32(&p[48], 45.0f);          /* heading_of_motion */
    write_f32(&p[52], 1.5f);           /* heading_accuracy  */
    write_f32(&p[56], pdop);
    p[60] = (uint8_t)(fix_ok & 0x01u); /* gnssFixOk:1 | invalidLLH:1 | rsv:2 | lastCorrAge:4 */
    p[61] = status;
    append_crc(p, 62);
    return 64;
}

static size_t build_debug_1_payload(uint8_t *p, uint8_t counter, uint32_t gxb, uint32_t mzb, uint8_t status) {
    memset(p, 0, 58);
    p[0] = make_byte0(OSCP_FRAME_DEBUG_1, OSCP_OP_MODE_MEDIUM, 0);
    p[1] = counter;
    write_u32(&p[2],  gxb);   /* gxb */
    write_u32(&p[46], mzb);   /* mzb = 12th u32, offset 2 + 11*4 = 46 */
    p[50] = 0x00;             /* gyro filter bitfields  */
    p[51] = 0x00;             /* accel filter bitfields */
    write_u16(&p[52], 0u);    /* reserved_0 */
    p[54] = 0u;               /* reserved_1 */
    p[55] = status;
    append_crc(p, 56);
    return 58;
}

static size_t build_debug_2_payload(uint8_t *p, uint8_t counter, uint32_t mxx, uint32_t fusion_gain, uint8_t status) {
    memset(p, 0, 58);
    p[0] = make_byte0(OSCP_FRAME_DEBUG_2, OSCP_OP_MODE_MEDIUM, 0);
    p[1] = counter;
    write_u32(&p[2],  mxx);          /* mxx */
    write_u32(&p[38], fusion_gain);  /* fusionGain = 10th u32, offset 2 + 9*4 = 38 */
    p[54] = 0x00;                    /* fusionConvention:4 | fusionHeadingSource:4 */
    p[55] = status;
    append_crc(p, 56);
    return 58;
}

static size_t build_startup_payload(uint8_t *p, const char *mark, uint16_t unit,
                                    uint8_t maj, uint8_t min, uint8_t pat,
                                    uint8_t enabled_frames, uint8_t status) {
    memset(p, 0, 40);
    p[0] = make_byte0(OSCP_FRAME_STARTUP, OSCP_OP_MODE_LOW, 0);
    memcpy(&p[1], mark, 10);
    write_u16(&p[11], unit);
    p[13] = maj;
    p[14] = min;
    p[15] = pat;
    p[16] = enabled_frames;
    /* bytes 17-36: DR, filters, AHRS — left zeroed */
    p[37] = status;
    append_crc(p, 38);
    return 40;
}

/* ==========================================================================
 * Transport A (UART) helpers — COBS-wrap a payload and feed the parser
 * ========================================================================*/

static size_t cobs_wrap(const uint8_t *raw, size_t raw_len, uint8_t *wire, size_t wire_max) {
    size_t enc_len = 0;
    oscp_cobs_encode(raw, raw_len, wire, wire_max - 1u, &enc_len);
    wire[enc_len] = OSCP_FRAME_DELIM;
    return enc_len + 1u;
}

static size_t build_ascii_response(const char *ascii, uint8_t *wire, size_t wire_max) {
    size_t enc_len = 0;
    oscp_cobs_encode((const uint8_t *)ascii, strlen(ascii), wire, wire_max - 1u, &enc_len);
    wire[enc_len] = OSCP_FRAME_DELIM;
    return enc_len + 1u;
}

static void feed_bytes(oscp_parser_t *p, const uint8_t *wire, size_t wire_len) {
    /* Prepend a 0x00 so the parser syncs before the first frame */
    oscp_parser_feed(p, OSCP_FRAME_DELIM);
    oscp_parser_feed_buf(p, wire, wire_len);
}

/* Feed a raw payload through the full UART path (COBS-wrap + sync + feed). */
static void feed_payload(oscp_parser_t *p, const uint8_t *payload, size_t len) {
    uint8_t wire[128];
    const size_t wlen = cobs_wrap(payload, len, wire, sizeof(wire));
    feed_bytes(p, wire, wlen);
}

/* ==========================================================================
 * SHARED — CRC
 * ========================================================================*/

void test_crc_empty_buffer(void) {
    /* Empty input must return the init value 0xFFFF */
    const uint8_t dummy = 0;
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, oscp_crc16(&dummy, 0));
}

/* Golden vectors: these PIN the polynomial (0xD175) and init (0xFFFF).
 * A weaker "is non-zero and deterministic" check would still pass if the
 * polynomial silently changed — these will not. */
void test_crc_golden_check_string(void) {
    /* "123456789" — the conventional CRC check string */
    const uint8_t d[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_HEX16(0x9DB1, oscp_crc16(d, sizeof(d)));
}

void test_crc_golden_abc(void) {
    const uint8_t d[] = {0x41, 0x42, 0x43};
    TEST_ASSERT_EQUAL_HEX16(0x5C67, oscp_crc16(d, sizeof(d)));
}

void test_crc_golden_single_zero(void) {
    const uint8_t d[] = {0x00};
    TEST_ASSERT_EQUAL_HEX16(0xF950, oscp_crc16(d, sizeof(d)));
}

/**
 * Independent bitwise CRC-16 reference.
 *
 * NOTE ON THE POLYNOMIAL: the library documents "polynomial 0xD175", which is
 * the KOOPMAN representation. The normal (MSB-first) form used by a textbook
 * shift-register implementation is (0xD175 << 1) | 1 == 0xA2EB. Using 0xD175
 * directly in the routine below produces different, wrong CRCs.
 */
#define OSCP_CRC_POLY_NORMAL 0xA2EBu

static uint16_t crc16_bitwise_ref(const uint8_t *d, const size_t n) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)d[i] << 8u));
        for (int b = 0; b < 8; b++) {
            const uint16_t shifted = (uint16_t)(crc << 1u);
            crc = (crc & 0x8000u) ? (uint16_t)(shifted ^ (uint16_t)OSCP_CRC_POLY_NORMAL)
                                  : shifted;
        }
    }
    return crc;
}

/**
 * Cross-check the 256-entry lookup table against the bitwise reference.
 *
 * Single-byte inputs exercise EVERY LUT entry: with init 0xFFFF the table index
 * is (crc >> 8) ^ byte == 0xFF ^ byte, which ranges over 0..255 as byte does.
 * The golden vectors above only reach ~13 of the 256 entries, so a corrupted
 * entry would otherwise slip through undetected.
 */
void test_crc_lut_matches_bitwise_reference_all_entries(void) {
    for (int b = 0; b < 256; b++) {
        const uint8_t x = (uint8_t)b;
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(crc16_bitwise_ref(&x, 1), oscp_crc16(&x, 1),
            "CRC LUT entry disagrees with bitwise reference");
    }
}

/* Multi-byte cross-check over deterministic pseudo-random buffers. */
void test_crc_lut_matches_bitwise_reference_multibyte(void) {
    uint8_t buf[64];
    uint32_t seed = 12345u;
    for (int trial = 0; trial < 200; trial++) {
        for (int i = 0; i < 64; i++) {
            seed = seed * 1103515245u + 12345u;
            buf[i] = (uint8_t)(seed >> 16);
        }
        TEST_ASSERT_EQUAL_HEX16(crc16_bitwise_ref(buf, sizeof(buf)), oscp_crc16(buf, sizeof(buf)));
    }
}

void test_crc_single_bit_flip_detected(void) {
    uint8_t data[] = {0x08, 0x01, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00,
                      0x00, 0x00, 0x3F, 0x80, 0x00, 0x00};
    const uint16_t crc1 = oscp_crc16(data, sizeof(data));
    data[0] ^= 0x01u;
    TEST_ASSERT_NOT_EQUAL(crc1, oscp_crc16(data, sizeof(data)));
}

/* Every single-bit flip across a full frame must change the CRC. */
void test_crc_all_single_bit_flips_detected(void) {
    uint8_t p[61];
    build_raw_payload(p, 0x11, 4242ULL, 1.0f, 2.0f, 3.0f, 25.0f, OSCP_STATUS_OK);
    const uint16_t good = oscp_crc16(p, 59);

    for (size_t byte = 0; byte < 59; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            p[byte] ^= (uint8_t)(1u << bit);
            TEST_ASSERT_NOT_EQUAL_MESSAGE(good, oscp_crc16(p, 59), "bit flip not detected");
            p[byte] ^= (uint8_t)(1u << bit); /* restore */
        }
    }
}

void test_crc_endianness_little_endian(void) {
    uint8_t frame[5] = {0x08, 0x01, 0x00, 0x00, 0x00};
    append_crc(frame, 3);
    const uint16_t stored_le = (uint16_t)(frame[3] | ((uint16_t)frame[4] << 8));
    TEST_ASSERT_EQUAL_HEX16(oscp_crc16(frame, 3), stored_le);
}

/* ==========================================================================
 * SHARED — COBS
 * ========================================================================*/

static void assert_cobs_roundtrip(const uint8_t *orig, const size_t len) {
    uint8_t enc[256], dec[256];
    size_t enc_len = 0, dec_len = 0;

    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cobs_encode(orig, len, enc, sizeof(enc), &enc_len));
    for (size_t i = 0; i < enc_len; i++) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, enc[i], "0x00 found inside COBS-encoded stream");
    }
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cobs_decode(enc, enc_len, dec, sizeof(dec), &dec_len));
    TEST_ASSERT_EQUAL(len, dec_len);
    TEST_ASSERT_EQUAL_MEMORY(orig, dec, len);
}

void test_cobs_roundtrip_no_zeros(void) {
    const uint8_t d[] = {0x11, 0x22, 0x33, 0x44};
    assert_cobs_roundtrip(d, sizeof(d));
}

void test_cobs_roundtrip_single_zero(void) {
    const uint8_t d[] = {0x00};
    assert_cobs_roundtrip(d, sizeof(d));
}

void test_cobs_roundtrip_all_zeros(void) {
    const uint8_t d[8] = {0};
    assert_cobs_roundtrip(d, sizeof(d));
}

void test_cobs_roundtrip_mixed(void) {
    const uint8_t d[] = {0x00, 0xAB, 0x00, 0xCD, 0x00};
    assert_cobs_roundtrip(d, sizeof(d));
}

void test_cobs_roundtrip_realistic_raw_payload(void) {
    uint8_t p[61];
    build_raw_payload(p, 0x2A, 1000ULL, -9.81f, 0.0f, 1.0f, 30.0f, OSCP_STATUS_OK);
    assert_cobs_roundtrip(p, 61);
}

/* Every OSCP frame length must survive a COBS round-trip within the parser's
 * buffer budget (OSCP_FRAME_MAX_LEN). Guards against a future frame growing
 * past what the parser can hold once COBS overhead is added. */
void test_cobs_max_frame_fits_parser_buffer(void) {
    uint8_t worst[64];
    memset(worst, 0x00, sizeof(worst)); /* all-zero = worst case COBS overhead */
    uint8_t enc[128];
    size_t enc_len = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cobs_encode(worst, sizeof(worst), enc, sizeof(enc), &enc_len));
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(OSCP_FRAME_MAX_LEN, enc_len,
        "worst-case COBS-encoded frame exceeds OSCP_FRAME_MAX_LEN");
}

/* ==========================================================================
 * TRANSPORT A — Byte-stream parser (RS-422 / UART)
 * ========================================================================*/

void test_parser_raw_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[61];
    const size_t len = build_raw_payload(payload, 0x2A, 1000ULL, 1.5f, -0.5f, -1.0f, 38.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_RAW, g_last_frame.type);

    const oscp_raw_t *r = &oscp_raw(&g_last_frame);
    TEST_ASSERT_EQUAL_UINT8(0x2A, r->counter);
    TEST_ASSERT_EQUAL_UINT64(1000, r->timestamp_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f,  1.5f, r->gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, r->gyro_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, r->accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 38.0f, r->temp);
    TEST_ASSERT_EQUAL_UINT8(OSCP_STATUS_OK, r->status);
}

void test_parser_euler_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[25];
    const size_t len = build_euler_payload(payload, 0x01, 500ULL, 10.0f, -5.0f, 270.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, g_last_frame.type);

    const oscp_euler_t *e = &oscp_euler(&g_last_frame);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f,  10.0f, e->roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f,  -5.0f, e->pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 270.0f, e->yaw);
}

void test_parser_quat_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    const size_t len = build_quat_payload(payload, 0x07, 2000ULL, 0.9239f, 0.3827f, 0.0f, 0.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION, g_last_frame.type);

    const oscp_quat_t *q = &oscp_quat(&g_last_frame);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9239f, q->w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3827f, q->x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f,    q->y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f,    q->z);
}

/* Rotation matrix was previously untested. Verifies row-major element order. */
void test_parser_rot_mat_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    /* Distinct values so a transposed decode would fail. */
    const float rm[3][3] = {{1.0f, 2.0f, 3.0f},
                            {4.0f, 5.0f, 6.0f},
                            {7.0f, 8.0f, 9.0f}};
    uint8_t payload[49];
    const size_t len = build_rot_mat_payload(payload, 0x03, 300ULL, rm, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_ROT_MATRIX, g_last_frame.type);

    const oscp_rot_mat_t *m = &oscp_rot_mat(&g_last_frame);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, rm[r][c], m->rm[r][c]);
}

/* GNSS was previously untested — covers int32 velocities and the bitfield byte. */
void test_parser_gnss_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[64];
    const size_t len = build_gnss_payload(payload, 0x09, 7000ULL,
                                          3, 12, -73.5673f, 45.5017f, 36000,
                                          1200, -800, 50, 1.8f, 1, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_GNSS, g_last_frame.type);

    const oscp_gnss_t *g = &oscp_gnss(&g_last_frame);
    TEST_ASSERT_EQUAL_UINT8(3,  g->gnss_fix_type);
    TEST_ASSERT_EQUAL_UINT8(12, g->num_satellites);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -73.5673f, g->longitude);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  45.5017f, g->latitude);
    TEST_ASSERT_EQUAL_INT32(36000, g->height);
    TEST_ASSERT_EQUAL_INT32( 1200, g->velocity_north);
    TEST_ASSERT_EQUAL_INT32( -800, g->velocity_east);   /* negative int32 */
    TEST_ASSERT_EQUAL_INT32(   50, g->velocity_down);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.8f, g->pdop);
    TEST_ASSERT_EQUAL_UINT8(1, g->gnssFixOk);
    TEST_ASSERT_EQUAL_UINT8(0, g->invalidLLH);
}

void test_parser_debug_1_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[58];
    const size_t len = build_debug_1_payload(payload, 0x0D, 0xDEADBEEFu, 0xCAFEBABEu, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_DEBUG_1, g_last_frame.type);

    const oscp_debug_1_t *d = &oscp_debug_1(&g_last_frame);
    TEST_ASSERT_EQUAL_UINT8(0x0D, d->counter);
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, d->gxb);
    TEST_ASSERT_EQUAL_HEX32(0xCAFEBABEu, d->mzb);
}

void test_parser_debug_2_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[58];
    const size_t len = build_debug_2_payload(payload, 0x0E, 0x11223344u, 0x55667788u, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_DEBUG_2, g_last_frame.type);

    const oscp_debug_2_t *d = &oscp_debug_2(&g_last_frame);
    TEST_ASSERT_EQUAL_UINT8(0x0E, d->counter);
    TEST_ASSERT_EQUAL_HEX32(0x11223344u, d->mxx);
    TEST_ASSERT_EQUAL_HEX32(0x55667788u, d->fusionGain);
}

void test_parser_startup_frame_decoded(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[40];
    const size_t len = build_startup_payload(payload, "OSCP-MK2E2", 42, 1, 2, 3,
                                             OSCP_ENABLE_RAW | OSCP_ENABLE_QUAT, OSCP_STATUS_OK);
    feed_payload(&p, payload, len);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_STARTUP, g_last_frame.type);

    const oscp_startup_t *s = &oscp_startup(&g_last_frame);
    TEST_ASSERT_EQUAL_UINT16(42, s->unit_number);
    TEST_ASSERT_EQUAL_UINT8(1, s->sw_major_ver);
    TEST_ASSERT_EQUAL_UINT8(2, s->sw_minor_ver);
    TEST_ASSERT_EQUAL_UINT8(3, s->sw_patch_ver);
    TEST_ASSERT_EQUAL_UINT8(OSCP_ENABLE_RAW | OSCP_ENABLE_QUAT, s->enabled_frames);
    TEST_ASSERT_EQUAL_STRING_LEN("OSCP-MK2E2", s->mark_number, 10);
}

void test_parser_header_accessors(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    build_quat_payload(payload, 0, 0ULL, 1.0f, 0, 0, 0, OSCP_STATUS_OK);
    /* MEDIUM (0b010) + misalign 0b01 -> byte0 = 0b01_010_010 */
    payload[0] = make_byte0(OSCP_FRAME_QUATERNION, OSCP_OP_MODE_MEDIUM, OSCP_MISALIGNMENT_CORR_ENABLED);
    append_crc(payload, 27);
    feed_payload(&p, payload, 29);

    TEST_ASSERT_EQUAL(1, g_call_count);
    const oscp_quat_t q = oscp_quat(&g_last_frame);
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION,           OSCP_HDR_FRAME_TYPE(q));
    TEST_ASSERT_EQUAL(OSCP_OP_MODE_MEDIUM,             OSCP_HDR_OPERATING_MODE(q));
    TEST_ASSERT_EQUAL(OSCP_MISALIGNMENT_CORR_ENABLED,  OSCP_HDR_MISALIGNMENT_CORR(q));
}

void test_parser_tagged_union_type_is_clean_enum(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    build_quat_payload(payload, 0x00, 0ULL, 1.0f, 0.0f, 0.0f, 0.0f, OSCP_STATUS_OK);
    payload[0] = make_byte0(OSCP_FRAME_QUATERNION, OSCP_OP_MODE_MEDIUM, 1); /* 0x52 */
    append_crc(payload, 27);
    feed_payload(&p, payload, 29);

    TEST_ASSERT_EQUAL(1, g_call_count);
    /* type must be the masked enum (2), not the raw header byte (0x52) */
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION, g_last_frame.type);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, oscp_quat(&g_last_frame).w);
}

/* --- Parser error paths --- */

void test_parser_crc_error_increments_counter(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[61];
    build_raw_payload(payload, 0x01, 100ULL, 0, 0, 0, 0, OSCP_STATUS_OK);
    payload[5] ^= 0xFFu; /* corrupt after CRC was appended */
    feed_payload(&p, payload, 61);

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->crc_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->frames_ok);
}

void test_parser_framing_error_wrong_length(void) {
    /* Quaternion type (2) but padded to raw length (61) -> expected 29, got 61 */
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[61];
    memset(payload, 0, sizeof(payload));
    payload[0] = make_byte0(OSCP_FRAME_QUATERNION, OSCP_OP_MODE_LOW, 0);
    append_crc(payload, 59);
    feed_payload(&p, payload, 61);

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->framing_errors);
}

void test_parser_framing_error_too_short(void) {
    /* Below OSCP_FRAME_MIN_LEN -> framing error, not a crash */
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[8];
    memset(payload, 0x01, sizeof(payload));
    payload[0] = make_byte0(OSCP_FRAME_EULER, OSCP_OP_MODE_LOW, 0);
    feed_payload(&p, payload, sizeof(payload));

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->framing_errors);
}

void test_parser_cobs_error_increments_counter(void) {
    /* A COBS overhead byte pointing past the end of the frame is invalid. */
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    const uint8_t bad[] = {0xFF, 0x01, 0x02};  /* overhead 0xFF but only 2 bytes follow */
    feed_bytes(&p, bad, sizeof(bad));
    oscp_parser_feed(&p, OSCP_FRAME_DELIM);

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->cobs_errors);
}

void test_parser_no_callback_before_first_delimiter(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    const uint8_t garbage[] = {0x04, 0xAB, 0xCD, 0xEF};
    oscp_parser_feed_buf(&p, garbage, sizeof(garbage));

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->frames_ok);
}

void test_parser_overflow_increments_counter(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    oscp_parser_feed(&p, OSCP_FRAME_DELIM);
    for (size_t i = 0; i < OSCP_FRAME_MAX_LEN + 1u; i++) {
        oscp_parser_feed(&p, 0x01u);
    }

    TEST_ASSERT_GREATER_THAN_UINT32(0u, oscp_parser_stats(&p)->overflows);
    TEST_ASSERT_EQUAL(0, g_call_count);
}

/* After an overflow the parser must resync and decode the NEXT frame cleanly. */
void test_parser_recovers_after_overflow(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    oscp_parser_feed(&p, OSCP_FRAME_DELIM);
    for (size_t i = 0; i < OSCP_FRAME_MAX_LEN + 1u; i++) oscp_parser_feed(&p, 0x01u);
    TEST_ASSERT_GREATER_THAN_UINT32(0u, oscp_parser_stats(&p)->overflows);

    uint8_t payload[25];
    build_euler_payload(payload, 0x42, 1ULL, 1.0f, 2.0f, 3.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, g_last_frame.type);
    TEST_ASSERT_EQUAL_UINT8(0x42, oscp_euler(&g_last_frame).counter);
}

/* After a CRC error the parser must still decode the NEXT frame. */
void test_parser_recovers_after_crc_error(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t bad[25];
    build_euler_payload(bad, 0x01, 1ULL, 0, 0, 0, OSCP_STATUS_OK);
    bad[3] ^= 0xFFu;
    feed_payload(&p, bad, 25);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->crc_errors);

    uint8_t good[25];
    build_euler_payload(good, 0x02, 2ULL, 7.0f, 8.0f, 9.0f, OSCP_STATUS_OK);
    feed_payload(&p, good, 25);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL_UINT8(0x02, oscp_euler(&g_last_frame).counter);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->frames_ok);
}

void test_parser_reset_clears_partial_frame(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    /* Sync, then feed half a frame */
    oscp_parser_feed(&p, OSCP_FRAME_DELIM);
    const uint8_t partial[] = {0x05, 0x06, 0x07};
    oscp_parser_feed_buf(&p, partial, sizeof(partial));

    /* Reset drops the partial frame and de-syncs */
    oscp_parser_reset(&p);

    /* Stats survive the reset */
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->frames_ok);

    /* Parser re-syncs and decodes the next complete frame */
    uint8_t payload[25];
    build_euler_payload(payload, 0x77, 5ULL, 1.0f, 1.0f, 1.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL_UINT8(0x77, oscp_euler(&g_last_frame).counter);
}

void test_parser_null_callback_is_safe(void) {
    /* cb == NULL must not crash; stats must still advance. */
    oscp_parser_t p;
    oscp_parser_init(&p, NULL, NULL);

    uint8_t payload[25];
    build_euler_payload(payload, 0x01, 1ULL, 1.0f, 2.0f, 3.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);

    TEST_ASSERT_EQUAL(0, g_call_count); /* our cb was never registered */
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->frames_ok);
}

/* File-scope: ISO C forbids nested functions (GCC extension only). */
static void ctx_probe_cb(const oscp_frame_t *f, void *ctx) {
    (void)f;
    *(int *)ctx = 0xDEF;
}

void test_parser_ctx_is_forwarded(void) {
    int marker = 0xABC;
    oscp_parser_t p;

    oscp_parser_init(&p, ctx_probe_cb, &marker);

    uint8_t payload[25];
    build_euler_payload(payload, 0x01, 1ULL, 0, 0, 0, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);

    TEST_ASSERT_EQUAL_HEX32(0xDEF, marker);
}

/* --- Parser streaming behaviour --- */

void test_parser_byte_by_byte_quat(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    build_quat_payload(payload, 0xBB, 9999ULL, 0.7071f, 0.7071f, 0.0f, 0.0f, OSCP_STATUS_OK);

    uint8_t wire[64];
    const size_t wlen = cobs_wrap(payload, 29, wire, sizeof(wire));

    oscp_parser_feed(&p, OSCP_FRAME_DELIM);
    for (size_t i = 0; i < wlen; i++) oscp_parser_feed(&p, wire[i]);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION, g_last_frame.type);
    TEST_ASSERT_EQUAL_UINT8(0xBB, oscp_quat(&g_last_frame).counter);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.7071f, oscp_quat(&g_last_frame).w);
}

/* feed() one byte at a time and feed_buf() in bulk must be equivalent. */
void test_parser_feed_buf_equals_feed_byte(void) {
    uint8_t payload[29];
    build_quat_payload(payload, 0x55, 77ULL, 0.5f, 0.5f, 0.5f, 0.5f, OSCP_STATUS_OK);
    uint8_t wire[64];
    const size_t wlen = cobs_wrap(payload, 29, wire, sizeof(wire));

    oscp_parser_t p1, p2;
    oscp_parser_init(&p1, frame_cb, NULL);
    oscp_parser_init(&p2, frame_cb, NULL);

    oscp_parser_feed(&p1, OSCP_FRAME_DELIM);
    for (size_t i = 0; i < wlen; i++) oscp_parser_feed(&p1, wire[i]);
    const oscp_frame_t byte_wise = g_last_frame;

    oscp_parser_feed(&p2, OSCP_FRAME_DELIM);
    oscp_parser_feed_buf(&p2, wire, wlen);
    const oscp_frame_t buf_wise = g_last_frame;

    TEST_ASSERT_EQUAL(2, g_call_count);
    TEST_ASSERT_EQUAL_MEMORY(&byte_wise, &buf_wise, sizeof(oscp_frame_t));
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p1)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p2)->frames_ok);
}

void test_parser_two_consecutive_frames(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t raw[61], quat[29];
    build_raw_payload(raw, 0x01, 100ULL, 0, 0, -1.0f, 25.0f, OSCP_STATUS_OK);
    build_quat_payload(quat, 0x02, 200ULL, 1.0f, 0, 0, 0, OSCP_STATUS_OK);

    uint8_t wire[256];
    size_t total = 0;
    total += cobs_wrap(raw,  61, wire + total, sizeof(wire) - total);
    total += cobs_wrap(quat, 29, wire + total, sizeof(wire) - total);
    feed_bytes(&p, wire, total);

    TEST_ASSERT_EQUAL(2, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(2u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION, g_last_frame.type);
}

void test_parser_back_to_back_delimiters_ignored(void) {
    /* Empty frames (0x00 0x00) must not produce callbacks or errors. */
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    for (int i = 0; i < 5; i++) oscp_parser_feed(&p, OSCP_FRAME_DELIM);

    uint8_t payload[25];
    build_euler_payload(payload, 0x09, 9ULL, 1.0f, 1.0f, 1.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->framing_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->cobs_errors);
}

void test_parser_status_byte_mems_error(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    build_quat_payload(payload, 0x00, 0ULL, 1.0f, 0, 0, 0, OSCP_STATUS_MEMS_ERR);
    feed_payload(&p, payload, 29);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_BITS(OSCP_STATUS_MEMS_ERR, OSCP_STATUS_MEMS_ERR, oscp_quat(&g_last_frame).status);
}

void test_parser_status_byte_multiple_flags(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    const uint8_t st = OSCP_STATUS_OVERRUN | OSCP_STATUS_OG_ERR;
    uint8_t payload[29];
    build_quat_payload(payload, 0x00, 0ULL, 1.0f, 0, 0, 0, st);
    feed_payload(&p, payload, 29);

    TEST_ASSERT_EQUAL(1, g_call_count);
    const uint8_t got = oscp_quat(&g_last_frame).status;
    TEST_ASSERT_BITS_HIGH(OSCP_STATUS_OVERRUN, got);
    TEST_ASSERT_BITS_HIGH(OSCP_STATUS_OG_ERR,  got);
    TEST_ASSERT_BITS_LOW(OSCP_STATUS_MEMS_ERR, got);
}

void test_parser_status_byte_ok_is_zero(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[29];
    build_quat_payload(payload, 0x00, 0ULL, 1.0f, 0, 0, 0, OSCP_STATUS_OK);
    feed_payload(&p, payload, 29);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL_UINT8(OSCP_STATUS_OK, oscp_quat(&g_last_frame).status);
}

void test_parser_stats_frames_ok(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);
    oscp_parser_feed(&p, OSCP_FRAME_DELIM);

    for (int i = 0; i < 5; i++) {
        uint8_t payload[29];
        build_quat_payload(payload, (uint8_t)i, (uint64_t)(i * 10), 1.0f, 0, 0, 0, OSCP_STATUS_OK);
        uint8_t wire[64];
        const size_t wlen = cobs_wrap(payload, 29, wire, sizeof(wire));
        oscp_parser_feed_buf(&p, wire, wlen);
    }

    TEST_ASSERT_EQUAL(5, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(5u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->crc_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->cobs_errors);
}

/* --- Command response frames (UART only: ASCII over COBS) --- */

void test_parser_cmd_success_delivered(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("OSCP-MK2M2: Command succeed.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_SUCCESS, g_last_frame.type);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->framing_errors);
}

void test_parser_cmd_failed_delivered(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("OSCP-MK2E2: Command failed.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_FAILED, g_last_frame.type);
}

void test_parser_cmd_unknown_delivered(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("OSCP-MK2M2: This command is erroneous.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_UNKNOWN, g_last_frame.type);
}

void test_parser_cmd_not_impl_delivered(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("OSCP-MK2E2: This command is not implemented yet.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_NOT_IMPL, g_last_frame.type);
}

void test_parser_cmd_success_suffix_only_matches(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("Command succeed.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_SUCCESS, g_last_frame.type);
}

/* A response whose suffix does not match any table entry must not be delivered. */
void test_parser_unrecognised_ascii_not_delivered(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t wire[64];
    const size_t wlen = build_ascii_response("Some other text.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);

    TEST_ASSERT_EQUAL(0, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(1u, oscp_parser_stats(&p)->framing_errors);
}

void test_parser_full_session(void) {
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);

    uint8_t payload[25], wire[128];
    size_t wlen;

    build_euler_payload(payload, 0x01, 100ULL, 1.0f, 2.0f, 3.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);
    TEST_ASSERT_EQUAL(1, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, g_last_frame.type);

    wlen = build_ascii_response("OSCP-MK2M2: Command succeed.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);
    TEST_ASSERT_EQUAL(2, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_SUCCESS, g_last_frame.type);

    wlen = build_ascii_response("OSCP-MK2M2: Command failed.\r\n", wire, sizeof(wire));
    feed_bytes(&p, wire, wlen);
    TEST_ASSERT_EQUAL(3, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_CMD_FAILED, g_last_frame.type);

    build_euler_payload(payload, 0x02, 200ULL, 4.0f, 5.0f, 6.0f, OSCP_STATUS_OK);
    feed_payload(&p, payload, 25);
    TEST_ASSERT_EQUAL(4, g_call_count);
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, g_last_frame.type);

    TEST_ASSERT_EQUAL_UINT32(4u, oscp_parser_stats(&p)->frames_ok);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->crc_errors);
    TEST_ASSERT_EQUAL_UINT32(0u, oscp_parser_stats(&p)->framing_errors);
}

/* ==========================================================================
 * TRANSPORT B — Message decode (CAN-FD)
 *
 * No parser, no COBS. Payloads may be padded up to a DLC bucket; the CRC must
 * be checked over the frame length only, never over the padding.
 * ========================================================================*/

/* Copy a payload into a CAN-FD message buffer and fill the padding with noise,
 * so any decoder that reads past the frame length will be caught. */
static void pad_to_dlc(const uint8_t *payload, size_t len, uint8_t *msg, size_t dlc) {
    memset(msg, 0xAA, dlc);
    memcpy(msg, payload, len);
}

void test_can_decode_euler_exact_length(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 0x2A, 123456ULL, 1.5f, -2.5f, 90.0f, OSCP_STATUS_OK);

    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(payload, 25, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, f.type);
    TEST_ASSERT_EQUAL_UINT8(0x2A, oscp_euler(&f).counter);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, oscp_euler(&f).yaw);
}

/* THE critical CAN-FD case: 25-byte frame inside a 32-byte DLC bucket. */
void test_can_decode_euler_with_dlc_padding(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 0x2A, 123456ULL, 1.5f, -2.5f, 90.0f, OSCP_STATUS_OK);

    uint8_t msg[32];
    pad_to_dlc(payload, 25, msg, sizeof(msg));

    oscp_frame_t f;
    TEST_ASSERT_EQUAL_MESSAGE(OSCP_OK, oscp_frame_decode(msg, 32, &f),
        "DLC-padded frame must decode; CRC must cover the frame only");
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, f.type);
    TEST_ASSERT_EQUAL_UINT8(0x2A, oscp_euler(&f).counter);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.5f,  oscp_euler(&f).roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.5f, oscp_euler(&f).pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f, oscp_euler(&f).yaw);
}

/* Padding content must be irrelevant: 0x00 padding and 0xAA padding decode alike. */
void test_can_decode_padding_content_irrelevant(void) {
    uint8_t payload[29];
    build_quat_payload(payload, 0x11, 7ULL, 1.0f, 0.0f, 0.0f, 0.0f, OSCP_STATUS_OK);

    uint8_t msg_a[32], msg_b[32];
    memset(msg_a, 0x00, sizeof(msg_a)); memcpy(msg_a, payload, 29);
    memset(msg_b, 0xFF, sizeof(msg_b)); memcpy(msg_b, payload, 29);

    oscp_frame_t fa, fb;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(msg_a, 32, &fa));
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(msg_b, 32, &fb));
    TEST_ASSERT_EQUAL_MEMORY(&fa, &fb, sizeof(oscp_frame_t));
}

void test_can_decode_raw_in_64_byte_message(void) {
    uint8_t payload[61];
    build_raw_payload(payload, 0x33, 555ULL, 1.0f, -1.0f, 9.81f, 42.0f, OSCP_STATUS_OK);

    uint8_t msg[64];
    pad_to_dlc(payload, 61, msg, sizeof(msg));

    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(msg, 64, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_RAW, f.type);
    TEST_ASSERT_EQUAL_UINT64(555, oscp_raw(&f).timestamp_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.81f, oscp_raw(&f).accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 42.0f, oscp_raw(&f).temp);
}

void test_can_decode_gnss_full_64(void) {
    /* GNSS is exactly 64 bytes: no padding possible. */
    uint8_t payload[64];
    build_gnss_payload(payload, 0x04, 8000ULL, 3, 9, -73.0f, 45.0f, 1000,
                       10, -20, 30, 2.1f, 1, OSCP_STATUS_OK);

    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(payload, 64, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_GNSS, f.type);
    TEST_ASSERT_EQUAL_INT32(-20, oscp_gnss(&f).velocity_east);
    TEST_ASSERT_EQUAL_UINT8(9, oscp_gnss(&f).num_satellites);
}

void test_can_decode_all_frame_types(void) {
    uint8_t p[64];
    oscp_frame_t f;

    build_raw_payload(p, 1, 1ULL, 0, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 61, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_RAW, f.type);

    build_euler_payload(p, 1, 1ULL, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 25, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_EULER, f.type);

    build_quat_payload(p, 1, 1ULL, 1.0f, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 29, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_QUATERNION, f.type);

    const float rm[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    build_rot_mat_payload(p, 1, 1ULL, rm, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 49, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_ROT_MATRIX, f.type);

    build_gnss_payload(p, 1, 1ULL, 3, 5, 0, 0, 0, 0, 0, 0, 1.0f, 1, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 64, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_GNSS, f.type);

    build_debug_1_payload(p, 1, 0xAAu, 0xBBu, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 58, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_DEBUG_1, f.type);

    build_debug_2_payload(p, 1, 0xCCu, 0xDDu, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 58, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_DEBUG_2, f.type);

    build_startup_payload(p, "OSCP-MK2M2", 1, 0, 1, 0, OSCP_ENABLE_RAW, 0);
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_frame_decode(p, 40, &f));
    TEST_ASSERT_EQUAL(OSCP_FRAME_STARTUP, f.type);
}

/* --- CAN-FD error paths --- */

void test_can_decode_rejects_truncated(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 1, 1ULL, 0, 0, 0, 0);

    oscp_frame_t f;
    /* len < frame length -> reject, never read past the buffer */
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(payload, 24, &f));
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(payload, 10, &f));
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(payload,  1, &f));
}

void test_can_decode_rejects_zero_length(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 1, 1ULL, 0, 0, 0, 0);
    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(payload, 0, &f));
}

void test_can_decode_rejects_bad_crc(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 1, 1ULL, 1.0f, 2.0f, 3.0f, 0);
    payload[5] ^= 0xFFu;

    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(payload, 25, &f));
}

/* Corruption anywhere in the frame body must be caught, padding must not matter. */
void test_can_decode_rejects_bad_crc_under_padding(void) {
    uint8_t payload[29];
    build_quat_payload(payload, 1, 1ULL, 1.0f, 0, 0, 0, 0);
    uint8_t msg[32];
    pad_to_dlc(payload, 29, msg, sizeof(msg));
    msg[3] ^= 0x01u; /* corrupt inside the frame, not the padding */

    oscp_frame_t f;
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(msg, 32, &f));
}

void test_can_decode_null_args(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 1, 1ULL, 0, 0, 0, 0);
    oscp_frame_t f;

    TEST_ASSERT_EQUAL(OSCP_ERR_NULL, oscp_frame_decode(NULL, 25, &f));
    TEST_ASSERT_EQUAL(OSCP_ERR_NULL, oscp_frame_decode(payload, 25, NULL));
}

/* oscp_frame_decode must not touch COBS: a COBS-encoded buffer should fail. */
void test_can_decode_does_not_expect_cobs(void) {
    uint8_t payload[25];
    build_euler_payload(payload, 1, 1ULL, 0, 0, 0, 0);

    uint8_t wire[64];
    const size_t wlen = cobs_wrap(payload, 25, wire, sizeof(wire));

    oscp_frame_t f;
    /* COBS-encoded bytes are not a valid raw frame -> CRC (or type) rejects it */
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_frame_decode(wire, wlen, &f));
}

/* ==========================================================================
 * EQUIVALENCE — both transports share dispatch_frame, so identical payload
 * bytes must produce byte-identical oscp_frame_t on either path.
 * ========================================================================*/

static void assert_transports_agree(const uint8_t *payload, size_t len) {
    /* Path A: UART parser (COBS-wrapped) */
    oscp_parser_t p;
    oscp_parser_init(&p, frame_cb, NULL);
    feed_payload(&p, payload, len);
    TEST_ASSERT_EQUAL_MESSAGE(1, g_call_count, "UART path did not deliver a frame");
    const oscp_frame_t via_uart = g_last_frame;

    /* Path B: CAN-FD direct decode, with padding to prove it is ignored */
    uint8_t msg[72];
    memset(msg, 0xA5, sizeof(msg));
    memcpy(msg, payload, len);
    oscp_frame_t via_can;
    TEST_ASSERT_EQUAL_MESSAGE(OSCP_OK, oscp_frame_decode(msg, sizeof(msg), &via_can),
        "CAN path did not decode the frame");

    TEST_ASSERT_EQUAL_MESSAGE(via_uart.type, via_can.type, "frame type differs between transports");
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(&via_uart, &via_can, sizeof(oscp_frame_t),
        "decoded frame differs between UART and CAN-FD");
}

void test_equivalence_raw(void) {
    uint8_t p[61];
    build_raw_payload(p, 0x7F, 987654ULL, -1.25f, 2.5f, -9.81f, 21.5f, OSCP_STATUS_OVERRUN);
    assert_transports_agree(p, 61);
}

void test_equivalence_euler(void) {
    uint8_t p[25];
    build_euler_payload(p, 0x08, 4321ULL, -179.9f, 89.9f, 359.9f, OSCP_STATUS_OK);
    assert_transports_agree(p, 25);
}

void test_equivalence_quat(void) {
    uint8_t p[29];
    build_quat_payload(p, 0x09, 1ULL, 0.7071f, 0.0f, 0.7071f, 0.0f, OSCP_STATUS_OK);
    assert_transports_agree(p, 29);
}

void test_equivalence_startup(void) {
    uint8_t p[40];
    build_startup_payload(p, "OSCP-MK2Z1", 1234, 9, 8, 7, OSCP_ENABLE_GNSS, OSCP_STATUS_OK);
    assert_transports_agree(p, 40);
}

/* ==========================================================================
 * COMMANDS
 * ========================================================================*/

/* UART: COBS-encoded, delimiter-terminated, no interior 0x00. */
static void assert_uart_cmd(const uint8_t *buf, size_t len, const char *expected_ascii) {
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(OSCP_FRAME_DELIM, buf[len - 1u], "missing 0x00 delimiter");
    for (size_t i = 0; i < len - 1u; i++) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, buf[i], "0x00 inside COBS-encoded command");
    }
    uint8_t decoded[64];
    size_t dec_len = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cobs_decode(buf, len - 1u, decoded, sizeof(decoded), &dec_len));
    TEST_ASSERT_EQUAL_size_t(strlen(expected_ascii), dec_len);
    TEST_ASSERT_EQUAL_MEMORY(expected_ascii, decoded, dec_len);
}

/* CAN: raw ASCII, no COBS, no delimiter. */
static void assert_can_cmd(const uint8_t *buf, size_t len, const char *expected_ascii) {
    TEST_ASSERT_EQUAL_size_t_MESSAGE(strlen(expected_ascii), len, "CAN command length mismatch");
    TEST_ASSERT_EQUAL_MEMORY(expected_ascii, buf, len);
}

/* Every command must encode correctly on BOTH transports. */
static void assert_cmd_both(oscp_err_t (*fn)(uint8_t *, size_t, size_t *, oscp_transport_t),
                            const char *ascii) {
    uint8_t buf[32];
    size_t n = 0;

    TEST_ASSERT_EQUAL(OSCP_OK, fn(buf, sizeof(buf), &n, OSCP_TRANSPORT_RS422));
    assert_uart_cmd(buf, n, ascii);

    n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, fn(buf, sizeof(buf), &n, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, ascii);
}

/* --- Nullary commands, both transports --- */

void test_cmd_exit(void)           { assert_cmd_both(oscp_cmd_exit,           "EXIT\r\n"); }
void test_cmd_reset(void)          { assert_cmd_both(oscp_cmd_reset,          "RESET\r\n"); }
void test_cmd_config(void)         { assert_cmd_both(oscp_cmd_config,         "CONFIG\r\n"); }
void test_cmd_suf(void)            { assert_cmd_both(oscp_cmd_suf,            "SUF\r\n"); }
void test_cmd_enable_mcorr(void)   { assert_cmd_both(oscp_cmd_enable_mcorr,   "EMCORR\r\n"); }
void test_cmd_disable_mcorr(void)  { assert_cmd_both(oscp_cmd_disable_mcorr,  "DMCORR\r\n"); }
void test_cmd_save(void)           { assert_cmd_both(oscp_cmd_save,           "SAVE\r\n"); }
void test_cmd_refs(void)           { assert_cmd_both(oscp_cmd_refs,           "REFS\r\n"); }

/* --- Parameterized commands --- */

void test_cmd_of_all_frame_selectors(void) {
    uint8_t buf[32];
    size_t n = 0;

    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_RAW, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFR\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_EULER, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFE\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_QUATERNION, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFQ\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_ROT_MATRIX, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFM\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_GNSS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFG\r\n");
    /* DEBUG is valid for OF (one-shot) */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_DEBUG, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OFD\r\n");

    /* UART framing too */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_of(buf, sizeof(buf), &n, OSCP_FRAME_SEL_RAW, OSCP_TRANSPORT_RS422));
    assert_uart_cmd(buf, n, "OFR\r\n");
}

void test_cmd_of_rejects_invalid_selector(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_of(buf, sizeof(buf), &n, (oscp_frame_sel_t)'Z', OSCP_TRANSPORT_CANFD));
}

void test_cmd_om_all_modes(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_om(buf, sizeof(buf), &n, OSCP_OM_IDLE, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OMI\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_om(buf, sizeof(buf), &n, OSCP_OM_LOW, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OML\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_om(buf, sizeof(buf), &n, OSCP_OM_MEDIUM, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "OMM\r\n");
}

void test_cmd_om_rejects_invalid_mode(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_om(buf, sizeof(buf), &n, (oscp_om_sel_t)'X', OSCP_TRANSPORT_CANFD));
}

void test_cmd_enable_oft(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_enable_oft(buf, sizeof(buf), &n, OSCP_FRAME_SEL_RAW, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "EOFTR\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_enable_oft(buf, sizeof(buf), &n, OSCP_FRAME_SEL_GNSS, OSCP_TRANSPORT_RS422));
    assert_uart_cmd(buf, n, "EOFTG\r\n");
}

void test_cmd_disable_oft(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_disable_oft(buf, sizeof(buf), &n, OSCP_FRAME_SEL_EULER, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DOFTE\r\n");
}

/* Debug frames cannot be persistently enabled/disabled — only one-shot via OF. */
void test_cmd_oft_rejects_debug_selector(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_enable_oft(buf, sizeof(buf), &n, OSCP_FRAME_SEL_DEBUG, OSCP_TRANSPORT_CANFD));
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_disable_oft(buf, sizeof(buf), &n, OSCP_FRAME_SEL_DEBUG, OSCP_TRANSPORT_CANFD));
}

void test_cmd_drg_all_ranges_zero_padded(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_125DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG0125\r\n");   /* zero-padded to 4 digits */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_250DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG0250\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_500DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG0500\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_1000DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG1000\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_2000DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG2000\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_drg(buf, sizeof(buf), &n, OSCP_GYRO_DR_4000DPS, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRG4000\r\n");   /* widest: exercises the exact buffer */
}

void test_cmd_drg_rejects_invalid_range(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_drg(buf, sizeof(buf), &n, (oscp_gyro_dr_t)999, OSCP_TRANSPORT_CANFD));
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_drg(buf, sizeof(buf), &n, (oscp_gyro_dr_t)0,   OSCP_TRANSPORT_CANFD));
}

void test_cmd_dra_all_ranges_zero_padded(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dra(buf, sizeof(buf), &n, OSCP_ACCEL_DR_2G, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRA02\r\n");     /* zero-padded to 2 digits */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dra(buf, sizeof(buf), &n, OSCP_ACCEL_DR_4G, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRA04\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dra(buf, sizeof(buf), &n, OSCP_ACCEL_DR_8G, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRA08\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dra(buf, sizeof(buf), &n, OSCP_ACCEL_DR_16G, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRA16\r\n");     /* widest */
}

void test_cmd_dra_rejects_invalid_range(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_dra(buf, sizeof(buf), &n, (oscp_accel_dr_t)32, OSCP_TRANSPORT_CANFD));
}

void test_cmd_dri_all_ranges_decimal_format(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dri(buf, sizeof(buf), &n, OSCP_INCL_DR_0G5, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRI0.5\r\n");    /* tenths -> "0.5" */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dri(buf, sizeof(buf), &n, OSCP_INCL_DR_1G0, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRI1.0\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dri(buf, sizeof(buf), &n, OSCP_INCL_DR_2G0, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRI2.0\r\n");
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_dri(buf, sizeof(buf), &n, OSCP_INCL_DR_3G0, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "DRI3.0\r\n");    /* widest */
}

void test_cmd_dri_rejects_invalid_range(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_dri(buf, sizeof(buf), &n, (oscp_incl_dr_t)7, OSCP_TRANSPORT_CANFD));
}

void test_cmd_wr_hex_formatting(void) {
    uint8_t buf[32];
    size_t n = 0;

    /* 8 uppercase hex digits, zero-padded */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, OSCP_USR_REG_GXB, 0x2Au, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "WRGXB0000002A\r\n");

    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, OSCP_USR_REG_GXB, 0u, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "WRGXB00000000\r\n");

    /* widest value: all F, exercises the exact 16-byte buffer */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, OSCP_USR_REG_AHP, 0xFFFFFFFFu, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "WRAHPFFFFFFFF\r\n");

    /* lowercase must NOT appear */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, OSCP_USR_REG_FGA, 0xDEADBEEFu, OSCP_TRANSPORT_CANFD));
    assert_can_cmd(buf, n, "WRFGADEADBEEF\r\n");

    /* UART framing */
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, OSCP_USR_REG_MZZ, 0x12345678u, OSCP_TRANSPORT_RS422));
    assert_uart_cmd(buf, n, "WRMZZ12345678\r\n");
}

/* Spot-check the register mnemonic table at both ends and a few interior points. */
void test_cmd_wr_register_mnemonics(void) {
    uint8_t buf[32];
    size_t n = 0;
    const struct { oscp_usr_reg_t reg; const char *ascii; } cases[] = {
        { OSCP_USR_REG_GXB, "WRGXB00000001\r\n" },  /* first  */
        { OSCP_USR_REG_GOB, "WRGOB00000001\r\n" },
        { OSCP_USR_REG_IYB, "WRIYB00000001\r\n" },
        { OSCP_USR_REG_MZZ, "WRMZZ00000001\r\n" },
        { OSCP_USR_REG_FCO, "WRFCO00000001\r\n" },
        { OSCP_USR_REG_FMR, "WRFMR00000001\r\n" },
        { OSCP_USR_REG_GFI, "WRGFI00000001\r\n" },
        { OSCP_USR_REG_AHP, "WRAHP00000001\r\n" },  /* last   */
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_wr(buf, sizeof(buf), &n, cases[i].reg, 1u, OSCP_TRANSPORT_CANFD));
        assert_can_cmd(buf, n, cases[i].ascii);
    }
}

void test_cmd_wr_rejects_invalid_register(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_INVALID, oscp_cmd_wr(buf, sizeof(buf), &n, (oscp_usr_reg_t)99, 0u, OSCP_TRANSPORT_CANFD));
}

/* --- Command argument guards --- */

void test_cmd_null_args_returns_error(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR_NULL, oscp_cmd_config(NULL, sizeof(buf), &n,   OSCP_TRANSPORT_RS422));
    TEST_ASSERT_EQUAL(OSCP_ERR_NULL, oscp_cmd_config(buf,  sizeof(buf), NULL, OSCP_TRANSPORT_RS422));
    TEST_ASSERT_EQUAL(OSCP_ERR_NULL, oscp_cmd_config(NULL, sizeof(buf), &n,   OSCP_TRANSPORT_CANFD));
}

/* Zero-length buffer must not underflow (enc_max_len - 1 would wrap to SIZE_MAX). */
void test_cmd_zero_length_buffer_returns_error(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_cmd_config(buf, 0, &n, OSCP_TRANSPORT_RS422));
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_cmd_config(buf, 0, &n, OSCP_TRANSPORT_CANFD));
}

void test_cmd_too_small_buffer_returns_error(void) {
    uint8_t buf[4];
    size_t n = 0;
    /* "CONFIG\r\n" is 8 bytes of ASCII; 4 bytes cannot hold it on either transport */
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_cmd_config(buf, sizeof(buf), &n, OSCP_TRANSPORT_CANFD));
    TEST_ASSERT_EQUAL(OSCP_ERR, oscp_cmd_config(buf, sizeof(buf), &n, OSCP_TRANSPORT_RS422));
}

/* A CAN-encoded command is raw ASCII: it must contain no COBS delimiter. */
void test_cmd_can_has_no_delimiter(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_config(buf, sizeof(buf), &n, OSCP_TRANSPORT_CANFD));
    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(OSCP_FRAME_DELIM, buf[i], "CAN command must not be COBS-framed");
    }
}

/* A UART-encoded command must round-trip back through the parser's response
 * matcher when echoed — sanity check that our COBS framing is self-consistent. */
void test_cmd_uart_roundtrips_through_cobs(void) {
    uint8_t buf[32];
    size_t n = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cmd_config(buf, sizeof(buf), &n, OSCP_TRANSPORT_RS422));

    uint8_t dec[64];
    size_t dec_len = 0;
    TEST_ASSERT_EQUAL(OSCP_OK, oscp_cobs_decode(buf, n - 1u, dec, sizeof(dec), &dec_len));
    TEST_ASSERT_EQUAL_size_t(strlen("CONFIG\r\n"), dec_len);
    TEST_ASSERT_EQUAL_MEMORY("CONFIG\r\n", dec, dec_len);
}

/** Main */
int main(void) {
    UNITY_BEGIN();

    /* SHARED — CRC */
    RUN_TEST(test_crc_empty_buffer);
    RUN_TEST(test_crc_golden_check_string);
    RUN_TEST(test_crc_golden_abc);
    RUN_TEST(test_crc_golden_single_zero);
    RUN_TEST(test_crc_lut_matches_bitwise_reference_all_entries);
    RUN_TEST(test_crc_lut_matches_bitwise_reference_multibyte);
    RUN_TEST(test_crc_single_bit_flip_detected);
    RUN_TEST(test_crc_all_single_bit_flips_detected);
    RUN_TEST(test_crc_endianness_little_endian);

    /* SHARED — COBS */
    RUN_TEST(test_cobs_roundtrip_no_zeros);
    RUN_TEST(test_cobs_roundtrip_single_zero);
    RUN_TEST(test_cobs_roundtrip_all_zeros);
    RUN_TEST(test_cobs_roundtrip_mixed);
    RUN_TEST(test_cobs_roundtrip_realistic_raw_payload);
    RUN_TEST(test_cobs_max_frame_fits_parser_buffer);

    /* TRANSPORT A — parser: frame decoding */
    RUN_TEST(test_parser_raw_frame_decoded);
    RUN_TEST(test_parser_euler_frame_decoded);
    RUN_TEST(test_parser_quat_frame_decoded);
    RUN_TEST(test_parser_rot_mat_frame_decoded);
    RUN_TEST(test_parser_gnss_frame_decoded);
    RUN_TEST(test_parser_debug_1_frame_decoded);
    RUN_TEST(test_parser_debug_2_frame_decoded);
    RUN_TEST(test_parser_startup_frame_decoded);
    RUN_TEST(test_parser_header_accessors);
    RUN_TEST(test_parser_tagged_union_type_is_clean_enum);

    /* TRANSPORT A — parser: error paths and recovery */
    RUN_TEST(test_parser_crc_error_increments_counter);
    RUN_TEST(test_parser_framing_error_wrong_length);
    RUN_TEST(test_parser_framing_error_too_short);
    RUN_TEST(test_parser_cobs_error_increments_counter);
    RUN_TEST(test_parser_no_callback_before_first_delimiter);
    RUN_TEST(test_parser_overflow_increments_counter);
    RUN_TEST(test_parser_recovers_after_overflow);
    RUN_TEST(test_parser_recovers_after_crc_error);
    RUN_TEST(test_parser_reset_clears_partial_frame);
    RUN_TEST(test_parser_null_callback_is_safe);
    RUN_TEST(test_parser_ctx_is_forwarded);

    /* TRANSPORT A — parser: streaming */
    RUN_TEST(test_parser_byte_by_byte_quat);
    RUN_TEST(test_parser_feed_buf_equals_feed_byte);
    RUN_TEST(test_parser_two_consecutive_frames);
    RUN_TEST(test_parser_back_to_back_delimiters_ignored);

    /* TRANSPORT A — parser: status byte */
    RUN_TEST(test_parser_status_byte_mems_error);
    RUN_TEST(test_parser_status_byte_multiple_flags);
    RUN_TEST(test_parser_status_byte_ok_is_zero);
    RUN_TEST(test_parser_stats_frames_ok);

    /* TRANSPORT A — parser: command responses */
    RUN_TEST(test_parser_cmd_success_delivered);
    RUN_TEST(test_parser_cmd_failed_delivered);
    RUN_TEST(test_parser_cmd_unknown_delivered);
    RUN_TEST(test_parser_cmd_not_impl_delivered);
    RUN_TEST(test_parser_cmd_success_suffix_only_matches);
    RUN_TEST(test_parser_unrecognised_ascii_not_delivered);
    RUN_TEST(test_parser_full_session);

    /* TRANSPORT B — CAN-FD message decode */
    RUN_TEST(test_can_decode_euler_exact_length);
    RUN_TEST(test_can_decode_euler_with_dlc_padding);
    RUN_TEST(test_can_decode_padding_content_irrelevant);
    RUN_TEST(test_can_decode_raw_in_64_byte_message);
    RUN_TEST(test_can_decode_gnss_full_64);
    RUN_TEST(test_can_decode_all_frame_types);
    RUN_TEST(test_can_decode_rejects_truncated);
    RUN_TEST(test_can_decode_rejects_zero_length);
    RUN_TEST(test_can_decode_rejects_bad_crc);
    RUN_TEST(test_can_decode_rejects_bad_crc_under_padding);
    RUN_TEST(test_can_decode_null_args);
    RUN_TEST(test_can_decode_does_not_expect_cobs);

    /* EQUIVALENCE — UART and CAN-FD must agree */
    RUN_TEST(test_equivalence_raw);
    RUN_TEST(test_equivalence_euler);
    RUN_TEST(test_equivalence_quat);
    RUN_TEST(test_equivalence_startup);

    /* COMMANDS — nullary, both transports */
    RUN_TEST(test_cmd_exit);
    RUN_TEST(test_cmd_reset);
    RUN_TEST(test_cmd_config);
    RUN_TEST(test_cmd_suf);
    RUN_TEST(test_cmd_enable_mcorr);
    RUN_TEST(test_cmd_disable_mcorr);
    RUN_TEST(test_cmd_save);
    RUN_TEST(test_cmd_refs);

    /* COMMANDS — parameterized */
    RUN_TEST(test_cmd_of_all_frame_selectors);
    RUN_TEST(test_cmd_of_rejects_invalid_selector);
    RUN_TEST(test_cmd_om_all_modes);
    RUN_TEST(test_cmd_om_rejects_invalid_mode);
    RUN_TEST(test_cmd_enable_oft);
    RUN_TEST(test_cmd_disable_oft);
    RUN_TEST(test_cmd_oft_rejects_debug_selector);
    RUN_TEST(test_cmd_drg_all_ranges_zero_padded);
    RUN_TEST(test_cmd_drg_rejects_invalid_range);
    RUN_TEST(test_cmd_dra_all_ranges_zero_padded);
    RUN_TEST(test_cmd_dra_rejects_invalid_range);
    RUN_TEST(test_cmd_dri_all_ranges_decimal_format);
    RUN_TEST(test_cmd_dri_rejects_invalid_range);
    RUN_TEST(test_cmd_wr_hex_formatting);
    RUN_TEST(test_cmd_wr_register_mnemonics);
    RUN_TEST(test_cmd_wr_rejects_invalid_register);

    /* COMMANDS — argument guards and framing invariants */
    RUN_TEST(test_cmd_null_args_returns_error);
    RUN_TEST(test_cmd_zero_length_buffer_returns_error);
    RUN_TEST(test_cmd_too_small_buffer_returns_error);
    RUN_TEST(test_cmd_can_has_no_delimiter);
    RUN_TEST(test_cmd_uart_roundtrips_through_cobs);

    return UNITY_END();
}
