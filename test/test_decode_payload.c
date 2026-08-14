/*
 * Payload-decoder segment tests.
 *
 * The validation suite only ever feeds well-formed symbols that this project
 * generated itself, so it cannot reach either of the defects covered here:
 * both need a symbol that passes Reed-Solomon and *then* carries a bit pattern
 * a real encoder would never emit.
 *
 *   - decode_alpha() indexed a 45-entry table with raw 6-bit (0..63) and
 *     11-bit (0..2047) fields.  Values 45..63 read past the array into
 *     adjacent .rodata and copied it into the payload; 2025..2047 landed on
 *     the NUL terminator and spliced an embedded NUL into a buffer callers
 *     treat as a C string.
 *
 *   - decode_payload() assigned data_type per segment instead of accumulating
 *     it.  A QR code may mix modes, so [Kanji][byte] reported BYTE and slipped
 *     past callers that reject Kanji by testing that field.
 *
 * Driving decode_payload() directly - rather than building images - keeps the
 * cases exact and deterministic: the bitstream under test is the one written
 * here, with no encoder, ECC or optics in between.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Pulled in as source so the static segment decoders are reachable.  The
 * build must therefore not also link k_quirc_decode.c into this target. */
#include "k_quirc_decode.c"

/* Version 1 keeps every count field at its narrow width: 9 bits for
 * alphanumeric, 8 for byte and Kanji.  See decode_alpha/byte/kanji. */
#define TEST_VERSION 1

#define MODE_NUMERIC K_QUIRC_DATA_TYPE_NUMERIC
#define MODE_ALPHA K_QUIRC_DATA_TYPE_ALPHA
#define MODE_BYTE K_QUIRC_DATA_TYPE_BYTE
#define MODE_KANJI K_QUIRC_DATA_TYPE_KANJI
#define MODE_ECI 7

static int failures;

static void fail(const char *what, const char *fmt, ...) {
  va_list ap;

  fprintf(stderr, "%s: ", what);
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  failures++;
}

/*
 * Bit writer - MSB first, matching take_bits().
 */
typedef struct {
  uint8_t bytes[K_QUIRC_MAX_PAYLOAD];
  int len;
} bitbuf_t;

static void put_bits(bitbuf_t *b, unsigned value, int count) {
  for (int i = count - 1; i >= 0; i--) {
    if (value & (1u << i))
      b->bytes[b->len >> 3] |= (uint8_t)(0x80 >> (b->len & 7));
    b->len++;
  }
}

/* Run a hand-built bitstream through the payload decoder. */
static k_quirc_error_t decode(const bitbuf_t *b, struct quirc_data *data) {
  static struct datastream ds;

  memset(&ds, 0, sizeof(ds));
  memcpy(ds.data, b->bytes, (size_t)((b->len + 7) / 8));
  ds.data_bits = b->len;

  memset(data, 0, sizeof(*data));
  data->version = TEST_VERSION;

  return decode_payload(data, &ds);
}

/*
 * PR303-001 - alphanumeric range checks.
 *
 * Odd character counts end in a 6-bit field; even ones pack two characters
 * into 11 bits.  Both are checked at the boundary and beyond it.
 */
typedef void (*alpha_builder_t)(bitbuf_t *, unsigned);

static void alpha_tail(bitbuf_t *b, unsigned d) {
  put_bits(b, MODE_ALPHA, 4);
  put_bits(b, 1, 9); /* one character */
  put_bits(b, d, 6);
}

static void alpha_pair(bitbuf_t *b, unsigned d) {
  put_bits(b, MODE_ALPHA, 4);
  put_bits(b, 2, 9); /* two characters */
  put_bits(b, d, 11);
}

static void check_alpha_accepted(const char *what, alpha_builder_t build,
                                 unsigned d, const char *expect) {
  bitbuf_t b = {{0}, 0};
  struct quirc_data data;
  const int want_len = (int)strlen(expect);

  build(&b, d);
  const k_quirc_error_t err = decode(&b, &data);

  if (err != K_QUIRC_SUCCESS) {
    fail(what, "err %d (%s), expected success", err, k_quirc_strerror(err));
    return;
  }
  if (data.payload_len != want_len) {
    fail(what, "payload_len %d, expected %d", data.payload_len, want_len);
    return;
  }
  if (memcmp(data.payload, expect, (size_t)want_len))
    fail(what, "payload \"%s\", expected \"%s\"", (const char *)data.payload,
         expect);
}

static void check_alpha_rejected(const char *what, alpha_builder_t build,
                                 unsigned d) {
  bitbuf_t b = {{0}, 0};
  struct quirc_data data;

  build(&b, d);
  const k_quirc_error_t err = decode(&b, &data);

  if (err != K_QUIRC_ERROR_INVALID_SYMBOL)
    fail(what, "err %d (%s), expected INVALID_SYMBOL", err,
         k_quirc_strerror(err));
}

static void test_alpha_bounds(void) {
  /* alpha_map[44] is ':', the last entry; [45] is the NUL terminator. */
  check_alpha_accepted("alpha 6-bit d=44", alpha_tail, 44, ":");
  check_alpha_rejected("alpha 6-bit d=45", alpha_tail, 45);
  check_alpha_rejected("alpha 6-bit d=63", alpha_tail, 63);

  /* 44*45 + 44 == 2024 is the largest encodable pair. */
  check_alpha_accepted("alpha 11-bit d=0", alpha_pair, 0, "00");
  check_alpha_accepted("alpha 11-bit d=2024", alpha_pair, 2024, "::");
  check_alpha_rejected("alpha 11-bit d=2025", alpha_pair, 2025);
  check_alpha_rejected("alpha 11-bit d=2047", alpha_pair, 2047);
}

/*
 * PR303-007 - data_type must be the OR of every segment mode present.
 */
static void put_kanji(bitbuf_t *b) {
  put_bits(b, MODE_KANJI, 4);
  put_bits(b, 1, 8);  /* one character */
  put_bits(b, 0, 13); /* decodes to Shift-JIS 0x8140 */
}

static void put_byte(bitbuf_t *b) {
  put_bits(b, MODE_BYTE, 4);
  put_bits(b, 1, 8); /* one byte */
  put_bits(b, 'A', 8);
}

static void put_numeric(bitbuf_t *b) {
  put_bits(b, MODE_NUMERIC, 4);
  put_bits(b, 1, 10); /* one digit */
  put_bits(b, 7, 4);
}

static void put_alpha(bitbuf_t *b) {
  put_bits(b, MODE_ALPHA, 4);
  put_bits(b, 1, 9);
  put_bits(b, 10, 6); /* 'A' */
}

static void check_data_type(const char *what, const bitbuf_t *b, int expect) {
  struct quirc_data data;
  const k_quirc_error_t err = decode(b, &data);

  if (err != K_QUIRC_SUCCESS) {
    fail(what, "err %d (%s), expected success", err, k_quirc_strerror(err));
    return;
  }
  if (data.data_type != expect)
    fail(what, "data_type %d, expected %d", data.data_type, expect);
}

static void test_data_type_accumulates(void) {
  bitbuf_t b;

  /* Single-mode symbols keep reporting exactly their own mode. */
  b = (bitbuf_t){{0}, 0};
  put_numeric(&b);
  check_data_type("numeric only", &b, MODE_NUMERIC);

  b = (bitbuf_t){{0}, 0};
  put_alpha(&b);
  check_data_type("alpha only", &b, MODE_ALPHA);

  b = (bitbuf_t){{0}, 0};
  put_byte(&b);
  check_data_type("byte only", &b, MODE_BYTE);

  b = (bitbuf_t){{0}, 0};
  put_kanji(&b);
  check_data_type("kanji only", &b, MODE_KANJI);

  /* The regressions: a trailing segment must not hide a leading one, and a
   * leading segment must not hide a trailing one. */
  b = (bitbuf_t){{0}, 0};
  put_kanji(&b);
  put_byte(&b);
  check_data_type("kanji then byte", &b, MODE_KANJI | MODE_BYTE);

  b = (bitbuf_t){{0}, 0};
  put_byte(&b);
  put_kanji(&b);
  check_data_type("byte then kanji", &b, MODE_BYTE | MODE_KANJI);

  b = (bitbuf_t){{0}, 0};
  put_numeric(&b);
  put_alpha(&b);
  check_data_type("numeric then alpha", &b, MODE_NUMERIC | MODE_ALPHA);

  b = (bitbuf_t){{0}, 0};
  put_kanji(&b);
  put_byte(&b);
  put_numeric(&b);
  check_data_type("kanji, byte, numeric", &b,
                  MODE_KANJI | MODE_BYTE | MODE_NUMERIC);
}

/* ECI is mode 7, an encoding declaration rather than a segment mode, and must
 * stay out of the mask - otherwise it would read as NUMERIC|ALPHA|BYTE. */
static void test_eci_excluded(void) {
  bitbuf_t b = {{0}, 0};
  struct quirc_data data;

  put_bits(&b, MODE_ECI, 4);
  put_bits(&b, 26, 8); /* ECI 26 - UTF-8 */
  put_byte(&b);

  const k_quirc_error_t err = decode(&b, &data);
  if (err != K_QUIRC_SUCCESS) {
    fail("eci then byte", "err %d (%s), expected success", err,
         k_quirc_strerror(err));
    return;
  }
  if (data.data_type != MODE_BYTE)
    fail("eci then byte", "data_type %d, expected %d", data.data_type,
         MODE_BYTE);
  if (data.eci != 26)
    fail("eci then byte", "eci %u, expected 26", data.eci);
}

/* A payload that reaches the end of the data without a terminator must still
 * succeed - erroring on the unknown-mode default would break symbols whose
 * data exactly fills the capacity. */
static void test_no_terminator(void) {
  bitbuf_t b = {{0}, 0};
  struct quirc_data data;

  put_byte(&b);

  const k_quirc_error_t err = decode(&b, &data);
  if (err != K_QUIRC_SUCCESS)
    fail("byte without terminator", "err %d (%s), expected success", err,
         k_quirc_strerror(err));
  else if (data.payload_len != 1 || data.payload[0] != 'A')
    fail("byte without terminator", "payload_len %d, expected 1",
         data.payload_len);
}

int main(void) {
  test_alpha_bounds();
  test_data_type_accumulates();
  test_eci_excluded();
  test_no_terminator();

  if (failures) {
    fprintf(stderr, "decode payload: %d failure(s)\n", failures);
    return 1;
  }
  printf("decode payload: alphanumeric bounds and segment-mode mask OK\n");
  return 0;
}
