//////////////////////////////////////
//// User-modifiable parameters //////
//////////////////////////////////////
static const uint8_t PROGMEM initial_balls = 6;
static const uint8_t PROGMEM max_balls = 13;
constexpr bool initial_velocity = false;
constexpr bool collision_detection = true;
static const unsigned long PROGMEM lockout_duration_usec = 200000;
static const unsigned long PROGMEM tick_duration_usec = 16667; // 16667 == 60 fps
static const float PROGMEM gravity = 0.14f;
static const float PROGMEM collide_restitution = 0.89f;
static const float PROGMEM wall_restitution = -0.89f;
static const float PROGMEM progression = 0.95f;
//////////////////////////////////////

#include <SPI.h>

// irresponsible but effective
static const uint32_t PROGMEM SPI_RATE = 16000000;
static const uint8_t PROGMEM WRITE = 3;
static const uint8_t PROGMEM CLS = 6;
static const uint8_t PROGMEM BTN_PWR = 12;
static const uint8_t PROGMEM BTN_L = 13;
static const uint8_t PROGMEM BTN_R = 8;

// Accelerometer Calibration
static const int PROGMEM X_MIN = 263;
static const int PROGMEM X_MAX = 392;
static const int PROGMEM Y_MIN = 257;
static const int PROGMEM Y_MAX = 389;
static const float PROGMEM X_RNG = static_cast<float>(X_MAX - X_MIN) * 0.5f;
static const float PROGMEM Y_RNG = static_cast<float>(Y_MAX - Y_MIN) * 0.5f;

// each row of this table is a compacted representation of the uint32_t that
// must be masked in for each of 21 pixel rows to draw an r==10 circle.
//
// there are 8 rows in the table because there are 8 possible starting bits for a circle (x & 7).
static const uint32_t PROGMEM circ_mask[] = {
  0xFFFFC07F, 0xFFFF001F, 0xFFFE000F, 0xFFFC0007, 0xFFF80003, 0xFFF00001, 0xFFF00001, 0xFFE00000,
  0xFFFF80FF, 0xFFFE003F, 0xFFFC001F, 0xFFF8000F, 0xFFF00007, 0xFFE00003, 0xFFE00003, 0xFFC00001,
  0xFFFF01FF, 0xFFFC007F, 0xFFF8003F, 0xFFF0001F, 0xFFE0000F, 0xFFC00007, 0xFFC00007, 0xFF800003,
  0xFFFE03FF, 0xFFF800FF, 0xFFF0007F, 0xFFE0003F, 0xFFC0001F, 0xFF80000F, 0xFF80000F, 0xFF000007,
  0xFFFC07FF, 0xFFF001FF, 0xFFE000FF, 0xFFC0007F, 0xFF80003F, 0xFF00001F, 0xFF00001F, 0xFE00000F,
  0xFFF80FFF, 0xFFE003FF, 0xFFC001FF, 0xFF8000FF, 0xFF00007F, 0xFE00003F, 0xFE00003F, 0xFC00001F,
  0xFFF01FFF, 0xFFC007FF, 0xFF8003FF, 0xFF0001FF, 0xFE0000FF, 0xFC00007F, 0xFC00007F, 0xF800003F,
  0xFFE03FFF, 0xFF800FFF, 0xFF0007FF, 0xFE0003FF, 0xFC0001FF, 0xF80000FF, 0xF80000FF, 0xF000007F
};

// 96 * 96 pixels but only 1 bit per pixel so 96*96/8 bytes
uint8_t buf[1152];

void push() {
  digitalWrite(SS, HIGH);
  SPI.transfer(WRITE);
  // write 1 whole line of 96 pixels (12 bytes) at a time
  for (uint8_t i = 1; i <= 96; ++i) {
    SPI.transfer(i);
    SPI.transfer(buf + 12 * (i - 1), 12);
    SPI.transfer(0);
  }
  digitalWrite(SS, LOW);
}

void fillCircle(int16_t x0, int16_t y0) {
  const int16_t x = x0 - 10;
  const int16_t y = y0 - 10;
  uint32_t* const __restrict base = reinterpret_cast<uint32_t* __restrict const>(buf + 12 * y + (x >> 3));
  uint32_t* const __restrict circ_shft = circ_mask + ((x & 7) << 3);
  uint8_t i = 0;
  for (; i < 7; ++i) base[3 * i] &= pgm_read_dword(circ_shft + i);
  const uint32_t mid = pgm_read_dword(circ_shft + 7);
  for (; i < 14; ++i) base[3 * i] &= mid;
  for (; i < 21; ++i) base[3 * i] &= pgm_read_dword(circ_shft + 20 - i);
}

struct Ball {
  float x, y, vx, vy;
};

Ball balls[max_balls];

uint8_t cur_balls;

bool addBall() {
  Ball& b1 = balls[cur_balls];
  uint16_t j;
  for (uint16_t x = 0; x < 100; ++x) {
    b1.x = static_cast<float>(random(1100, 8501)) * 0.01f;
    b1.y = static_cast<float>(random(1100, 8501)) * 0.01f;
    for (j = 0; j < cur_balls; ++j) {
      Ball& b2 = balls[j];
      const float dx = b1.x - b2.x;
      const float dy = b1.y - b2.y;
      if (dx * dx + dy * dy < 400.0f) break;
    }
    if (j == cur_balls) break;
  }
  if (j != cur_balls) return false;
  b1.vx = initial_velocity ? static_cast<float>(random(-400, 401)) * 0.01f : 0.0f;
  b1.vy = initial_velocity ? static_cast<float>(random(-400, 401)) * 0.01f : 0.0f;
  ++cur_balls;
  return true;
}

void setup_balls() {
  cur_balls = 0;
  for (;;) {
    randomSeed(analogRead(0));
    randomSeed(analogRead(random(0, 16)));
    uint16_t i = 0;
    for (; i < initial_balls; ++i) {
      if (!addBall()) break;
    }
    if (i == initial_balls) break;
  }
}

void tick() {
  const float ax = -static_cast<float>(analogRead(0) - X_MIN - X_RNG) / X_RNG;
  const float ay = static_cast<float>(analogRead(1) - Y_MIN - Y_RNG) / Y_RNG;
  for (uint8_t i = 0; i < cur_balls; ++i) {
    Ball& b = balls[i];
    const float oldx = b.x;
    const float oldy = b.y;

    b.x += b.vx;
    b.y += b.vy;
    if (b.x > 85.0f) {
      b.x = 85.0f;
      b.vx *= wall_restitution;
      b.vy *= progression;
    }
    if (b.x < 10.0f) {
      b.x = 10.0f;
      b.vx *= wall_restitution;
      b.vy *= progression;
    }
    if (b.y > 85.0f) {
      b.y = 85.0f;
      b.vy *= wall_restitution;
      b.vx *= progression;
    }
    if (b.y < 10.0f) {
      b.y = 10.0f;
      b.vy *= wall_restitution;
      b.vx *= progression;
    }

    b.vx += gravity * ax;
    b.vy += gravity * ay;

    if (collision_detection) {
      for (uint8_t j = 0; j < cur_balls; ++j) {
        if (j == i) continue;
        Ball& b2 = balls[j];
        const float dx = b.x - b2.x;
        const float dy = b.y - b2.y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < 400.0f) {
          const float dvx = b.vx - b2.vx;
          const float dvy = b.vy - b2.vy;
          if (dvx * dx + dvy * dy < 0.0f) {
            const float c = (dvx * dx + dvy * dy) / d2;
            const float cx = c * dx;
            const float cy = c * dy;
            b.vx -= cx;
            b.vy -= cy;
            b2.vx += cx;
            b2.vy += cy;
            b.vx *= collide_restitution;
            b.vy *= collide_restitution;
            b2.vx *= collide_restitution;
            b2.vy *= collide_restitution;
            b.x = oldx;
            b.y = oldy;
          }
        }
      }
    }
  }
}

void paint() {
  memset(buf, 0xFF, 1152);
  for (uint8_t i = 0; i < cur_balls; ++i) {
    fillCircle(balls[i].x + 0.5f, balls[i].y + 0.5f);
  }
}

int main() {
  init();
  pinMode(BTN_PWR, OUTPUT);
  pinMode(BTN_L, INPUT);
  pinMode(BTN_R, INPUT);
  digitalWrite(BTN_PWR, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_RATE, LSBFIRST, SPI_MODE0));

  setup_balls();
  unsigned long lockout = micros() + lockout_duration_usec;
  unsigned long next_tick = micros() + tick_duration_usec;
  for (;;) {
    tick();
    paint();
    push();
    while (micros() < next_tick) {
      if (micros() > lockout) {
        const bool L = digitalRead(BTN_L);
        const bool R = digitalRead(BTN_R);
        if (L ^ R) {
          if (cur_balls > 0 && L) --cur_balls;
          if (cur_balls < max_balls && R) addBall();
          lockout = micros() + lockout_duration_usec;
        }
      }
    }
    next_tick += tick_duration_usec;
  }
}

