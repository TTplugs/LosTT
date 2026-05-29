#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define LOS_TT_URI "urn:ttlab:lv2:lostt"
#define TWO_PI 6.28318530717958647692f
#define MAX_DELAY 16384
#define DELAY_MASK (MAX_DELAY - 1)

enum PortIndex {
    PORT_IN_L = 0,
    PORT_IN_R = 1,
    PORT_OUT_L = 2,
    PORT_OUT_R = 3,
    PORT_MODEL = 4,
    PORT_WOW = 5,
    PORT_FLUTTER = 6,
    PORT_SATURATE = 7,
    PORT_FAILURE = 8,
    PORT_VOLUME = 9,
    PORT_DRY = 10,
    PORT_NOISE = 11,
    PORT_AUX_MODE = 12,
    PORT_AUX_ACTIVE = 13,
    PORT_AUX_ONSET = 14,
    PORT_SPREAD = 15,
    PORT_DRY_TYPE = 16,
    PORT_DROP_BYPASS = 17,
    PORT_SNAG_BYPASS = 18,
    PORT_HUM_BYPASS = 19,
    PORT_HISS_LEVEL = 20,
    PORT_MECH_LEVEL = 21,
    PORT_MECH_TYPE = 22,
    PORT_CRINKLE_POP_LEVEL = 23,
    PORT_INPUT_GAIN_MODE = 24,
    PORT_COUNT = 25
};

enum ControlIndex {
    CTRL_MODEL = 0,
    CTRL_WOW,
    CTRL_FLUTTER,
    CTRL_SATURATE,
    CTRL_FAILURE,
    CTRL_VOLUME,
    CTRL_DRY,
    CTRL_NOISE,
    CTRL_AUX_MODE,
    CTRL_AUX_ACTIVE,
    CTRL_AUX_ONSET,
    CTRL_SPREAD,
    CTRL_DRY_TYPE,
    CTRL_DROP_BYPASS,
    CTRL_SNAG_BYPASS,
    CTRL_HUM_BYPASS,
    CTRL_HISS_LEVEL,
    CTRL_MECH_LEVEL,
    CTRL_MECH_TYPE,
    CTRL_CRINKLE_POP_LEVEL,
    CTRL_INPUT_GAIN_MODE,
    CTRL_COUNT
};

typedef struct {
    float drive;
    float comp;
    float lp_hz;
    float hp_hz;
    float wow_ms;
    float wow_rate;
    float flutter_ms;
    float flutter_rate;
    float hiss;
    float mech;
    float dropout;
    float snag;
    float crackle;
    float width;
    float crush;
} TapeProfile;

static const TapeProfile kProfiles[12] = {
    {1.16f, 0.58f, 13200.0f, 55.0f, 2.6f, 0.34f, 0.34f, 6.8f, 0.16f, 0.18f, 0.16f, 0.14f, 0.07f, 1.05f, 0.00f},
    {1.30f, 0.50f, 9600.0f, 42.0f, 1.6f, 0.24f, 0.22f, 4.6f, 0.13f, 0.10f, 0.09f, 0.08f, 0.05f, 1.00f, 0.00f},
    {1.10f, 0.64f, 7100.0f, 130.0f, 2.1f, 0.42f, 0.42f, 8.5f, 0.20f, 0.09f, 0.18f, 0.14f, 0.06f, 0.90f, 0.00f},
    {1.40f, 0.80f, 5200.0f, 240.0f, 2.3f, 0.50f, 0.35f, 7.0f, 0.21f, 0.07f, 0.23f, 0.11f, 0.09f, 0.86f, 0.00f},
    {1.56f, 0.86f, 4200.0f, 190.0f, 4.2f, 0.74f, 0.86f, 9.2f, 0.28f, 0.10f, 0.45f, 0.30f, 0.17f, 0.80f, 0.03f},
    {1.20f, 0.38f, 9000.0f, 68.0f, 1.8f, 0.30f, 0.28f, 5.8f, 0.15f, 0.11f, 0.10f, 0.10f, 0.05f, 0.96f, 0.00f},
    {1.65f, 0.70f, 6400.0f, 120.0f, 5.8f, 0.96f, 1.20f, 10.5f, 0.32f, 0.12f, 0.85f, 0.75f, 0.28f, 0.76f, 0.10f},
    {1.08f, 0.30f, 15000.0f, 30.0f, 0.9f, 0.16f, 0.10f, 3.6f, 0.09f, 0.09f, 0.03f, 0.03f, 0.03f, 1.12f, 0.00f},
    {1.22f, 0.62f, 9800.0f, 58.0f, 2.7f, 0.40f, 0.31f, 6.0f, 0.17f, 0.16f, 0.22f, 0.18f, 0.08f, 1.02f, 0.00f},
    {1.44f, 0.54f, 7800.0f, 82.0f, 1.8f, 0.35f, 0.28f, 5.2f, 0.19f, 0.11f, 0.16f, 0.14f, 0.12f, 0.93f, 0.02f},
    {1.34f, 0.52f, 7200.0f, 88.0f, 3.7f, 0.60f, 0.74f, 9.0f, 0.24f, 0.11f, 0.30f, 0.24f, 0.15f, 0.88f, 0.03f},
    {1.38f, 0.24f, 17800.0f, 24.0f, 0.0f, 0.05f, 0.0f, 0.08f, 0.06f, 0.02f, 0.01f, 0.01f, 0.02f, 1.00f, 0.88f}
};

typedef struct {
    float* in_l;
    float* in_r;
    float* out_l;
    float* out_r;
    const float* control[PORT_COUNT];

    double sample_rate;
    float inv_sample_rate;

    float target[CTRL_COUNT];
    float smooth[CTRL_COUNT];

    float delay_l[MAX_DELAY];
    float delay_r[MAX_DELAY];
    uint32_t write_idx;

    float wow_value;
    float wow_target;
    uint32_t wow_count;

    float flutter_value;
    float flutter_target;
    uint32_t flutter_count;

    float aux_env;
    float transport;

    float lp_l;
    float lp_r;
    float hp_l;
    float hp_r;

    float comp_env;

    uint32_t drop_count_l;
    uint32_t drop_count_r;
    float drop_gain_l;
    float drop_gain_r;
    float drop_target_l;
    float drop_target_r;

    uint32_t snag_count;
    float snag_offset;

    uint32_t crackle_count;
    float crackle_env;

    float hiss_lp_l;
    float hiss_lp_r;
    float mech_lp;

    float hum_phase1;
    float hum_phase2;

    uint32_t crush_count;
    float crush_hold_l;
    float crush_hold_r;

    uint32_t noise_state;
} LosTT;

static inline float clampf(const float x, const float lo, const float hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline float sanitize(const float x) {
    return isfinite(x) ? x : 0.0f;
}

static inline float soft_sat(const float x) {
    const float v = sanitize(x);
    return v / (1.0f + fabsf(v));
}

static inline float port_value(const float* p, const float fallback) {
    return p ? *p : fallback;
}

static inline float quantize01(float x) {
    x = clampf(x, 0.0f, 1.0f);
    return floorf((x * 100.0f) + 0.5f) * 0.01f;
}

static inline int select3(const float x) {
    int v = (int)floorf(x + 0.5f);
    if (v < 0) {
        v = 0;
    }
    if (v > 2) {
        v = 2;
    }
    return v;
}

static inline float lerpf(const float a, const float b, const float t) {
    return a + (b - a) * t;
}

static inline uint32_t wrap_inc(const uint32_t i) {
    return (i + 1u) & DELAY_MASK;
}

static inline float lcg_noise(LosTT* self) {
    self->noise_state = self->noise_state * 1664525u + 1013904223u;
    const uint32_t v = (self->noise_state >> 8u) & 0x00FFFFFFu;
    return ((float)v * (1.0f / 8388608.0f)) - 1.0f;
}

static inline float one_pole_coeff(const float hz, const float sr) {
    const float c = clampf(hz, 10.0f, 20000.0f);
    return expf(-TWO_PI * c / sr);
}

static inline float hp_process(float* state, const float x, const float coeff) {
    const float blend = 1.0f - coeff;
    *state += (x - *state) * blend;
    return x - *state;
}

static inline float lp_process(float* state, const float x, const float coeff) {
    const float blend = 1.0f - coeff;
    *state += (x - *state) * blend;
    return *state;
}

static inline float read_delay(const float* buf, const uint32_t write_idx, float delay_samp) {
    delay_samp = clampf(delay_samp, 0.0f, (float)(MAX_DELAY - 2));
    float rp = (float)write_idx - delay_samp;
    while (rp < 0.0f) {
        rp += (float)MAX_DELAY;
    }

    const int i0 = (int)rp;
    const int i1 = (i0 + 1) & DELAY_MASK;
    const float f = rp - (float)i0;
    return sanitize(buf[i0] * (1.0f - f) + buf[i1] * f);
}

static inline float bits_quant(const float x, const int bits) {
    int b = bits;
    if (b < 3) {
        b = 3;
    }
    if (b > 16) {
        b = 16;
    }
    const int levels = 1 << (b - 1);
    const float c = clampf(x, -1.0f, 1.0f);
    const float q = (c >= 0.0f) ? (c * (float)levels + 0.5f) : (c * (float)levels - 0.5f);
    return floorf(q) / (float)levels;
}

static inline float volume_gain(const float x) {
    const float v = clampf(x, 0.0f, 1.0f);
    return v * 2.0f;
}

static inline float dry_gain_mode(const int dry_mode) {
    switch (dry_mode) {
        case 0:
            return 0.0f;
        case 1:
            return 0.25f;
        default:
            return 1.0f;
    }
}

static inline float noise_gain_mode(const int noise_mode) {
    switch (noise_mode) {
        case 0:
            return 0.0f;
        case 1:
            return 0.5f;
        default:
            return 1.0f;
    }
}

static inline float input_gain_mode(const int mode) {
    switch (mode) {
        case 0:
            return 1.0f;
        case 1:
            return 1.35f;
        default:
            return 2.15f;
    }
}

static inline float comp_gain(LosTT* self, const float in_abs, const float amount) {
    const float atk = 0.0010f + amount * 0.015f;
    const float rel = 0.0002f + amount * 0.0032f;
    if (in_abs > self->comp_env) {
        self->comp_env += (in_abs - self->comp_env) * atk;
    } else {
        self->comp_env += (in_abs - self->comp_env) * rel;
    }

    const float threshold = 0.52f - amount * 0.30f;
    const float over = clampf(self->comp_env - threshold, 0.0f, 4.0f);
    return 1.0f / (1.0f + over * (1.2f + amount * 4.5f));
}

static void morph_profile(const float model, TapeProfile* out) {
    const float x = clampf(model, 0.0f, 11.0f);
    int i = (int)floorf(x);
    if (i < 0) {
        i = 0;
    }
    if (i > 11) {
        i = 11;
    }
    const int j = (i < 11) ? (i + 1) : i;
    const float t = x - (float)i;
    const TapeProfile* a = &kProfiles[i];
    const TapeProfile* b = &kProfiles[j];

    out->drive = lerpf(a->drive, b->drive, t);
    out->comp = lerpf(a->comp, b->comp, t);
    out->lp_hz = lerpf(a->lp_hz, b->lp_hz, t);
    out->hp_hz = lerpf(a->hp_hz, b->hp_hz, t);
    out->wow_ms = lerpf(a->wow_ms, b->wow_ms, t);
    out->wow_rate = lerpf(a->wow_rate, b->wow_rate, t);
    out->flutter_ms = lerpf(a->flutter_ms, b->flutter_ms, t);
    out->flutter_rate = lerpf(a->flutter_rate, b->flutter_rate, t);
    out->hiss = lerpf(a->hiss, b->hiss, t);
    out->mech = lerpf(a->mech, b->mech, t);
    out->dropout = lerpf(a->dropout, b->dropout, t);
    out->snag = lerpf(a->snag, b->snag, t);
    out->crackle = lerpf(a->crackle, b->crackle, t);
    out->width = lerpf(a->width, b->width, t);
    out->crush = lerpf(a->crush, b->crush, t);
}

static void update_targets(LosTT* self) {
    self->target[CTRL_MODEL] = clampf(port_value(self->control[PORT_MODEL], 0.0f), 0.0f, 11.0f);
    self->target[CTRL_WOW] = quantize01(port_value(self->control[PORT_WOW], 0.35f));
    self->target[CTRL_FLUTTER] = quantize01(port_value(self->control[PORT_FLUTTER], 0.30f));
    self->target[CTRL_SATURATE] = quantize01(port_value(self->control[PORT_SATURATE], 0.40f));
    self->target[CTRL_FAILURE] = quantize01(port_value(self->control[PORT_FAILURE], 0.30f));
    self->target[CTRL_VOLUME] = quantize01(port_value(self->control[PORT_VOLUME], 0.50f));

    self->target[CTRL_DRY] = clampf(port_value(self->control[PORT_DRY], 0.0f), 0.0f, 2.0f);
    self->target[CTRL_NOISE] = clampf(port_value(self->control[PORT_NOISE], 1.0f), 0.0f, 2.0f);
    self->target[CTRL_AUX_MODE] = clampf(port_value(self->control[PORT_AUX_MODE], 0.0f), 0.0f, 2.0f);
    self->target[CTRL_AUX_ACTIVE] = quantize01(port_value(self->control[PORT_AUX_ACTIVE], 0.0f));

    self->target[CTRL_AUX_ONSET] = quantize01(port_value(self->control[PORT_AUX_ONSET], 0.20f));
    self->target[CTRL_SPREAD] = quantize01(port_value(self->control[PORT_SPREAD], 0.45f));
    self->target[CTRL_DRY_TYPE] = quantize01(port_value(self->control[PORT_DRY_TYPE], 0.0f));
    self->target[CTRL_DROP_BYPASS] = quantize01(port_value(self->control[PORT_DROP_BYPASS], 0.0f));
    self->target[CTRL_SNAG_BYPASS] = quantize01(port_value(self->control[PORT_SNAG_BYPASS], 0.0f));
    self->target[CTRL_HUM_BYPASS] = quantize01(port_value(self->control[PORT_HUM_BYPASS], 0.0f));

    self->target[CTRL_HISS_LEVEL] = quantize01(port_value(self->control[PORT_HISS_LEVEL], 0.50f));
    self->target[CTRL_MECH_LEVEL] = quantize01(port_value(self->control[PORT_MECH_LEVEL], 0.50f));
    self->target[CTRL_MECH_TYPE] = clampf(port_value(self->control[PORT_MECH_TYPE], 0.0f), -1.0f, 1.0f);
    self->target[CTRL_CRINKLE_POP_LEVEL] = quantize01(port_value(self->control[PORT_CRINKLE_POP_LEVEL], 0.45f));
    self->target[CTRL_INPUT_GAIN_MODE] = clampf(port_value(self->control[PORT_INPUT_GAIN_MODE], 1.0f), 0.0f, 2.0f);
}

static void reset_state(LosTT* self) {
    memset(self->delay_l, 0, sizeof(self->delay_l));
    memset(self->delay_r, 0, sizeof(self->delay_r));
    self->write_idx = 0;

    self->wow_value = 0.0f;
    self->wow_target = 0.0f;
    self->wow_count = 1u;

    self->flutter_value = 0.0f;
    self->flutter_target = 0.0f;
    self->flutter_count = 1u;

    self->aux_env = 0.0f;
    self->transport = 1.0f;

    self->lp_l = 0.0f;
    self->lp_r = 0.0f;
    self->hp_l = 0.0f;
    self->hp_r = 0.0f;

    self->comp_env = 0.0f;

    self->drop_count_l = 0u;
    self->drop_count_r = 0u;
    self->drop_gain_l = 1.0f;
    self->drop_gain_r = 1.0f;
    self->drop_target_l = 1.0f;
    self->drop_target_r = 1.0f;

    self->snag_count = 0u;
    self->snag_offset = 0.0f;

    self->crackle_count = 0u;
    self->crackle_env = 0.0f;

    self->hiss_lp_l = 0.0f;
    self->hiss_lp_r = 0.0f;
    self->mech_lp = 0.0f;

    self->hum_phase1 = 0.0f;
    self->hum_phase2 = 0.0f;

    self->crush_count = 0u;
    self->crush_hold_l = 0.0f;
    self->crush_hold_r = 0.0f;

    self->noise_state = 0x7BADB002u;
}

static LV2_Handle instantiate(const LV2_Descriptor* descriptor,
                              double rate,
                              const char* bundle_path,
                              const LV2_Feature* const* features) {
    (void)descriptor;
    (void)bundle_path;
    (void)features;

    LosTT* self = (LosTT*)calloc(1u, sizeof(LosTT));
    if (!self) {
        return NULL;
    }

    self->sample_rate = (rate > 1000.0) ? rate : 48000.0;
    self->inv_sample_rate = 1.0f / (float)self->sample_rate;
    reset_state(self);
    return (LV2_Handle)self;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data) {
    LosTT* self = (LosTT*)instance;
    if (!self || port >= PORT_COUNT) {
        return;
    }

    switch (port) {
        case PORT_IN_L:
            self->in_l = (float*)data;
            break;
        case PORT_IN_R:
            self->in_r = (float*)data;
            break;
        case PORT_OUT_L:
            self->out_l = (float*)data;
            break;
        case PORT_OUT_R:
            self->out_r = (float*)data;
            break;
        default:
            self->control[port] = (const float*)data;
            break;
    }
}

static void activate(LV2_Handle instance) {
    LosTT* self = (LosTT*)instance;
    if (!self) {
        return;
    }

    update_targets(self);
    for (int i = 0; i < CTRL_COUNT; ++i) {
        self->smooth[i] = self->target[i];
    }
    reset_state(self);
}

static void run(LV2_Handle instance, uint32_t n_samples) {
    LosTT* self = (LosTT*)instance;
    if (!self || !self->in_l || !self->in_r || !self->out_l || !self->out_r) {
        return;
    }

    update_targets(self);

    const float smooth_coeff = expf(-1.0f / (0.012f * (float)self->sample_rate));
    const float smooth_blend = 1.0f - smooth_coeff;

    for (uint32_t i = 0; i < n_samples; ++i) {
        for (int c = 0; c < CTRL_COUNT; ++c) {
            self->smooth[c] += (self->target[c] - self->smooth[c]) * smooth_blend;
        }

        TapeProfile p;
        morph_profile(self->smooth[CTRL_MODEL], &p);

        const float wow = self->smooth[CTRL_WOW];
        const float flutter = self->smooth[CTRL_FLUTTER];
        const float saturate = self->smooth[CTRL_SATURATE];
        float failure = self->smooth[CTRL_FAILURE];
        const float spread = self->smooth[CTRL_SPREAD];

        const int dry_mode = select3(self->smooth[CTRL_DRY]);
        const int noise_mode = select3(self->smooth[CTRL_NOISE]);
        const int aux_mode = select3(self->smooth[CTRL_AUX_MODE]);
        const int gain_mode = select3(self->smooth[CTRL_INPUT_GAIN_MODE]);

        const int drop_byp = (self->smooth[CTRL_DROP_BYPASS] > 0.5f) ? 1 : 0;
        const int snag_byp = (self->smooth[CTRL_SNAG_BYPASS] > 0.5f) ? 1 : 0;
        const int hum_byp = (self->smooth[CTRL_HUM_BYPASS] > 0.5f) ? 1 : 0;
        const int dry_type = (self->smooth[CTRL_DRY_TYPE] > 0.5f) ? 1 : 0;

        const float aux_target = (self->smooth[CTRL_AUX_ACTIVE] > 0.5f) ? 1.0f : 0.0f;
        const float aux_onset_s = 0.001f + self->smooth[CTRL_AUX_ONSET] * 1.6f;
        const float aux_coeff = expf(-1.0f / (aux_onset_s * (float)self->sample_rate));
        const float aux_blend = 1.0f - aux_coeff;
        self->aux_env += (aux_target - self->aux_env) * aux_blend;
        const float aux_env = clampf(self->aux_env, 0.0f, 1.0f);

        if (aux_mode == 2) {
            if (aux_env > failure) {
                failure = aux_env;
            }
        }

        const float stop_amt = (aux_mode == 0) ? aux_env : 0.0f;
        const float target_transport = 1.0f - stop_amt * 0.95f;
        self->transport += (target_transport - self->transport) * 0.0008f;
        const float transport = clampf(self->transport, 0.04f, 1.0f);

        float in_l = sanitize(self->in_l[i]);
        float in_r = sanitize(self->in_r[i]);

        const float pre_gain = input_gain_mode(gain_mode);
        in_l *= pre_gain;
        in_r *= pre_gain;

        const float sat_drive = (0.75f + saturate * 3.4f) * p.drive;
        float sat_l = soft_sat(in_l * sat_drive);
        float sat_r = soft_sat(in_r * sat_drive);

        if (self->wow_count == 0u) {
            const float sec = 0.05f + (1.2f / (0.15f + p.wow_rate * (0.5f + wow * 2.0f)));
            self->wow_count = (uint32_t)(sec * (float)self->sample_rate);
            if (self->wow_count < 1u) {
                self->wow_count = 1u;
            }
            self->wow_target = lcg_noise(self);
        }
        self->wow_count--;
        self->wow_value += (self->wow_target - self->wow_value) * 0.0007f;

        if (self->flutter_count == 0u) {
            const float sec = 0.005f + (0.11f / (0.3f + p.flutter_rate * (0.5f + flutter * 2.2f)));
            self->flutter_count = (uint32_t)(sec * (float)self->sample_rate);
            if (self->flutter_count < 1u) {
                self->flutter_count = 1u;
            }
            self->flutter_target = lcg_noise(self);
        }
        self->flutter_count--;
        self->flutter_value += (self->flutter_target - self->flutter_value) * 0.015f;

        if (self->snag_count == 0u && !snag_byp) {
            const float p_snag = p.snag * (0.00005f + failure * 0.0020f);
            if (fabsf(lcg_noise(self)) < p_snag) {
                self->snag_count = 8u + (uint32_t)(fabsf(lcg_noise(self)) * (300.0f + failure * 2600.0f));
                self->snag_offset = lcg_noise(self) * (80.0f + failure * 2400.0f);
            }
        }
        if (self->snag_count > 0u) {
            self->snag_count--;
            self->snag_offset *= 0.988f;
        } else {
            self->snag_offset *= 0.95f;
        }

        const float wow_depth = p.wow_ms * (0.15f + wow * 1.9f) * (0.35f + failure) * 0.001f * (float)self->sample_rate;
        const float flutter_depth = p.flutter_ms * (0.10f + flutter * 1.9f) * (0.30f + failure) * 0.001f * (float)self->sample_rate;

        float delay_mod = self->wow_value * wow_depth + self->flutter_value * flutter_depth;
        delay_mod *= transport;
        if (!snag_byp) {
            delay_mod += self->snag_offset;
        }
        delay_mod += lcg_noise(self) * (0.03f + failure * 0.20f);

        const float base_delay = 8.0f + (1.0f - transport) * 2600.0f + failure * 14.0f;
        const float dly_l = base_delay + delay_mod;
        const float dly_r = base_delay - delay_mod;

        self->delay_l[self->write_idx] = sat_l;
        self->delay_r[self->write_idx] = sat_r;

        float wet_l = read_delay(self->delay_l, self->write_idx, dly_l);
        float wet_r = read_delay(self->delay_r, self->write_idx, dly_r);

        self->write_idx = wrap_inc(self->write_idx);

        const float hp_hz = p.hp_hz + failure * 240.0f;
        const float lp_hz = p.lp_hz - failure * 1300.0f;
        const float hp_c = one_pole_coeff(hp_hz, (float)self->sample_rate);
        const float lp_c = one_pole_coeff(lp_hz, (float)self->sample_rate);

        float model_l = hp_process(&self->hp_l, wet_l, hp_c);
        float model_r = hp_process(&self->hp_r, wet_r, hp_c);
        model_l = lp_process(&self->lp_l, model_l, lp_c);
        model_r = lp_process(&self->lp_r, model_r, lp_c);

        float filter_bypass_mix = 0.0f;
        if (aux_mode == 1) {
            filter_bypass_mix = aux_env;
        }
        wet_l = lerpf(model_l, wet_l, filter_bypass_mix);
        wet_r = lerpf(model_r, wet_r, filter_bypass_mix);

        const float level_abs = fmaxf(fabsf(wet_l), fabsf(wet_r));
        const float comp_amt = clampf(p.comp * (0.25f + saturate * 1.45f) + failure * 0.55f, 0.0f, 1.8f);
        const float cg = comp_gain(self, level_abs, comp_amt);
        wet_l *= cg;
        wet_r *= cg;

        const float noise_gain = noise_gain_mode(noise_mode);
        const float hiss_amt = p.hiss * noise_gain * self->smooth[CTRL_HISS_LEVEL] * (0.25f + failure * 1.2f);
        const float hiss_in_l = lcg_noise(self);
        const float hiss_in_r = lcg_noise(self);
        self->hiss_lp_l += (hiss_in_l - self->hiss_lp_l) * 0.18f;
        self->hiss_lp_r += (hiss_in_r - self->hiss_lp_r) * 0.18f;

        const float mech_amt = p.mech * noise_gain * self->smooth[CTRL_MECH_LEVEL] * (0.25f + failure * 1.2f);
        const float mech_type = self->smooth[CTRL_MECH_TYPE];

        self->hum_phase1 += 50.0f * self->inv_sample_rate * transport;
        self->hum_phase2 += 100.0f * self->inv_sample_rate * transport;
        if (self->hum_phase1 >= 1.0f) {
            self->hum_phase1 -= 1.0f;
        }
        if (self->hum_phase2 >= 1.0f) {
            self->hum_phase2 -= 1.0f;
        }

        float hum = (sinf(TWO_PI * self->hum_phase1) * 0.75f + sinf(TWO_PI * self->hum_phase2) * 0.25f);
        float vcr_noise = lcg_noise(self);
        self->mech_lp += (vcr_noise - self->mech_lp) * 0.045f;
        vcr_noise = vcr_noise - self->mech_lp;

        const float hum_mix = clampf((mech_type + 1.0f) * 0.5f, 0.0f, 1.0f);
        float mech = lerpf(vcr_noise, hum, hum_mix) * mech_amt;
        if (hum_byp) {
            mech = 0.0f;
        }

        if (self->crackle_count == 0u) {
            const float crackle_prob = p.crackle * self->smooth[CTRL_CRINKLE_POP_LEVEL] * (0.0001f + failure * 0.0025f);
            if (fabsf(lcg_noise(self)) < crackle_prob) {
                self->crackle_count = 3u + (uint32_t)(fabsf(lcg_noise(self)) * 150.0f);
                self->crackle_env = 0.7f + fabsf(lcg_noise(self)) * 0.3f;
            }
        }
        float crack = 0.0f;
        if (self->crackle_count > 0u) {
            crack = lcg_noise(self) * self->crackle_env;
            self->crackle_env *= 0.84f;
            self->crackle_count--;
        }
        crack *= self->smooth[CTRL_CRINKLE_POP_LEVEL] * (0.3f + failure);

        wet_l += self->hiss_lp_l * hiss_amt + mech + crack;
        wet_r += self->hiss_lp_r * hiss_amt + mech + crack;

        if (!drop_byp) {
            if (self->drop_count_l == 0u) {
                const float pd = p.dropout * (0.0001f + failure * 0.0042f);
                if (fabsf(lcg_noise(self)) < pd) {
                    self->drop_count_l = 8u + (uint32_t)(fabsf(lcg_noise(self)) * (3000.0f + failure * 6000.0f));
                    self->drop_target_l = 0.02f + fabsf(lcg_noise(self)) * 0.30f;
                }
            }
            if (self->drop_count_r == 0u) {
                const float pd = p.dropout * (0.0001f + (failure + spread * 0.25f) * 0.0042f);
                if (fabsf(lcg_noise(self)) < pd) {
                    self->drop_count_r = 8u + (uint32_t)(fabsf(lcg_noise(self)) * (3000.0f + failure * 6000.0f));
                    self->drop_target_r = 0.02f + fabsf(lcg_noise(self)) * 0.30f;
                }
            }
        } else {
            self->drop_count_l = 0u;
            self->drop_count_r = 0u;
        }

        if (self->drop_count_l > 0u) {
            self->drop_count_l--;
            self->drop_gain_l += (self->drop_target_l - self->drop_gain_l) * 0.06f;
        } else {
            self->drop_gain_l += (1.0f - self->drop_gain_l) * 0.003f;
        }

        if (self->drop_count_r > 0u) {
            self->drop_count_r--;
            self->drop_gain_r += (self->drop_target_r - self->drop_gain_r) * 0.06f;
        } else {
            self->drop_gain_r += (1.0f - self->drop_gain_r) * 0.003f;
        }

        wet_l *= self->drop_gain_l;
        wet_r *= self->drop_gain_r;

        const float crush_amt = clampf(p.crush, 0.0f, 1.0f);
        if (crush_amt > 0.0001f) {
            const int bits = 16 - (int)(crush_amt * 12.0f);
            const uint32_t hold_n = 1u + (uint32_t)(crush_amt * 62.0f);
            if (self->crush_count == 0u) {
                self->crush_hold_l = bits_quant(wet_l, bits);
                self->crush_hold_r = bits_quant(wet_r, bits);
                self->crush_count = hold_n;
            } else {
                self->crush_count--;
            }
            wet_l = self->crush_hold_l;
            wet_r = self->crush_hold_r;
        } else {
            self->crush_count = 0u;
        }

        const float width = clampf(p.width * (0.45f + spread * 1.3f), 0.0f, 1.8f);
        const float mid = 0.5f * (wet_l + wet_r);
        const float side = 0.5f * (wet_l - wet_r) * width;
        wet_l = mid + side;
        wet_r = mid - side;

        float dry_l = sanitize(self->in_l[i]);
        float dry_r = sanitize(self->in_r[i]);

        if (dry_type) {
            float dsl = soft_sat(dry_l * sat_drive);
            float dsr = soft_sat(dry_r * sat_drive);

            float dml = hp_process(&self->hp_l, dsl, hp_c);
            float dmr = hp_process(&self->hp_r, dsr, hp_c);
            dml = lp_process(&self->lp_l, dml, lp_c);
            dmr = lp_process(&self->lp_r, dmr, lp_c);

            dml *= self->drop_gain_l;
            dmr *= self->drop_gain_r;

            dml += self->hiss_lp_l * hiss_amt + mech + crack;
            dmr += self->hiss_lp_r * hiss_amt + mech + crack;

            dry_l = dml;
            dry_r = dmr;
        }

        const float wet_gain = volume_gain(self->smooth[CTRL_VOLUME]) * powf(transport, 0.35f);
        wet_l = soft_sat(wet_l * wet_gain * 1.20f);
        wet_r = soft_sat(wet_r * wet_gain * 1.20f);

        const float dg = dry_gain_mode(dry_mode);
        float out_l = wet_l + dry_l * dg;
        float out_r = wet_r + dry_r * dg;

        if (!isfinite(out_l) || fabsf(out_l) < 1.0e-20f) {
            out_l = 0.0f;
        }
        if (!isfinite(out_r) || fabsf(out_r) < 1.0e-20f) {
            out_r = 0.0f;
        }

        self->out_l[i] = clampf(out_l, -1.0f, 1.0f);
        self->out_r[i] = clampf(out_r, -1.0f, 1.0f);
    }
}

static void cleanup(LV2_Handle instance) {
    free(instance);
}

static const void* extension_data(const char* uri) {
    (void)uri;
    return NULL;
}

static const LV2_Descriptor DESCRIPTOR = {
    LOS_TT_URI,
    instantiate,
    connect_port,
    activate,
    run,
    NULL,
    cleanup,
    extension_data
};

extern "C" LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    return (index == 0u) ? &DESCRIPTOR : NULL;
}
