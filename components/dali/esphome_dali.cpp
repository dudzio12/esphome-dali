#include <esphome.h>
#include <esp_task_wdt.h>
#include <cstring>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "esphome/components/text_sensor/text_sensor.h"

static const char *const TAG = "dali";
static const bool DEBUG_LOG_RXTX = false;
static const bool DEBUG_LOG_RXTX_FULL = false;

using namespace esphome;
using namespace dali;

// courtesy of https://github.com/petrinm/ESP32Dali

/*******************************************************************************
 * Typedefs and defines
 ******************************************************************************/

#define BI_PHASE_HIGH           0b01
#define BI_PHASE_LOW            0b10
#define BI_PHASE_MASK           0b11


/*******************************************************************************
 * Define DALI timing values. One Bit on a DALI bus is
 * 833.33µs (1200 bauds) which corresponds to 2TE.
 ******************************************************************************/

#define TE            (417)                     // half bit time = 417 usec

/* Transmission timing, option to compensate for unequal physical layer delay in rising or falling edge*/
#define TE_HIGH       (TE + 0)
#define TE_LOW        (TE - 0)

#define DALI_STOP_BIT_TIME          (4*TE)
#define MIN_FORWARD_FRAME_DELAY     (22*TE)     // minimum time between two forward frames - 9.17ms ==> 22 x TE time
#define BACKWARD_FRAME_DELAY        (12*TE)     // time between forward frame and backward frame >= 7TE

#if 0 /* strict receive timing according to specification  */
  #define MIN_TE      (TE - 42)                 // minimum half bit time
  #define MAX_TE      (TE + 42)                 // maximum half bit time
  #define MIN_2TE     (2*TE - 83)               // minimum full bit time
  #define MAX_2TE     (2*TE + 83)               // maximum full bit time
#else /* More relaxed receive timing */
  #define MIN_TE      (300)                     // minimum half bit time
  #define MAX_TE      (550)                     // maximum half bit time
  #define MIN_2TE     (2*TE - (2*(TE/5)))       // minimum full bit time
  #define MAX_2TE     (2*TE + (2*(TE/5)))       // maximum full bit time
#endif

#define BACKWARD_FRAME_BIT_LENGTH     18        // (1 start bit + 8 data bits) * 2 symbols per bit

/*******************************************************************************
 * Timer-based RX: static instance pointer, reset, ISR
 ******************************************************************************/

// Static pointer to the singleton — needed because ISR must be a static function
static DaliBusComponent* g_dali_instance = nullptr;

void DaliTimerRxState::reset() {
    this->rxpos = 0;
    this->rxbitcnt = 0;
    this->rxbyte = 0;
    this->rxidle = 0;
    this->rxstate = 0; // EMPTY
    memset((void*)this->rxdata, 0, RX_BUF_SIZE);
}

void IRAM_ATTR DaliBusComponent::onTimerISR() {
    if (!g_dali_instance) return;
    if (g_dali_instance->m_tx_active) return; // ignore samples during TX
    auto& rx = g_dali_instance->m_rx_state;

    uint8_t busishigh = rx.rx_pin->digital_read() ? 1 : 0;

    // State EMPTY — waiting for start bit (bus goes LOW)
    if (rx.rxstate == 0) {
        if (!busishigh) {
            // Start of frame — transition to RECEIVING
            rx.rxpos = 0;
            rx.rxbitcnt = 0;
            rx.rxbyte = 0;
            rx.rxidle = 0;
            rx.rxstate = 1; // RECEIVING
            // Record first sample
            rx.rxbyte = (rx.rxbyte << 1) | busishigh;
            rx.rxbitcnt++;
        }
        return;
    }

    // State RECEIVING — sample the bus
    if (rx.rxstate == 1) {
        rx.rxbyte = (rx.rxbyte << 1) | busishigh;
        rx.rxbitcnt++;

        if (rx.rxbitcnt >= 8) {
            if (rx.rxpos < DaliTimerRxState::RX_BUF_SIZE) {
                rx.rxdata[rx.rxpos] = rx.rxbyte;
                rx.rxpos++;
            }
            rx.rxbitcnt = 0;
            rx.rxbyte = 0;
        }

        // Stop detection (16 consecutive HIGH samples = 2 Te)
        if (busishigh) {
            rx.rxidle++;
            if (rx.rxidle >= 16) {
                // Save last incomplete byte
                if (rx.rxbitcnt > 0 && rx.rxpos < DaliTimerRxState::RX_BUF_SIZE) {
                    rx.rxdata[rx.rxpos] = rx.rxbyte << (8 - rx.rxbitcnt);
                    rx.rxpos++;
                }
                rx.rxstate = 2; // COMPLETED
            }
        } else {
            rx.rxidle = 0;
        }
    }
    // State COMPLETED (2) — do nothing, wait for main code to read
}

void DaliBusComponent::setupTimer() {
    g_dali_instance = this;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // Arduino-ESP32 v3.x API
    m_timer = timerBegin(9600000);
    timerAttachInterrupt(m_timer, &DaliBusComponent::onTimerISR);
    timerAlarm(m_timer, 1000, true, 0);
#else
    // Arduino-ESP32 v2.x API
    // Timer 0, prescaler 80 (80MHz/80 = 1 MHz tick), count up
    m_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(m_timer, &DaliBusComponent::onTimerISR, true);
    timerAlarmWrite(m_timer, 104, true); // alarm every 104 us ≈ 9615 Hz
    timerAlarmEnable(m_timer);
#endif

    ESP_LOGI(TAG, "DALI timer started at ~9600 Hz");
}

/*******************************************************************************
 * Manchester decoder (weight-based with adaptive sync)
 ******************************************************************************/

uint8_t DaliBusComponent::manWeight(uint8_t sample) {
    // Analyzes 8 samples (1 Manchester bit at 8x oversampling)
    // Center samples weighted higher (more reliable)
    // Manchester '1' = LH (low-to-high): negative result → odd value
    // Manchester '0' = HL (high-to-low): positive result → even value
    int8_t w = 0;
    w += ((sample >> 7) & 1) ? 1 : -1;   // oldest sample
    w += ((sample >> 6) & 1) ? 2 : -2;   // center - higher weight
    w += ((sample >> 5) & 1) ? 2 : -2;
    w += ((sample >> 4) & 1) ? 1 : -1;
    w -= ((sample >> 3) & 1) ? 1 : -1;   // second half of bit
    w -= ((sample >> 2) & 1) ? 2 : -2;
    w -= ((sample >> 1) & 1) ? 2 : -2;
    w -= ((sample >> 0) & 1) ? 1 : -1;   // newest sample

    w *= 2;
    if (w < 0) w = -w + 1;  // negative → odd (bit=1), positive → even (bit=0)
    return (uint8_t)w;
}

uint8_t DaliBusComponent::manSample(const uint8_t* edata, uint16_t bitpos, uint8_t* stop_coll) {
    uint8_t pos = bitpos >> 3;
    uint8_t shift = bitpos & 0x7;
    uint8_t sample;
    if (shift == 0) {
        sample = edata[pos];
    } else if (pos + 1 < DaliTimerRxState::RX_BUF_SIZE) {
        sample = (edata[pos] << shift) | (edata[pos + 1] >> (8 - shift));
    } else {
        sample = edata[pos] << shift;  // pad with zeros at end of buffer
    }
    if (sample == 0xFF) *stop_coll = 1;  // stop bit (all high)
    if (sample == 0x00) *stop_coll = 2;  // collision (all low)
    return sample;
}

uint8_t DaliBusComponent::manDecode(const uint8_t* edata, uint8_t ebitlen, uint8_t* ddata) {
    // ebitlen = number of bytes (each = 8 samples = 1 Manchester bit)
    // ddata = output buffer for decoded bytes
    // Returns number of decoded bits

    uint8_t dbitlen = 0;
    uint16_t ebitpos = 1; // skip start bit (starts from sample 1)

    while (ebitpos + 8 <= (uint16_t)ebitlen * 8) {
        uint8_t stop_coll = 0;

        // Check 3 positions: -1, 0, +1 sample (adaptive sync)
        uint8_t sample = manSample(edata, ebitpos, &stop_coll);
        uint8_t weightmax = manWeight(sample);
        uint8_t pmax = 8; // nominal position of next bit

        if (ebitpos > 0) {
            sample = manSample(edata, ebitpos - 1, &stop_coll);
            uint8_t w = manWeight(sample);
            if (weightmax < w) { weightmax = w; pmax = 7; }
        }

        if (ebitpos + 9 <= (uint16_t)ebitlen * 8) {
            sample = manSample(edata, ebitpos + 1, &stop_coll);
            uint8_t w = manWeight(sample);
            if (weightmax < w) { weightmax = w; pmax = 9; }
        }

        if (stop_coll == 1) break;  // stop bit
        if (stop_coll == 2) return 0; // collision

        // Decode bit: weightmax & 1 → 1=Manchester '1', 0=Manchester '0'
        if (dbitlen > 0) { // skip start bit
            uint8_t bytepos = (dbitlen - 1) >> 3;
            uint8_t bitshift = 7 - ((dbitlen - 1) & 0x7);
            if (((dbitlen - 1) & 0x7) == 0)
                ddata[bytepos] = 0;
            if (weightmax & 1)
                ddata[bytepos] |= (1 << bitshift);
        }
        dbitlen++;
        ebitpos += pmax;
    }

    if (dbitlen > 0) dbitlen--; // subtract start bit
    return dbitlen;
}

/*******************************************************************************
 * Utility
 ******************************************************************************/

// Prints the DALI QUERY_STATUS (0x90) response in a human-readable format
void print_dali_status(uint8_t status, uint8_t address) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "DALI[%.2d] Status: %s | %s | %s | %s | %s | %s | %s | %s",
           address,
           (status & 0x01) ? "Gear Failed" : "Gear OK",
           (status & 0x02) ? "Lamp Failed" : "Lamp OK",
           (status & 0x04) ? "Lamp On" : "Lamp Off",
           (status & 0x08) ? "Limit Error" : "Level OK",
           (status & 0x10) ? "Fading" : "Not Fading",
           (status & 0x20) ? "Reset State" : "Not Reset",
           (status & 0x40) ? "No Address" : "Address OK",
           (status & 0x80) ? "Power Failure" : "Power OK");
  DALI_LOGI("%s", buffer);
}

/*******************************************************************************
 * setup()
 ******************************************************************************/

void DaliBusComponent::setup() {
    m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
    m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);

    // Initialize timer RX state
    m_rx_state.rx_pin = m_rxPin;
    m_rx_state.reset();

    // Start 9600 Hz timer
    setupTimer();

    // === TX polarity auto-detect ===
    // Test: set TX HIGH, wait, read RX. Then TX LOW, wait, read RX.
    // If RX reacts to TX, we can determine polarity.
    ESP_LOGW(TAG, "=== DALI TX POLARITY AUTO-DETECT ===");

    // Test 1: TX=HIGH (sehraf: assert, waveshare: release)
    m_txPin->digital_write(HIGH);
    delay(50);
    bool rx_when_tx_high = m_rxPin->digital_read();
    ESP_LOGW(TAG, "TX=HIGH => RX=%s", rx_when_tx_high ? "HIGH" : "LOW");

    // Test 2: TX=LOW (sehraf: release, waveshare: assert)
    m_txPin->digital_write(LOW);
    delay(50);
    bool rx_when_tx_low = m_rxPin->digital_read();
    ESP_LOGW(TAG, "TX=LOW  => RX=%s", rx_when_tx_low ? "HIGH" : "LOW");

    if (rx_when_tx_high != rx_when_tx_low) {
        // RX reacts to TX — we can determine polarity
        if (rx_when_tx_high && !rx_when_tx_low) {
            // TX HIGH → RX HIGH (bus released), TX LOW → RX LOW (bus asserted)
            // This means: HIGH=release, LOW=assert → non-inverted (Waveshare)
            m_tx_inverted = false;
            ESP_LOGW(TAG, "Detected: NON-INVERTED (Waveshare-style)");
        } else {
            // TX HIGH → RX LOW (bus asserted), TX LOW → RX HIGH (bus released)
            // This means: HIGH=assert, LOW=release → inverted (opto-isolator)
            m_tx_inverted = true;
            ESP_LOGW(TAG, "Detected: INVERTED (opto-isolator-style)");
        }
    } else {
        ESP_LOGW(TAG, "Cannot detect polarity (RX=%s regardless of TX). Defaulting to inverted.",
                 rx_when_tx_high ? "HIGH" : "LOW");
        m_tx_inverted = true; // default to sehraf original
    }

    // Restore TX to idle (release)
    bool release_level = m_tx_inverted ? LOW : HIGH;
    m_txPin->digital_write(release_level);
    delay(50);

    ESP_LOGW(TAG, "TX idle level: %s, RX idle: %s",
             release_level ? "HIGH" : "LOW",
             m_rxPin->digital_read() ? "HIGH" : "LOW");

    DALI_LOGI("DALI bus ready (timer RX), discovery=%s, init_addr=%d, tx_inv=%d",
              m_discovery ? "true" : "false",
              static_cast<int>(m_initialize_addresses),
              m_tx_inverted);

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGW("No control gear detected on bus!");
        }

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL);
            }
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                // Only randomize devices without an assigned short address
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED);
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            // Seem to need a delay to allow time for devices to randomize...
            delay(50);
        }

        DALI_LOGI("Begin device discovery...");
        dali.bus_manager.startAddressScan(); // All devices

        // Keep track of short addresses to detect duplicates
        bool duplicate_detected = false;
        bool is_discovered[ADDR_SHORT_MAX+1];
        for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
            is_discovered[i] = false;
        }

        uint8_t count = 0;
        short_addr_t short_addr = 0xFF;
        uint32_t long_addr = 0;
        while (dali.bus_manager.findNextAddress(short_addr, long_addr)) {
            count++;
            delay(1); // yield to ESP stack
            esp_task_wdt_reset();

            if (short_addr <= ADDR_SHORT_MAX) {
                DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                // Duplicate detection
                if (is_discovered[short_addr]) {
                    if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                        DALI_LOGW("  WARNING: Duplicate short address detected!");
                        duplicate_detected = true;
                        // TODO: Maybe don't register the component in this case?
                        // Brightness control will work, but reported capabilities will not be correct.
                    }
                    else {
                        // Assign a new address for this
                        short_addr++;
                        DALI_LOGD("  Duplicate short address detected, assigning a new address: %.2x", short_addr);

                        if (!dali.bus_manager.programShortAddress(short_addr)) {
                            DALI_LOGE("  Could not program short address");
                            short_addr = 0xFF;
                            continue;
                        }
                    }
                }
                else {
                    is_discovered[short_addr] = true;
                }

                uint8_t status = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_STATUS);
                print_dali_status(status, short_addr);
                // dump_device_info(short_addr); // info now exposed via text sensors

                // Dynamic component creation (if not defined in YAML)
                if (m_addresses[short_addr]) {
                    DALI_LOGD("  Ignoring, already defined");
                    if (m_addresses[short_addr] == 0xffffff) {
                        m_addresses[short_addr] = long_addr;
                    }
                }
                else {
                    m_addresses[short_addr] = long_addr;
                    create_light_component(short_addr, long_addr);
                }
            }
            else if (short_addr == 0xFF) {
                if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                    DALI_LOGI("  Device %.6x @ --", long_addr);
                    // You'll need to assign a short address before the device will respond to commands.
                    // However it will still respond to BROADCAST brightness updates...
                    DALI_LOGW("  No short address assigned!");
                    continue;
                }
                else {
                    short_addr = count;
                    DALI_LOGI("  Assigning short address: %.2x", short_addr);

                    if (!dali.bus_manager.programShortAddress(short_addr)) {
                        DALI_LOGE("  Could not program short address");
                        short_addr = 0xFF;
                        continue;
                    }

                    DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);
                    uint8_t status = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_STATUS);
                    print_dali_status(status, short_addr);
                    // dump_device_info(short_addr); // info now exposed via text sensors

                    // Dynamic component creation (if not defined in YAML)
                    if (m_addresses[short_addr]) {
                        DALI_LOGD("  Ignoring, already defined");
                        if (m_addresses[short_addr] == 0xffffff) {
                            m_addresses[short_addr] = long_addr;
                        }
                    }
                    else {
                        m_addresses[short_addr] = long_addr;
                        create_light_component(short_addr, long_addr);
                    }
                }
            }
            // Remove this device from the search
            dali.bus_manager.withdraw(long_addr);
        }

        DALI_LOGD("No more devices found!");
        dali.bus_manager.endAddressScan();

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
            DALI_LOGW("  Devices may report inconsistent capabilities.");
            DALI_LOGW("  You should fix your address assignments.");
        }
    }

    // Register HA service calls (groups, scenes, identify, etc.)
    register_services();
}

void DaliBusComponent::dump_device_info(short_addr_t addr) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  DALI Device Report — addr %d (0x%02x)     ", addr, addr);
    ESP_LOGI(TAG, "╠══════════════════════════════════════════╣");

    // Basic info
    uint8_t version = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_VERSION_NUMBER);
    uint8_t device_type = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_DEVICE_TYPE);
    uint8_t light_source = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_LIGHT_SOURCE_TYPE);
    uint8_t operating_mode = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_OPERATING_MODE);

    const char* source_name = "Unknown";
    switch (light_source) {
        case 0: source_name = "Fluorescent"; break;
        case 1: source_name = "Emergency"; break;
        case 2: source_name = "HID"; break;
        case 3: source_name = "LV Halogen"; break;
        case 4: source_name = "Incandescent"; break;
        case 5: source_name = "Digital"; break;
        case 6: source_name = "LED"; break;
        case 8: source_name = "Color (DT8)"; break;
        case 255: source_name = "N/A"; break;
    }

    ESP_LOGI(TAG, "║  Version: %d.%d  Device Type: %d", version >> 4, version & 0x0F, device_type);
    ESP_LOGI(TAG, "║  Light source: %s (%d)", source_name, light_source);
    ESP_LOGI(TAG, "║  Operating mode: %d", operating_mode);

    // Levels
    uint8_t actual = dali.lamp.getCurrentLevel(addr);
    uint8_t min_level = dali.lamp.getMinLevel(addr);
    uint8_t max_level = dali.lamp.getMaxLevel(addr);
    uint8_t phys_min = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_PHYSICAL_MINIMUM);
    uint8_t power_on = dali.lamp.getPowerOnLevel(addr);
    uint8_t sys_fail = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_SYSTEM_FAILURE_LEVEL);

    ESP_LOGI(TAG, "╠── Levels ─────────────────────────────────");
    ESP_LOGI(TAG, "║  Actual: %d  Min: %d  Max: %d  Phys.Min: %d", actual, min_level, max_level, phys_min);
    ESP_LOGI(TAG, "║  Power-on: %d  System failure: %d", power_on, sys_fail);

    // Fade
    uint8_t fade_info = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_FADE_TIME_FADE_RATE);
    uint8_t fade_time = (fade_info >> 4) & 0x0F;
    uint8_t fade_rate = fade_info & 0x0F;
    uint8_t ext_fade = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_EXTENDED_FADE_TIME);

    ESP_LOGI(TAG, "╠── Fade ───────────────────────────────────");
    ESP_LOGI(TAG, "║  Fade time: %d  Fade rate: %d  Extended fade: 0x%02x", fade_time, fade_rate, ext_fade);

    // Dimming curve (DT6 LED)
    uint8_t dimming_curve = dali.port.sendExtendedQuery(addr, DaliLedCommand::QUERY_DIMMING_CURVE);
    ESP_LOGI(TAG, "║  Dimming curve: %s (%d)", dimming_curve == 1 ? "Linear" : "Logarithmic", dimming_curve);

    // Groups
    uint8_t groups_0_7 = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_GROUPS_0_7);
    uint8_t groups_8_15 = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_GROUPS_8_15);
    ESP_LOGI(TAG, "╠── Groups ────────────────────────────────");
    ESP_LOGI(TAG, "║  Groups 0-7: 0x%02x  Groups 8-15: 0x%02x", groups_0_7, groups_8_15);

    // Status
    uint8_t status = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_STATUS);
    ESP_LOGI(TAG, "╠── Status (0x%02x) ─────────────────────────", status);
    ESP_LOGI(TAG, "║  Gear: %s  Lamp: %s  Power: %s",
             (status & 0x01) ? "FAIL" : "OK",
             (status & 0x02) ? "FAIL" : "OK",
             (status & 0x04) ? "ON" : "OFF");
    ESP_LOGI(TAG, "║  Limit err: %s  Fading: %s  Reset: %s  Addr: %s",
             (status & 0x08) ? "YES" : "no",
             (status & 0x10) ? "YES" : "no",
             (status & 0x20) ? "YES" : "no",
             (status & 0x40) ? "MISSING" : "OK");

    // DT8 Color info
    bool has_color = dali.color.supportsExtendedColor(addr);
    if (has_color) {
        uint8_t color_features = dali.port.sendExtendedQuery(addr, DaliColorCommand::QUERY_COLOR_FEATURES);
        uint8_t color_status = dali.port.sendExtendedQuery(addr, DaliColorCommand::QUERY_COLOR_STATUS);
        bool tc_cap = (color_features & 0x02) != 0;
        bool xy_cap = (color_features & 0x01) != 0;

        ESP_LOGI(TAG, "╠── DT8 Color ─────────────────────────────");
        ESP_LOGI(TAG, "║  Features: 0x%02x (TC:%s XY:%s)", color_features,
                 tc_cap ? "yes" : "no", xy_cap ? "yes" : "no");
        ESP_LOGI(TAG, "║  Status: 0x%02x (TC active:%s XY active:%s RGBWAF active:%s)",
                 color_status,
                 (color_status & 0x20) ? "yes" : "no",
                 (color_status & 0x10) ? "yes" : "no",
                 (color_status & 0x40) ? "yes" : "no");

        if (tc_cap) {
            uint16_t tc_coolest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcCoolest);
            uint16_t tc_warmest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcWarmest);
            uint16_t tc_phys_coolest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcPhysicalCoolest);
            uint16_t tc_phys_warmest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcPhysicalWarmest);
            uint16_t tc_current = dali.color.getColorTemperature(addr);

            ESP_LOGI(TAG, "║  TC current: %d mireds", tc_current);
            ESP_LOGI(TAG, "║  TC range: %d-%d mireds (phys: %d-%d)",
                     tc_coolest, tc_warmest, tc_phys_coolest, tc_phys_warmest);
        }
    } else {
        ESP_LOGI(TAG, "╠── DT8 Color: not supported ──────────────");
    }

    // DT6 LED specific queries
    uint8_t gear_type = dali.port.sendExtendedQuery(addr, DaliLedCommand::QUERY_GEAR_TYPE);
    uint8_t features = dali.port.sendExtendedQuery(addr, DaliLedCommand::QUERY_FEATURES);
    uint8_t failure = dali.port.sendExtendedQuery(addr, DaliLedCommand::QUERY_FAILURE_STATUS);

    ESP_LOGI(TAG, "╠── DT6 LED ───────────────────────────────");
    ESP_LOGI(TAG, "║  Gear type: 0x%02x  Features: 0x%02x  Failure: 0x%02x", gear_type, features, failure);

    // Scenes (0-15)
    ESP_LOGI(TAG, "╠── Scenes ────────────────────────────────");
    char scene_buf[128];
    int pos = 0;
    for (uint8_t s = 0; s < 16; s++) {
        uint8_t scene_level = dali.port.sendQueryCommand(addr, (DaliCommand)(0xB0 + s));
        if (scene_level != 0xFF) {
            pos += snprintf(scene_buf + pos, sizeof(scene_buf) - pos, " S%d=%d", s, scene_level);
        }
    }
    if (pos > 0) {
        ESP_LOGI(TAG, "║ %s", scene_buf);
    } else {
        ESP_LOGI(TAG, "║  (no scenes configured)");
    }

    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
}

/*******************************************************************************
 * Service calls — callable from HA as esphome.<device>_dali_*
 ******************************************************************************/

#ifdef USE_API_CUSTOM_SERVICES
void DaliBusComponent::register_services() {
    register_service(&DaliBusComponent::on_dali_add_to_group, "dali_add_to_group",
                     {"address", "group"});
    register_service(&DaliBusComponent::on_dali_remove_from_group, "dali_remove_from_group",
                     {"address", "group"});
    register_service(&DaliBusComponent::on_dali_store_scene, "dali_store_scene",
                     {"address", "scene", "is_group"});
    register_service(&DaliBusComponent::on_dali_remove_scene, "dali_remove_scene",
                     {"address", "scene", "is_group"});
    register_service(&DaliBusComponent::on_dali_go_to_scene, "dali_go_to_scene",
                     {"address", "scene", "is_group"});
    register_service(&DaliBusComponent::on_dali_set_brightness, "dali_set_brightness",
                     {"address", "level", "is_group"});
    register_service(&DaliBusComponent::on_dali_identify, "dali_identify",
                     {"address", "is_group"});
    register_service(&DaliBusComponent::on_dali_set_fade_time, "dali_set_fade_time",
                     {"address", "fade_time", "is_group"});
    register_service(&DaliBusComponent::on_dali_set_power_on_level, "dali_set_power_on_level",
                     {"address", "level", "is_group"});
    ESP_LOGI(TAG, "DALI service calls registered");
}
#else
void DaliBusComponent::register_services() {
    ESP_LOGW(TAG, "Service calls disabled (api: custom_services: true required)");
}
#endif

void DaliBusComponent::on_dali_add_to_group(int32_t address, int32_t group) {
    if (address < 0 || address > 63) { ESP_LOGW(TAG, "Address must be 0-63"); return; }
    if (group < 0 || group > 15) { ESP_LOGW(TAG, "Group must be 0-15"); return; }
    ESP_LOGI(TAG, "Service: add addr %d to group %d", address, group);
    dali.scene.addToGroup((short_addr_t)address, (uint8_t)group);
    m_group_mask[address] |= (1 << group);
}

void DaliBusComponent::on_dali_remove_from_group(int32_t address, int32_t group) {
    if (address < 0 || address > 63) { ESP_LOGW(TAG, "Address must be 0-63"); return; }
    if (group < 0 || group > 15) { ESP_LOGW(TAG, "Group must be 0-15"); return; }
    ESP_LOGI(TAG, "Service: remove addr %d from group %d", address, group);
    dali.scene.removeFromGroup((short_addr_t)address, (uint8_t)group);
    m_group_mask[address] &= ~(1 << group);
}

void DaliBusComponent::on_dali_store_scene(int32_t address, int32_t scene, int32_t is_group) {
    if (scene < 0 || scene > 15) { ESP_LOGW(TAG, "Scene must be 0-15"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: store scene %d for %s %d", scene, is_group ? "group" : "addr", address);
    dali.scene.storeScene(addr, (uint8_t)scene);
}

void DaliBusComponent::on_dali_remove_scene(int32_t address, int32_t scene, int32_t is_group) {
    if (scene < 0 || scene > 15) { ESP_LOGW(TAG, "Scene must be 0-15"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: remove scene %d from %s %d", scene, is_group ? "group" : "addr", address);
    dali.scene.removeScene(addr, (uint8_t)scene);
}

void DaliBusComponent::on_dali_go_to_scene(int32_t address, int32_t scene, int32_t is_group) {
    if (scene < 0 || scene > 15) { ESP_LOGW(TAG, "Scene must be 0-15"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: go to scene %d for %s %d", scene, is_group ? "group" : "addr", address);
    dali.scene.goToScene(addr, (uint8_t)scene);

    // Update HA state by querying actual levels after scene recall
    delay(50);  // allow DALI devices to reach target level
    for (auto& dl : m_dynamic_lights) {
        bool match = is_group ? (m_group_mask[dl.addr] & (1 << (uint8_t)address))
                              : (dl.addr == (short_addr_t)address);
        if (!match) continue;
        uint8_t actual = dali.lamp.getCurrentLevel(dl.addr);
        if (actual == 0xFF) continue;
        dl.output->set_suppress_write(true);
        auto call = dl.state->make_call();
        if (actual == 0) {
            call.set_state(false);
        } else {
            call.set_state(true);
            call.set_brightness((float)actual / 254.0f);
        }
        call.perform();
    }
}

void DaliBusComponent::on_dali_set_brightness(int32_t address, int32_t level, int32_t is_group) {
    if (level < 0 || level > 254) { ESP_LOGW(TAG, "Level must be 0-254"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: set brightness %s %d level %d", is_group ? "group" : "addr", address, level);
    dali.lamp.setBrightness(addr, (uint8_t)level);

    // Update HA state for affected lights
    for (auto& dl : m_dynamic_lights) {
        bool match = is_group ? (m_group_mask[dl.addr] & (1 << (uint8_t)address))
                              : (dl.addr == (short_addr_t)address);
        if (!match) continue;
        dl.output->set_suppress_write(true);
        auto call = dl.state->make_call();
        if (level == 0) {
            call.set_state(false);
        } else {
            call.set_state(true);
            call.set_brightness((float)level / 254.0f);
        }
        call.perform();
    }
}

void DaliBusComponent::on_dali_identify(int32_t address, int32_t is_group) {
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: identify %s %d", is_group ? "group" : "addr", address);
    dali.port.sendControlCommand(addr, DaliCommand::IDENTIFY_DEVICE);
}

void DaliBusComponent::on_dali_set_fade_time(int32_t address, int32_t fade_time, int32_t is_group) {
    if (fade_time < 0 || fade_time > 15) { ESP_LOGW(TAG, "Fade time must be 0-15"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: set fade_time %s %d value %d", is_group ? "group" : "addr", address, fade_time);
    dali.lamp.setFadeTime(addr, (uint8_t)fade_time);
}

void DaliBusComponent::on_dali_set_power_on_level(int32_t address, int32_t level, int32_t is_group) {
    if (level < 0 || level > 255) { ESP_LOGW(TAG, "Level must be 0-255"); return; }
    if (is_group ? (address < 0 || address > 15) : (address < 0 || address > 63)) {
        ESP_LOGW(TAG, "Address out of range"); return;
    }
    short_addr_t addr = is_group ? (ADDR_GROUP | (uint8_t)address) : (short_addr_t)address;
    ESP_LOGI(TAG, "Service: set power_on_level %s %d level %d", is_group ? "group" : "addr", address, level);
    dali.lamp.setPowerOnLevel(addr, (uint8_t)level);
}

static text_sensor::TextSensor* create_diagnostic_text_sensor(const char* name_fmt, short_addr_t addr) {
    char name[48];
    snprintf(name, sizeof(name), name_fmt, addr);
    auto* ts = new text_sensor::TextSensor();
    ts->set_name(strdup(name));
    ts->set_entity_category(ENTITY_CATEGORY_DIAGNOSTIC);
    ts->set_disabled_by_default(false);
    App.register_text_sensor(ts);
    return ts;
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr);
    m_addresses[short_addr] = long_addr;

    // Apply bus-level defaults to auto-discovered lights
    if (m_default_fade_time.has_value()) {
        dali_light->set_fade_time(m_default_fade_time.value());
    }
    if (m_default_fade_rate.has_value()) {
        dali_light->set_fade_rate(m_default_fade_rate.value());
    }
    if (m_default_brightness_curve.has_value()) {
        dali_light->set_brightness_curve(m_default_brightness_curve.value());
    }

    // Query device type to auto-detect CCT support
    bool tc_capable = dali.color.isTcCapable(short_addr);
    if (tc_capable) {
        DALI_LOGI("Device %d supports color temperature (DT8 CCT)", short_addr);
        dali_light->set_color_mode(DaliColorMode::COLOR_TEMPERATURE);

        // Set cold/warm white temperature from device-reported TC range (mireds)
        uint16_t tc_coolest = dali.color.queryParameter(short_addr, DaliColorParam::ColourTemperatureTcCoolest);
        uint16_t tc_warmest = dali.color.queryParameter(short_addr, DaliColorParam::ColourTemperatureTcWarmest);
        if (tc_coolest > 0 && tc_coolest <= COLOR_MIREK_WARMEST &&
            tc_warmest > 0 && tc_warmest <= COLOR_MIREK_WARMEST) {
            dali_light->set_cold_white_temperature((float)tc_coolest);
            dali_light->set_warm_white_temperature((float)tc_warmest);
            DALI_LOGI("Device %d TC range: %d-%d mireds (%dK-%dK)",
                      short_addr, tc_coolest, tc_warmest,
                      1000000 / tc_coolest, 1000000 / tc_warmest);
        }
    } else {
        DALI_LOGI("Device %d: brightness only", short_addr);
    }

    const int MAX_STR_LEN = 32;
    char* name = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);

    auto* light_state = new light::LightState { dali_light };
    light_state->set_component_source(LOG_STR("light"));
    App.register_light(light_state);
    App.register_component(light_state);
    light_state->set_name(name);
    light_state->set_disabled_by_default(false);
    light_state->set_restore_mode(light::LIGHT_ALWAYS_ON);
    light_state->add_effects({});

    // Manually call setup() since we're registering after ESPHome's setup phase
    light_state->setup();

    // After setup, correct the HA state to match actual DALI device state
    // (restore mode may have set wrong state)
    {
        uint8_t actual = dali.lamp.getCurrentLevel(short_addr);
        dali_light->set_suppress_write(true);
        auto call = light_state->make_call();
        if (actual == 0) {
            call.set_state(false);
        } else {
            call.set_state(true);
            call.set_brightness((float)actual / 254.0f);
        }
        call.perform();
    }

    // Query group membership for bitmap filtering
    {
        uint8_t g07 = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_GROUPS_0_7);
        uint8_t g815 = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_GROUPS_8_15);
        m_group_mask[short_addr] = ((uint16_t)g815 << 8) | g07;
        if (m_group_mask[short_addr]) {
            DALI_LOGI("Device %d groups: 0x%04x", short_addr, m_group_mask[short_addr]);
        }
    }

    // Store for proxying loop() calls and bus listening
    m_dynamic_lights.push_back({light_state, dali_light, short_addr});

    DALI_LOGI("Created light '%s' @ addr %d (tc=%s)", name, short_addr, tc_capable ? "yes" : "no");

    // --- Create empty diagnostic text sensors (data populated later in loop()) ---
    DaliDiagSensors diag;
    diag.addr = short_addr;
    diag.long_addr = long_addr;
    diag.tc_capable = tc_capable;
    diag.device_type     = create_diagnostic_text_sensor("DALI %d Device Type", short_addr);
    diag.light_source    = create_diagnostic_text_sensor("DALI %d Light Source", short_addr);
    diag.version         = create_diagnostic_text_sensor("DALI %d Version", short_addr);
    diag.min_level       = create_diagnostic_text_sensor("DALI %d Min Level", short_addr);
    diag.max_level       = create_diagnostic_text_sensor("DALI %d Max Level", short_addr);
    diag.power_on_level  = create_diagnostic_text_sensor("DALI %d Power On Level", short_addr);
    diag.fade_time       = create_diagnostic_text_sensor("DALI %d Fade Time", short_addr);
    diag.fade_rate       = create_diagnostic_text_sensor("DALI %d Fade Rate", short_addr);
    diag.dimming_curve   = create_diagnostic_text_sensor("DALI %d Dimming Curve", short_addr);
    diag.groups          = create_diagnostic_text_sensor("DALI %d Groups", short_addr);
    if (tc_capable) {
        diag.color_temp_range = create_diagnostic_text_sensor("DALI %d Color Temp Range", short_addr);
    }
    m_diag_sensors.push_back(diag);

    ESP_LOGI(TAG, "Device %d: %d diagnostic sensors created (deferred)", short_addr, tc_capable ? 11 : 10);
}

/*******************************************************************************
 * Deferred diagnostic sensor population (state machine)
 *
 * Runs one DALI query per call (~15-50ms each). Called from loop() with
 * throttling so normal light control remains responsive.
 * Each step = 1 query → 1 sensor published immediately.
 *
 * Steps per device:
 *   0: QUERY_DEVICE_TYPE        → device_type
 *   1: QUERY_LIGHT_SOURCE_TYPE  → light_source
 *   2: QUERY_VERSION_NUMBER     → version
 *   3: QUERY_MIN_LEVEL          → min_level
 *   4: QUERY_MAX_LEVEL          → max_level
 *   5: QUERY_POWER_ON_LEVEL     → power_on_level
 *   6: QUERY_FADE_TIME_FADE_RATE → fade_time + fade_rate (1 query, 2 sensors)
 *   7: QUERY_DIMMING_CURVE      → dimming_curve
 *   8: QUERY_GROUPS_0_7 + 8_15  → groups (2 queries)
 *   9: TC coolest+warmest       → color_temp_range (if tc_capable, 2 queries)
 ******************************************************************************/

static const char* dali_source_name(uint8_t v) {
    switch (v) {
        case 0: return "Fluorescent";
        case 1: return "Emergency";
        case 2: return "HID";
        case 3: return "LV Halogen";
        case 4: return "Incandescent";
        case 5: return "Digital";
        case 6: return "LED";
        case 8: return "Color (DT8)";
        case 255: return "N/A";
        default: return "Unknown";
    }
}

static const char* dali_type_name(uint8_t v) {
    switch (v) {
        case 0: return "Fluorescent";
        case 1: return "Emergency";
        case 2: return "Discharge (HID)";
        case 3: return "LV Halogen";
        case 4: return "Dimmer";
        case 5: return "DC Converter";
        case 6: return "LED Driver";
        case 7: return "Relay";
        case 8: return "Color (DT8)";
        case 255: return "Multi-type";
        default: return "Unknown";
    }
}

static const char* fade_time_str[] = {
    "0 (none)", "0.7s", "1.0s", "1.4s", "2.0s", "2.8s", "4.0s", "5.7s",
    "8.0s", "11.3s", "16.0s", "22.6s", "32.0s", "45.3s", "64.0s", "90.5s"
};

void DaliBusComponent::diag_loop_step() {
    if (m_diag_done || m_diag_sensors.empty()) return;

    auto& ds = m_diag_sensors[m_diag_device_idx];
    short_addr_t addr = ds.addr;
    char buf[64];

    switch (m_diag_query_step) {
    case 0: {
        uint8_t v = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_DEVICE_TYPE);
        ds.device_type->publish_state(dali_type_name(v));
        break;
    }
    case 1: {
        uint8_t v = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_LIGHT_SOURCE_TYPE);
        ds.light_source->publish_state(dali_source_name(v));
        break;
    }
    case 2: {
        uint8_t v = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_VERSION_NUMBER);
        snprintf(buf, sizeof(buf), "%d.%d", v >> 4, v & 0x0F);
        ds.version->publish_state(buf);
        break;
    }
    case 3: {
        uint8_t v = dali.lamp.getMinLevel(addr);
        snprintf(buf, sizeof(buf), "%d", v);
        ds.min_level->publish_state(buf);
        break;
    }
    case 4: {
        uint8_t v = dali.lamp.getMaxLevel(addr);
        snprintf(buf, sizeof(buf), "%d", v);
        ds.max_level->publish_state(buf);
        break;
    }
    case 5: {
        uint8_t v = dali.lamp.getPowerOnLevel(addr);
        snprintf(buf, sizeof(buf), "%d", v);
        ds.power_on_level->publish_state(buf);
        break;
    }
    case 6: {
        // 1 query → 2 sensors (fade time in upper nibble, fade rate in lower)
        uint8_t v = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_FADE_TIME_FADE_RATE);
        uint8_t ft = (v >> 4) & 0x0F;
        uint8_t fr = v & 0x0F;
        ds.fade_time->publish_state(fade_time_str[ft]);
        snprintf(buf, sizeof(buf), "%d", fr);
        ds.fade_rate->publish_state(buf);
        break;
    }
    case 7: {
        uint8_t v = dali.port.sendExtendedQuery(addr, DaliLedCommand::QUERY_DIMMING_CURVE);
        ds.dimming_curve->publish_state(v == 1 ? "Linear" : "Logarithmic");
        break;
    }
    case 8: {
        // 2 queries → groups membership (16 groups encoded in 2 bytes)
        uint8_t g07 = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_GROUPS_0_7);
        uint8_t g815 = dali.port.sendQueryCommand(addr, DaliCommand::QUERY_GROUPS_8_15);
        uint16_t groups = ((uint16_t)g815 << 8) | g07;
        if (groups == 0) {
            ds.groups->publish_state("None");
        } else {
            int pos = 0;
            for (uint8_t g = 0; g < 16; g++) {
                if (groups & (1 << g)) {
                    if (pos > 0) pos += snprintf(buf + pos, sizeof(buf) - pos, ", ");
                    pos += snprintf(buf + pos, sizeof(buf) - pos, "%d", g);
                }
            }
            ds.groups->publish_state(buf);
        }

        // If no TC — advance to next device
        if (!ds.tc_capable) {
            m_diag_query_step = 0;
            m_diag_device_idx++;
            if (m_diag_device_idx >= m_diag_sensors.size()) {
                m_diag_done = true;
                ESP_LOGI(TAG, "All diagnostic sensors populated (%d devices)", m_diag_sensors.size());
            }
            return;
        }
        break;
    }
    case 9: {
        // 2 queries for TC range → 1 sensor
        uint16_t coolest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcCoolest);
        uint16_t warmest = dali.color.queryParameter(addr, DaliColorParam::ColourTemperatureTcWarmest);
        snprintf(buf, sizeof(buf), "%d-%d mireds (%dK-%dK)",
                 coolest, warmest,
                 coolest > 0 ? 1000000 / coolest : 0,
                 warmest > 0 ? 1000000 / warmest : 0);
        ds.color_temp_range->publish_state(buf);

        // Advance to next device
        m_diag_query_step = 0;
        m_diag_device_idx++;
        if (m_diag_device_idx >= m_diag_sensors.size()) {
            m_diag_done = true;
            ESP_LOGI(TAG, "All diagnostic sensors populated (%d devices)", m_diag_sensors.size());
        }
        return;
    }
    }

    m_diag_query_step++;
}

/*******************************************************************************
 * loop()
 ******************************************************************************/

/*******************************************************************************
 * Forward frame listening — detect commands from other DALI masters
 *
 * When another master (e.g. LTECH LT-424) sends a forward frame on the bus,
 * the ISR captures it. We decode it and update the corresponding LightState.
 ******************************************************************************/

void DaliBusComponent::check_bus_activity() {
    // Only check when we didn't just send something ourselves
    if (m_tx_active) return;
    if (m_rx_state.rxstate != 2) return;  // no completed frame

    // Copy ISR data and reset immediately to avoid race condition
    uint8_t rxpos = m_rx_state.rxpos;
    uint8_t raw_copy[DaliTimerRxState::RX_BUF_SIZE];
    memcpy(raw_copy, (const void*)m_rx_state.rxdata, rxpos);
    m_rx_state.reset();

    // Decode from local copy (ISR may already be filling new frame)
    uint8_t decoded[4] = {0};
    uint8_t bitlen = manDecode(raw_copy, rxpos, decoded);

    // Forward frames are 16 bits, backward frames are 8 bits
    if (bitlen < 16) return;

    uint8_t addr_byte = decoded[0];
    uint8_t data_byte = decoded[1];

    // Hex dump of raw ISR samples for debugging
    char hex[DaliTimerRxState::RX_BUF_SIZE * 3 + 1];
    int hpos = 0;
    for (uint8_t i = 0; i < rxpos && hpos < (int)sizeof(hex) - 3; i++) {
        hpos += snprintf(hex + hpos, sizeof(hex) - hpos, "%02x ", raw_copy[i]);
    }
    ESP_LOGD(TAG, "BUS: %d bits [0x%02x 0x%02x] from %d raw bytes: %s",
             bitlen, addr_byte, data_byte, rxpos, hex);

    // Parse address: YAAA AAAS
    // Y=0: individual short address, Y=1xx: group, Y=111: broadcast
    // S=0: DAPC (Direct Arc Power Control), S=1: command
    bool is_dapc = (addr_byte & 0x01) == 0;
    uint8_t raw_addr = addr_byte >> 1;  // 7-bit address field

    // Determine if this targets a known device
    bool is_broadcast = (raw_addr == ADDR_BROADCAST);
    bool is_group = !is_broadcast && ((raw_addr & ADDR_GROUP_MASK) == ADDR_GROUP);
    uint8_t group_id = is_group ? (raw_addr & 0x0F) : 0;
    short_addr_t short_addr = raw_addr;  // for individual addressing

    // Fire HA event for every forward frame from the bus
#ifdef USE_API_HOMEASSISTANT_SERVICES
    {
        char addr_str[8], data_str[8], group_str[8];
        snprintf(addr_str, sizeof(addr_str), "%d", is_group ? group_id : (int)short_addr);
        snprintf(data_str, sizeof(data_str), "%d", data_byte);

        std::string type;
        std::map<std::string, std::string> event_data;
        event_data["address"] = addr_str;
        event_data["is_broadcast"] = is_broadcast ? "true" : "false";
        event_data["is_group"] = is_group ? "true" : "false";

        if (is_dapc) {
            event_data["type"] = "dapc";
            event_data["level"] = data_str;
        } else {
            event_data["type"] = "command";
            event_data["command"] = data_str;
            if (data_byte >= 0x10 && data_byte <= 0x1F) {
                char scene_str[4];
                snprintf(scene_str, sizeof(scene_str), "%d", data_byte - 0x10);
                event_data["scene"] = scene_str;
            }
        }
        fire_homeassistant_event("esphome.dali_command", event_data);
    }
#endif

    if (is_dapc) {
        // DAPC: data_byte is the brightness level (0-254, 255=MASK)
        uint8_t level = data_byte;
        ESP_LOGD(TAG, "BUS: DAPC addr=0x%02x level=%d %s",
                 raw_addr, level,
                 is_broadcast ? "(broadcast)" : is_group ? "(group)" : "");

        for (auto& dl : m_dynamic_lights) {
            bool match = is_broadcast ||
                         (is_group && (m_group_mask[dl.addr] & (1 << group_id))) ||
                         (!is_group && !is_broadcast && dl.addr == short_addr);

            if (!match) continue;

            if (level > 0 && m_restore_state_on_toggle) {
                // Restore last HA state instead of accepting external brightness
                auto call = dl.state->make_call();
                call.set_state(true);
                call.perform();  // write_state() sends stored brightness+CCT to DALI
            } else {
                dl.output->set_suppress_write(true);
                auto call = dl.state->make_call();
                if (level == 0) {
                    call.set_state(false);
                } else {
                    call.set_state(true);
                    call.set_brightness((float)level / 254.0f);
                }
                call.perform();
            }
        }
    } else {
        // Command frame
        DaliCommand cmd = static_cast<DaliCommand>(data_byte);
        ESP_LOGD(TAG, "BUS: CMD addr=0x%02x cmd=0x%02x %s",
                 raw_addr, data_byte,
                 is_broadcast ? "(broadcast)" : is_group ? "(group)" : "");

        // Determine which commands affect light state
        bool is_off_cmd = (cmd == DaliCommand::OFF || cmd == DaliCommand::STEP_DOWN_AND_OFF);
        bool is_scene_cmd = (data_byte >= 0x10 && data_byte <= 0x1F);  // GO_TO_SCENE 0-15
        bool is_level_cmd = (cmd == DaliCommand::RECALL_MAX_LEVEL ||
                             cmd == DaliCommand::RECALL_MIN_LEVEL ||
                             cmd == DaliCommand::ON_AND_STEP_UP ||
                             cmd == DaliCommand::UP ||
                             cmd == DaliCommand::DOWN ||
                             cmd == DaliCommand::STEP_UP ||
                             cmd == DaliCommand::STEP_DOWN ||
                             cmd == DaliCommand::CONTINUOUS_UP ||
                             cmd == DaliCommand::CONTINUOUS_DOWN ||
                             cmd == DaliCommand::GO_TO_LAST_ACTIVE_LEVEL);

        if (!is_off_cmd && !is_level_cmd && !is_scene_cmd) return;  // not a state-changing command

        for (auto& dl : m_dynamic_lights) {
            bool match = is_broadcast ||
                         (is_group && (m_group_mask[dl.addr] & (1 << group_id))) ||
                         (!is_group && !is_broadcast && dl.addr == short_addr);

            if (!match) continue;

            // For all matching devices (individual, group, broadcast),
            // query actual level to get real state
            uint8_t actual = dali.lamp.getCurrentLevel(dl.addr);
            ESP_LOGD(TAG, "BUS: queried DALI %d actual level = %d", dl.addr, actual);
            if (actual == 0xFF) continue;  // no response

            dl.output->set_suppress_write(true);
            auto call = dl.state->make_call();
            if (actual == 0 || is_off_cmd) {
                call.set_state(false);
            } else {
                call.set_state(true);
                call.set_brightness((float)actual / 254.0f);
            }
            call.perform();
        }
    }
}

/*******************************************************************************
 * loop()
 ******************************************************************************/

void DaliBusComponent::loop() {
    // Check for forward frames from other DALI masters
    check_bus_activity();

    // Proxy loop() calls for dynamically created light components.
    for (auto& dl : m_dynamic_lights) {
        dl.state->loop();
    }

    // Deferred diagnostic sensor population — one DALI query per 100ms
    if (!m_diag_done && millis() - m_diag_last_query_ms >= 100) {
        m_diag_last_query_ms = millis();
        diag_loop_step();
    }
}

/*******************************************************************************
 * dump_config()
 ******************************************************************************/

void DaliBusComponent::dump_config() {
    static const char *const TAG = "dali";

    ESP_LOGCONFIG(TAG, "DALI Bus:");
    LOG_PIN("  TX Pin: ", m_txPin);
    LOG_PIN("  RX Pin: ", m_rxPin);
    ESP_LOGCONFIG(TAG, "  Discovery: %s", m_discovery ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Dynamic lights: %d", m_dynamic_lights.size());
    ESP_LOGCONFIG(TAG, "  Diag sensors: %d devices", m_diag_sensors.size());
    ESP_LOGCONFIG(TAG, "  assigned short addresses:");
    for (int i = 0; i < ADDR_SHORT_MAX; i++) {
        if (m_addresses[i] > 0) {
            ESP_LOGCONFIG(TAG, "   - %.2u = %.6x", i, m_addresses[i]);
        }
    }
}

/*******************************************************************************
 * TX / RX
 ******************************************************************************/

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::writeBit(bool bit) {
    // Manchester encoding:
    //   bit=1: first half = assert (bus low), second half = release (bus high)
    //   bit=0: first half = release (bus high), second half = assert (bus low)
    //
    // m_tx_inverted=true  (sehraf/opto-isolator): HIGH=assert, LOW=release
    // m_tx_inverted=false (Waveshare direct):     LOW=assert,  HIGH=release

    bool assert_level = m_tx_inverted ? HIGH : LOW;
    bool release_level = m_tx_inverted ? LOW : HIGH;

    #define OFFSET 10
    m_txPin->digital_write(bit ? assert_level : release_level);
    delayMicroseconds(HALF_BIT_PERIOD - OFFSET);
    m_txPin->digital_write(bit ? release_level : assert_level);
    delayMicroseconds(HALF_BIT_PERIOD - OFFSET);
}

void DaliBusComponent::writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    bool assert_level = m_tx_inverted ? HIGH : LOW;
    bool release_level = m_tx_inverted ? LOW : HIGH;
    m_txPin->digital_write(assert_level); // hold bus asserted
    delay(1000);
    m_txPin->digital_write(release_level); // release
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (DEBUG_LOG_RXTX) {
        ESP_LOGD(TAG, "TX: addr=0x%02x data=0x%02x (inv=%d, RX=%s)",
                 address, data, m_tx_inverted, m_rxPin->digital_read() ? "HIGH" : "LOW");
    }

    bool release_level = m_tx_inverted ? LOW : HIGH;

    // Minimum inter-frame gap: 22 half-bit periods (~9.17ms at 1200 baud)
    uint32_t min_gap_us = HALF_BIT_PERIOD * 22;
    uint32_t elapsed_us = (millis() - m_last_rx_ts) * 1000;
    if (elapsed_us < min_gap_us) {
        delayMicroseconds(min_gap_us - elapsed_us);
    }

    // Suppress RX sampling during TX to avoid collecting TX echo
    m_tx_active = true;

    {
        // This is timing critical
        InterruptLock lock;

        writeBit(1); // START bit
        writeByte(address);
        writeByte(data);
        m_txPin->digital_write(release_level); // bus idle
    }

    // Stop bits
    delayMicroseconds(HALF_BIT_PERIOD * 2);

    // TX done — reset RX state and enable sampling for backward frame
    m_rx_state.reset();
    m_tx_active = false;

    m_last_rx_ts = millis();
    delayMicroseconds(BIT_PERIOD * 4);
}


uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    // Wait for complete frame or timeout
    uint32_t start_time = millis();
    while (m_rx_state.rxstate != 2) { // 2 = COMPLETED
        delay(1);
        if (millis() - start_time > timeout_ms) {
            if (DEBUG_LOG_RXTX) {
                ESP_LOGD(TAG, "RX timeout (%lums), rxstate=%d, rxpos=%d, RX=%s",
                         timeout_ms,
                         m_rx_state.rxstate,
                         m_rx_state.rxpos,
                         m_rxPin->digital_read() ? "HIGH" : "LOW");
            }
            m_rx_state.reset();
            return 0;
        }
    }

    // We have a complete frame — decode Manchester
    uint8_t decoded[4] = {0};
    uint8_t bitlen = manDecode((const uint8_t*)m_rx_state.rxdata, m_rx_state.rxpos, decoded);

    if (DEBUG_LOG_RXTX) {
        ESP_LOGD(TAG, "RX: %d raw bytes, decoded %d bits => 0x%02x",
                 m_rx_state.rxpos, bitlen, decoded[0]);
    }

    m_rx_state.reset();

    if (bitlen >= 8) {
        delayMicroseconds(BIT_PERIOD * 7); // minimum inter-frame delay
        return decoded[0];
    }

    return 0;
}
