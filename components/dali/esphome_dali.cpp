#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

//static const char *const TAG = "dali";
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

void DaliInterruptState::gpio_intr(DaliInterruptState *queue) {

    if (queue->bitcount >= BACKWARD_FRAME_BIT_LENGTH) {
        // already received enough bits
        return;
    }

    // Read pin state and timestamp
    uint32_t ts = micros();
    bool level = queue->rx_pin->digital_read();

    if (queue->timestamp == 0) {
        // start bit (should be low)
        queue->frame <<= 1;
        queue->frame |= level ? BI_PHASE_HIGH : 0;
        queue->bitcount++;
    } else {
        // ongoing reception
        uint32_t diff = ts - queue->timestamp;

        // Check the pulse width for TE or 2TE time
        if (MIN_2TE < diff && diff < MAX_2TE) {
            // This is a 2TE pulse, we need to shift to bits shift both bits to the left...
            queue->frame <<= 2;

            if (level)
                queue->frame |= BI_PHASE_HIGH;
            else
                queue->frame |= BI_PHASE_LOW;

            // increment position counter by 2
           queue->bitcount += 2;
        } else if (MIN_TE < diff && diff < MAX_TE) {
            // This is a TE pulse, we just need to shift one bit == current LEVEL
            // shift one bit to the left...
            queue->frame <<= 1;

            if (level)
                queue->frame |= BI_PHASE_HIGH;

            // Increment position counter by 1
            queue->bitcount += 1;
        }
        else
        {
            // This pulse is not a valid DALI bi-phase pulse. We stop receiving
            // by setting the state to DALI_PD_STATE_IDLE and wait for the next one.
            // The idle timeout (MR0) will set us back to receive mode...
            queue->bitcount = BACKWARD_FRAME_BIT_LENGTH; // force end of reception
            queue->frame = 0;
        }
    }

    // Save timestamp for next edge
    queue->timestamp = ts;
}

void DaliInterruptState::reset()
{
    this->frame = 0;
    this->bitcount = 0;
    this->timestamp = 0;
}

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

void DaliBusComponent::setup() {
    m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
    m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);

    this->m_interrupt_state.rx_pin = m_rxPin;
    this->m_interrupt_state.reset();
    this->armInterrupt();

    DALI_LOGI("DALI bus ready");

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
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr);
    // the previous call overrides the long address with `0xffff`, set it again, since we know it here
    m_addresses[short_addr] = long_addr;

    const int MAX_STR_LEN = 20;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
    // NOTE: Not freeing these strings, they will be owned by LightState.

    auto* light_state = new light::LightState { dali_light };
    light_state->set_component_source(LOG_STR("light"));
    App.register_light(light_state);
    App.register_component(light_state);
    light_state->set_name(name);
    light_state->set_object_id(id);
    light_state->set_disabled_by_default(false);
    light_state->set_restore_mode(light::LIGHT_RESTORE_DEFAULT_ON);
    light_state->add_effects({});

    DALI_LOGI("Created light component '%s' (%s)", name, id);
#else
    // Make sure you set discovery: true, or specify a light component somewhere in your YAML!
    DALI_LOGE("Cannot add light component - not enabled");
#endif
}

void DaliBusComponent::loop() {
    this->disable_loop();
}

void DaliBusComponent::dump_config() {
    static const char *const TAG = "dali";

    ESP_LOGCONFIG(TAG, "DALI Bus:");
    LOG_PIN("  TX Pin: ", m_txPin);
    LOG_PIN("  RX Pin: ", m_rxPin);
    ESP_LOGCONFIG(TAG, "  assigned short addresses:");
    for (int i = 0; i < ADDR_SHORT_MAX; i++) {
        if (m_addresses[i] > 0) {
            ESP_LOGCONFIG(TAG, "   - %.2u = %.6x\n", i, m_addresses[i]);
        }
    }
}

void inline DaliBusComponent::armInterrupt() {
    this->m_rxPin->attach_interrupt(DaliInterruptState::gpio_intr, &this->m_interrupt_state, gpio::INTERRUPT_ANY_EDGE);
}

void inline DaliBusComponent::disarmInterrupt() {
    this->m_rxPin->detach_interrupt();
}

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::writeBit(bool bit) {
    #define OFFSET 10
    // NOTE: output is inverted - HIGH will pull the bus to 0V (logic low)
    bit = !bit;
    m_txPin->digital_write(bit ? LOW : HIGH);
    delayMicroseconds(HALF_BIT_PERIOD-OFFSET);
    m_txPin->digital_write(bit ? HIGH : LOW);
    delayMicroseconds(HALF_BIT_PERIOD-OFFSET);
}

void DaliBusComponent::writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    m_txPin->digital_write(HIGH);
    delay(1000);
    m_txPin->digital_write(LOW);
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("TX: %02x %02x", address, data);
    }

    // Minimum time before we can read a backward frame
    uint32_t delay = millis() - m_last_rx_ts;
    if (delay < (HALF_BIT_PERIOD*22)/1000) { // _BIT_PERIOD is in us, convert to ms
        delayMicroseconds(delay);
    }

    this->disarmInterrupt();
    this->m_interrupt_state.reset();
    {
        // This is timing critical
        InterruptLock lock;

        writeBit(1); // START bit
        writeByte(address);
        writeByte(data);
        m_txPin->digital_write(LOW);
    }

    // Non critical delay
    delayMicroseconds(HALF_BIT_PERIOD*2);
    this->armInterrupt();
    m_last_rx_ts = millis();
    delayMicroseconds(BIT_PERIOD*4); // Optional, for clarity in scope trace
}


uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    uint8_t data = 0;

    // Wait for complete frame or timeout
    uint32_t start_time = millis();
    while (this->m_interrupt_state.bitcount < BACKWARD_FRAME_BIT_LENGTH) {
        // Wait for complete frame or timeout
        delay(1);
        if (millis() - start_time > timeout_ms) {
            if (DEBUG_LOG_RXTX) {
                DALI_LOGD("DALI: receiveBackwardFrame timeout");
            }
            return 0; // timeout
        }
    }

    // copy received data
    uint32_t raw_data = this->m_interrupt_state.frame;
    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX raw: %03x", raw_data);
    }

    for (int i = 0; i < 8; i++)
    {
        data >>= 1;
        switch (raw_data & BI_PHASE_MASK)
        {
        case BI_PHASE_HIGH:
            // We shift a 1 into backward_frame from MSB to LSB position
            data |= 0x80;
            break;
        case BI_PHASE_LOW:
            break;
        default:
            return false;
        }
        raw_data >>= 2;
    }

    this->m_interrupt_state.reset();

    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX: %02x", data);
    }

    delayMicroseconds(BIT_PERIOD*7); // Minimum time before we can send another forward frame
    return data;
}
