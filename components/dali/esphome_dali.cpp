#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

//static const char *const TAG = "dali";
static const bool DEBUG_LOG_RXTX = false;
static const bool DEBUG_LOG_RXTX_FULL = false;

using namespace esphome;
using namespace dali;

void DaliInterrupt::gpio_intr(DaliInterrupt *queue) {
    // Ignore if not initialized
    if (!queue->init) return;

    // Ignore if queue is full
    if (queue->received_queue_pos >= NUM_ENTRIES) return;

    // Read pin state and timestamp
    uint32_t ts = micros();
    bool level = queue->rx_pin->digital_read();

    // Add to queue
    queue->received_queue[queue->received_queue_pos] = { ts, level };

    // Advance position until end is reached, do not reset
    if (queue->received_queue_pos < NUM_ENTRIES) {
        queue->received_queue_pos++;
    }
}

void DaliInterrupt::reset()
{
    // cheap way to disable interrupts while resetting
    this->init = false;

    std::memset(this->received_queue, 0, sizeof(this->received_queue));
    this->received_queue_pos = 0;

    this->init = true;
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

    this->m_interrupt_queue.rx_pin = m_rxPin;
    this->m_interrupt_queue.reset();
    this->armInterrupt();

    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
            // {
            //     // reset everything and initialize all devices
            //     this->dali.reset(ADDR_BROADCAST);
            //     this->m_initialize_addresses = DaliInitMode::InitializeAll
            // }
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGW("No control gear detected on bus!");
        }

        // for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
        //     if (m_addresses[i] != 0) {
        //         DALI_LOGD("Static config addr: %.2x", i);
        //     }
        // }

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
    light_state->set_component_source("light");
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
    this->m_rxPin->attach_interrupt(DaliInterrupt::gpio_intr, &this->m_interrupt_queue, gpio::INTERRUPT_ANY_EDGE);
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

uint8_t DaliBusComponent::readByte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= m_rxPin->digital_read();
        delayMicroseconds(BIT_PERIOD); // 1/1200 seconds
    }
    return byte;
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
        // delayMicroseconds(BIT_PERIOD*8);
        //Serial.print("TX: "); Serial.print(address, HEX); Serial.print(" "); Serial.println(data, HEX);
    }

    // Minimum time before we can read a backward frame
    uint32_t delay = millis() - m_last_rx_ts;
    if (delay < (HALF_BIT_PERIOD*22)/1000) { // _BIT_PERIOD is in us, convert to ms
        delayMicroseconds(delay);
    }

    this->disarmInterrupt();
    this->m_interrupt_queue.reset();
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


const uint32_t DELTA = 41; // 10% us tolerance
// const uint32_t DELTA = 60; // FIXME
/// @brief Check if the time difference is a valid short or long period.
/// @param diff Time difference in microseconds.
/// @param long Set to true if the period is a long period, false if short period.
/// @return true if the period is valid, false otherwise.
bool valid_diff_period (uint32_t diff, bool &is_long) {
    if (diff >= HALF_BIT_PERIOD - DELTA && diff <= HALF_BIT_PERIOD + DELTA) {
        // valid short level change (same symbol / entry transition)
        is_long = false;
        return true;
    } else if (diff >= BIT_PERIOD - DELTA && diff <= BIT_PERIOD + DELTA) {
        // valid long level change (different symbol / no entry transition)
        is_long = true;
        return true;
    }
    return false; // timing weird
};

void dump_queue(DaliInterrupt &queue) {
    DALI_LOGD("  RX[%02d]: %10u %5s", 0, queue.received_queue[0].ts, queue.received_queue[0].level ? "HIGH" : "LOW");
    bool lng = false;
    for (size_t i = 1; i < queue.received_queue_pos; i++) {
        uint32_t diff = queue.received_queue[i].ts - queue.received_queue[i-1].ts;
        DALI_LOGD("  RX[%02d]: %10u %5s %7u %6s", i, queue.received_queue[i].ts, queue.received_queue[i].level ? "HIGH" : "LOW", diff, (valid_diff_period(diff, lng) ? (lng ? "LONG" : "SHORT") : "INVAL") );
    }
}

void filter_flukes(DaliInterrupt &queue) {
    #define THRESHOLD 100 // us
    // remove glitches (very short pulses)
    for (size_t i = 1; i < queue.received_queue_pos; i++) {
        uint32_t diff = queue.received_queue[i].ts - queue.received_queue[i-1].ts;
        if (diff < THRESHOLD) {
            // remove this entry by shifting all following entries one position to the left
            for (size_t j = i; j < queue.received_queue_pos-1; j++) {
                queue.received_queue[j] = queue.received_queue[j+1];
            }
            queue.received_queue[queue.received_queue_pos-1] = {0, false};
            queue.received_queue_pos--;
            i--; // recheck this position
        }
    }
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    /*
    SIGNAL CHARACTERISTICS
    High Level: 9.5 to 22.5 V (Typical 16 V)
    Low Level: -6.5 to + 6.5 V (Typical 0 V)
    Te = half cycle = 416.67 us +/- 10 %
    10 us <= tfall <= 100 us
    10 us <= trise <= 100 us

    BIT TIMING
    msb send first
    logical 1 = 1Te Low 1Te High
    logical 0 = 1Te High 1Te Low
    Start bit = logical 1
    Stop bit = 2Te High

    FRAME TIMING
    FF: TX Forward Frame 2 bytes (38Te) = 2*(1start+16bits+2stop)
    BF: RX Backward Frame 1 byte (22Te) = 2*(1start+8bits+2stop)
    no reply: FF >22Te pause FF
    with reply: FF >7Te <22Te pause BF >22Te pause FF
    */

    // Using interrupt-driven reception instead of timing-critical polling
    DaliInterrupt &queue = this->m_interrupt_queue;
    uint8_t bits_received = 0; // number of bits received so far, max 8
    bool last_level = false;   // used for sanity checks
    bool long_out = false;     // if the symbol ends with a long period
    bool long_in = false;      // if the symbol starts with a long period
    uint32_t diff_out = 0;
    uint32_t diff_in = 0;
    uint8_t data = 0;

    // pos should always point to the middle of the symbol which always exists (due to Manchester encoding)
    // The start bit starts with a high to low transition since the bus idle state is high.
    // Therefore the position 0 is the begin of the start bit and position 1 is the middle of the start symbol.
    size_t pos = 1;

    // wait for enough data or timeout
    uint32_t startTime = millis();
    uint32_t last_received_us = queue.received_queue[queue.received_queue_pos-1].ts;
    // 10 is the minumum number of transitions possible
    // FIXME: right now the code does not handle waiting periods good enough, therefore check if we are receiving data right now
    while (queue.received_queue_pos < 10 || micros() - last_received_us < BIT_PERIOD) {
        // don't exit if we are still receiving data
        if (millis() - startTime >= timeout_ms && micros() - last_received_us >= BIT_PERIOD * 2) {
            //Serial.println("No reply");
            if (DEBUG_LOG_RXTX) {
                DALI_LOGD("RX: 00 (NACK, timeout) queue len = %u, %u us", queue.received_queue_pos, (micros() - queue.received_queue[queue.received_queue_pos-1].ts));
            }
            return 0;
        }
        delay(1);
        last_received_us = queue.received_queue[queue.received_queue_pos-1].ts;
    }

    if (DEBUG_LOG_RXTX_FULL) {
        dump_queue(queue);
    }
    filter_flukes(queue);
    if (DEBUG_LOG_RXTX_FULL) {
        DALI_LOGD("After filtering:");
        dump_queue(queue);
    }

    // find start bit, first to be low level
    // TODO: is this still needed? Haven't seen this being hit in a while
    while (queue.received_queue[pos-1].level != LOW) {
        DALI_LOGD("RX: searching start bit, skipped one");
        pos++;
    }

    // expect start bit = logical 1
    diff_in = queue.received_queue[pos].ts - queue.received_queue[pos-1].ts;
    diff_out = queue.received_queue[pos+1].ts - queue.received_queue[pos].ts;
    if (!valid_diff_period(diff_in, long_in) || !valid_diff_period(diff_out, long_out)) {
        // timing weird (period does not match)
        DALI_LOGW("RX: 00 (NACK, no start bit, timing) pos = %u", pos);
        return 0; // no start bit
    }
    if (long_in) {
        // timing weird (this transition is supposed to be short = HALF_BIT_PERIOD)
        DALI_LOGW("RX: 00 (NACK, no start bit, timing long_in) pos = %u", pos);
        return 0; // no start bit
    }
    if (queue.received_queue[pos-1].level == HIGH || queue.received_queue[pos].level == LOW) {
        // gpio level weird
        DALI_LOGW("RX: 00 (NACK, no start bit, level) pos = %u", pos);
        return 0; // no start bit
    }

    pos += long_out ? 1 : 2; // move pos to the middle of the next symbol
    last_level = HIGH; // end of start bit

    // expect data bits
    for (;bits_received < 8;) {
        // diff to previous entry
        diff_in = queue.received_queue[pos].ts - queue.received_queue[pos-1].ts;
        if (valid_diff_period(diff_in, long_in)) {
            // timings are valid

            // sanity checks:
            // depending on the previous symbol, there night be a level change at the start of the current symbol or not
            if (long_in == false) {
                // short period, therefore level change at the beginning of the current symbol
                // -> should be the same symbol than before
                if (queue.received_queue[pos-1].level == last_level
                    || queue.received_queue[pos].level != last_level) {
                    // level did not change, but should have
                    DALI_LOGW("RX: 00 (NACK, level did not change) pos = %u", pos);
                    dump_queue(queue);
                    return 0;
                }
            } else {
                // long period, therefore no level change at the beginning of the current symbol
                // -> should be a different symbol than before
                if (queue.received_queue[pos-1].level != last_level
                    || queue.received_queue[pos].level == last_level) {
                    // level did change, but should not have
                    DALI_LOGW("RX: 00 (NACK, level changed) pos = %u", pos);
                    dump_queue(queue);
                    return 0;
                }
            }

            data = (data << 1) | (queue.received_queue[pos-1].level == LOW ? 1 : 0);
            bits_received++;
            if (DEBUG_LOG_RXTX_FULL) {
                DALI_LOGD("  RX: bit %d = %d", bits_received, data & 0x1);
            }

            if (queue.received_queue[pos+1].ts != 0) {
                diff_out = queue.received_queue[pos+1].ts - queue.received_queue[pos].ts;
                valid_diff_period(diff_out, long_out);
            } else {
                // edge case, end of byte
                // the stop bit is a long level high, therefore the usual timings don't fit
                // only hit when the current symbol ends with a high level
                if (bits_received != 8) {
                    DALI_LOGW("RX: 00 (NACK, byte incomplete) pos = %u", pos);
                    return 0; // byte incomplete
                }
                break;
            }

            last_level = queue.received_queue[pos].level;
            pos += long_out ? 1 : 2; // move pos to the middle of the next symbol
        } else {
            // timing weird
            DALI_LOGW("RX: 00 (NACK, timing weird) pos = %u", pos);
            dump_queue(queue);
            return 0;
        }
    }

    // expect stop bits
    // we have no entries for this

    // cleanup and prepare for next reception
    this->m_interrupt_queue.reset();

    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX: %02x", data);
    }

    delayMicroseconds(BIT_PERIOD*7); // Minimum time before we can send another forward frame
    return data;
}
