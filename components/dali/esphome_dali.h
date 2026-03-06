#pragma once

#include <esphome.h>
#include <vector>
#include "esphome/components/light/light_state.h"
#include "esphome/components/text_sensor/text_sensor.h"
#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif
#include "dali.h"

namespace esphome {
namespace dali {

class DaliLight;  // forward declaration

enum class DaliInitMode {
    DiscoverOnly,
    InitializeUnassigned,
    InitializeAll
};

struct DaliTimerRxState {
    // Bufor próbek RX — max 32 bajtów (256 próbek = 32 bitów * 8 oversample)
    // Backward frame: 1 start + 8 data + 2 stop = 11 bitów * 8 = 88 próbek = 11 bajtów
    // Forward frame (nasłuch): 1 start + 16 data + 2 stop = 19 bitów * 8 = 152 = 19 bajtów
    // Z marginesem: 32 bajty wystarczy
    static constexpr uint8_t RX_BUF_SIZE = 32;
    volatile uint8_t rxdata[RX_BUF_SIZE];
    volatile uint8_t rxpos;        // pozycja w rxdata (bajt)
    volatile uint8_t rxbitcnt;     // bit counter w bieżącym bajcie (0-7)
    volatile uint8_t rxbyte;       // bieżący bajt (budowany z próbek)
    volatile uint8_t rxidle;       // licznik kolejnych HIGH próbek (stop detection)
    volatile uint8_t rxstate;      // 0=EMPTY, 1=RECEIVING, 2=COMPLETED

    InternalGPIOPin* rx_pin{nullptr};

    void reset();
};


class DaliBusComponent : public Component, public DaliPort
#ifdef USE_API_CUSTOM_SERVICES
    , public api::CustomAPIDevice
#endif
{
public:
    DaliBusComponent()
        : Component { }
        , dali { *this }
    { }

    void setup() override;
    void loop() override;
    void dump_config() override;

    void set_tx_pin(GPIOPin* tx_pin) { m_txPin = tx_pin; }
    void set_rx_pin(InternalGPIOPin* rx_pin) { m_rxPin = rx_pin; }

    /// @brief Perform automatic device discovery on setup.
    /// Light components will automatically be created and appear in HomeAssistant
    void do_device_discovery() { m_discovery = true; }

    /// @brief Initialize long and short addresses for devices on the bus.
    /// @param mode
    //          InitializeUnassigned - only devices that do not yet have an assigned short address
    ///         InitializeAll - all devices on the bus
    /// @note
    void do_initialize_addresses(DaliInitMode mode = DaliInitMode::InitializeUnassigned) { m_initialize_addresses = mode; }

    void set_default_fade_time(uint16_t v) { m_default_fade_time = v; }
    void set_default_fade_rate(uint16_t v) { m_default_fade_rate = v; }
    void set_default_brightness_curve(DaliLedDimmingCurve v) { m_default_brightness_curve = v; }
    void set_restore_state_on_toggle(bool v) { m_restore_state_on_toggle = v; }

    // NOTE: Must have a higher priority number than the components that depend on this.
    // ie, this must be initialized first.
    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    void register_static_addr(short_addr_t short_addr) {
        if (short_addr < ADDR_SHORT_MAX) {
            m_addresses[short_addr] = 0xFFFFFF;
        }
    }

    DaliMaster dali;

public: // DaliPort
    void resetBus() override;
    void sendForwardFrame(uint8_t address, uint8_t data) override;
    uint8_t receiveBackwardFrame(unsigned long timeout_ms = 100) override;

private:
    void writeBit(bool bit);
    void writeByte(uint8_t b);
    uint8_t readByte();

    static void IRAM_ATTR onTimerISR();
    void setupTimer();

    static uint8_t manWeight(uint8_t sample);
    static uint8_t manSample(const uint8_t* data, uint16_t bitpos, uint8_t* stop_coll);
    uint8_t manDecode(const uint8_t* edata, uint8_t ebitlen, uint8_t* ddata);

    void create_light_component(short_addr_t short_addr, uint32_t long_addr);
    void dump_device_info(short_addr_t addr);
    void register_services();

    // Service call handlers
    void on_dali_add_to_group(int32_t address, int32_t group);
    void on_dali_remove_from_group(int32_t address, int32_t group);
    void on_dali_store_scene(int32_t address, int32_t scene, int32_t is_group);
    void on_dali_remove_scene(int32_t address, int32_t scene, int32_t is_group);
    void on_dali_go_to_scene(int32_t address, int32_t scene, int32_t is_group);
    void on_dali_set_brightness(int32_t address, int32_t level, int32_t is_group);
    void on_dali_identify(int32_t address, int32_t is_group);
    void on_dali_set_fade_time(int32_t address, int32_t fade_time, int32_t is_group);
    void on_dali_set_power_on_level(int32_t address, int32_t level, int32_t is_group);

    InternalGPIOPin* m_rxPin;
    GPIOPin* m_txPin;
    uint32_t m_last_rx_ts = 0;

    bool m_discovery = false;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};
    uint16_t m_group_mask[ADDR_SHORT_MAX+1] = {0};  // per-device group membership bitmap (bit N = group N)

    // Default parameters for auto-discovered lights
    optional<uint16_t> m_default_fade_time{};
    optional<uint16_t> m_default_fade_rate{};
    optional<DaliLedDimmingCurve> m_default_brightness_curve{};
    bool m_restore_state_on_toggle{false};

    DaliTimerRxState m_rx_state;
    hw_timer_t* m_timer{nullptr};
    volatile bool m_tx_active{false}; // ISR ignores samples when true
    bool m_tx_inverted{true}; // true = HIGH=assert (opto-isolator), false = LOW=assert (Waveshare)

    // Dynamically created lights — we proxy loop() calls since ESPHome's
    // looping_components_ FixedVector may not include late-registered components
    struct DynamicLight {
        light::LightState* state;
        DaliLight* output;
        short_addr_t addr;
    };
    std::vector<DynamicLight> m_dynamic_lights;

    // Forward frame listening — detect commands from other DALI masters
    void check_bus_activity();

    // --- Deferred text sensor population (state machine in loop()) ---
    struct DaliDiagSensors {
        short_addr_t addr;
        uint32_t long_addr;
        bool tc_capable;
        // Individual sensors — one per parameter
        text_sensor::TextSensor* device_type{nullptr};
        text_sensor::TextSensor* light_source{nullptr};
        text_sensor::TextSensor* version{nullptr};
        text_sensor::TextSensor* min_level{nullptr};
        text_sensor::TextSensor* max_level{nullptr};
        text_sensor::TextSensor* power_on_level{nullptr};
        text_sensor::TextSensor* fade_time{nullptr};
        text_sensor::TextSensor* fade_rate{nullptr};
        text_sensor::TextSensor* dimming_curve{nullptr};
        text_sensor::TextSensor* groups{nullptr};
        text_sensor::TextSensor* color_temp_range{nullptr};  // only if tc_capable
    };
    std::vector<DaliDiagSensors> m_diag_sensors;
    uint8_t m_diag_device_idx{0};   // which device we're querying
    uint8_t m_diag_query_step{0};   // which query step within a device
    uint32_t m_diag_last_query_ms{0};
    bool m_diag_done{false};        // all sensors populated

    void diag_loop_step();  // one query step per call
};

}  // namespace dali
}  // namespace esphome
