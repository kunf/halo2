#pragma once

#include <array>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "lr1121.h"
#include "protocol.h"

#ifdef USE_LIGHT
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace halo2 {

enum class Lamp : uint8_t { FRONT, BACK };
enum class SwitchKind : uint8_t { POWER, ULTRASONIC };
enum class ButtonKind : uint8_t { AUTO, DISCOVER };

class Halo2Light;
class Halo2Switch;

/// Bridge between Home Assistant and a BenQ ScreenBar HALO 2 lamp. Port of
/// src/main.py and src/benq_halo/__init__.py.
///
/// The lamp only talks in replies to our packets, so its state is polled every
/// `poll_interval`. Packets between the remote control and the lamp are sniffed in
/// between. A change from Home Assistant is queued, sent once, and then confirmed by
/// polling every 500 ms until the lamp reports it (or 5 polls pass).
class Halo2Component : public Component, public Halo2SPIDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_reset_pin(GPIOPin *pin) { this->radio_.set_reset_pin(pin); }
  void set_busy_pin(GPIOPin *pin) { this->radio_.set_busy_pin(pin); }
  void set_irq_pin(GPIOPin *pin) { this->radio_.set_irq_pin(pin); }
  void set_tcxo_voltage_code(uint8_t code) { this->radio_.set_tcxo_voltage_code(code); }
  void set_address(const Address &register_order);
  void set_poll_interval(uint32_t ms) { this->poll_interval_ = ms; }

#ifdef USE_LIGHT
  void register_light(Lamp lamp, Halo2Light *light);
#endif
#ifdef USE_SWITCH
  void register_switch(SwitchKind kind, Halo2Switch *sw);
#endif
#ifdef USE_TEXT_SENSOR
  void set_address_text_sensor(text_sensor::TextSensor *s) { this->address_sensor_ = s; }
  void set_status_text_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }
#endif

  // Called by the entities on a Home Assistant command
  void on_light_command(Lamp lamp, bool on, uint8_t brightness, uint16_t color_temp);
  void on_switch_command(SwitchKind kind, bool on);
  void on_button_press(ButtonKind kind);

  /// Entity state derived from the lamp state
  bool light_is_on(Lamp lamp) const;
  uint8_t light_brightness(Lamp lamp) const;
  uint16_t color_temp() const { return this->state_.color_temp; }
  bool switch_state(SwitchKind kind) const;

 protected:
  enum class Mode : uint8_t { DISCOVERY, RUN };

  void start_discovery_();
  void loop_discovery_();
  void finish_discovery_(const Address &address);
  void use_address_(const Address &address);

  void request_update_(uint8_t command, bool auto_mode = false);
  bool is_syncing_() const { return this->pending_command_ != 0xFF || this->sync_polls_ > 0; }
  void process_updates_();
  bool exchange_(uint8_t command, const LampState &state, Payload *reply, bool auto_mode = false);
  void poll_lamp_(uint8_t command);
  void sniff_();
  void apply_lamp_state_(const LampState &state, const char *source);
  void publish_entities_(bool force = false);
  void set_status_(const char *status);

  LR1121Radio radio_;
  ESPPreferenceObject address_pref_;
  Address address_{};  // On-air order
  bool address_from_config_{false};
  bool have_address_{false};
  Mode mode_{Mode::RUN};

  // Address discovery: majority vote over 5 captures, like find_halo2_address.py
  std::array<Address, 5> discovery_hits_{};
  uint8_t discovery_count_{0};
  uint32_t discovery_last_log_{0};

  LampState state_{};      // Best knowledge of the lamp (and our requested changes)
  bool ready_{false};      // Set after the first poll; commands are ignored before that
  int8_t lamp_reachable_{-1};  // -1: not reported yet
  uint32_t poll_interval_{5000};
  uint32_t last_poll_{0};
  uint32_t last_reply_{0};
  uint32_t last_undecoded_log_{0};
  uint32_t last_osc_check_{0};
  uint8_t pid_{0};

  // Queued update (request_update / process_updates in benq_halo/__init__.py)
  uint8_t pending_command_{0xFF};  // 0xFF: none
  bool pending_auto_{false};
  Payload sync_payload_{};
  uint8_t sync_polls_{0};
  uint32_t last_sync_poll_{0};
  bool have_sync_reply_{false};
  Payload last_sync_reply_{};

  const char *status_{nullptr};

#ifdef USE_LIGHT
  std::array<Halo2Light *, 2> lights_{};
#endif
#ifdef USE_SWITCH
  std::array<Halo2Switch *, 2> switches_{};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *address_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
#endif
};

#ifdef USE_LIGHT
class Halo2Light : public light::LightOutput {
 public:
  Halo2Light(Halo2Component *parent, Lamp lamp) : parent_(parent), lamp_(lamp) {}
  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override { this->state_ = state; }
  void write_state(light::LightState *state) override;
  /// Show the lamp's state in Home Assistant. The resulting write_state() call matches
  /// the lamp state, so nothing is sent back to the lamp.
  void publish(bool on, uint8_t brightness, uint16_t color_temp, bool force = false);

 protected:
  struct Published {
    bool valid{false};
    bool on{false};
    uint8_t brightness{0};
    uint16_t color_temp{0};
    bool operator==(const Published &o) const {
      return valid == o.valid && on == o.on && brightness == o.brightness && color_temp == o.color_temp;
    }
  };

  Halo2Component *parent_;
  Lamp lamp_;
  light::LightState *state_{nullptr};
  Published last_{};
};
#endif

#ifdef USE_SWITCH
class Halo2Switch : public switch_::Switch {
 public:
  Halo2Switch(Halo2Component *parent, SwitchKind kind) : parent_(parent), kind_(kind) {}

 protected:
  void write_state(bool state) override;
  Halo2Component *parent_;
  SwitchKind kind_;
};
#endif

#ifdef USE_BUTTON
class Halo2Button : public button::Button {
 public:
  Halo2Button(Halo2Component *parent, ButtonKind kind) : parent_(parent), kind_(kind) {}

 protected:
  void press_action() override { this->parent_->on_button_press(this->kind_); }
  Halo2Component *parent_;
  ButtonKind kind_;
};
#endif

}  // namespace halo2
}  // namespace esphome
