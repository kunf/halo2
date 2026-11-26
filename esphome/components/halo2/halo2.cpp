#include "halo2.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace halo2 {

static const char *const TAG = "halo2";

static constexpr uint8_t NO_COMMAND = 0xFF;
static constexpr uint32_t SYNC_POLL_INTERVAL_MS = 500;  // Between polls while the lamp applies an update
static constexpr uint8_t SYNC_MAX_POLLS = 5;
static constexpr uint8_t COMMAND_ATTEMPTS = 3;          // Sends of a command without a reply            // Give up confirming an update after this
static constexpr uint32_t UNREACHABLE_AFTER_MS = 30000;
static constexpr uint32_t DISCOVERY_HINT_INTERVAL_MS = 15000;

static void format_address(const Address &air, char *buf, size_t len) {
  // Register order, the same notation as HALO2_ADDRESS in halo2_address.py
  snprintf(buf, len, "0x%02x, 0x%02x, 0x%02x, 0x%02x", air[3], air[2], air[1], air[0]);
}

void Halo2Component::set_address(const Address &register_order) {
  this->address_ = {register_order[3], register_order[2], register_order[1], register_order[0]};
  this->address_from_config_ = true;
}

void Halo2Component::setup() {
  this->spi_setup();
  this->radio_.set_spi(this);
  this->address_pref_ = global_preferences->make_preference<Address>(fnv1_hash("halo2_address"));

  Address initial = this->address_;
  bool have_address = this->address_from_config_;
  if (!have_address) {
    Address saved{};
    if (this->address_pref_.load(&saved)) {
      initial = saved;
      have_address = true;
    }
  }

  if (!this->radio_.init(have_address ? initial : DISCOVERY_SYNC_WORD)) {
    this->status_set_error(LOG_STR("LR1121 setup failed"));
    this->set_status_("Radio error: LR1121 setup failed");
    this->mark_failed();
    return;
  }

  if (have_address) {
    this->use_address_(initial);
  } else {
    this->start_discovery_();
  }
  // First poll a second after boot
  this->last_poll_ = millis() - this->poll_interval_ + 1000;
}

void Halo2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BenQ ScreenBar HALO 2 (LR1121):");
  const auto &v = this->radio_.version();
  ESP_LOGCONFIG(TAG, "  LR1121 version: hw %02X, use case %02X, firmware %u.%02u", v[0], v[1], v[2], v[3]);
  static const char *const VOLTAGES[] = {"1.6", "1.7", "1.8", "2.2", "2.4", "2.7", "3.0", "3.3"};
  const uint8_t tcxo = this->radio_.active_tcxo_code();
  if (tcxo <= 7) {
    ESP_LOGCONFIG(TAG, "  Clock: TCXO %s V", VOLTAGES[tcxo]);
  } else {
    ESP_LOGCONFIG(TAG, "  Clock: crystal");
  }
  if (this->radio_.tx_check_ok()) {
    ESP_LOGCONFIG(TAG, "  TX check: OK");
  } else {
    ESP_LOGE(TAG, "  TX check FAILED (errors 0x%04X)%s", this->radio_.tx_check_errors(),
             (this->radio_.tx_check_errors() & 0x0020) ? ": the 32 MHz oscillator stops when transmitting; check the "
                                                         "3.3V/GND wiring and power supply"
                                                       : "");
  }
  if (this->radio_.reinit_count() > 0)
    ESP_LOGW(TAG, "  Radio re-initialized %" PRIu32 " times after the oscillator stopped", this->radio_.reinit_count());
  if (this->radio_.clock_check_errors() == 0) {
    ESP_LOGCONFIG(TAG, "  Clock check: OK");
  } else {
    ESP_LOGE(TAG, "  Clock check FAILED (errors 0x%04X)%s", this->radio_.clock_check_errors(),
             (this->radio_.clock_check_errors() & 0x0020) ? ": 32 MHz oscillator did not start, try another tcxo_voltage" : "");
  }
  if (this->radio_.errors() != 0)
    ESP_LOGW(TAG, "  LR1121 errors during setup: 0x%04X", this->radio_.errors());
  if (this->radio_.is_failed()) {
    // Nothing below applies
  } else if (this->have_address_) {
    char buf[40];
    format_address(this->address_, buf, sizeof(buf));
    ESP_LOGCONFIG(TAG, "  Lamp address: %s (%s)", buf,
                  this->address_from_config_ ? "from config" : "discovered");
  } else {
    ESP_LOGCONFIG(TAG, "  Lamp address: not known yet, discovery running");
  }
  ESP_LOGCONFIG(TAG, "  Poll interval: %" PRIu32 " ms", this->poll_interval_);
  ESP_LOGCONFIG(TAG, "  TX power: %d dBm", this->radio_.tx_power());
  ESP_LOGCONFIG(TAG, "  Transmissions: %" PRIu32 " sent, %" PRIu32 " completed, %" PRIu32 " replies",
                this->radio_.tx_count(), this->radio_.tx_done_count(), this->radio_.reply_count());
  if (this->radio_.is_failed())
    ESP_LOGE(TAG, "  Radio failed: %s", this->radio_.failure());
}

void Halo2Component::loop() {
  if (this->radio_.is_failed()) {
    this->status_set_error(LOG_STR("LR1121 stopped responding"));
    this->set_status_("Radio error: LR1121 not responding");
    this->mark_failed();
    return;
  }
  if (this->mode_ == Mode::DISCOVERY) {
    this->loop_discovery_();
    return;
  }

  // Send queued updates and confirm them; at most one short RF exchange per loop
  this->process_updates_();
  if (this->is_syncing_())
    return;

  this->sniff_();

  const uint32_t now = millis();
  if (now - this->last_poll_ >= this->poll_interval_) {
    this->last_poll_ = now;
    this->poll_lamp_(this->ready_ ? CMD_SYNC : CMD_WAKE);
    if (!this->ready_) {
      this->ready_ = true;
      this->publish_entities_(true);
    }
  }

  if (!this->ready_)
    return;
  // Fresh timestamp: a reply received during this loop is newer than `now`
  const bool reachable = this->last_reply_ != 0 && millis() - this->last_reply_ < UNREACHABLE_AFTER_MS;
  if (this->lamp_reachable_ != static_cast<int8_t>(reachable)) {
    this->lamp_reachable_ = static_cast<int8_t>(reachable);
    if (reachable) {
      this->status_clear_warning();
    } else {
      this->status_set_warning(LOG_STR("Lamp not responding"));
    }
  }
  this->set_status_(reachable ? "OK" : "Lamp not responding");
}

// ---------------------------------------------------------------- address discovery

void Halo2Component::start_discovery_() {
  this->mode_ = Mode::DISCOVERY;
  this->discovery_count_ = 0;
  this->discovery_last_log_ = 0;
  this->pending_command_ = NO_COMMAND;
  this->sync_polls_ = 0;
  this->radio_.set_sync_word(DISCOVERY_SYNC_WORD);
  this->set_status_("Discovering address: set back lamp to 10% and 3925 K on the remote");
}

void Halo2Component::loop_discovery_() {
  const uint32_t now = millis();
  if (this->discovery_last_log_ == 0 || now - this->discovery_last_log_ >= DISCOVERY_HINT_INTERVAL_MS) {
    this->discovery_last_log_ = now;
    ESP_LOGI(TAG, "Address discovery: on the HALO 2 remote, set the back lamp brightness to 10%% and the colour "
                  "temperature to 3925 K (%u/5 captures)",
             this->discovery_count_);
  }

  uint8_t buf[SNIFF_LEN];
  const size_t len = this->radio_.read_raw(buf, sizeof(buf));
  Address found;
  if (len == 0 || !extract_address(buf, len, &found))
    return;
  char text[40];
  format_address(found, text, sizeof(text));
  ESP_LOGD(TAG, "Address candidate: %s", text);
  this->discovery_hits_[this->discovery_count_++] = found;
  if (this->discovery_count_ < this->discovery_hits_.size())
    return;

  // Most frequent candidate
  size_t best = 0, best_count = 0;
  for (size_t i = 0; i < this->discovery_hits_.size(); i++) {
    size_t count = 0;
    for (const auto &other : this->discovery_hits_)
      count += (other == this->discovery_hits_[i]);
    if (count > best_count) {
      best = i;
      best_count = count;
    }
  }
  this->finish_discovery_(this->discovery_hits_[best]);
}

void Halo2Component::finish_discovery_(const Address &address) {
  char text[40];
  format_address(address, text, sizeof(text));
  ESP_LOGI(TAG, "Lamp address discovered: HALO2_ADDRESS = [%s]", text);
  if (this->address_from_config_)
    ESP_LOGW(TAG, "An address is set in the YAML config; it will be used again after a reboot. Remove it to keep "
                  "the discovered one.");
  this->address_pref_.save(&address);
  global_preferences->sync();
  this->mode_ = Mode::RUN;
  this->use_address_(address);
  this->last_poll_ = millis() - this->poll_interval_;  // Poll right away
  this->set_status_("Address discovered");
}

void Halo2Component::use_address_(const Address &address) {
  this->address_ = address;
  this->have_address_ = true;
  this->mode_ = Mode::RUN;
  this->radio_.set_sync_word(address);
#ifdef USE_TEXT_SENSOR
  if (this->address_sensor_ != nullptr) {
    char text[40];
    format_address(address, text, sizeof(text));
    this->address_sensor_->publish_state(text);
  }
#endif
}

// ---------------------------------------------------------------- radio exchanges

bool Halo2Component::exchange_(uint8_t command, const LampState &state, Payload *reply, bool auto_mode) {
  const Payload payload = build_payload(command, state, auto_mode);
  const RawPacket packet = compile_packet(this->address_, this->pid_, payload);
  this->pid_ = (this->pid_ + 1) & 0x03;
  Packet p;
  if (!this->radio_.transact(this->address_, packet, &p))
    return false;
  if (!validate_payload(p.payload)) {
    ESP_LOGV(TAG, "Ignoring reply with invalid payload");
    return false;
  }
  this->last_reply_ = millis();
  if (this->last_reply_ == 0)
    this->last_reply_ = 1;
  *reply = p.payload;
  return true;
}

void Halo2Component::poll_lamp_(uint8_t command) {
  Payload reply;
  if (this->exchange_(command, this->state_, &reply)) {
    this->apply_lamp_state_(parse_payload(reply), "poll");
  } else {
    ESP_LOGV(TAG, "No reply to status poll");
  }
}

void Halo2Component::sniff_() {
  uint8_t buf[SNIFF_LEN];
  const size_t len = this->radio_.read_raw(buf, sizeof(buf));
  // A capture holds the remote control's request followed by the lamp's reply; use the
  // first CRC-valid one with NO_ACK set, as lr1121.receive_without_ack() does.
  if (len == 0)
    return;
  Packet p = parse_packet(this->address_, buf, len);
  if (!(p.crc_valid && p.pkt_len == PAYLOAD_LEN && p.no_ack)) {
    if (!find_second_packet(this->address_, buf, len, &p) || !p.no_ack) {
      // Something matched our address but didn't decode: log it (rate limited) for diagnosis
      const uint32_t now = millis();
      if (now - this->last_undecoded_log_ >= 1000) {
        this->last_undecoded_log_ = now;
        char hex[SNIFF_LEN * 3];
        to_hex(buf, len, hex, sizeof(hex));
        ESP_LOGV(TAG, "Undecoded capture (%u bytes, first packet CRC %s): %s", (unsigned) len,
                 parse_packet(this->address_, buf, len).crc_valid ? "ok" : "bad", hex);
      }
      return;
    }
  }
  if (!validate_payload(p.payload)) {
    char hex[PAYLOAD_LEN * 3];
    to_hex(p.payload.data(), PAYLOAD_LEN, hex, sizeof(hex));
    ESP_LOGD(TAG, "Sniffed packet with unexpected payload: %s", hex);
    return;
  }
  ESP_LOGD(TAG, "Remote control exchange sniffed (command 0x%02X)", p.payload[0]);
  this->last_reply_ = millis();
  this->apply_lamp_state_(parse_payload(p.payload), "remote");
  this->last_poll_ = millis();
}

// ---------------------------------------------------------------- queued updates

void Halo2Component::request_update_(uint8_t command, bool auto_mode) {
  // Several requests before the update is sent are merged; on/off (0x02) takes precedence
  if (this->pending_command_ != CMD_POWER)
    this->pending_command_ = command;
  this->pending_auto_ = this->pending_auto_ || auto_mode;
}

void Halo2Component::process_updates_() {
  const uint32_t now = millis();
  if (this->pending_command_ != NO_COMMAND) {
    const uint8_t command = this->pending_command_;
    const bool auto_mode = this->pending_auto_;
    this->pending_command_ = NO_COMMAND;
    this->pending_auto_ = false;
    this->sync_payload_ = build_payload(command, this->state_, auto_mode);
    ESP_LOGD(TAG, "Sending command 0x%02X: power=%d front=%d back=%d front=%u%% back=%u%% %uK ultrasonic=%d%s",
             command, this->state_.power, this->state_.front, this->state_.back, this->state_.front_brightness,
             this->state_.back_brightness, this->state_.color_temp, this->state_.ultrasonic,
             auto_mode ? " auto" : "");
    // The lamp occasionally misses a packet; resend like the remote does
    Payload reply;
    for (uint8_t attempt = 1; attempt <= COMMAND_ATTEMPTS; attempt++) {
      if (this->exchange_(command, this->state_, &reply, auto_mode))
        break;
      ESP_LOGD(TAG, "No reply to command (attempt %u/%u)", attempt, COMMAND_ATTEMPTS);
    }
    this->sync_polls_ = SYNC_MAX_POLLS;
    this->have_sync_reply_ = false;
    this->last_sync_poll_ = now - SYNC_POLL_INTERVAL_MS;  // First confirmation poll on the next loop
    return;
  }

  if (this->sync_polls_ == 0 || now - this->last_sync_poll_ < SYNC_POLL_INTERVAL_MS)
    return;
  this->sync_polls_--;
  this->last_sync_poll_ = now;

  // Poll with the requested state, as process_updates() in benq_halo/__init__.py
  const auto &sp = this->sync_payload_;
  const bool auto_mode = sp[1] & 0x02;
  LampState requested = parse_payload(sp);
  Payload reply;
  if (this->exchange_(CMD_SYNC, requested, &reply, auto_mode)) {
    if (std::equal(sp.begin() + 1, sp.end(), reply.begin() + 1)) {
      ESP_LOGD(TAG, "Lamp confirmed the update");
      this->sync_polls_ = 0;
      this->last_poll_ = now;
      return;
    }
    this->have_sync_reply_ = true;
    this->last_sync_reply_ = reply;
  }
  if (this->sync_polls_ == 0) {
    ESP_LOGD(TAG, "Lamp did not confirm the update");
    this->last_poll_ = now;
    if (this->have_sync_reply_)
      this->apply_lamp_state_(parse_payload(this->last_sync_reply_), "poll");
  }
}

void Halo2Component::apply_lamp_state_(const LampState &state, const char *source) {
  if (state == this->state_)
    return;
  ESP_LOGD(TAG, "Lamp state (%s): power=%d front=%d back=%d front=%u%% back=%u%% %uK ultrasonic=%d", source,
           state.power, state.front, state.back, state.front_brightness, state.back_brightness, state.color_temp,
           state.ultrasonic);
  this->state_ = state;
  if (this->ready_)
    this->publish_entities_();
}

// ---------------------------------------------------------------- Home Assistant side

bool Halo2Component::light_is_on(Lamp lamp) const {
  return this->state_.power && (lamp == Lamp::FRONT ? this->state_.front : this->state_.back);
}

uint8_t Halo2Component::light_brightness(Lamp lamp) const {
  const uint8_t b = lamp == Lamp::FRONT ? this->state_.front_brightness : this->state_.back_brightness;
  return clamp<uint8_t>(b, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
}

bool Halo2Component::switch_state(SwitchKind kind) const {
  return kind == SwitchKind::POWER ? this->state_.power : this->state_.ultrasonic;
}

void Halo2Component::publish_entities_(bool force) {
#ifdef USE_LIGHT
  for (uint8_t i = 0; i < 2; i++) {
    const Lamp lamp = static_cast<Lamp>(i);
    if (this->lights_[i] != nullptr)
      this->lights_[i]->publish(this->light_is_on(lamp), this->light_brightness(lamp), this->state_.color_temp, force);
  }
#endif
#ifdef USE_SWITCH
  for (uint8_t i = 0; i < 2; i++) {
    Halo2Switch *sw = this->switches_[i];
    const bool value = this->switch_state(static_cast<SwitchKind>(i));
    if (sw != nullptr && (force || sw->state != value))
      sw->publish_state(value);
  }
#endif
}

void Halo2Component::set_status_(const char *status) {
  if (this->status_ == status)
    return;
  this->status_ = status;
#ifdef USE_TEXT_SENSOR
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
#endif
}

void Halo2Component::on_light_command(Lamp lamp, bool on, uint8_t brightness, uint16_t color_temp) {
  if (!this->ready_ || this->mode_ != Mode::RUN) {
    ESP_LOGD(TAG, "Ignoring light command until the lamp state is known");
    return;
  }
  LampState &s = this->state_;
  bool &status = lamp == Lamp::FRONT ? s.front : s.back;
  const bool other_status = lamp == Lamp::FRONT ? s.back : s.front;
  uint8_t command = NO_COMMAND;

  if (on != this->light_is_on(lamp)) {
    if (on) {
      status = true;
      if (!s.power) {
        s.power = true;
        command = CMD_POWER;
      } else {
        command = CMD_SET;
      }
    } else if (other_status) {
      status = false;
      command = CMD_SET;
    } else {
      // Turning off the last lit lamp powers off the whole lamp
      s.power = false;
      command = CMD_POWER;
    }
  }

  uint8_t &current_brightness = lamp == Lamp::FRONT ? s.front_brightness : s.back_brightness;
  if (brightness != this->light_brightness(lamp)) {
    current_brightness = brightness;
    if (command == NO_COMMAND)
      command = CMD_SET;
  }
  if (lamp == Lamp::FRONT && std::abs(int(color_temp) - int(s.color_temp)) > 1) {
    s.color_temp = color_temp;
    if (command == NO_COMMAND)
      command = CMD_SET;
  }

  if (command != NO_COMMAND) {
    this->request_update_(command);
    // Other entities may have changed (e.g. turning on a light powers the lamp on)
    this->defer([this]() { this->publish_entities_(); });
  }
}

void Halo2Component::on_switch_command(SwitchKind kind, bool on) {
  if (!this->ready_ || this->mode_ != Mode::RUN) {
    ESP_LOGD(TAG, "Ignoring switch command until the lamp state is known");
    return;
  }
  if (kind == SwitchKind::POWER) {
    this->state_.power = on;
    this->request_update_(CMD_POWER);
  } else {
    this->state_.ultrasonic = on;
    this->request_update_(CMD_SET);
  }
  this->defer([this]() { this->publish_entities_(); });
}

void Halo2Component::on_button_press(ButtonKind kind) {
  if (kind == ButtonKind::DISCOVER) {
    this->start_discovery_();
    return;
  }
  if (!this->ready_ || this->mode_ != Mode::RUN)
    return;
  this->request_update_(CMD_SET, true);
}

#ifdef USE_LIGHT
void Halo2Component::register_light(Lamp lamp, Halo2Light *light) { this->lights_[static_cast<uint8_t>(lamp)] = light; }

light::LightTraits Halo2Light::get_traits() {
  light::LightTraits traits;
  if (this->lamp_ == Lamp::FRONT) {
    traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
    traits.set_min_mireds(1e6f / COLOR_TEMP_MAX);
    traits.set_max_mireds(1e6f / COLOR_TEMP_MIN);
  } else {
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  }
  return traits;
}

void Halo2Light::write_state(light::LightState *state) {
  // remote_values is the target of a transition, so transitions don't flood the radio
  const auto &values = state->remote_values;
  const bool on = values.is_on();
  const uint8_t brightness =
      clamp<uint8_t>(static_cast<uint8_t>(lroundf(values.get_brightness() * 100.0f)), BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  uint16_t color_temp = this->parent_->color_temp();
  if (this->lamp_ == Lamp::FRONT && values.get_color_temperature() > 0.0f) {
    const long kelvin = lroundf(1e6f / values.get_color_temperature());
    color_temp = static_cast<uint16_t>(clamp<long>(kelvin, COLOR_TEMP_MIN, COLOR_TEMP_MAX));
  }
  this->last_ = {true, on, brightness, this->lamp_ == Lamp::FRONT ? color_temp : uint16_t(0)};
  this->parent_->on_light_command(this->lamp_, on, brightness, color_temp);
}

void Halo2Light::publish(bool on, uint8_t brightness, uint16_t color_temp, bool force) {
  if (this->state_ == nullptr)
    return;
  if (this->lamp_ == Lamp::BACK)
    color_temp = 0;  // Not shown for the back lamp
  const Published now{true, on, brightness, color_temp};
  if (!force && now == this->last_)
    return;
  this->last_ = now;
  auto call = this->state_->make_call();
  call.set_state(on);
  call.set_brightness(brightness / 100.0f);
  if (this->lamp_ == Lamp::FRONT)
    call.set_color_temperature(1e6f / color_temp);
  call.set_transition_length(0);
  call.perform();
}
#endif

#ifdef USE_SWITCH
void Halo2Component::register_switch(SwitchKind kind, Halo2Switch *sw) {
  this->switches_[static_cast<uint8_t>(kind)] = sw;
}

void Halo2Switch::write_state(bool state) {
  this->parent_->on_switch_command(this->kind_, state);
  this->publish_state(this->parent_->switch_state(this->kind_));
}
#endif

}  // namespace halo2
}  // namespace esphome
