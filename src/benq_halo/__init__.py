import time
import config

HALO2_PKT_END = [0x01, 0x02]

SYNC_POLL_INTERVAL_MS = 500  # Interval between status polls while the lamp applies an update
SYNC_MAX_POLLS = 5           # Give up waiting for the lamp to report the requested state after this

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, int(round(value))))

class benq_halo():
    def __init__(self):
        self._debug = config.DEBUG

        # Load driver for RF module
        module_name = config.RF_TRANSCEIVER
        import_file = __import__(module_name, None, None, [module_name])
        driver_class = getattr(import_file, module_name)
        self._rf_module = driver_class()

        # Optional machine.WDT, fed from the main loop
        self.watchdog = None

        # Queued lamp update (see request_update / process_updates)
        self._pending_command = None
        self._pending_auto = False
        self._sync_payload = None
        self._sync_polls = 0
        self._last_sync_poll = 0

        self.ultrasonic_sensor_status = True
        self.front_lamp_status = True
        self.onoff_status = True
        self.back_lamp_status = False

        self.front_lamp_brightness = 1
        self.back_lamp_brightness = 1
        self.front_lamp_brightness_min = 1
        self.front_lamp_brightness_max = 100
        self.back_lamp_brightness_min = 1
        self.back_lamp_brightness_max = 100
        self.front_lamp_color_temp = 4000
        self.front_lamp_color_temp_min = 2700
        self.front_lamp_color_temp_max = 6500

    def __str__(self):
        return  f"On/Off status: {self.onoff_status}\n" \
                f"Ultrasonic status: {self.ultrasonic_sensor_status}\n" \
                f"Front lamp status: {self.front_lamp_status}\n" \
                f"Back lamp status: {self.back_lamp_status}\n" \
                f"Front lamp brightness: {self.front_lamp_brightness}\n" \
                f"Back lamp brightness: {self.back_lamp_brightness}\n" \
                f"Front lamp color temp: {self.front_lamp_color_temp}"

    def feed_watchdog(self):
        if self.watchdog:
            self.watchdog.feed()

    def print_hex(self, data, title):
        print(f"{title}: ", end="")
        for byte in data:
            print("{:02X} ".format(byte), end="")
        print()

    def parse_lamp_status(self, ack_data):
        """
        Parse packet and update internal lamp state
        """
        self.onoff_status = ack_data[1] & 0b00000001
        self.ultrasonic_sensor_status = (ack_data[1] & 0b00100000) >> 5
        #auto_status = (ack_data[1] & 0b00000010) >> 1
        #favorite_status = (ack_data[1] & 0b00000100) >> 2
        lamp_status = (ack_data[1] & 0b00011000) >> 3
        self.front_lamp_status = (lamp_status == 0) or (lamp_status == 2)
        self.back_lamp_status = (lamp_status == 1) or (lamp_status == 2)
        self.front_lamp_brightness = ack_data[2]
        self.back_lamp_brightness = ack_data[5]
        self.front_lamp_color_temp = ack_data[3] * 0x100 + ack_data[4]

    def prepare_payload(self):
        lamp_status = 0
        if self.back_lamp_status:
            lamp_status = self.back_lamp_status + self.front_lamp_status
        control = (self.ultrasonic_sensor_status << 5) | (lamp_status << 3) | (self.onoff_status)
        color_temp_byte1 = self.front_lamp_color_temp >> 8
        color_temp_byte2 = self.front_lamp_color_temp & 0x00FF
        return (control, color_temp_byte1, color_temp_byte2)

    def validate_packet(self, ack_data):
        """
        Validate packet length, commands, Brightness and Color Temp
        """
        if len(ack_data) != 10:
            return False
        if ack_data[0] > 5:
            return False
        if (ack_data[2] > self.front_lamp_brightness_max) or (ack_data[5] > self.back_lamp_brightness_max):
            return False
        if (ack_data[8:10] != bytearray(HALO2_PKT_END)):
            return False
        color_temp = ack_data[3] << 8 | ack_data[4]
        if (color_temp < self.front_lamp_color_temp_min) or (color_temp > self.front_lamp_color_temp_max):
            return False
        color_temp = ack_data[6] << 8 | ack_data[7]
        if (color_temp < self.front_lamp_color_temp_min) or (color_temp > self.front_lamp_color_temp_max):
            return False
        return True

    def request_lamp_status(self, command=0x04, parse_lamp_status=True):
        """
        Request status from the lamp
        """
        # Prepare packet payload and send it
        (control, color_temp_byte1, color_temp_byte2) = self.prepare_payload()
        self._rf_module.send_with_ack([ command, control, self.front_lamp_brightness, color_temp_byte1, color_temp_byte2, 
                             self.back_lamp_brightness, color_temp_byte1, color_temp_byte2 ] + HALO2_PKT_END)
        ack_data_raw = self._rf_module.receive_packet()
        if ack_data_raw:
            ack_data = bytearray(ack_data_raw)
            if self.validate_packet(ack_data):
                if self._debug:
                    self.print_hex(ack_data, "Rcvt")
                if parse_lamp_status:
                    self.parse_lamp_status(ack_data)
        return self.get_lamp_status()

    def sniff_mode(self):
        # Don't let remote control packets overwrite a state we are still sending to the lamp
        if self.is_syncing():
            return False
        rcvd_packet = self._rf_module.receive_without_ack()
        if rcvd_packet and (rcvd_packet["no_ack"] == 1):
            self.parse_lamp_status(rcvd_packet["payload"])
            return True

    def get_lamp_status(self):
        """
        Fill in dictinary with lamp status
        """
        lamp_status = {
            'ultrasonic_sensor_status': self.ultrasonic_sensor_status,
            'onoff_status': self.onoff_status,
            'front_lamp_status': self.front_lamp_status,
            'back_lamp_status': self.back_lamp_status,
            'front_lamp_brightness': self.front_lamp_brightness,
            'back_lamp_brightness': self.back_lamp_brightness,
            'front_lamp_color_temp': self.front_lamp_color_temp
        }
        return lamp_status

    def build_payload(self, auto=False):
        """
        Build the 9 payload bytes (everything after the command byte) from our internal state
        """
        (control, color_temp_byte1, color_temp_byte2) = self.prepare_payload()
        if auto:
            control = control | 0b00000010
        return [ control, self.front_lamp_brightness, color_temp_byte1, color_temp_byte2,
                 self.back_lamp_brightness, color_temp_byte1, color_temp_byte2 ] + HALO2_PKT_END

    def request_update(self, command=0x03, auto=False):
        """
        Queue an update of the lamp with our internal state. It is sent by process_updates()
        from the main loop, so MQTT callbacks never block on RF. Several requests made before
        the update is sent are merged into one RF transaction; 0x02 (on/off) takes precedence.
        """
        if self._pending_command != 0x02:
            self._pending_command = command
        self._pending_auto = self._pending_auto or auto

    def is_syncing(self):
        """
        True while an update is queued or we are waiting for the lamp to apply it
        """
        return (self._pending_command is not None) or (self._sync_polls > 0)

    def process_updates(self):
        """
        Non-blocking replacement for the old update loop, call it from the main loop:
        sends a queued update, then polls the lamp every SYNC_POLL_INTERVAL_MS until it reports
        the requested state or SYNC_MAX_POLLS is reached. Each call does at most one RF exchange.
        """
        try:
            if self._pending_command is not None:
                command = self._pending_command
                self._sync_payload = self.build_payload(self._pending_auto)
                self._pending_command = None
                self._pending_auto = False
                self._rf_module._tx_mode = True
                self._rf_module.send_with_ack([command] + self._sync_payload)
                self._sync_polls = SYNC_MAX_POLLS
                # First status poll on the next call, like the old loop polled right away
                self._last_sync_poll = time.ticks_add(time.ticks_ms(), -SYNC_POLL_INTERVAL_MS)
                return

            if self._sync_polls <= 0:
                return
            if time.ticks_diff(time.ticks_ms(), self._last_sync_poll) < SYNC_POLL_INTERVAL_MS:
                return

            self._sync_polls -= 1
            self._last_sync_poll = time.ticks_ms()
            self._rf_module.send_with_ack([0x04] + self._sync_payload)
            ack_data_raw = self._rf_module.receive_packet()
            if ack_data_raw:
                ack_data = bytearray(ack_data_raw)
                if self._debug:
                    self.print_hex(ack_data, "Rcvt")
                if bytearray(self._sync_payload) == ack_data[1:]:
                    self._sync_polls = 0
        except Exception:
            self._sync_polls = 0
            raise
        finally:
            # Re-enable sniffing (BC5602) once the sync is over, even after an error
            self._rf_module._tx_mode = self.is_syncing()

class OnOff:
    """
    On/Off switch
    """
    def __init__(self, benq_halo):
        self._benq_halo = benq_halo
        if self._benq_halo._debug:
            print("OnOff init")

    def on(self):
        self._benq_halo.onoff_status = True
        self._benq_halo.request_update(command=0x02)
        if self._benq_halo._debug:
            print("onoff_status True")

    def off(self):
        self._benq_halo.onoff_status = False
        self._benq_halo.request_update(command=0x02)
        if self._benq_halo._debug:
            print("onoff_status False")

class UltrasonicSensor:
    """
    Ultrasonic sensor switch
    """
    def __init__(self, benq_halo):
        self._benq_halo = benq_halo
        if self._benq_halo._debug:
            print("UltrasonicSensor init")

    def on(self):
        self._benq_halo.ultrasonic_sensor_status = True
        self._benq_halo.request_update()
        if self._benq_halo._debug:
            print("ultrasonic_sensor_status True")

    def off(self):
        self._benq_halo.ultrasonic_sensor_status = False
        self._benq_halo.request_update()
        if self._benq_halo._debug:
            print("ultrasonic_sensor_status False")

class _Lamp:
    """
    Common logic for the front and back lamps. Subclasses set the attribute names below.
    """
    _status_attr = None      # e.g. "front_lamp_status"
    _other_status_attr = None
    _brightness_attr = None
    _name = None

    def __init__(self, benq_halo):
        self._benq_halo = benq_halo
        if self._benq_halo._debug:
            print(f"{self._name} init")

    def set(self, state=None, brightness=None, color_temp=None):
        """
        Apply on/off state, brightness (percent, HA uses brightness_scale 100) and color
        temperature (Kelvin) from one HA command as a single lamp update.
        Returns the brightness/color_temp values actually applied after clamping.
        """
        halo = self._benq_halo
        command = None
        applied = {}

        if state == "ON":
            if halo.onoff_status and not getattr(halo, self._status_attr):
                setattr(halo, self._status_attr, True)
                command = 0x03
        elif state == "OFF":
            if halo.onoff_status:
                if getattr(halo, self._other_status_attr):
                    setattr(halo, self._status_attr, False)
                    command = 0x03
                else:
                    # Turning off the last lit lamp powers off the whole lamp
                    halo.onoff_status = False
                    command = 0x02

        if brightness is not None:
            value = clamp(brightness, getattr(halo, self._brightness_attr + "_min"), getattr(halo, self._brightness_attr + "_max"))
            setattr(halo, self._brightness_attr, value)
            applied['brightness'] = value
            command = command or 0x03

        if color_temp is not None:
            value = clamp(color_temp, halo.front_lamp_color_temp_min, halo.front_lamp_color_temp_max)
            halo.front_lamp_color_temp = value
            applied['color_temp'] = value
            command = command or 0x03

        if command is not None:
            halo.request_update(command=command)
        if halo._debug:
            print(f"{self._name} set state={state} {applied} -> command {command}")
        return applied

class BackLamp(_Lamp):
    """
    Back lamp class
    """
    _status_attr = "back_lamp_status"
    _other_status_attr = "front_lamp_status"
    _brightness_attr = "back_lamp_brightness"
    _name = "BackLamp"

class FrontLamp(_Lamp):
    """
    Front lamp class
    """
    _status_attr = "front_lamp_status"
    _other_status_attr = "back_lamp_status"
    _brightness_attr = "front_lamp_brightness"
    _name = "FrontLamp"

class AutoConfig:
    """
    Auto mode button
    """
    def __init__(self, benq_halo):
        self._benq_halo = benq_halo
        if self._benq_halo._debug:
            print("AutoConfig init")

    def start_auto(self):
        self._benq_halo.request_update(auto=True)
        if self._benq_halo._debug:
            print("start_auto")
