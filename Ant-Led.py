import time
import neopixel
from machine import Pin, I2C
import ssd1306

# ----------------------------
# Pin-Definitionen
# ----------------------------
ROTARY_CLK = 2
ROTARY_DT  = 3
ROTARY_SW  = 8          # Taster
LED_STRIP_PIN = 14
NUM_LEDS = 19

# ----------------------------
# Hardware-Setup
# ----------------------------
clk = Pin(ROTARY_CLK, Pin.IN, Pin.PULL_UP)
dt  = Pin(ROTARY_DT,  Pin.IN, Pin.PULL_UP)
sw  = Pin(ROTARY_SW,  Pin.IN, Pin.PULL_UP)

strip = neopixel.NeoPixel(Pin(LED_STRIP_PIN), NUM_LEDS)

# I2C + OLED
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
oled = None
try:
    devices = i2c.scan()
    print("I2C devices:", [hex(d) for d in devices])
    if 0x3C in devices:
        oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3C)
        print("OLED @0x3C (128x32)")
    elif 0x3D in devices:
        oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=0x3D)
        print("OLED @0x3D (erzwinge 128x32)")
    else:
        print("Kein OLED gefunden")
except Exception as e:
    print("OLED init Fehler:", e)

# ----------------------------
# Globale Variablen
# ----------------------------
brightness         = 100          # Startwert (kannst du ändern)
target_brightness  = brightness
led_state          = False
color_mode         = 0           # 0 = Rot, 1 = Weiß  (Start immer Rot)

# Button/Press-Handling
press_start_ms     = None
last_button_state  = 1
LONGPRESS_MS       = 500

# Display/Scroll
display_scroll_idx = 0
last_scroll_time   = 0
SCROLL_MS          = 750
last_disp_upd      = 0
DISP_UPD_MS        = 120

# ----------------------------
# Encoder: robuster Quadratur-Decoder
# ----------------------------
# Zustands-Tabelle (von Ben Buxton inspiriert)
# index = (old_state<<2) | new_state
# value: -1, 0, +1 pro Flanke
_ENC_TABLE = (
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
)
last_enc_code = (clk.value() << 1) | dt.value()
enc_accum = 0
STEPS_PER_NOTCH = 4    # viele Encoder liefern 4 Flanken pro Raster

def read_encoder_step():
    """
    Liefert -1 / +1 wenn ein Detent erreicht wurde, sonst 0.
    Unterdrückt Bounces & Richtungswechsel.
    """
    global last_enc_code, enc_accum
    code = (clk.value() << 1) | dt.value()
    if code == last_enc_code:
        return 0

    idx = (last_enc_code << 2) | code
    step = _ENC_TABLE[idx]
    last_enc_code = code

    if step:
        enc_accum += step
        if enc_accum >= STEPS_PER_NOTCH:
            enc_accum = 0
            return 1
        elif enc_accum <= -STEPS_PER_NOTCH:
            enc_accum = 0
            return -1
    return 0

# ----------------------------
# LEDs aktualisieren
# ----------------------------
def update_leds():
    if led_state:
        if color_mode == 0:
            col = (brightness, 0, 0)  # Rot
        else:
            col = (brightness, brightness, brightness)  # Weiß
        for i in range(NUM_LEDS):
            strip[i] = col
    else:
        for i in range(NUM_LEDS):
            strip[i] = (0, 0, 0)
    strip.write()

# ----------------------------
# Display-Zeilen zusammenbauen
# ----------------------------
def build_display_lines():
    status  = "ON" if led_state else "OFF"
    coltxt  = "RED" if color_mode == 0 else "WHT"
    lines = [
        "Ant LED Ctrl",
        f"St:{status}  Cl:{coltxt}",
    ]
    if led_state:
        pct = int(brightness * 100 / 255)
        lines.append(f"Br:{pct:3d}%")
    else:
        lines.append("Btn: Press = ON")
    return lines

# ----------------------------
# Display aktualisieren (mit Auto-Scroll)
# ----------------------------
def update_display(force=False):
    global display_scroll_idx, last_scroll_time

    if oled is None:
        return

    now = time.ticks_ms()
    try:
        oled.fill(0)
        w = 128
        h = 32
        usable_px = h - 4
        max_lines = usable_px // 8
        lines = build_display_lines()

        if len(lines) > max_lines:
            if force or time.ticks_diff(now, last_scroll_time) > SCROLL_MS:
                display_scroll_idx = (display_scroll_idx + 1) % (len(lines) - max_lines + 1)
                last_scroll_time = now
        else:
            display_scroll_idx = 0

        y = 0
        for i in range(display_scroll_idx, min(display_scroll_idx + max_lines, len(lines))):
            oled.text(lines[i], 0, y)
            y += 8

        if led_state:
            bar_y = h - 4
            oled.rect(0, bar_y - 1, w, 4, 1)
            filled = int((w - 2) * brightness / 255)
            oled.fill_rect(1, bar_y, filled, 2, 1)

        oled.show()
    except Exception as e:
        print("Display error:", e)

# ----------------------------
# Hauptprogramm
# ----------------------------
def main():
    global brightness, target_brightness, led_state, color_mode
    global press_start_ms, last_button_state, last_disp_upd

    print("Starte Ant LED Controller...")
    
    update_display(force=True)
    print("Setup fertig!")

    while True:
        now = time.ticks_ms()

        # ---------- Encoder ----------
        step = read_encoder_step()
        if step and led_state:
            # schneller Dreh => viele Steps -> große Änderung
            target_brightness = max(0, min(255, target_brightness + step * 10))

        # Soft-Dimming (sanft zum Zielwert)
        if brightness != target_brightness and led_state:
            diff = target_brightness - brightness
            # Schrittgröße dynamisch (mind. 1)
            inc = max(1, abs(diff) // 6)
            brightness += inc if diff > 0 else -inc
            update_leds()

        # ---------- Button ----------
        button_state = sw.value()

        # Press start
        if button_state == 0 and last_button_state == 1:
            if press_start_ms is None:
                press_start_ms = now

        # Release
        if button_state == 1 and last_button_state == 0 and press_start_ms is not None:
            duration = time.ticks_diff(now, press_start_ms)
            if duration >= LONGPRESS_MS and led_state:
                # AUS, nichts zurücksetzen außer LED-State
                led_state = False
                update_leds()
                print("LED OFF")
            else:
                if led_state:
                    # kurzer Druck: Farbe toggeln
                    color_mode = 1 - color_mode
                    print("Color:", "WHITE" if color_mode else "RED")
                    update_leds()
                else:
                    # Einschalten: immer ROT, Helligkeit bleibt
                    led_state = True
                    color_mode = 0
                    print("LED ON (RED)")
                    update_leds()

            press_start_ms = None

        last_button_state = button_state

        # ---------- Display ----------
        if oled is not None and time.ticks_diff(now, last_disp_upd) > DISP_UPD_MS:
            update_display()
            last_disp_upd = now

        time.sleep(0.002)  # kleine Pause

# ----------------------------
# Start
# ----------------------------
if __name__ == "__main__":
    main()
