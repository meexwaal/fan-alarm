/*
 * Power notes:
 * LCD: 5V, ~1 mA for LCD but ?100-200? mA with backlight
 * WiFi: 3.3V, 80? mA average, 200+ mA peak
 * RF: 3.3V, 130 mA for the highest power, not sure how low we can go but maybe 50 mA
 *
 * Teensy:
 *   can power with 5V external, but not with USB connected
 *   5V power is just from USB, probably 500 mA
 *   3.3V pin can power 100 mA
 *
 * So teensy could power the LCD (passing through USB's 5V), and Maybe RF and wifi on average,
 * but not at peak. Maybe some capacitor something? But just get an external supply.
 * Watch out for dropping voltage even with that... Could easily get smart and make sure that
 * only one of {LCD backlight, wifi, RF} is on at a time.
 *
 * Wifi is connected to Serial2, pins 9 (Teensy RX2) and 10 (TX2).
 *
 * Note: I think I modified my radiohead library? Lines 528-534, sending headers, are commented
 */


#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <Wire.h>

// RFM69HCW
// Library: http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html
// Datasheet: https://cdn.sparkfun.com/datasheets/Wireless/General/RFM69HCW-V1.1.pdf
//
// Radio driver uses:
// SPI pins: MOSI 7, MISO 8, SCK 14, SS 17
// Interrupt pin 15
RH_RF69 rf69(17, 15);

// Fan power levels: 1111 for fan off, 1110 for min, 0110 for max
constexpr bool bedroom_fan_on[27] = {
    0,1,0,0,0,0,0,1,1,0,0,1,0,0,1,0,1, // ID
    1,0,0,1,                           // Fan power level mid
    1,0,
    1,1,1,1                            // Noop/fan command
};
constexpr bool bedroom_fan_off[27] = {
    0,1,0,0,0,0,0,1,1,0,0,1,0,0,1,0,1, // ID
    1,1,1,1,                           // Fan power off
    1,0,
    1,1,1,1                            // Noop/fan command
};
constexpr bool kitchen_light[27] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // ID
    0,1,0,1,                           // Fan power level noop
    1,0,
    1,0,1,1                            // Light command
};

// Each logical bit is transmitted with 10 bits (slightly wrong because the Tx
// bitrate is 9.6 kbps, not 10, and each logical bit takes 1 ms)
constexpr size_t num_logical_bits =
    sizeof(kitchen_light) / sizeof(kitchen_light[0]);
constexpr size_t tx_per_logical_bit = 10;
constexpr size_t num_tx_bits = num_logical_bits * tx_per_logical_bit;
constexpr size_t num_tx_bytes = (num_tx_bits + 8 - 1) / 8;

constexpr uint16_t logical_zero = 0b0000000111;
constexpr uint16_t logical_one  = 0b0001111111;


// Set the LCD address to 0x27 for a 16 chars and 2 line display.
// Teensy LC uses pins 18 (SDA0) and 19 (SCL0) for I2C. The LCD is 5V
// so route these through a voltage level shifter.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Turn the backlight off after 10s of inactivity.
// When not active, dial/button turn on the backlight instead of taking effect.
constexpr time_t ACTIVE_PERIOD = 10;
bool active;
time_t deactivate_time;

//    0123456789012345
//   .----------------.
// 0 |> Stop |11:32 PM|
// 1 |Set    |10:00 AM|
//   '----------------'

// Left half is the menu. Dial to change selected entry, button to select.
// Right half is time (top) and, if set, alarm (bottom).
// Set: sets one-time alarm
// Stop: turns off alarm

// Dial uses pins 2 and 3 for the encoder, pin 4 for the button
Encoder enc(2, 3);
constexpr int BUTTON_PIN = 4;

constexpr size_t MENU_SIZE = 2;
const String MENU[MENU_SIZE] = {
    "Set ",
    "Stop",
};
enum {
    SET_ALARM = 0,
    STOP_ALARM = 1,
};

// Number of bytes to allocate for time strings
// "11:11 PM" needs 9 chars with null, round up
constexpr size_t TIME_STR_LEN = 16;

TimeChangeRule PT_DST = {"PDT", Second, Sun, Mar, 2, -420}; // Daylight time = UTC - 7 hours
TimeChangeRule PT_STD = {"PST", First, Sun, Nov, 2, -480};  // Standard time = UTC - 8 hours
Timezone timezone_PT(PT_DST, PT_STD);

char wifi_buf[256] = {0};
int ip_addr[4];

// Take a UTC time_t, convert to PT, format as "HH:MM xM"
void format_time(const time_t time_in, char* const str_out) {
    const time_t local_time = timezone_PT.toLocal(time_in);

    snprintf(str_out, TIME_STR_LEN,
             "%2d:%02d %cM",
             hourFormat12(local_time),
             minute(local_time),
             isAM(local_time) ? 'A' : 'P');
}

// Start/stop the fan
void start_alarm() {
    Serial.println("Starting alarm");
    radio_tx(bedroom_fan_on);
}
void stop_alarm() {
    Serial.println("Stopping alarm");
    radio_tx(bedroom_fan_off);
}

// Draw the menu (left half of the screen), with MENU[idx] on the first row.
// If active=true, render "> " in front of the first row.
void render_menu(const int idx, const bool active) {
    lcd.setCursor(0,0);
    if (active) {
        lcd.print("> ");
        lcd.print(MENU[idx % MENU_SIZE]);
    } else {
        lcd.print(MENU[idx % MENU_SIZE]);
        lcd.print("  ");
    }
    lcd.setCursor(0,1);
    lcd.print(MENU[(idx + 1) % MENU_SIZE]);
    lcd.print("  ");
}

// Draw the current time (upper right)
void render_time(const time_t t) {
    static char time_str_buf[TIME_STR_LEN];
    format_time(t, time_str_buf);
    lcd.setCursor(8,0);
    lcd.print(time_str_buf);
}

// Draw the alarm time (lower right)
void render_alarm(const time_t t) {
    static char time_str_buf[TIME_STR_LEN];
    format_time(t, time_str_buf);
    lcd.setCursor(8,1);
    lcd.print(time_str_buf);
}

// Print a command, send it to the wifi chip, get the response in wifi_buf and print it.
// Ensures that wifi_buf is a null-terminated string.
void cmd_wifi(const char* const cmd) {
    Serial2.print(cmd);
    Serial.print(">> ");
    Serial.println(cmd); // Some spare \r\n's but oh well
    Serial.print("<< ");

    const size_t wifi_buf_strlen = Serial2.readBytes(wifi_buf, sizeof(wifi_buf) - 1);
    wifi_buf[wifi_buf_strlen] = 0;
    Serial.println(wifi_buf);

    if (wifi_buf_strlen == sizeof(wifi_buf) - 1)
        Serial.println("ERROR: wifi_buf not large enough");
    // todo add flag to check for "OK"
}

void hexdump(const char* const buf, const size_t n = 0) {
    size_t i = 0;
    char hex[4];
    while ((n > 0) ? (i < n) : buf[i] != 0) {
        snprintf(hex, 4, "%02X.", buf[i]);
        Serial.print(hex);
        i++;
    }
    Serial.println();
}

// Get the time from NTP over wifi
time_t get_ntp_time() {
    time_t new_time = 0;

    Serial.println("Getting NTP time...");

    // Flush any available data
    if (Serial2.available()) {
        Serial.println("WARNING: stale data on Serial2:");
        while(Serial2.available()) {
            Serial.print(Serial2.read());
            delay(1);
        }
        Serial.println();
    }

    cmd_wifi("AT+CIPSTART=4,\"UDP\",\"pool.ntp.org\",123\r\n");
    // todo getting (benign) error about already connected, should we disconnect?
    cmd_wifi("AT+CIPSEND=4,48\r\n");
    // todo can get "busy" response, when previous command doesn't respond "OK".
    // For now, adding delay after init seems to work well.

    // This 48-byte packet comes from https://seriot.ch/projects/tiny_ntp_client.html.
    // Perhaps a bit of an abuse to use spaces instead of nulls, but easier to write.
    Serial2.print("c                                               ");
    // Response will look like:
    // '\r\nRecv 48 bytes\r\n\r\nSEND OK\r\n\r\n+IPD,4,48:' (40 bytes), then the 48-byte response
    size_t bytes_read = Serial2.readBytes(wifi_buf, 40);
    wifi_buf[bytes_read] = 0;
    Serial.println(wifi_buf);
    if (bytes_read != 40) {
        Serial.println("No/short NTP response");
    } else if (wifi_buf[39] != ':') {
        Serial.println("Bad NTP response");
    } else {
        bytes_read = Serial2.readBytes(wifi_buf, 48);
        if (bytes_read == 48) {
            // We got a full response, now parse it
            unsigned long secs_since_1900;
            // Convert four bytes starting at location 40 in the packet to a 32b
            // integer. This is the seconds field of the transmit timestamp
            // (when the packet left the NTP server).
            secs_since_1900 =  (unsigned long)wifi_buf[40] << 24;
            secs_since_1900 |= (unsigned long)wifi_buf[41] << 16;
            secs_since_1900 |= (unsigned long)wifi_buf[42] << 8;
            secs_since_1900 |= (unsigned long)wifi_buf[43];
            new_time = secs_since_1900 - 2208988800UL;

            Serial.print("Got ");
            Serial.print(secs_since_1900);
            Serial.print("/");
            Serial.print(new_time);
            Serial.print(": ");
            Serial.print(year(new_time));
            Serial.print("-");
            Serial.print(month(new_time));
            Serial.print("-");
            Serial.print(day(new_time));
            Serial.print(" ");
            Serial.print(hour(new_time));
            Serial.print(":");
            Serial.print(minute(new_time));
            Serial.print(":");
            Serial.print(second(new_time));
            Serial.println(" UTC");
        } else {
            Serial.println("Short NTP packet");
        }
    }

    // Flush any available data
    if (Serial2.available()) {
        Serial.println("WARNING: stale data left on Serial2:");
        while(Serial2.available()) {
            Serial.print(Serial2.read());
            delay(1);
        }
        Serial.println();
    }

    Serial.println("\nDone getting NTP time");

    return new_time;
}

// Transmit a 27-bit message
void radio_tx(const bool* const bits) {
    Serial.print("Radio transmitting... ");

    uint8_t data[num_tx_bytes];
    size_t data_idx = 0;

    // Pack bits (10 tx bits per logical bit) to bytes.
    // Shift register: new bits in on the right.
    uint16_t unwritten_data = 0; // 16b is just enough for these numbers
    size_t num_unwritten_bits = 0;
    for (size_t i = 0; i < num_logical_bits; i++) {
        unwritten_data <<= tx_per_logical_bit;
        unwritten_data |= bits[i] ? logical_one : logical_zero;
        num_unwritten_bits += tx_per_logical_bit;
        while (num_unwritten_bits >= 8) {
            // Shift out the newest (num_unwritten_bits - 8) bits to make the
            // new data word from the oldest 8 bits
            data[data_idx]
                = (unwritten_data >> (num_unwritten_bits - 8)) & 0xFF;
            data_idx++;
            num_unwritten_bits -= 8;
        }
    }

    if (data_idx < num_tx_bytes) {
        // Shift in 8 zeroes and use the same logic to make the last byte
        unwritten_data <<= 8;
        num_unwritten_bits += 8;

        // Shift out the newest (num_unwritten_bits - 8) bits to make the
        // new data word from the oldest 8 bits
        data[data_idx]
            = (unwritten_data >> (num_unwritten_bits - 8)) & 0xFF;
        data_idx++;
        num_unwritten_bits -= 8;
    }
    if (data_idx < num_tx_bytes) {
        Serial.println("ERROR: Bad data_idx");
    }

    // Send 10 times with a 9 ms gap
    for (int i = 0; i < 10; i++) {
        if (!rf69.send(data, sizeof(data))) {
            Serial.println("Error sending");
        }
        rf69.waitPacketSent();
        delay(9);
    }

    Serial.println("Done");
}

void setup() {
    Serial.begin(115200);

    // Initialize the lcd
    Serial.print("Initializing LCD... ");
    lcd.init();
    lcd.backlight();
    Serial.println("Done");


    // Initialize radio
    Serial.print("Initializing radio... ");
    lcd.setCursor(0,0);
    lcd.print("Initing radio...");
    SPI.setMOSI(7);
    SPI.setMISO(8);
    SPI.setSCK(14);

    if (!rf69.init())
        Serial.println("init failed");
    if (!rf69.setFrequency(434.0))
        Serial.println("setFrequency failed");
    if (!rf69.setModemConfig(RH_RF69::OOK_Rb9_6Bw19_2))
        Serial.println("setModemConfig failed");

    rf69.setPreambleLength(0);
    rf69.setSyncWords();

    // Set the following bits to 0:
    //  7   fixed packet length
    //  6-5 no bit scrambling/encodeing
    //  4   no CRC
    rf69.spiWrite(RH_RF69_REG_37_PACKETCONFIG1,
                  rf69.spiRead(RH_RF69_REG_37_PACKETCONFIG1) & ~0xF0);

    // Set the following bits to 0:
    //  7   disable
    //  5-3 sync size
    rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG,
                  rf69.spiRead(RH_RF69_REG_2E_SYNCCONFIG) & ~0xB8);

    // Payload length
    rf69.spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, num_tx_bytes);

    // Unset bit 0, AES
    rf69.spiWrite(RH_RF69_REG_3D_PACKETCONFIG2,
                  rf69.spiRead(RH_RF69_REG_3D_PACKETCONFIG2) & ~0x01);

    // Use the minimum power (-2 dBm). This is a high power RF69 so set the
    // ishighpowermodule flag true.
    rf69.setTxPower(-2, true);
    // TODO we don't need this much power (-5 dBm or lower is pry ok), but the
    // library limits how low we can go.
    // Could try setting RegPaLevel (0x11) to {0b100 (pa0 on, pa1 pa2 off), 0b01101 (-5 dBm)}.

    Serial.println("Done");

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize wifi
    Serial.println("Initializing wifi... ");
    lcd.setCursor(0,0);
    lcd.print("Initing wifi...");
    Serial2.begin(115200);
    Serial2.setTimeout(200); // 100 ms timeout

    // Test command to make sure device is responding
    cmd_wifi("AT\r\n");

    // Wait until we get an IP
    while (true) {
        delay(1000);
        cmd_wifi("AT+CIFSR\r\n");

        // When we haven't gotten an IP, the response starts:
        // `AT+CIFSR\r\r\n+CIFSR:STAIP,"0.0.0.0"`
        const char* ip_start = strstr(wifi_buf, "STAIP,\"");
        if (ip_start == NULL) continue;

        // Move to start of IP, read IP
        ip_start += 7;
        const int bytes_found = sscanf(
            ip_start, "%3d.%3d.%3d.%3d", &ip_addr[0], &ip_addr[1], &ip_addr[2], &ip_addr[3]);
        if (bytes_found != 4) continue;

        if (ip_addr[0] != 0) break;
        if (ip_addr[1] != 0) break;
        if (ip_addr[2] != 0) break;
        if (ip_addr[3] != 0) break;
    }
    delay(1000); // Once we have an IP, give the chip some time to get set up

    // Enable multiple connections
    cmd_wifi("AT+CIPMUX=1\r\n");

    Serial.println("Done");

    // Initialize system time over the network
    Serial.println("Initializing time... ");
    lcd.setCursor(0,0);
    lcd.print("Getting time...");
    // TODO if this fails, provide fallback or better diagnostic
    setSyncProvider(get_ntp_time);
    // Wait until the time is set by the sync provider
    while (timeStatus() == timeNotSet);
    // Re-sync to NTP every hour or so
    setSyncInterval(3333);
    Serial.println("Done initializing time");

    // Set up the screen
    lcd.clear();
    lcd.setCursor(7,0);
    lcd.write('|');
    lcd.setCursor(7,1);
    lcd.write('|');
    render_menu(0, true);
    render_time(now());
    active = true;
    deactivate_time = now() + ACTIVE_PERIOD;
}

void loop() {
    // todo use timer to not busy loop this
    delay(20);

    // True when at menu top level, false when in a sub menu (at index menu_idx)
    static bool at_menu = true;
    static size_t menu_idx = 0;

    static time_t alarm_time = 0;
    static bool alarm_set = false;

    static long dial_pos = 0;
    static int button = 0;

    static int prev_min = -1;

    // Flag to turn on the backlight.
    bool activate = false;

    // Use one time throughout the loop
    const time_t current_time = now();

    // Once a minute, update clock
    const int min = minute(current_time);
    if (min != prev_min) {
        prev_min = min;
        render_time(current_time);
    }

    // If the alarm time is reached, start it
    if (alarm_set && current_time >= alarm_time) {
        start_alarm();
        alarm_set = false;
        activate = true;
    }

    // How much has dial moved since the last time we checked
    const long new_dial_pos = (enc.read()+2)/4; // Each tick increases reading by 4
    const long dial_delta = new_dial_pos - dial_pos;
    dial_pos = new_dial_pos;
    if (dial_delta != 0) activate = true;

    // Render dial changes
    if (dial_delta != 0 && active) {
        if (at_menu) {
            menu_idx = (menu_idx + dial_delta) % MENU_SIZE;
            render_menu(menu_idx, true);
        } else {
            switch (menu_idx) {
            case SET_ALARM:
                // Change alarm in 5 minute increments
                alarm_time += dial_delta * (5 * 60);
                render_alarm(alarm_time);
                break;

            default:
                Serial.print("ERROR: dial change, !at_menu, invalid idx ");
                Serial.println(menu_idx);
                break;
            }
        }
    }

    // Process button on release
    const int button_new = !digitalRead(BUTTON_PIN);
    if (!button_new && button && active) {
        Serial.print("Button at index ");
        Serial.println(menu_idx);

        switch (menu_idx) {
        case SET_ALARM:
            if (at_menu) {
                // Enter the set alarm screen
                at_menu = false;

                //    0123456789012345
                //   .----------------.
                // 0 |Set    |11:32 PM|
                // 1 |Stop   >10:00 AM|
                //   '----------------'

                // First, render the menu as inactive (no '>')
                render_menu(menu_idx, false);

                // If set, keep the current alarm time. Otherwise, default alarm
                // time is 8 hours from now, truncated to a mulitple of 5 min.
                if (!alarm_set) {
                    alarm_time = (current_time / (5 * 60)) * 5 * 60 + (8 * 60 * 60);
                }

                lcd.setCursor(7,1);
                lcd.write('>');
                render_alarm(alarm_time);
            } else {
                // todo should handle alarm time in past or >24 hours in future

                // Exit the set alarm scren, set the alarm
                at_menu = true;
                alarm_set = true;
                lcd.setCursor(7,1);
                lcd.write('|');
                render_menu(menu_idx, true);
            }
            break;

        case STOP_ALARM:
            // Stops the alarm.
            // Unlike set alarm, stays at the menu.

            // Clear alarm text
            lcd.setCursor(8,1);
            lcd.print("        ");

            stop_alarm();
            alarm_set = false;
            break;

        default:
            Serial.print("ERROR: Button at invalid idx ");
            Serial.println(menu_idx);
            break;
        }
    }
    if (!button_new && button) activate = true;
    button = button_new;

    if (activate) {
        if (!active) {
            lcd.backlight();
        }
        active = true;
        deactivate_time = current_time + ACTIVE_PERIOD;
    }
    if (active && current_time > deactivate_time) {
        active = false;
        lcd.noBacklight();
    }
}


/*

/*
 * AT
AT\r

OK
 * AT+CIFSR
AT+CIFSR\r
+CIFSR:STAIP,"192.168.1.129"
+CIFSR:STAMAC,"84:cc:a8:81:c4:42"

OK
 * AT+CIPMUX=1
AT+CIPMUX=1\r

OK
 * AT+CIPSTART=4,"UDP","pool.ntp.org",123
AT+CIPSTART=4,"UDP","pool.ntp.org",123\r
4,CONNECT

OK
 * AT+CIPSEND=4,48
AT+CIPSEND=4,48\r

OK
>
 *`c                                               `
Recv 48 bytes

SEND OK

+IPD,4,48: ...
 * /

void setup() {
  Serial.begin(112500);
  Serial2.begin(115200);
}

int idx = 0;
char buf[500];

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
    Serial2.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial2.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    char c = Serial2.read();   // read it and send it out Serial (USB)
    Serial.write(c);
    if (idx < 500) {
      buf[idx] = c;
      idx++;
    }
    if (c == '\n') {
      for (int i = 0; i < idx; i++) {
        String s = String(buf[i], HEX);
        Serial.print(s+'.');
      }
      Serial.println();
      idx = 0;
    }
  }
}
*/
