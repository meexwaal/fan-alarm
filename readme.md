This was a weekend project to make an alarm clock which turns on my bedroom fan,
for a gentle start to the day. The fan has a remote which uses a simple RF
on-off signal. I used an SDR and gnuradio to scope the signal and determine
enough of its structure to control the fan.

Various alarm clock features/components:
 * 16x2 LCD screen
 * Very satisfying encoder dial & button
 * ESP8266 wifi-over-serial chip, used to get current time via NTP
 * RFM69HCW radio transceiver (with a simple wire antenna) to control fan
 * Menu to set/stop alarm (room for more future features)
