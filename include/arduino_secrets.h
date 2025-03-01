/*
 * Copy this template into arduino_secrets.h and change values accordingly.
 *
 * This file holds settings following the Arduino convention, see
 * https://docs.arduino.cc/arduino-cloud/tutorials/store-your-sensitive-data-safely-when-sharing
 *
 */

// Settings: Wi-Fi credentials
#define SECRET_WIFI_SSID        "motorola edge 40_4386"               // Replace "eduroam" for other Wi-Fi
#define SECRET_WIFI_PASSWORD    "motowifi"              // Wi-Fi or university account password
#define SECRET_WIFI_ANONYMOUSID "wlan@tu-berlin.de"     // Don't change, only used for Eduroam
#define SECRET_WIFI_EDUROAMID   "susmitha_suresh@tu-berlin.de" // TU account name, only used for Eduroam

// Settings: MQTT
#define SECRET_MQTT_BROKER   "heide.bastla.net" // Server hostname (FQDN)
#define SECRET_MQTT_PORT     8883                 // Server TLS port
#define SECRET_MQTT_USER     "mse24"           // Server credentials
#define SECRET_MQTT_PASSWORD "aura"
#define SECRET_MQTT_PREFIX   "cadse"              // Prefix for topic and client ID
#define SECRET_MQTT_YEAR     2024                 // Current year (beginning of course)
#define SECRET_MQTT_BOARDID  11                 // [ID]: Assigned PCB number
