#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// WIFI
// ============================================================

const char* ssid     = "LOKI";
const char* password = "88888888";

// ============================================================
// AI THINKER ESP32-CAM PINS
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_GPIO_NUM     4

// ============================================================
// WEB SERVER
// ============================================================

WebServer server(80);

// ============================================================
// STREAM SETTINGS
// ============================================================

#define STREAM_BOUNDARY "123456789000000000000987654321"

const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;

const char* STREAM_BOUNDARY_STRING =
    "\r\n--" STREAM_BOUNDARY "\r\n";

// ============================================================
// ROOT HTML PAGE
// ============================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
      content="width=device-width, initial-scale=1">

<title>ESP32-CAM Rover</title>

<style>

body {
    margin: 0;
    padding: 0;
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
}

h1 {
    margin-top: 20px;
}

img {
    width: 320px;
    height: 240px;
    object-fit: contain;
    border: 2px solid #555;
}

button {
    margin-top: 20px;
    padding: 12px 25px;
    font-size: 16px;
}

.status {
    margin-top: 15px;
    font-size: 18px;
}

</style>

</head>

<body>

<h1>ESP32-CAM Rover</h1>

<img src="/stream">

<div class="status">
Camera: ONLINE
</div>

<br>

<button onclick="window.open('/capture','_blank')">
Capture Image
</button>

</body>

</html>
)rawliteral";

// ============================================================
// ROOT PAGE
// ============================================================

void handleRoot()
{
    server.send(
        200,
        "text/html",
        INDEX_HTML
    );
}

// ============================================================
// CAPTURE IMAGE
// ============================================================

void handleCapture()
{
    Serial.println(
        "[CAPTURE] Request received"
    );

    camera_fb_t *fb =
        esp_camera_fb_get();

    if (fb == NULL)
    {
        Serial.println(
            "[CAPTURE] Capture failed"
        );

        server.send(
            500,
            "text/plain",
            "Camera capture failed"
        );

        return;
    }

    Serial.printf(
        "[CAPTURE] %u bytes\n",
        (unsigned int)fb->len
    );

    WiFiClient client =
        server.client();

    // --------------------------------------------------------
    // HTTP response
    // --------------------------------------------------------

    client.println(
        "HTTP/1.1 200 OK"
    );

    client.println(
        "Content-Type: image/jpeg"
    );

    client.println(
        "Content-Disposition: inline; filename=capture.jpg"
    );

    client.println(
        "Cache-Control: no-cache"
    );

    client.println(
        "Access-Control-Allow-Origin: *"
    );

    client.print(
        "Content-Length: "
    );

    client.println(
        fb->len
    );

    client.println();

    // --------------------------------------------------------
    // Send image
    // --------------------------------------------------------

    client.write(
        fb->buf,
        fb->len
    );

    // --------------------------------------------------------
    // Return frame buffer
    // --------------------------------------------------------

    esp_camera_fb_return(fb);

    Serial.println(
        "[CAPTURE] Image sent"
    );
}

// ============================================================
// MJPEG STREAM
// ============================================================

void handleStream()
{
    Serial.println(
        "[STREAM] Client connected"
    );

    WiFiClient client =
        server.client();

    // --------------------------------------------------------
    // HTTP headers
    // --------------------------------------------------------

    client.println(
        "HTTP/1.1 200 OK"
    );

    client.print(
        "Content-Type: "
    );

    client.println(
        STREAM_CONTENT_TYPE
    );

    client.println(
        "Cache-Control: no-cache"
    );

    client.println(
        "Pragma: no-cache"
    );

    client.println(
        "Access-Control-Allow-Origin: *"
    );

    client.println(
        "Connection: close"
    );

    client.println();

    // --------------------------------------------------------
    // Stream loop
    // --------------------------------------------------------

    while (client.connected())
    {
        // ----------------------------------------------------
        // Capture frame
        // ----------------------------------------------------

        camera_fb_t *fb =
            esp_camera_fb_get();

        if (fb == NULL)
        {
            Serial.println(
                "[STREAM] Camera capture failed"
            );

            break;
        }

        // ----------------------------------------------------
        // Check JPEG
        // ----------------------------------------------------

        if (fb->format != PIXFORMAT_JPEG)
        {
            Serial.println(
                "[STREAM] Frame is not JPEG"
            );

            esp_camera_fb_return(fb);

            break;
        }

        // ----------------------------------------------------
        // Send boundary
        // --------------------------------------------------------

        client.print(
            STREAM_BOUNDARY_STRING
        );

        // ----------------------------------------------------
        // Send frame header
        // --------------------------------------------------------

        client.print(
            "Content-Type: image/jpeg\r\n"
        );

        client.print(
            "Content-Length: "
        );

        client.print(
            fb->len
        );

        client.print(
            "\r\n\r\n"
        );

        // ----------------------------------------------------
        // Send JPEG
        // ----------------------------------------------------

        size_t written =
            client.write(
                fb->buf,
                fb->len
            );

        // ----------------------------------------------------
        // Return frame buffer
        // ----------------------------------------------------

        esp_camera_fb_return(fb);

        // ----------------------------------------------------
        // Check connection
        // ----------------------------------------------------

        if (written == 0)
        {
            Serial.println(
                "[STREAM] Client disconnected"
            );

            break;
        }

        // ----------------------------------------------------
        // Small delay
        // ----------------------------------------------------

        delay(20);
    }

    Serial.println(
        "[STREAM] Stream ended"
    );
}

// ============================================================
// 404
// ============================================================

void handleNotFound()
{
    server.send(
        404,
        "text/plain",
        "404 - Not Found"
    );
}

// ============================================================
// START SERVER
// ============================================================

void startWebServer()
{
    Serial.println();
    Serial.println(
        "[HTTP] Starting WebServer..."
    );

    // --------------------------------------------------------
    // Routes
    // --------------------------------------------------------

    server.on(
        "/",
        HTTP_GET,
        handleRoot
    );

    server.on(
        "/capture",
        HTTP_GET,
        handleCapture
    );

    server.on(
        "/stream",
        HTTP_GET,
        handleStream
    );

    server.onNotFound(
        handleNotFound
    );

    // --------------------------------------------------------
    // Start
    // --------------------------------------------------------

    server.begin();

    Serial.println(
        "[HTTP] WebServer started"
    );

    Serial.println(
        "[HTTP] Port: 80"
    );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    // ========================================================
    // SERIAL
    // ========================================================

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "       ESP32-CAM ROVER CAMERA"
    );

    Serial.println(
        "========================================"
    );

    // ========================================================
    // FLASH LED
    // ========================================================

    pinMode(
        FLASH_GPIO_NUM,
        OUTPUT
    );

    digitalWrite(
        FLASH_GPIO_NUM,
        LOW
    );

    // ========================================================
    // CAMERA CONFIGURATION
    // ========================================================

    camera_config_t config;

    config.ledc_channel =
        LEDC_CHANNEL_0;

    config.ledc_timer =
        LEDC_TIMER_0;

    // --------------------------------------------------------
    // Data pins
    // --------------------------------------------------------

    config.pin_d0 =
        Y2_GPIO_NUM;

    config.pin_d1 =
        Y3_GPIO_NUM;

    config.pin_d2 =
        Y4_GPIO_NUM;

    config.pin_d3 =
        Y5_GPIO_NUM;

    config.pin_d4 =
        Y6_GPIO_NUM;

    config.pin_d5 =
        Y7_GPIO_NUM;

    config.pin_d6 =
        Y8_GPIO_NUM;

    config.pin_d7 =
        Y9_GPIO_NUM;

    // --------------------------------------------------------
    // Control pins
    // --------------------------------------------------------

    config.pin_xclk =
        XCLK_GPIO_NUM;

    config.pin_pclk =
        PCLK_GPIO_NUM;

    config.pin_vsync =
        VSYNC_GPIO_NUM;

    config.pin_href =
        HREF_GPIO_NUM;

    config.pin_sccb_sda =
        SIOD_GPIO_NUM;

    config.pin_sccb_scl =
        SIOC_GPIO_NUM;

    config.pin_pwdn =
        PWDN_GPIO_NUM;

    config.pin_reset =
        RESET_GPIO_NUM;

    // --------------------------------------------------------
    // Clock
    // --------------------------------------------------------

    config.xclk_freq_hz =
        20000000;

    // --------------------------------------------------------
    // JPEG
    // --------------------------------------------------------

    config.pixel_format =
        PIXFORMAT_JPEG;

    // ========================================================
    // PSRAM
    // ========================================================

    if (psramFound())
    {
        Serial.println(
            "[CAMERA] PSRAM detected"
        );

        config.frame_size =
            FRAMESIZE_QVGA;

        config.jpeg_quality =
            12;

        config.fb_count =
            2;

        config.grab_mode =
            CAMERA_GRAB_LATEST;
    }
    else
    {
        Serial.println(
            "[CAMERA] PSRAM NOT detected"
        );

        config.frame_size =
            FRAMESIZE_QQVGA;

        config.jpeg_quality =
            15;

        config.fb_count =
            1;

        config.grab_mode =
            CAMERA_GRAB_WHEN_EMPTY;
    }

    // ========================================================
    // CAMERA INIT
    // ========================================================

    Serial.println(
        "[CAMERA] Initializing..."
    );

    esp_err_t result =
        esp_camera_init(
            &config
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "[CAMERA] Initialization FAILED: 0x%x\n",
            result
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "[CAMERA] Initialization OK"
    );

    // ========================================================
    // CAMERA SENSOR
    // ========================================================

    sensor_t *sensor =
        esp_camera_sensor_get();

    if (sensor != NULL)
    {
        sensor->set_framesize(
            sensor,
            FRAMESIZE_QVGA
        );

        sensor->set_quality(
            sensor,
            12
        );

        sensor->set_brightness(
            sensor,
            0
        );

        sensor->set_contrast(
            sensor,
            0
        );

        sensor->set_saturation(
            sensor,
            0
        );
    }

    // ========================================================
    // CAMERA TEST
    // ========================================================

    Serial.println(
        "[CAMERA] Testing frame capture..."
    );

    camera_fb_t *test_fb =
        esp_camera_fb_get();

    if (test_fb == NULL)
    {
        Serial.println(
            "[CAMERA] TEST FAILED"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.printf(
        "[CAMERA] TEST OK - %u bytes\n",
        (unsigned int)test_fb->len
    );

    esp_camera_fb_return(
        test_fb
    );

    // ========================================================
    // WIFI
    // ========================================================

    Serial.println();

    Serial.println(
        "[WIFI] Connecting..."
    );

    WiFi.mode(
        WIFI_STA
    );

    WiFi.setSleep(
        false
    );

    WiFi.begin(
        ssid,
        password
    );

    int attempts = 0;

    while (
        WiFi.status() != WL_CONNECTED
    )
    {
        delay(500);

        Serial.print(
            "."
        );

        attempts++;

        if (attempts >= 60)
        {
            Serial.println();

            Serial.println(
                "[WIFI] Connection FAILED"
            );

            delay(1000);

            ESP.restart();
        }
    }

    Serial.println();

    Serial.println(
        "[WIFI] Connected"
    );

    // --------------------------------------------------------
    // IP
    // --------------------------------------------------------

    Serial.print(
        "[WIFI] IP Address: "
    );

    Serial.println(
        WiFi.localIP()
    );

    // --------------------------------------------------------
    // RSSI
    // --------------------------------------------------------

    Serial.print(
        "[WIFI] RSSI: "
    );

    Serial.print(
        WiFi.RSSI()
    );

    Serial.println(
        " dBm"
    );

    // ========================================================
    // START WEB SERVER
    // ========================================================

    startWebServer();

    // ========================================================
    // READY
    // ========================================================

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "          CAMERA READY"
    );

    Serial.println(
        "========================================"
    );

    Serial.print(
        "Web Page : http://"
    );

    Serial.println(
        WiFi.localIP()
    );

    Serial.print(
        "Capture  : http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/capture"
    );

    Serial.print(
        "Stream   : http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/stream"
    );

    Serial.println(
        "========================================"
    );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Handle HTTP clients
    // --------------------------------------------------------

    server.handleClient();

    // --------------------------------------------------------
    // Wi-Fi monitoring
    // --------------------------------------------------------

    static unsigned long lastWifiCheck = 0;

    if (
        millis() - lastWifiCheck > 5000
    )
    {
        lastWifiCheck = millis();

        if (
            WiFi.status() != WL_CONNECTED
        )
        {
            Serial.println(
                "[WIFI] Connection lost"
            );

            WiFi.reconnect();
        }
    }

    delay(2);
}
