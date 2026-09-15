#include <WiFi.h>

// ============================================================
// DRILLPULSE ESP32 ROVER WIFI <-> UART2 BRIDGE
// Multi-client version: joystick + central telemetry node
// ============================================================

const char* WIFI_SSID     = "vicky";
const char* WIFI_PASSWORD = "87654321";

WiFiServer tcpServer(5000);
WiFiClient clients[2];

HardwareSerial ArduinoSerial(2);
#define ESP32_RX2 16
#define ESP32_TX2 17
#define UART_BAUD 115200

String tcpRxBuffer[2];
String uartRxBuffer = "";

unsigned long lastPingTime = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastDebugTime = 0;

const unsigned long PING_INTERVAL = 5000;
const unsigned long WIFI_CHECK_INTERVAL = 5000;
const unsigned long DEBUG_INTERVAL = 5000;

void connectWiFi();
void acceptClients();
void readTCP();
void readClientTCP(int i);
void readArduinoUART();
void sendToArduino(const String& message);
void broadcastToTCP(const String& message);
void sendPing();
void checkWiFi();
void printStatus();
void dropClient(int i);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" DRILLPULSE ESP32 ROVER BRIDGE - V2");
    Serial.println("==========================================");

    ArduinoSerial.begin(UART_BAUD, SERIAL_8N1, ESP32_RX2, ESP32_TX2);

    Serial.println("[UART2] RX = GPIO16");
    Serial.println("[UART2] TX = GPIO17");
    Serial.println("[UART2] Baud = 115200");

    for (int i = 0; i < 2; ++i) {
        clients[i] = WiFiClient();
        tcpRxBuffer[i] = "";
    }

    connectWiFi();
    tcpServer.begin();

    Serial.println("[TCP] Server started");
    Serial.println("[TCP] Port = 5000");
    Serial.println("[TCP] Max clients = 2 (joystick + central)");

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" BRIDGE READY");
    Serial.println("==========================================");
}

void loop()
{
    acceptClients();
    readTCP();
    readArduinoUART();
    sendPing();
    checkWiFi();
    printStatus();
    delay(1);
}

void connectWiFi()
{
    Serial.println("[WiFi] Connecting...");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttempt = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        if (millis() - startAttempt > 30000)
        {
            Serial.println();
            Serial.println("[WiFi] Connection timeout - retrying");
            WiFi.disconnect(true);
            delay(1000);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            startAttempt = millis();
        }
    }

    Serial.println();
    Serial.println("[WiFi] CONNECTED");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

void acceptClients()
{
    WiFiClient incoming = tcpServer.available();
    if (!incoming)
        return;

    incoming.setNoDelay(true);

    int slot = -1;
    for (int i = 0; i < 2; ++i)
    {
        if (!clients[i] || !clients[i].connected())
        {
            slot = i;
            break;
        }
    }

    if (slot < 0)
    {
        Serial.println("[TCP] Client limit reached - rejecting new client");
        incoming.stop();
        return;
    }

    clients[slot] = incoming;
    tcpRxBuffer[slot] = "";

    Serial.print("[TCP] CLIENT ");
    Serial.print(slot);
    Serial.print(" CONNECTED: ");
    Serial.println(clients[slot].remoteIP());
}

void readTCP()
{
    for (int i = 0; i < 2; ++i)
        readClientTCP(i);
}

void readClientTCP(int i)
{
    if (!clients[i])
        return;

    if (!clients[i].connected())
    {
        dropClient(i);
        return;
    }

    while (clients[i].available())
    {
        char c = clients[i].read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (tcpRxBuffer[i].length() > 0)
            {
                String command = tcpRxBuffer[i];
                tcpRxBuffer[i] = "";
                command.trim();

                if (command.length() > 0)
                {
                    Serial.print("[TCP ");
                    Serial.print(i);
                    Serial.print(" -> UART2] ");
                    Serial.println(command);

                    // Only recognized control commands are forwarded.
                    // Central node normally sends nothing.
                    if (command.startsWith("CMD,") ||
                        command.startsWith("SPEED,") ||
                        command == "STOP" ||
                        command == "PING")
                    {
                        sendToArduino(command);
                    }
                    else
                    {
                        Serial.print("[TCP] Ignored non-control message: ");
                        Serial.println(command);
                    }
                }
            }
        }
        else
        {
            tcpRxBuffer[i] += c;

            if (tcpRxBuffer[i].length() > 200)
            {
                tcpRxBuffer[i] = "";
                Serial.println("[TCP] RX buffer overflow - cleared");
            }
        }
    }
}

void sendToArduino(const String& message)
{
    if (message.length() == 0)
        return;

    ArduinoSerial.print(message);
    ArduinoSerial.print('\n');
}

void readArduinoUART()
{
    while (ArduinoSerial.available())
    {
        char c = ArduinoSerial.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            if (uartRxBuffer.length() > 0)
            {
                String message = uartRxBuffer;
                uartRxBuffer = "";
                message.trim();

                if (message.length() > 0)
                {
                    Serial.print("[UART2 -> TCP] ");
                    Serial.println(message);
                    broadcastToTCP(message);
                }
            }
        }
        else
        {
            uartRxBuffer += c;

            if (uartRxBuffer.length() > 300)
            {
                uartRxBuffer = "";
                Serial.println("[UART2] RX buffer overflow - cleared");
            }
        }
    }
}

void broadcastToTCP(const String& message)
{
    for (int i = 0; i < 2; ++i)
    {
        if (clients[i] && clients[i].connected())
        {
            clients[i].print(message);
            clients[i].print('\n');
        }
    }
}

void sendPing()
{
    unsigned long now = millis();

    if (now - lastPingTime < PING_INTERVAL)
        return;

    lastPingTime = now;

    // Keep Arduino's safety timer alive only while at least one TCP client
    // is connected. The joystick also sends motion keepalives every 250 ms.
    bool anyClient = false;
    for (int i = 0; i < 2; ++i)
        if (clients[i] && clients[i].connected())
            anyClient = true;

    if (anyClient)
    {
        ArduinoSerial.println("PING");
        Serial.println("[BRIDGE] PING -> Arduino");
    }
}

void checkWiFi()
{
    unsigned long now = millis();

    if (now - lastWiFiCheck < WIFI_CHECK_INTERVAL)
        return;

    lastWiFiCheck = now;

    if (WiFi.status() == WL_CONNECTED)
        return;

    Serial.println("[WiFi] CONNECTION LOST");

    for (int i = 0; i < 2; ++i)
        dropClient(i);

    WiFi.disconnect();
    delay(500);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
    {
        delay(250);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("[WiFi] RECONNECTED");
        Serial.print("[WiFi] New IP: ");
        Serial.println(WiFi.localIP());

        tcpServer.begin();
        Serial.println("[TCP] Server restarted");
    }
    else
    {
        Serial.println("[WiFi] Reconnection failed");
    }
}

void dropClient(int i)
{
    if (clients[i])
        clients[i].stop();

    tcpRxBuffer[i] = "";
}

void printStatus()
{
    unsigned long now = millis();

    if (now - lastDebugTime < DEBUG_INTERVAL)
        return;

    lastDebugTime = now;

    Serial.println();
    Serial.println("------------- BRIDGE STATUS -------------");
    Serial.print("WiFi       : ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");

    Serial.print("IP         : ");
    Serial.println(WiFi.localIP());

    Serial.print("RSSI       : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    for (int i = 0; i < 2; ++i)
    {
        Serial.print("TCP Client ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println((clients[i] && clients[i].connected()) ? "CONNECTED" : "WAITING");
    }

    Serial.println("UART2      : 115200");
    Serial.println("UART RX2   : GPIO16");
    Serial.println("UART TX2   : GPIO17");
    Serial.println("-----------------------------------------");
}
