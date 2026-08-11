#include <Arduino.h>
#include <WiFi.h>
#include "VideoStream.h"

#include "AppConfig.h"
#include "FcLink.h"

namespace {

constexpr uint8_t CAMERA_CHANNEL = 0;
constexpr char STREAM_BOUNDARY[] = "bw21frame";
constexpr uint32_t REQUEST_TIMEOUT_MS = 750;
constexpr uint32_t CLIENT_WRITE_TIMEOUT_MS = 2500;
constexpr uint32_t SERIAL_STATUS_INTERVAL_MS = 5000;

struct StreamProfile {
    const char* name;
    uint8_t fps;
    uint8_t jpegQuality;
};

const StreamProfile PROFILES[] = {
    {"Smooth", 30, 3},
    {"Balanced", 20, 5},
    {"Detail", 10, 8},
};

constexpr int PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);
constexpr int DEFAULT_PROFILE = 1;

VideoSetting streamConfig(BW21CAM_STREAM_WIDTH, BW21CAM_STREAM_HEIGHT,
                          PROFILES[DEFAULT_PROFILE].fps, VIDEO_JPEG, 1);
WiFiServer controlServer(BW21CAM_CONTROL_PORT);
WiFiServer streamServer(BW21CAM_STREAM_PORT);

volatile int activeProfile = DEFAULT_PROFILE;
volatile int pendingProfile = -1;
volatile uint32_t frameCount = 0;
volatile uint32_t streamErrorCount = 0;
volatile uint32_t streamClientCount = 0;
volatile uint32_t streamRejectCount = 0;

#if BW21CAM_USE_ACCESS_POINT
char apSsid[] = BW21CAM_AP_SSID;
char apPassword[] = BW21CAM_AP_PASSWORD;
char apChannel[] = BW21CAM_AP_CHANNEL;
#else
char stationSsid[] = BW21CAM_STATION_SSID;
char stationPassword[] = BW21CAM_STATION_PASSWORD;
#endif

const char INDEX_HTML[] = R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BW21 Camera</title>
  <style>
    *{box-sizing:border-box}body{margin:0;background:#111418;color:#f5f7fa;font:15px Arial,sans-serif}
    main{width:min(920px,100%);margin:auto;padding:16px}header{display:flex;align-items:end;justify-content:space-between;gap:12px;margin-bottom:12px}
    h1{margin:0;font-size:24px;letter-spacing:0}.state{color:#aeb7c2;font-size:13px;text-align:right}
    .viewer{width:100%;aspect-ratio:16/9;background:#050607;border:1px solid #30363d;overflow:hidden}
    .viewer img{display:block;width:100%;height:100%;object-fit:contain}.bar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:12px 0;flex-wrap:wrap}
    .modes{display:inline-grid;grid-template-columns:repeat(3,1fr);border:1px solid #3c444e}.modes button{min-width:96px;padding:10px 13px;border:0;border-right:1px solid #3c444e;background:#1b2026;color:#e8edf2;font-weight:700;cursor:pointer}
    .modes button:last-child{border-right:0}.modes button.active{background:#54c27a;color:#07120b}.modes button:disabled{opacity:.55;cursor:wait}
    .metrics{display:flex;gap:16px;color:#c5ced8;font-size:13px}.metrics b{color:#f3c969;font-weight:700}@media(max-width:520px){header{align-items:start}.modes{width:100%}.modes button{min-width:0;padding:10px 6px}.metrics{width:100%;justify-content:space-between}}
  </style>
</head>
<body>
  <main>
    <header><h1>BW21 Camera</h1><div id="state" class="state">Connecting</div></header>
    <div class="viewer"><img id="stream" alt="Live camera"></div>
    <div class="bar">
      <div class="modes" aria-label="Video profile">
        <button data-profile="0">Smooth</button><button data-profile="1">Balanced</button><button data-profile="2">Detail</button>
      </div>
      <div class="metrics"><span><b id="fps">--</b> FPS</span><span>Quality <b id="quality">--</b>/9</span><span><b>1280x720</b></span></div>
    </div>
  </main>
  <script>
    const state=document.getElementById('state'),buttons=[...document.querySelectorAll('button[data-profile]')];
    const stream=document.getElementById('stream');
    let retryTimer=0,lastFrames=-1,lastProgressAt=Date.now();
    function markLive(text){if(retryTimer){clearTimeout(retryTimer);retryTimer=0}state.textContent=text}
    function connectStream(){retryTimer=0;lastProgressAt=Date.now();state.textContent='Connecting';stream.src='http://'+location.hostname+':81/stream?t='+Date.now()}
    function reconnect(delay=1000){if(retryTimer)return;state.textContent='Reconnecting';retryTimer=setTimeout(connectStream,delay)}
    stream.onload=()=>{lastProgressAt=Date.now();markLive('Live')};
    stream.onerror=()=>reconnect();
    async function status(){try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw new Error('status');const s=await r.json();
      document.getElementById('fps').textContent=s.fps;document.getElementById('quality').textContent=s.quality;
      buttons.forEach((b,i)=>b.classList.toggle('active',i===s.profile));const now=Date.now();
      if(lastFrames<0){lastFrames=s.frames;lastProgressAt=now}else if(s.frames!==lastFrames){lastFrames=s.frames;lastProgressAt=now;markLive('Live | '+s.frames+' frames')}else if(now-lastProgressAt>5000){reconnect(250)}
    }catch(e){if(!retryTimer)state.textContent='Control link lost'}}
    buttons.forEach(b=>b.onclick=async()=>{buttons.forEach(x=>x.disabled=true);try{const r=await fetch('/api/profile?id='+b.dataset.profile,{cache:'no-store'});if(!r.ok)throw new Error('profile');await status()}finally{buttons.forEach(x=>x.disabled=false)}});
    connectStream();status();setInterval(status,2000);
  </script>
</body>
</html>)HTML";

bool writeAll(WiFiClient& client, const uint8_t* data, size_t length)
{
    size_t sent = 0;
    uint32_t lastProgressMs = millis();
    while (sent < length && client.connected()) {
        size_t block = length - sent;
        if (block > 1460) {
            block = 1460;
        }
        const bool written = client.write(data + sent, block) > 0;
        if (written) {
            // AmebaPro2 reports write success as 1, not the transmitted byte count.
            sent += block;
            lastProgressMs = millis();
        } else {
            if (millis() - lastProgressMs >= CLIENT_WRITE_TIMEOUT_MS) {
                return false;
            }
            delay(1);
        }
    }
    return sent == length;
}

bool writeText(WiFiClient& client, const char* text)
{
    return writeAll(client, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

bool readRequestLine(WiFiClient& client, char* line, size_t capacity)
{
    const uint32_t startedMs = millis();
    size_t position = 0;
    while (client.connected() && millis() - startedMs < REQUEST_TIMEOUT_MS) {
        while (client.available() > 0) {
            const char value = static_cast<char>(client.read());
            if (value == '\n') {
                line[position] = 0;
                return position > 0;
            }
            if (value != '\r' && position + 1 < capacity) {
                line[position++] = value;
            }
        }
        delay(1);
    }
    line[position] = 0;
    return false;
}

bool requestTargets(const char* request, const char* path)
{
    if (!request || strncmp(request, "GET ", 4) != 0) {
        return false;
    }

    const char* target = request + 4;
    const size_t pathLength = strlen(path);
    return strncmp(target, path, pathLength) == 0 &&
           (target[pathLength] == ' ' || target[pathLength] == '?' ||
            target[pathLength] == 0);
}

void drainHeaders(WiFiClient& client)
{
    char line[160];
    while (readRequestLine(client, line, sizeof(line))) {
        if (line[0] == 0) {
            break;
        }
    }
}

void sendResponse(WiFiClient& client, int code, const char* reason,
                  const char* contentType, const char* body)
{
    char header[240];
    snprintf(header, sizeof(header),
             "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\n"
             "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
             code, reason, contentType, static_cast<unsigned long>(strlen(body)));
    writeText(client, header);
    writeText(client, body);
}

void sendStatus(WiFiClient& client)
{
    const int profileId = activeProfile;
    const StreamProfile& profile = PROFILES[profileId];
    char json[460];
    snprintf(json, sizeof(json),
             "{\"version\":\"%s\",\"profile\":%d,\"name\":\"%s\","
             "\"fps\":%u,\"quality\":%u,\"width\":%u,\"height\":%u,"
             "\"frames\":%lu,\"stream_clients\":%lu,\"stream_errors\":%lu,"
             "\"stream_rejects\":%lu,"
             "\"fc_enabled\":%s,\"fc_connected\":%s,\"fc_requests\":%lu,"
             "\"fc_responses\":%lu,\"fc_crc_errors\":%lu}",
             BW21CAM_VERSION, profileId, profile.name, profile.fps, profile.jpegQuality,
             BW21CAM_STREAM_WIDTH, BW21CAM_STREAM_HEIGHT,
             static_cast<unsigned long>(frameCount),
             static_cast<unsigned long>(streamClientCount),
             static_cast<unsigned long>(streamErrorCount),
             static_cast<unsigned long>(streamRejectCount),
             BW21CAM_ENABLE_FC_LINK ? "true" : "false",
             FcLink::connected() ? "true" : "false",
             static_cast<unsigned long>(FcLink::requestCount()),
             static_cast<unsigned long>(FcLink::responseCount()),
             static_cast<unsigned long>(FcLink::checksumErrorCount()));
    sendResponse(client, 200, "OK", "application/json", json);
}

void handleControlClient(WiFiClient& client)
{
    char request[192];
    if (!readRequestLine(client, request, sizeof(request))) {
        client.stop();
        return;
    }
    drainHeaders(client);

    if (requestTargets(request, "/api/status")) {
        sendStatus(client);
    } else if (requestTargets(request, "/api/profile")) {
        const char* id = strstr(request, "?id=");
        char* end = nullptr;
        const long requested = id ? strtol(id + 4, &end, 10) : -1;
        const bool valid = id && end != id + 4 && (*end == ' ' || *end == '&') &&
                           requested >= 0 && requested < PROFILE_COUNT;
        if (valid) {
            pendingProfile = requested;
            sendResponse(client, 202, "Accepted", "application/json", "{\"accepted\":true}");
        } else {
            sendResponse(client, 400, "Bad Request", "application/json", "{\"accepted\":false}");
        }
    } else if (requestTargets(request, "/")) {
        sendResponse(client, 200, "OK", "text/html; charset=utf-8", INDEX_HTML);
    } else {
        sendResponse(client, 404, "Not Found", "text/plain", "Not found");
    }
    delay(1);
    client.stop();
}

void applyPendingProfile()
{
    const int requested = pendingProfile;
    if (requested < 0 || requested >= PROFILE_COUNT || requested == activeProfile) {
        if (requested == activeProfile) {
            pendingProfile = -1;
        }
        return;
    }

    pendingProfile = -1;
    const StreamProfile& profile = PROFILES[requested];
    streamConfig._fps = profile.fps;
    streamConfig.setJpegQuality(profile.jpegQuality);
    Camera.configVideoChannel(CAMERA_CHANNEL, streamConfig);
    Camera.updateVideoParams(CAMERA_CHANNEL);
    Camera.setFPS(profile.fps);
    activeProfile = requested;

    Serial.print("Video profile: ");
    Serial.print(profile.name);
    Serial.print(" fps=");
    Serial.print(profile.fps);
    Serial.print(" jpeg_quality=");
    Serial.println(profile.jpegQuality);
}

bool sendStreamHeader(WiFiClient& client)
{
    char header[240];
    snprintf(header, sizeof(header),
             "HTTP/1.1 200 OK\r\nCache-Control: no-store, no-cache\r\n"
             "Pragma: no-cache\r\nAccess-Control-Allow-Origin: *\r\n"
             "Content-Type: multipart/x-mixed-replace; boundary=%s\r\n"
             "Connection: close\r\n\r\n--%s\r\n",
             STREAM_BOUNDARY, STREAM_BOUNDARY);
    return writeText(client, header);
}

void controlTask(void*)
{
    for (;;) {
        WiFiClient client = controlServer.available();
        if (client) {
            handleControlClient(client);
        }
    }
}

void streamTask(void*)
{
    for (;;) {
        applyPendingProfile();
        WiFiClient client = streamServer.available();
        if (!client) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        char request[160];
        if (!readRequestLine(client, request, sizeof(request))) {
            streamRejectCount++;
            Serial.println("Stream request timed out");
            client.stop();
            continue;
        }
        drainHeaders(client);
        if (!requestTargets(request, "/stream")) {
            streamRejectCount++;
            Serial.print("Rejected stream request: ");
            Serial.println(request);
            sendResponse(client, 404, "Not Found", "text/plain", "Use /stream");
            client.stop();
            continue;
        }

        streamClientCount++;
        Serial.println("Camera viewer connected");
        if (!sendStreamHeader(client)) {
            streamErrorCount++;
            client.stop();
            continue;
        }

        while (client.connected()) {
            applyPendingProfile();
            const uint32_t frameStartedMs = millis();
            uint32_t imageAddress = 0;
            uint32_t imageLength = 0;
            Camera.getImage(CAMERA_CHANNEL, &imageAddress, &imageLength);
            if (!imageAddress || !imageLength) {
                streamErrorCount++;
                vTaskDelay(2 / portTICK_PERIOD_MS);
                continue;
            }

            char partHeader[128];
            snprintf(partHeader, sizeof(partHeader),
                     "Content-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n",
                     static_cast<unsigned long>(imageLength));
            if (!writeText(client, partHeader) ||
                !writeAll(client, reinterpret_cast<const uint8_t*>(imageAddress), imageLength) ||
                !writeText(client, "\r\n--bw21frame\r\n")) {
                streamErrorCount++;
                break;
            }
            frameCount++;

            const uint32_t frameIntervalMs = 1000U / PROFILES[activeProfile].fps;
            const uint32_t elapsedMs = millis() - frameStartedMs;
            if (elapsedMs < frameIntervalMs) {
                vTaskDelay((frameIntervalMs - elapsedMs) / portTICK_PERIOD_MS);
            }
        }

        client.stop();
        Serial.println("Camera viewer disconnected");
    }
}

void startWifi()
{
    int status = WL_IDLE_STATUS;
#if BW21CAM_USE_ACCESS_POINT
    Serial.print("Starting access point: ");
    Serial.println(apSsid);
    while (status != WL_CONNECTED) {
        status = WiFi.apbegin(apSsid, apPassword, apChannel, 0);
        if (status != WL_CONNECTED) {
            Serial.println("Access point start failed; retrying");
            delay(3000);
        }
    }
#else
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(stationSsid);
    while (status != WL_CONNECTED) {
        status = WiFi.begin(stationSsid, stationPassword);
        if (status != WL_CONNECTED) {
            Serial.println("Wi-Fi connection failed; retrying");
            delay(3000);
        }
    }
#endif
    WiFi.disablePowerSave();

    Serial.print("Web page: http://");
    Serial.println(WiFi.localIP());
    Serial.print("MJPEG stream: http://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.print(BW21CAM_STREAM_PORT);
    Serial.println("/stream");
}

void startCamera()
{
    streamConfig.setJpegQuality(PROFILES[DEFAULT_PROFILE].jpegQuality);
    Camera.configVideoChannel(CAMERA_CHANNEL, streamConfig);
    Camera.videoInit();
    Camera.channelBegin(CAMERA_CHANNEL);
    Camera.printInfo();
}

void printPeriodicStatus()
{
    static uint32_t lastStatusMs = 0;
    const uint32_t now = millis();
    if (now - lastStatusMs < SERIAL_STATUS_INTERVAL_MS) {
        return;
    }
    lastStatusMs = now;

    const StreamProfile& profile = PROFILES[activeProfile];
    Serial.print("CAM profile=");
    Serial.print(profile.name);
    Serial.print(" fps=");
    Serial.print(profile.fps);
    Serial.print(" quality=");
    Serial.print(profile.jpegQuality);
    Serial.print(" frames=");
    Serial.print(frameCount);
    Serial.print(" stream_errors=");
    Serial.print(streamErrorCount);
    Serial.print(" stream_clients=");
    Serial.print(streamClientCount);
    Serial.print(" stream_rejects=");
    Serial.print(streamRejectCount);
#if BW21CAM_ENABLE_FC_LINK
    Serial.print(" fc=");
    Serial.print(FcLink::connected() ? "connected" : "waiting");
    Serial.print(" fc_requests=");
    Serial.print(FcLink::requestCount());
    Serial.print(" fc_responses=");
    Serial.print(FcLink::responseCount());
#endif
    Serial.println();
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.print("BW21-CBV Cam Drone ");
    Serial.println(BW21CAM_VERSION);

    startWifi();
    startCamera();
    FcLink::begin();

    controlServer.begin();
    streamServer.begin();
    const BaseType_t controlStarted =
        xTaskCreate(controlTask, "CameraControl", 4 * 1024, nullptr, 1, nullptr);
    const BaseType_t streamStarted =
        xTaskCreate(streamTask, "CameraStream", 10 * 1024, nullptr, 2, nullptr);
    if (controlStarted != pdPASS || streamStarted != pdPASS) {
        Serial.print("Server task start failed control=");
        Serial.print(controlStarted);
        Serial.print(" stream=");
        Serial.println(streamStarted);
    }
    Serial.println("Camera test ready");
}

void loop()
{
    FcLink::update();

    printPeriodicStatus();
    delay(1);
}
