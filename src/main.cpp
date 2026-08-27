#include <Arduino.h>
#include <WiFi.h>
#include "VideoStream.h"
#include "server_drv.h"
#include <lwip/sockets.h>

extern "C" {
#include "wifi_conf.h"
}

#undef read
#undef write

#include <stdarg.h>

#include "AppConfig.h"
#include "FcLink.h"
#include "OnDeviceVision.h"

namespace {

constexpr uint8_t CAMERA_CHANNEL = 0;
constexpr char STREAM_BOUNDARY[] = "bw21frame";
constexpr uint32_t REQUEST_TIMEOUT_MS = 2000;
constexpr uint32_t CLIENT_WRITE_TIMEOUT_MS = 2000;
constexpr uint32_t SERIAL_STATUS_INTERVAL_MS = 5000;
constexpr uint32_t FRAME_STALL_THRESHOLD_MS = 250;
constexpr size_t TCP_WRITE_BLOCK_BYTES = 1460;
constexpr uint8_t TCP_WRITE_BLOCKS_PER_YIELD = 8;
constexpr uint8_t MAX_CONTROL_CLIENTS_PER_PASS = 4;

class DriverClient {
public:
    DriverClient(ServerDrv& driver, int socket) : driver_(driver), socket_(socket) {}
    ~DriverClient() { stop(); }

    DriverClient(const DriverClient&) = delete;
    DriverClient& operator=(const DriverClient&) = delete;

    explicit operator bool() const { return socket_ >= 0; }

    bool connected() const { return socket_ >= 0; }

    int read(uint8_t* data, size_t length)
    {
        if (socket_ < 0 || !data || !length) return -1;
        const int result = driver_.getDataBuf(socket_, data, length);
        if (result <= 0) stop();
        return result;
    }

    size_t write(const uint8_t* data, size_t length)
    {
        if (socket_ < 0 || !data || !length) return 0;
        const int result = driver_.sendData(socket_, data, length);
        if (result > 0) return static_cast<size_t>(result);
        Serial.print("TCP send failed fd=");
        Serial.print(socket_);
        Serial.print(" requested=");
        Serial.print(length);
        Serial.print(" result=");
        Serial.print(result);
        Serial.print(" errno=");
        Serial.println(driver_.getLastErrno(socket_));
        stop();
        return 0;
    }

    void stop()
    {
        if (socket_ < 0) return;
        driver_.stopSocket(socket_);
        socket_ = -1;
    }

private:
    ServerDrv& driver_;
    int socket_;
};

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
constexpr int DEFAULT_PROFILE = 0;

VideoSetting streamConfig(BW21CAM_STREAM_WIDTH, BW21CAM_STREAM_HEIGHT,
                          PROFILES[DEFAULT_PROFILE].fps, VIDEO_JPEG, 1);
CameraSetting cameraSettings;
ServerDrv controlDriver;
ServerDrv streamDriver;
int controlListenSocket = -1;
int streamListenSocket = -1;

volatile int activeProfile = DEFAULT_PROFILE;
volatile int pendingProfile = -1;
volatile uint32_t frameCount = 0;
volatile uint32_t cameraFrameCount = 0;
volatile uint32_t captureErrorCount = 0;
volatile uint32_t streamErrorCount = 0;
volatile uint32_t streamClientCount = 0;
volatile uint32_t streamRejectCount = 0;
volatile uint32_t streamStallCount = 0;
volatile uint32_t maxCaptureMs = 0;
volatile uint32_t maxSendMs = 0;
volatile uint32_t maxFrameGapMs = 0;
volatile uint32_t lastCameraFrameMs = 0;

void configureClientSocket(ServerDrv& driver, int socket)
{
    if (socket < 0) return;
    const int timeoutMs = CLIENT_WRITE_TIMEOUT_MS;
    driver.setSockRecvTimeout(socket, REQUEST_TIMEOUT_MS);
    lwip_setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeoutMs, sizeof(timeoutMs));
}

int createListenSocket(ServerDrv& driver, uint16_t port)
{
    return driver.startServer(port, TCP_MODE, NON_BLOCKING_MODE);
}

bool socketReadable(int socket)
{
    if (socket < 0) return false;
    fd_set sockets;
    FD_ZERO(&sockets);
    FD_SET(socket, &sockets);
    timeval timeout = {};
    return lwip_select(socket + 1, &sockets, nullptr, nullptr, &timeout) > 0;
}

int acceptSocket(ServerDrv& driver, int listenSocket, const char* label)
{
    if (!socketReadable(listenSocket)) return -1;
    const int socket = driver.getAvailable(listenSocket);
    if (socket < 0) return -1;
    Serial.print(label);
    Serial.print(" socket accepted fd=");
    Serial.println(socket);
    configureClientSocket(driver, socket);
    return socket;
}

#if BW21CAM_USE_ACCESS_POINT
char apSsid[] = BW21CAM_AP_SSID;
char apPassword[] = BW21CAM_AP_PASSWORD;
char apChannel[] = BW21CAM_AP_CHANNEL;
char apFallbackChannel[] = BW21CAM_AP_FALLBACK_CHANNEL;
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
    *{box-sizing:border-box}html,body{max-width:100%;overflow-x:hidden}body{margin:0;background:#101317;color:#f4f6f8;font:15px Arial,sans-serif}
    main{width:min(960px,100%);margin:auto;padding:14px}header,.toolbar{display:flex;align-items:center;justify-content:space-between;gap:12px}
    header{margin-bottom:10px}h1{margin:0;font-size:22px;letter-spacing:0}.state{color:#aab4bf;font-size:13px}
    .viewer{position:relative;width:100%;aspect-ratio:16/9;background:#050607;border:1px solid #333a43;overflow:hidden}
    .viewer img,.viewer canvas{position:absolute;inset:0;width:100%;height:100%}.viewer img{display:block;object-fit:contain}.viewer img.mirrored{transform:scaleX(-1)}.viewer canvas{pointer-events:none}
    .toolbar{padding:10px 0;flex-wrap:wrap}.controls{display:flex;gap:8px;flex-wrap:wrap}.modes,.vision-modes{display:inline-grid;border:1px solid #3d4650}.modes{grid-template-columns:repeat(3,1fr)}.vision-modes{grid-template-columns:repeat(2,1fr)}
    button{height:38px;padding:0 12px;border:1px solid #3d4650;background:#1c2229;color:#eef2f5;font-weight:700;cursor:pointer}button.active{background:#50bd78;color:#07120b;border-color:#50bd78}button:disabled{opacity:.55;cursor:wait}
    .modes,.modes button,.vision-modes,.vision-modes button{min-width:0}.modes button,.vision-modes button{border-width:0 1px 0 0}.modes button:last-child,.vision-modes button:last-child{border-right:0}.metrics{display:flex;gap:14px;color:#b9c3cd;font-size:13px}.metrics b{color:#f1c75b}
    .results{border-top:1px solid #303741;padding-top:10px}.qr{font-size:14px;color:#b9c3cd;overflow-wrap:anywhere}.qr b{color:#f4f6f8}.qr span{margin-left:8px;color:#f1c75b;font-size:12px}.objects{display:flex;gap:8px;flex-wrap:wrap;margin-top:9px;min-height:30px}
    .object{padding:6px 8px;border-left:3px solid #50bd78;background:#1a2026;color:#e6ebef;font-size:13px}.empty{color:#8f9aa5;font-size:13px}
    @media(max-width:560px){main{padding:10px}header{align-items:flex-start;flex-wrap:wrap}.toolbar{align-items:flex-start}.controls,.modes,.vision-modes{width:100%}.modes button,.vision-modes button{padding:0 4px}.metrics{width:100%;justify-content:space-between;gap:6px}}
  </style>
</head>
<body>
  <main>
    <header><h1>BW21 Camera</h1><div id="state" class="state">Connecting</div></header>
    <div id="viewer" class="viewer"><img id="stream" alt="Live camera"><canvas id="overlay"></canvas></div>
    <div class="toolbar">
      <div class="controls">
        <div class="modes" aria-label="Video profile">
          <button data-profile="0">Smooth</button><button data-profile="1">Balanced</button><button data-profile="2">Detail</button>
        </div>
        <div class="vision-modes" aria-label="Vision mode">
          <button data-vision="0">Camera</button><button data-vision="1">Vision</button>
        </div>
        <button id="mirror" type="button" aria-pressed="false" title="Mirror preview">Mirror</button>
      </div>
      <div class="metrics"><span><b id="fps">--</b> FPS</span><span>Q<b id="quality">--</b></span><span><b>1280x720</b></span></div>
    </div>
    <div class="results"><div class="qr">QR <b id="qr">--</b><span id="qrmeta"></span></div><div id="objects" class="objects"><span class="empty">No objects</span></div></div>
  </main>
  <script>
    const state=document.getElementById('state'),stream=document.getElementById('stream'),overlay=document.getElementById('overlay'),ctx=overlay.getContext('2d');
    const buttons=[...document.querySelectorAll('button[data-profile]')],visionButtons=[...document.querySelectorAll('button[data-vision]')],mirror=document.getElementById('mirror'),objects=document.getElementById('objects'),qrmeta=document.getElementById('qrmeta');
    let retryTimer=0,lastFrames=-1,lastProgressAt=Date.now(),pollBusy=false,visionSwitchBusy=false,mirrored=false;
    function markLive(text){if(retryTimer){clearTimeout(retryTimer);retryTimer=0}state.textContent=text}
    function connectStream(){retryTimer=0;lastProgressAt=Date.now();state.textContent='Connecting';stream.src='http://'+location.hostname+':81/stream?t='+Date.now()}
    function reconnect(delay=1000){if(retryTimer)return;state.textContent='Reconnecting';retryTimer=setTimeout(connectStream,delay)}
    stream.onload=()=>{lastProgressAt=Date.now();markLive('Live')};
    stream.onerror=()=>reconnect();
    function draw(v){const w=overlay.width=overlay.clientWidth*devicePixelRatio,h=overlay.height=overlay.clientHeight*devicePixelRatio;ctx.clearRect(0,0,w,h);if(!v.enabled)return;ctx.lineWidth=2*devicePixelRatio;ctx.font=(12*devicePixelRatio)+'px Arial';ctx.textBaseline='top';
      v.objects.forEach(o=>{const x=(mirrored?1-o.box[2]:o.box[0])*w,y=o.box[1]*h,bw=(o.box[2]-o.box[0])*w,bh=(o.box[3]-o.box[1])*h,color=v.color_enabled&&o.color&&o.color!=='unknown'?' '+o.color:'',label=o.name+' '+o.score+'%'+color;ctx.strokeStyle=o.name==='face'?'#f1c75b':'#50bd78';ctx.fillStyle=ctx.strokeStyle;ctx.strokeRect(x,y,bw,bh);const tw=ctx.measureText(label).width+8*devicePixelRatio,lh=18*devicePixelRatio;ctx.fillRect(x,Math.max(0,y-lh),tw,lh);ctx.fillStyle='#07120b';ctx.fillText(label,x+4*devicePixelRatio,Math.max(0,y-lh)+2*devicePixelRatio)})}
    function render(v){document.getElementById('fps').textContent=v.fps;document.getElementById('quality').textContent=v.quality;buttons.forEach((b,i)=>b.classList.toggle('active',i===v.profile));visionButtons.forEach((b,i)=>{b.classList.toggle('active',i===Number(v.enabled));b.disabled=visionSwitchBusy||!v.ready});document.getElementById('qr').textContent=v.enabled&&v.qr.fresh?v.qr.payload:'--';qrmeta.textContent=v.enabled&&v.qr.fresh&&v.qr.geometry.valid?'x '+v.qr.geometry.center_permille[0]+' y '+v.qr.geometry.center_permille[1]+' size '+v.qr.geometry.side_permille+(v.qr.geometry.fisheye_corrected?' fisheye':''):'';objects.replaceChildren();
      if(!v.enabled){const empty=document.createElement('span');empty.className='empty';empty.textContent=v.ready?'Vision off':'Vision unavailable';objects.appendChild(empty)}else if(v.objects.length){v.objects.forEach(o=>{const item=document.createElement('span');item.className='object';item.textContent=o.name+' '+o.score+'%'+(v.color_enabled?' | '+o.color+(o.color_confidence?' '+o.color_confidence+'%':''):'');objects.appendChild(item)})}else{const empty=document.createElement('span');empty.className='empty';empty.textContent='No person or face';objects.appendChild(empty)}draw(v)}
    async function poll(){if(pollBusy)return;pollBusy=true;try{const r=await fetch('/api/vision',{cache:'no-store'});if(!r.ok)throw new Error('vision');const v=await r.json();render(v);const now=Date.now();
      if(lastFrames<0){lastFrames=v.stream_frames;lastProgressAt=now}else if(v.stream_frames!==lastFrames){lastFrames=v.stream_frames;lastProgressAt=now;markLive('Live')}else if(now-lastProgressAt>5000){reconnect(250)}
    }catch(e){if(!retryTimer)state.textContent='Control link lost'}finally{pollBusy=false}}
    buttons.forEach(b=>b.onclick=async()=>{buttons.forEach(x=>x.disabled=true);try{const r=await fetch('/api/profile?id='+b.dataset.profile,{cache:'no-store'});if(!r.ok)throw new Error('profile');await poll()}finally{buttons.forEach(x=>x.disabled=false)}});
    visionButtons.forEach(b=>b.onclick=async()=>{visionSwitchBusy=true;visionButtons.forEach(x=>x.disabled=true);try{const r=await fetch('/api/vision-mode?enabled='+b.dataset.vision,{cache:'no-store'});if(!r.ok)throw new Error('vision mode')}catch(e){state.textContent='Mode change failed'}finally{visionSwitchBusy=false;await poll()}});
    mirror.onclick=()=>{mirrored=!mirrored;stream.classList.toggle('mirrored',mirrored);mirror.classList.toggle('active',mirrored);mirror.setAttribute('aria-pressed',String(mirrored));poll()};
    addEventListener('resize',poll);connectStream();poll();setInterval(poll,1250);
  </script>
</body>
</html>)HTML";

bool writeAll(DriverClient& client, const uint8_t* data, size_t length)
{
    size_t sent = 0;
    uint8_t blocksSinceYield = 0;
    const uint32_t startedMs = millis();
    uint32_t lastProgressMs = millis();
    while (sent < length && client) {
        if (millis() - startedMs >= CLIENT_WRITE_TIMEOUT_MS) {
            return false;
        }
        size_t block = length - sent;
        if (block > TCP_WRITE_BLOCK_BYTES) {
            block = TCP_WRITE_BLOCK_BYTES;
        }
        const size_t written = client.write(data + sent, block);
        if (written > 0 && written <= block) {
            sent += written;
            lastProgressMs = millis();
            if (++blocksSinceYield >= TCP_WRITE_BLOCKS_PER_YIELD) {
                blocksSinceYield = 0;
                vTaskDelay(1);
            }
        } else {
            if (!client.connected()) return false;
            if (millis() - lastProgressMs >= CLIENT_WRITE_TIMEOUT_MS) {
                return false;
            }
            delay(1);
        }
    }
    return sent == length;
}

bool writeText(DriverClient& client, const char* text)
{
    return writeAll(client, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

bool readHttpRequest(DriverClient& client, char* requestLine, size_t capacity)
{
    const uint32_t startedMs = millis();
    size_t position = 0;
    bool requestLineComplete = false;
    uint32_t headerTail = 0;
    while (client.connected() && millis() - startedMs < REQUEST_TIMEOUT_MS) {
        uint8_t buffer[192];
        const int received = client.read(buffer, sizeof(buffer));
        if (received <= 0) break;
        for (int index = 0; index < received; index++) {
            const char value = static_cast<char>(buffer[index]);
            headerTail = (headerTail << 8) | static_cast<uint8_t>(value);
            if (!requestLineComplete) {
                if (value == '\n') {
                    requestLine[position] = 0;
                    requestLineComplete = position > 0;
                } else if (value != '\r' && position + 1 < capacity) {
                    requestLine[position++] = value;
                }
            }
            if (requestLineComplete &&
                (headerTail == 0x0d0a0d0aU || (headerTail & 0xffffU) == 0x0a0aU)) {
                return true;
            }
        }
    }
    requestLine[position] = 0;
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

void sendResponse(DriverClient& client, int code, const char* reason,
                  const char* contentType, const char* body)
{
    char header[240];
    snprintf(header, sizeof(header),
             "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %lu\r\n"
             "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
             code, reason, contentType, static_cast<unsigned long>(strlen(body)));
    const bool sent = writeText(client, header) && writeText(client, body);
    Serial.print("HTTP response code=");
    Serial.print(code);
    Serial.print(" body_bytes=");
    Serial.print(strlen(body));
    Serial.print(" sent=");
    Serial.println(sent ? 1 : 0);
}

void sendQrFrame(DriverClient& client)
{
    const uint8_t* jpeg = nullptr;
    size_t length = 0;
    if (!OnDeviceVision::lockQrJpeg(jpeg, length)) {
        sendResponse(client, 503, "Service Unavailable", "text/plain",
                     "QR frame unavailable");
        return;
    }

    char header[240];
    snprintf(header, sizeof(header),
             "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %lu\r\n"
             "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
             static_cast<unsigned long>(length));
    const bool sent = writeText(client, header) && writeAll(client, jpeg, length);
    OnDeviceVision::unlockQrJpeg();
    Serial.print("QR input frame bytes=");
    Serial.print(length);
    Serial.print(" sent=");
    Serial.println(sent ? 1 : 0);
}

void sendStatus(DriverClient& client)
{
    const int profileId = activeProfile;
    const StreamProfile& profile = PROFILES[profileId];
    static OnDeviceVision::Status vision;
    OnDeviceVision::getStatus(vision);
    static char json[760];
    snprintf(json, sizeof(json),
             "{\"version\":\"%s\",\"variant\":\"%s\",\"profile\":%d,\"name\":\"%s\","
             "\"fps\":%u,\"quality\":%u,\"width\":%u,\"height\":%u,"
             "\"frames\":%lu,\"camera_frames\":%lu,\"capture_errors\":%lu,"
             "\"stream_clients\":%lu,\"stream_errors\":%lu,"
             "\"stream_rejects\":%lu,\"stream_stalls\":%lu,"
             "\"max_capture_ms\":%lu,\"max_send_ms\":%lu,\"max_gap_ms\":%lu,"
             "\"vision_enabled\":%s,\"vision_ready\":%s,\"qr_self_test\":%s,"
             "\"quirc_ready\":%s,\"quirc_self_test\":%s,\"vision_frames\":%lu,"
             "\"fc_enabled\":%s,\"fc_connected\":%s,\"fc_requests\":%lu,"
             "\"fc_responses\":%lu,\"fc_crc_errors\":%lu}",
             BW21CAM_VERSION, BW21CAM_BUILD_VARIANT, profileId, profile.name,
             profile.fps, profile.jpegQuality,
             BW21CAM_STREAM_WIDTH, BW21CAM_STREAM_HEIGHT,
             static_cast<unsigned long>(frameCount),
             static_cast<unsigned long>(cameraFrameCount),
             static_cast<unsigned long>(captureErrorCount),
             static_cast<unsigned long>(streamClientCount),
             static_cast<unsigned long>(streamErrorCount),
             static_cast<unsigned long>(streamRejectCount),
             static_cast<unsigned long>(streamStallCount),
             static_cast<unsigned long>(maxCaptureMs),
             static_cast<unsigned long>(maxSendMs),
             static_cast<unsigned long>(maxFrameGapMs),
             vision.enabled ? "true" : "false",
             vision.ready ? "true" : "false",
             vision.qrSelfTestPassed ? "true" : "false",
             vision.quircReady ? "true" : "false",
             vision.quircSelfTestPassed ? "true" : "false",
             static_cast<unsigned long>(vision.analyzedFrames),
             BW21CAM_ENABLE_FC_LINK ? "true" : "false",
             FcLink::connected() ? "true" : "false",
             static_cast<unsigned long>(FcLink::requestCount()),
             static_cast<unsigned long>(FcLink::responseCount()),
             static_cast<unsigned long>(FcLink::checksumErrorCount()));
    sendResponse(client, 200, "OK", "application/json", json);
}

size_t appendText(char* destination, size_t capacity, size_t used, const char* format, ...)
{
    if (used >= capacity) return capacity;
    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(destination + used, capacity - used, format, arguments);
    va_end(arguments);
    if (written < 0) return used;
    const size_t amount = static_cast<size_t>(written);
    return amount >= capacity - used ? capacity : used + amount;
}

size_t appendJsonString(char* destination, size_t capacity, size_t used, const char* value)
{
    used = appendText(destination, capacity, used, "\"");
    for (const uint8_t* cursor = reinterpret_cast<const uint8_t*>(value); *cursor; cursor++) {
        if (*cursor == '\\' || *cursor == '"') {
            used = appendText(destination, capacity, used, "\\%c", *cursor);
        } else if (*cursor >= 32 && *cursor != 127) {
            used = appendText(destination, capacity, used, "%c", *cursor);
        }
    }
    return appendText(destination, capacity, used, "\"");
}

void sendVision(DriverClient& client)
{
    static OnDeviceVision::Status vision;
    OnDeviceVision::getStatus(vision);
    const uint32_t now = millis();
    const uint32_t qrAge = vision.qrSeenAtMs ? now - vision.qrSeenAtMs : 0;
    const uint32_t objectAge = vision.objectSeenAtMs ? now - vision.objectSeenAtMs : 0;
    const bool qrFresh = vision.enabled && vision.qrSeenAtMs &&
                         qrAge <= BW21CAM_QR_STALE_MS;
    const bool objectsFresh = vision.enabled && vision.objectSeenAtMs &&
                              objectAge <= BW21CAM_OBJECT_STALE_MS;
    const uint32_t qrAccountedFrames = vision.qrCheckedFrames + vision.jpegDecodeErrors;
    const uint32_t qrBypassedFrames = vision.analyzedFrames > qrAccountedFrames
                                         ? vision.analyzedFrames - qrAccountedFrames
                                         : 0;

    static char json[2800];
    size_t used = appendText(
        json, sizeof(json), 0,
        "{\"profile\":%d,\"fps\":%u,\"quality\":%u,\"stream_frames\":%lu,"
        "\"enabled\":%s,\"ready\":%s,\"color_enabled\":%s,"
        "\"frame_sequence\":%lu,\"frames\":%lu,"
        "\"jpeg_errors\":%lu,\"process_ms\":%lu,\"max_process_ms\":%lu,"
        "\"jpeg_decode_ms\":%lu,\"qr_scan_ms\":%lu,"
        "\"qr_self_test\":%s,\"quirc_ready\":%s,\"quirc_self_test\":%s,"
        "\"quirc_fast_self_test\":%s,\"quirc_full_self_test\":%s,"
        "\"zbar_self_test\":%s,\"qr_luma\":[%u,%u,%u],\"qr_last_detail\":%d,"
        "\"qr_checked_frames\":%lu,\"qr_scan_passes\":%lu,"
        "\"quirc_scans\":%lu,\"quirc_fast_scans\":%lu,\"quirc_full_scans\":%lu,"
        "\"quirc_candidates\":%lu,"
        "\"quirc_errors\":%lu,\"quirc_mirrored\":%lu,\"zbar_fallbacks\":%lu,"
        "\"qr_enhanced_scans\":%lu,\"fisheye_scans\":%lu,"
        "\"fisheye_decodes\":%lu,\"fisheye_profile\":%u,"
        "\"qr_bypassed_frames\":%lu,"
        "\"qr\":{\"fresh\":%s,\"age_ms\":%lu,\"sequence\":%lu,\"payload\":",
        activeProfile, PROFILES[activeProfile].fps, PROFILES[activeProfile].jpegQuality,
        static_cast<unsigned long>(frameCount),
        vision.enabled ? "true" : "false", vision.ready ? "true" : "false",
        BW21CAM_ENABLE_COLOR_DETECTION ? "true" : "false",
        static_cast<unsigned long>(vision.frameSequence),
        static_cast<unsigned long>(vision.analyzedFrames),
        static_cast<unsigned long>(vision.jpegDecodeErrors),
        static_cast<unsigned long>(vision.lastProcessMs),
        static_cast<unsigned long>(vision.maxProcessMs),
        static_cast<unsigned long>(vision.lastJpegDecodeMs),
        static_cast<unsigned long>(vision.lastQrScanMs),
        vision.qrSelfTestPassed ? "true" : "false",
        vision.quircReady ? "true" : "false",
        vision.quircSelfTestPassed ? "true" : "false",
        vision.quircFastSelfTestPassed ? "true" : "false",
        vision.quircFullSelfTestPassed ? "true" : "false",
        vision.zbarSelfTestPassed ? "true" : "false",
        vision.qrDarkest, vision.qrMean, vision.qrBrightest,
        static_cast<int>(vision.qrLastScanDetail),
        static_cast<unsigned long>(vision.qrCheckedFrames),
        static_cast<unsigned long>(vision.qrScanPasses),
        static_cast<unsigned long>(vision.quircScanPasses),
        static_cast<unsigned long>(vision.quircFastScans),
        static_cast<unsigned long>(vision.quircFullScans),
        static_cast<unsigned long>(vision.quircCandidates),
        static_cast<unsigned long>(vision.quircDecodeErrors),
        static_cast<unsigned long>(vision.quircMirroredDecodes),
        static_cast<unsigned long>(vision.zbarFallbackScans),
        static_cast<unsigned long>(vision.qrEnhancedScans),
        static_cast<unsigned long>(vision.qrFisheyeScans),
        static_cast<unsigned long>(vision.qrFisheyeDecodes),
        vision.qrFisheyeProfile,
        static_cast<unsigned long>(qrBypassedFrames),
        qrFresh ? "true" : "false",
        static_cast<unsigned long>(qrAge), static_cast<unsigned long>(vision.qrSequence));
    used = appendJsonString(json, sizeof(json), used, qrFresh ? vision.qrPayload : "");
    used = appendText(
        json, sizeof(json), used,
        ",\"geometry\":{\"valid\":%s,\"full_resolution\":%s,"
        "\"mirrored\":%s,\"zbar_fallback\":%s,\"fisheye_corrected\":%s,"
        "\"center_permille\":[%u,%u],"
        "\"side_permille\":%u,\"area_permille\":%u,\"rotation_cdeg\":%d},"
        "\"candidates\":%lu,\"no_finder\":%lu,\"decodes\":%lu,"
        "\"decode_errors\":%lu,\"duplicates\":%lu},"
        "\"objects_fresh\":%s,\"objects_age_ms\":%lu,"
        "\"person_frames\":%lu,\"face_frames\":%lu,\"objects\":[",
        vision.qrGeometry.valid ? "true" : "false",
        vision.qrGeometry.fullResolution ? "true" : "false",
        vision.qrGeometry.mirrored ? "true" : "false",
        vision.qrGeometry.zbarFallback ? "true" : "false",
        vision.qrGeometry.fisheyeCorrected ? "true" : "false",
        vision.qrGeometry.centerXPermille, vision.qrGeometry.centerYPermille,
        vision.qrGeometry.sidePermille, vision.qrGeometry.areaPermille,
        static_cast<int>(vision.qrGeometry.rotationCdeg),
        static_cast<unsigned long>(vision.qrCandidates),
        static_cast<unsigned long>(vision.qrNoFinderCenters),
        static_cast<unsigned long>(vision.qrDecodes),
        static_cast<unsigned long>(vision.qrDecodeErrors),
        static_cast<unsigned long>(vision.qrDuplicates), objectsFresh ? "true" : "false",
        static_cast<unsigned long>(objectAge), static_cast<unsigned long>(vision.yoloFrames),
        static_cast<unsigned long>(vision.faceFrames));
    if (objectsFresh) {
        for (uint8_t i = 0; i < vision.objectCount; i++) {
            const OnDeviceVision::ObjectResult& object = vision.objects[i];
            used = appendText(json, sizeof(json), used, "%s{\"name\":", i ? "," : "");
            used = appendJsonString(json, sizeof(json), used, object.name);
#if BW21CAM_ENABLE_COLOR_DETECTION
            used = appendText(json, sizeof(json), used, ",\"score\":%u,\"color\":",
                              object.score);
            used = appendJsonString(json, sizeof(json), used, object.color);
            used = appendText(
                json, sizeof(json), used,
                ",\"color_confidence\":%u",
                object.colorConfidence);
#else
            used = appendText(json, sizeof(json), used, ",\"score\":%u",
                              object.score);
#endif
            used = appendText(json, sizeof(json), used,
                              ",\"box\":[%.3f,%.3f,%.3f,%.3f]}",
                              object.xMin, object.yMin, object.xMax, object.yMax);
        }
    }
    appendText(json, sizeof(json), used, "]}");
    json[sizeof(json) - 1] = 0;
    sendResponse(client, 200, "OK", "application/json", json);
}

void handleControlClient(DriverClient& client)
{
    char request[192];
    if (!readHttpRequest(client, request, sizeof(request))) {
        Serial.println("HTTP request headers timed out");
        return;
    }
    Serial.print("HTTP request: ");
    Serial.println(request);
    if (requestTargets(request, "/api/status")) {
        sendStatus(client);
    } else if (requestTargets(request, "/api/vision")) {
        sendVision(client);
    } else if (requestTargets(request, "/api/qr-frame.jpg")) {
        sendQrFrame(client);
    } else if (requestTargets(request, "/api/vision-mode")) {
        const char* enabled = strstr(request, "?enabled=");
        const bool valid = enabled && (enabled[9] == '0' || enabled[9] == '1') &&
                           (enabled[10] == ' ' || enabled[10] == '&');
        if (!valid) {
            sendResponse(client, 400, "Bad Request", "application/json",
                         "{\"accepted\":false}");
        } else if (!OnDeviceVision::setEnabled(enabled[9] == '1')) {
            sendResponse(client, 503, "Service Unavailable", "application/json",
                         "{\"accepted\":false,\"reason\":\"vision_unavailable\"}");
        } else {
            sendResponse(client, 200, "OK", "application/json",
                         "{\"accepted\":true}");
        }
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
}

void serviceControlClients()
{
    for (uint8_t handled = 0; handled < MAX_CONTROL_CLIENTS_PER_PASS; handled++) {
        const int socket = acceptSocket(controlDriver, controlListenSocket, "Control");
        if (socket < 0) break;
        DriverClient client(controlDriver, socket);
        handleControlClient(client);
    }
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
    activeProfile = requested;

    Serial.print("Video profile: ");
    Serial.print(profile.name);
    Serial.print(" fps=");
    Serial.print(profile.fps);
    Serial.print(" jpeg_quality=");
    Serial.println(profile.jpegQuality);
}

bool sendStreamHeader(DriverClient& client)
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

bool captureCameraFrame(uint32_t& imageAddress, uint32_t& imageLength,
                        uint32_t& frameStartedMs)
{
    frameStartedMs = millis();
    Camera.getImage(CAMERA_CHANNEL, &imageAddress, &imageLength);
    const uint32_t captureDurationMs = millis() - frameStartedMs;
    if (captureDurationMs > maxCaptureMs) maxCaptureMs = captureDurationMs;
    if (!imageAddress || !imageLength) {
        captureErrorCount++;
        return false;
    }

    cameraFrameCount++;
    const uint32_t capturedMs = millis();
    if (lastCameraFrameMs) {
        const uint32_t frameGapMs = capturedMs - lastCameraFrameMs;
        if (frameGapMs > maxFrameGapMs) maxFrameGapMs = frameGapMs;
        if (frameGapMs > FRAME_STALL_THRESHOLD_MS) streamStallCount++;
    }
    lastCameraFrameMs = capturedMs;
    return true;
}

void finishFrameInterval(uint32_t frameStartedMs)
{
    const uint32_t frameIntervalMs = 1000U / PROFILES[activeProfile].fps;
    const uint32_t elapsedMs = millis() - frameStartedMs;
    if (elapsedMs < frameIntervalMs) {
        const uint32_t waitMs = frameIntervalMs - elapsedMs;
        TickType_t waitTicks = waitMs / portTICK_PERIOD_MS;
        if (!waitTicks) waitTicks = 1;
        vTaskDelay(waitTicks);
    }
}

void handleStreamClient(DriverClient& client)
{
    char request[160];
    if (!readHttpRequest(client, request, sizeof(request))) {
        streamRejectCount++;
        Serial.println("Stream request headers timed out");
        return;
    }
    if (!requestTargets(request, "/stream")) {
        streamRejectCount++;
        Serial.print("Rejected stream request: ");
        Serial.println(request);
        sendResponse(client, 404, "Not Found", "text/plain", "Use /stream");
        return;
    }

    streamClientCount++;
    Serial.println("Camera viewer connected");
    if (!sendStreamHeader(client)) {
        streamErrorCount++;
        return;
    }

    while (client.connected()) {
        applyPendingProfile();
        serviceControlClients();
        uint32_t imageAddress = 0;
        uint32_t imageLength = 0;
        uint32_t frameStartedMs = 0;
        if (!captureCameraFrame(imageAddress, imageLength, frameStartedMs)) {
            vTaskDelay(2 / portTICK_PERIOD_MS);
            continue;
        }

        char partHeader[128];
        snprintf(partHeader, sizeof(partHeader),
                 "Content-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n",
                 static_cast<unsigned long>(imageLength));
        const uint32_t sendStartedMs = millis();
        const bool frameSent =
            writeText(client, partHeader) &&
            writeAll(client, reinterpret_cast<const uint8_t*>(imageAddress), imageLength) &&
            writeText(client, "\r\n--bw21frame\r\n");
        const uint32_t sendDurationMs = millis() - sendStartedMs;
        if (sendDurationMs > maxSendMs) maxSendMs = sendDurationMs;
        if (!frameSent) {
            streamErrorCount++;
            break;
        }
        frameCount++;
        finishFrameInterval(frameStartedMs);
    }
    Serial.println("Camera viewer disconnected");
}

void networkTask(void*)
{
    for (;;) {
        applyPendingProfile();
        serviceControlClients();
        const int socket = acceptSocket(streamDriver, streamListenSocket, "Stream");
        if (socket < 0) {
            vTaskDelay(1);
            continue;
        }
        DriverClient client(streamDriver, socket);
        handleStreamClient(client);
    }
}

void startWifi()
{
    int status = WL_IDLE_STATUS;
#if BW21CAM_USE_ACCESS_POINT
    char* selectedChannel = apChannel;
    bool tried5G = strcmp(apChannel, apFallbackChannel) != 0;
    Serial.print("Starting access point: ");
    Serial.print(apSsid);
    Serial.print(" channel=");
    Serial.println(selectedChannel);
    while (status != WL_CONNECTED) {
        status = WiFi.apbegin(apSsid, apPassword, selectedChannel, 0);
        if (status != WL_CONNECTED) {
            if (tried5G) {
                tried5G = false;
                selectedChannel = apFallbackChannel;
                Serial.println("5 GHz AP failed; falling back to 2.4 GHz channel 6");
            } else {
                Serial.println("Access point start failed; retrying");
            }
            delay(3000);
        }
    }
    uint8_t activeChannel = 0;
    wifi_get_channel(&activeChannel);
    Serial.print("Access point active on ");
    Serial.print(activeChannel > 14 ? "5 GHz" : "2.4 GHz");
    Serial.print(" channel ");
    Serial.println(activeChannel);
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
    OnDeviceVision::configureCamera();
    Camera.videoInit();
#if BW21CAM_ENABLE_LENS_DISTORTION_CORRECTION
    cameraSettings.setLDC(1);
    cameraSettings.getLDC();
#endif
    Camera.channelBegin(CAMERA_CHANNEL);
    if (!OnDeviceVision::begin()) {
        Serial.println("On-device vision initialization failed");
    }
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
    Serial.print(" camera_frames=");
    Serial.print(cameraFrameCount);
    Serial.print(" capture_errors=");
    Serial.print(captureErrorCount);
    Serial.print(" stream_errors=");
    Serial.print(streamErrorCount);
    Serial.print(" stream_clients=");
    Serial.print(streamClientCount);
    Serial.print(" stream_rejects=");
    Serial.print(streamRejectCount);
    Serial.print(" stalls=");
    Serial.print(streamStallCount);
    Serial.print(" max_capture_ms=");
    Serial.print(maxCaptureMs);
    Serial.print(" max_send_ms=");
    Serial.print(maxSendMs);
    Serial.print(" max_gap_ms=");
    Serial.print(maxFrameGapMs);
#if BW21CAM_ENABLE_ONDEVICE_VISION
    OnDeviceVision::Status vision = {};
    OnDeviceVision::getStatus(vision);
    const uint32_t qrAccountedFrames = vision.qrCheckedFrames + vision.jpegDecodeErrors;
    const uint32_t qrBypassedFrames = vision.analyzedFrames > qrAccountedFrames
                                         ? vision.analyzedFrames - qrAccountedFrames
                                         : 0;
    Serial.print(" vision=");
    Serial.print(vision.enabled ? "ON" : "OFF");
    Serial.print(" vision_frames=");
    Serial.print(vision.analyzedFrames);
    Serial.print(" qr_checked=");
    Serial.print(vision.qrCheckedFrames);
    Serial.print(" qr_bypassed=");
    Serial.print(qrBypassedFrames);
    Serial.print(" vision_ms=");
    Serial.print(vision.lastProcessMs);
    Serial.print(" jpeg_ms=");
    Serial.print(vision.lastJpegDecodeMs);
    Serial.print(" qr_ms=");
    Serial.print(vision.lastQrScanMs);
    Serial.print(" qr_decodes=");
    Serial.print(vision.qrDecodes);
    Serial.print(" quirc=");
    Serial.print(vision.quircScanPasses);
    Serial.print(" fast=");
    Serial.print(vision.quircFastScans);
    Serial.print(" full=");
    Serial.print(vision.quircFullScans);
    Serial.print('/');
    Serial.print(vision.quircCandidates);
    Serial.print('/');
    Serial.print(vision.quircDecodeErrors);
    Serial.print(" mirrored=");
    Serial.print(vision.quircMirroredDecodes);
    Serial.print(" zbar_fb=");
    Serial.print(vision.zbarFallbackScans);
    Serial.print(" fisheye=");
    Serial.print(vision.qrFisheyeScans);
    Serial.print('/');
    Serial.print(vision.qrFisheyeDecodes);
    Serial.print('/');
    Serial.print(vision.qrFisheyeProfile);
    Serial.print(" qr_self_test=");
    Serial.print(vision.qrSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" quirc_self=");
    Serial.print(vision.quircSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" fast_self=");
    Serial.print(vision.quircFastSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" full_self=");
    Serial.print(vision.quircFullSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" zbar_self=");
    Serial.print(vision.zbarSelfTestPassed ? "PASS" : "FAIL");
    Serial.print(" qr_detail=");
    Serial.print(static_cast<int>(vision.qrLastScanDetail));
    Serial.print(" qr_luma=");
    Serial.print(vision.qrDarkest);
    Serial.print('/');
    Serial.print(vision.qrMean);
    Serial.print('/');
    Serial.print(vision.qrBrightest);
    Serial.print(" objects=");
    Serial.print(vision.objectCount);
    Serial.print(" person_frames=");
    Serial.print(vision.yoloFrames);
    Serial.print(" face_frames=");
    Serial.print(vision.faceFrames);
#endif
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

void printVisionEvents()
{
#if BW21CAM_ENABLE_ONDEVICE_VISION
    static uint32_t lastCheckMs = 0;
    static uint32_t lastQrSequence = 0;
    static uint32_t lastObjectReportMs = 0;
    static bool hadObjects = false;
    const uint32_t now = millis();
    if (now - lastCheckMs < 100) return;
    lastCheckMs = now;

    OnDeviceVision::Status vision = {};
    OnDeviceVision::getStatus(vision);
    if (vision.qrSequence != lastQrSequence && vision.qrPayload[0]) {
        lastQrSequence = vision.qrSequence;
        Serial.print("VISION QR payload=");
        Serial.print(vision.qrPayload);
        Serial.print(" geometry=");
        if (vision.qrGeometry.valid) {
            Serial.print(vision.qrGeometry.centerXPermille);
            Serial.print(',');
            Serial.print(vision.qrGeometry.centerYPermille);
            Serial.print(" side=");
            Serial.print(vision.qrGeometry.sidePermille);
            Serial.print(" rot_cdeg=");
            Serial.println(vision.qrGeometry.rotationCdeg);
        } else {
            Serial.println("unavailable");
        }
    }

    if (!vision.objectCount) {
        if (hadObjects) Serial.println("VISION objects=none");
        hadObjects = false;
        return;
    }
    if (now - lastObjectReportMs < 1000) return;

    lastObjectReportMs = now;
    hadObjects = true;
    Serial.print("VISION objects=");
    Serial.print(vision.objectCount);
    for (uint8_t i = 0; i < vision.objectCount; i++) {
        const OnDeviceVision::ObjectResult& object = vision.objects[i];
        Serial.print(" | ");
        Serial.print(object.name);
        Serial.print(" score=");
        Serial.print(object.score);
#if BW21CAM_ENABLE_COLOR_DETECTION
        Serial.print(" color=");
        Serial.print(object.color);
        Serial.print(" color_conf=");
        Serial.print(object.colorConfidence);
#endif
        Serial.print(" box=");
        Serial.print(object.xMin, 3);
        Serial.print(',');
        Serial.print(object.yMin, 3);
        Serial.print(',');
        Serial.print(object.xMax, 3);
        Serial.print(',');
        Serial.print(object.yMax, 3);
    }
    Serial.println();
#endif
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.print("BW21-CBV Cam Drone ");
    Serial.println(BW21CAM_VERSION);
    Serial.print("Build variant: ");
    Serial.println(BW21CAM_BUILD_VARIANT);

    startWifi();
    startCamera();
    FcLink::begin();

    controlListenSocket = createListenSocket(controlDriver, BW21CAM_CONTROL_PORT);
    streamListenSocket = createListenSocket(streamDriver, BW21CAM_STREAM_PORT);
    if (controlListenSocket < 0 || streamListenSocket < 0) {
        Serial.print("Listen socket start failed control=");
        Serial.print(controlListenSocket);
        Serial.print(" stream=");
        Serial.println(streamListenSocket);
    } else {
        Serial.print("HTTP listeners ready control_fd=");
        Serial.print(controlListenSocket);
        Serial.print(" stream_fd=");
        Serial.println(streamListenSocket);
        const BaseType_t networkStarted =
            xTaskCreate(networkTask, "CameraNetwork", 14 * 1024, nullptr, 2, nullptr);
        if (networkStarted != pdPASS) {
            Serial.print("Network task start failed status=");
            Serial.println(networkStarted);
        }
    }
    Serial.println("Camera test ready");
}

void loop()
{
    FcLink::update();

#if BW21CAM_ENABLE_ONDEVICE_VISION && BW21CAM_ENABLE_FC_LINK
    static uint32_t publishedQrSequence = 0;
    OnDeviceVision::Status vision = {};
    OnDeviceVision::getStatus(vision);
    const FcLink::QrObservation observation = {
        vision.qrPayload,
        vision.qrGeometry.valid,
        vision.qrGeometry.fullResolution,
        vision.qrGeometry.mirrored,
        vision.qrGeometry.zbarFallback,
        vision.qrGeometry.centerXPermille,
        vision.qrGeometry.centerYPermille,
        vision.qrGeometry.sidePermille,
        vision.qrGeometry.areaPermille,
        vision.qrGeometry.rotationCdeg
    };
    if (vision.qrSequence != publishedQrSequence && vision.qrPayload[0] &&
        FcLink::publishQr(observation)) {
        publishedQrSequence = vision.qrSequence;
    }
#endif

    printPeriodicStatus();
    printVisionEvents();
    delay(1);
}
