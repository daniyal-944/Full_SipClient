#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "usb_device_uac.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "mbedtls/md5.h"

static const char *TAG = "UAC2-SIP-HEADSET";

// Configuration defaults - fallback values if not defined in sdkconfig
#ifndef CONFIG_UAC_AUDIO_CHANNELS
#define CONFIG_UAC_AUDIO_CHANNELS 1
#endif

#ifndef CONFIG_UAC_BITS_PER_SAMPLE
#define CONFIG_UAC_BITS_PER_SAMPLE 16
#endif

#ifndef CONFIG_UAC_DEVICE_NAME
#define CONFIG_UAC_DEVICE_NAME "ESP32-S3 SIP Headset"
#endif

#ifndef CONFIG_UAC_MANUFACTURER_NAME
#define CONFIG_UAC_MANUFACTURER_NAME "ESP32 Audio Systems"
#endif

#ifndef CONFIG_UAC_SAMPLE_RATE
#define CONFIG_UAC_SAMPLE_RATE 48000
#endif

// Audio configuration
#define SAMPLE_RATE CONFIG_UAC_SAMPLE_RATE
#define CHANNELS CONFIG_UAC_AUDIO_CHANNELS
#define BITS_PER_SAMPLE CONFIG_UAC_BITS_PER_SAMPLE
#define DMA_BUF_COUNT 24
#define DMA_BUF_LEN 256

// Wi-Fi Configuration
#define WIFI_SSID "ayounas"
#define WIFI_PASSWORD "asdf1234"
#define WIFI_RETRY_MAX 10

// SIP Configuration
#define SIP_SERVER_HOST "78.46.21.240"  
#define SIP_USER        "1001"           
#define SIP_PASSWORD    "3569test@@"    
#define SIP_SERVER_PORT 35060

// #define SIP_SERVER_HOST "sip.linphone.org"  // Free test server
// #define SIP_SERVER_PORT 5060                // Standard SIP port
// #define SIP_USER        "test"              // Test username
// #define SIP_PASSWORD    "test"              // Test password


// RTP Configuration
#define RTP_PORT_BASE 20000
#define RTP_MAX_PACKET_SIZE 1500
#define RTP_AUDIO_BUFFER_SIZE 960 
#define RTP_SEND_INTERVAL_MS 20   
#define RTP_PAYLOAD_TYPE_PCMU 0   // G.711 μ-law
#define RTP_PAYLOAD_TYPE_OPUS 96  // Dynamic payload type for Opus

// Hardware pins - using fallback defaults if not defined
#ifdef CONFIG_UAC_ENABLE_I2S_SPK
#ifndef CONFIG_UAC_SPK_GPIO_BCLK
#define CONFIG_UAC_SPK_GPIO_BCLK 2
#endif
#ifndef CONFIG_UAC_SPK_GPIO_WS
#define CONFIG_UAC_SPK_GPIO_WS 3
#endif
#ifndef CONFIG_UAC_SPK_GPIO_DOUT
#define CONFIG_UAC_SPK_GPIO_DOUT 4
#endif
#define I2S_SPK_BCLK CONFIG_UAC_SPK_GPIO_BCLK
#define I2S_SPK_WS CONFIG_UAC_SPK_GPIO_WS
#define I2S_SPK_DOUT CONFIG_UAC_SPK_GPIO_DOUT
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
#ifndef CONFIG_UAC_MIC_GPIO_SCK
#define CONFIG_UAC_MIC_GPIO_SCK 5
#endif
#ifndef CONFIG_UAC_MIC_GPIO_WS
#define CONFIG_UAC_MIC_GPIO_WS 6
#endif
#ifndef CONFIG_UAC_MIC_GPIO_SD
#define CONFIG_UAC_MIC_GPIO_SD 7
#endif
#define I2S_MIC_BCLK CONFIG_UAC_MIC_GPIO_SCK
#define I2S_MIC_WS CONFIG_UAC_MIC_GPIO_WS
#define I2S_MIC_DIN CONFIG_UAC_MIC_GPIO_SD
#endif

// Mute button configuration
#define MUTE_BUTTON_GPIO 10
#define MUTE_BUTTON_ACTIVE_LOW 1
#define DEBOUNCE_TIME_MS 50
#define LONG_PRESS_TIME_MS 2000

// Volume boost configuration
#ifdef CONFIG_UAC_ENABLE_VOLUME_BOOST
#define VOLUME_BOOST_PERCENT CONFIG_UAC_VOLUME_BOOST_PERCENT
#else
#define VOLUME_BOOST_PERCENT 100
#endif

#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
#define BUFFER_RESET_INTERVAL_MS (CONFIG_UAC_BUFFER_RESET_INTERVAL_MIN * 60 * 1000)
#else
#define BUFFER_RESET_INTERVAL_MS 0
#endif

#define STARTUP_FADE_CALLBACKS 20

// ============================================================================
// RTP STRUCTURES AND DEFINITIONS
// ============================================================================

// RTP Header Structure (RFC 3550)
typedef struct __attribute__((packed))
{
    uint8_t version : 2;   // Version (always 2)
    uint8_t padding : 1;   // Padding flag
    uint8_t extension : 1; // Extension flag
    uint8_t cc : 4;        // CSRC count
    uint8_t marker : 1;    // Marker bit
    uint8_t pt : 7;        // Payload type
    uint16_t sequence;     // Sequence number
    uint32_t timestamp;    // Timestamp
    uint32_t ssrc;         // Synchronization source identifier
} rtp_header_t;

// RTP Session State
typedef struct
{
    int send_socket;                // Socket for sending RTP packets
    int recv_socket;                // Socket for receiving RTP packets
    uint16_t send_sequence;         // Outgoing sequence number
    uint16_t recv_sequence;         // Expected incoming sequence number
    uint32_t send_timestamp;        // Outgoing timestamp
    uint32_t recv_timestamp;        // Last received timestamp
    uint32_t send_ssrc;             // Our SSRC identifier
    uint32_t recv_ssrc;             // Remote SSRC identifier
    struct sockaddr_in remote_addr; // Remote RTP endpoint
    bool session_active;            // RTP session status
    uint8_t payload_type;           // Audio codec payload type

    // Audio buffers
    int16_t send_buffer[RTP_AUDIO_BUFFER_SIZE];
    int16_t recv_buffer[RTP_AUDIO_BUFFER_SIZE];
    size_t send_buffer_pos;
    size_t recv_buffer_len;

    // Statistics
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t bytes_sent;
    uint32_t bytes_received;
} rtp_session_t;

// ============================================================================
// SIP CLIENT STATE (Enhanced with RTP support)
// ============================================================================

typedef enum
{
    SIP_STATE_DISCONNECTED,
    SIP_STATE_REGISTERING,
    SIP_STATE_REGISTERED,
    SIP_STATE_CALLING,
    SIP_STATE_IN_CALL,
    SIP_STATE_ERROR
} sip_state_t;

typedef struct
{
    sip_state_t state;
    bool is_connected;
    bool call_active;
    uint32_t last_keepalive;
    uint32_t registration_retry_count;
    int socket_fd;
    char call_id[64];
    char from_tag[32];
    char to_tag[32]; // Added for call state
    uint32_t cseq;

    // RTP session
    rtp_session_t rtp_session;
    bool rtp_ready;
    uint16_t rtp_local_port;
    uint16_t rtp_remote_port;
    char remote_ip[16];
} sip_client_t;

// Wi-Fi State
typedef struct
{
    bool connected;
    int retry_count;
    esp_ip4_addr_t ip;
} wifi_state_t;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Audio Handles
#ifdef CONFIG_UAC_ENABLE_I2S_SPK
static i2s_chan_handle_t i2s_spk_handle = NULL;
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
static i2s_chan_handle_t i2s_mic_handle = NULL;
#endif

// Mute button state
typedef enum
{
    BUTTON_STATE_RELEASED,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_LONG_PRESS
} button_state_t;

typedef struct
{
    bool is_muted;
    bool button_pressed;
    uint32_t press_start_time;
    uint32_t last_toggle_time;
    button_state_t state;
} mute_control_t;

static mute_control_t mute_ctrl = {
    .is_muted = false,
    .button_pressed = false,
    .press_start_time = 0,
    .last_toggle_time = 0,
    .state = BUTTON_STATE_RELEASED};

// Global state variables
static sip_client_t sip_client = {0};
static wifi_state_t wifi_state = {0};
static bool initial_test_done = false;

// Statistics
static uint32_t mic_callback_count = 0;
static uint32_t spk_callback_count = 0;
static uint32_t mute_toggle_count = 0;

#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
static uint32_t last_buffer_reset = 0;
static uint32_t consecutive_errors = 0;
#endif

// Audio buffers - only declare if microphone is enabled
#ifdef CONFIG_UAC_ENABLE_I2S_MIC
static int16_t mic_buffer[SAMPLE_RATE / 100];
#endif

// RTP Timer for periodic packet sending
static TimerHandle_t rtp_send_timer = NULL;
static QueueHandle_t usb_to_sip_queue = NULL;
static QueueHandle_t sip_to_usb_queue = NULL;

#define AUDIO_QUEUE_SIZE 10
#define AUDIO_FRAME_SIZE 480 // 10ms at 48kHz mono

typedef struct
{
    int16_t audio_data[AUDIO_FRAME_SIZE];
    size_t sample_count;
    uint32_t timestamp;
} audio_frame_t;

// ============================================================================
// RTP AUDIO IMPLEMENTATION
// ============================================================================

/**
 * @brief Initialize RTP session
 */
static esp_err_t rtp_session_init(rtp_session_t *session)
{
    if (!session)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing RTP session...");

    memset(session, 0, sizeof(rtp_session_t));

    // Generate random SSRC
    session->send_ssrc = esp_random();
    session->send_sequence = esp_random() & 0xFFFF;
    session->send_timestamp = esp_random();
    session->payload_type = RTP_PAYLOAD_TYPE_PCMU; // Start with G.711 μ-law

    // Create RTP send socket
    session->send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (session->send_socket < 0)
    {
        ESP_LOGE(TAG, "Failed to create RTP send socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Create RTP receive socket
    session->recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (session->recv_socket < 0)
    {
        ESP_LOGE(TAG, "Failed to create RTP receive socket: errno %d", errno);
        close(session->send_socket);
        return ESP_FAIL;
    }

    // Bind receive socket to local RTP port
    struct sockaddr_in local_addr = {0};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(RTP_PORT_BASE);

    if (bind(session->recv_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        ESP_LOGE(TAG, "Failed to bind RTP receive socket: errno %d", errno);
        close(session->send_socket);
        close(session->recv_socket);
        return ESP_FAIL;
    }

    // Set receive socket to non-blocking
    int flags = fcntl(session->recv_socket, F_GETFL, 0);
    fcntl(session->recv_socket, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "RTP session initialized:");
    ESP_LOGI(TAG, "  SSRC: 0x%08" PRIx32, session->send_ssrc);
    ESP_LOGI(TAG, "  Local port: %d", RTP_PORT_BASE);
    ESP_LOGI(TAG, "  Payload type: %d (%s)", session->payload_type,
             session->payload_type == RTP_PAYLOAD_TYPE_PCMU ? "G.711 μ-law" : "Unknown");

    return ESP_OK;
}

/**
 * @brief Configure RTP session for active call
 */
static esp_err_t rtp_session_configure(rtp_session_t *session, const char *remote_ip, uint16_t remote_port)
{
    if (!session || !remote_ip)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Configuring RTP session for %s:%d", remote_ip, remote_port);

    // Configure remote endpoint
    session->remote_addr.sin_family = AF_INET;
    session->remote_addr.sin_port = htons(remote_port);
    if (inet_aton(remote_ip, &session->remote_addr.sin_addr) == 0)
    {
        ESP_LOGE(TAG, "Invalid remote IP address: %s", remote_ip);
        return ESP_ERR_INVALID_ARG;
    }

    session->session_active = true;
    session->recv_buffer_len = 0;
    session->send_buffer_pos = 0;

    ESP_LOGI(TAG, "RTP session configured for active call");
    return ESP_OK;
}

/**
 * @brief Convert 16-bit linear PCM to G.711 μ-law
 */
static uint8_t linear_to_ulaw(int16_t sample)
{
    const int16_t BIAS = 0x84;
    const int16_t CLIP = 32635;

    static const uint8_t exp_lut[256] = {
        0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};

    int sign = (sample >> 8) & 0x80;
    if (sign != 0)
        sample = -sample;
    if (sample > CLIP)
        sample = CLIP;

    sample = sample + BIAS;
    uint8_t exponent = exp_lut[(sample >> 7) & 0xFF];
    uint8_t mantissa = (sample >> (exponent + 3)) & 0x0F;
    uint8_t ulaw = ~(sign | (exponent << 4) | mantissa);

    return ulaw;
}

/**
 * @brief Convert G.711 μ-law to 16-bit linear PCM
 */
static int16_t ulaw_to_linear(uint8_t ulaw)
{
    static const int16_t ulaw_lut[256] = {
        -32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956,
        -23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
        -15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412,
        -11900, -11388, -10876, -10364, -9852, -9340, -8828, -8316,
        -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140,
        -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092,
        -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004,
        -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
        -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436,
        -1372, -1308, -1244, -1180, -1116, -1052, -988, -924,
        -876, -844, -812, -780, -748, -716, -684, -652,
        -620, -588, -556, -524, -492, -460, -428, -396,
        -372, -356, -340, -324, -308, -292, -276, -260,
        -244, -228, -212, -196, -180, -164, -148, -132,
        -120, -112, -104, -96, -88, -80, -72, -64,
        -56, -48, -40, -32, -24, -16, -8, 0,
        32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956,
        23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764,
        15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412,
        11900, 11388, 10876, 10364, 9852, 9340, 8828, 8316,
        7932, 7676, 7420, 7164, 6908, 6652, 6396, 6140,
        5884, 5628, 5372, 5116, 4860, 4604, 4348, 4092,
        3900, 3772, 3644, 3516, 3388, 3260, 3132, 3004,
        2876, 2748, 2620, 2492, 2364, 2236, 2108, 1980,
        1884, 1820, 1756, 1692, 1628, 1564, 1500, 1436,
        1372, 1308, 1244, 1180, 1116, 1052, 988, 924,
        876, 844, 812, 780, 748, 716, 684, 652,
        620, 588, 556, 524, 492, 460, 428, 396,
        372, 356, 340, 324, 308, 292, 276, 260,
        244, 228, 212, 196, 180, 164, 148, 132,
        120, 112, 104, 96, 88, 80, 72, 64,
        56, 48, 40, 32, 24, 16, 8, 0};

    return ulaw_lut[ulaw];
}

/**
 * @brief Send RTP audio packet
 */
static esp_err_t rtp_send_audio(int16_t *audio_data, size_t num_samples)
{
    if (!sip_client.rtp_session.session_active || !audio_data || num_samples == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    rtp_session_t *session = &sip_client.rtp_session;

    // Calculate packet size based on codec
    size_t payload_size;
    uint8_t packet_buffer[RTP_MAX_PACKET_SIZE];
    rtp_header_t *rtp_header = (rtp_header_t *)packet_buffer;
    uint8_t *payload = packet_buffer + sizeof(rtp_header_t);

    // Fill RTP header
    memset(rtp_header, 0, sizeof(rtp_header_t));
    rtp_header->version = 2;
    rtp_header->pt = session->payload_type;
    rtp_header->sequence = htons(session->send_sequence++);
    rtp_header->timestamp = htonl(session->send_timestamp);
    rtp_header->ssrc = htonl(session->send_ssrc);

    // Encode audio based on payload type
    if (session->payload_type == RTP_PAYLOAD_TYPE_PCMU)
    {
        // G.711 μ-law encoding
        payload_size = num_samples;
        for (size_t i = 0; i < num_samples && i < (RTP_MAX_PACKET_SIZE - sizeof(rtp_header_t)); i++)
        {
            payload[i] = linear_to_ulaw(audio_data[i]);
        }
    }
    else
    {
        // For now, fallback to G.711 if other codecs not implemented
        ESP_LOGW(TAG, "Unsupported payload type %d, using G.711 μ-law", session->payload_type);
        payload_size = num_samples;
        for (size_t i = 0; i < num_samples && i < (RTP_MAX_PACKET_SIZE - sizeof(rtp_header_t)); i++)
        {
            payload[i] = linear_to_ulaw(audio_data[i]);
        }
    }

    // Send RTP packet
    size_t packet_size = sizeof(rtp_header_t) + payload_size;
    int sent = sendto(session->send_socket, packet_buffer, packet_size, 0,
                      (struct sockaddr *)&session->remote_addr, sizeof(session->remote_addr));

    if (sent < 0)
    {
        ESP_LOGW(TAG, "RTP send failed: errno %d", errno);
        return ESP_FAIL;
    }

    // Update statistics and timestamp
    session->packets_sent++;
    session->bytes_sent += sent;
    session->send_timestamp += num_samples; // Increment by sample count

    // Log periodically
    if (session->packets_sent % 100 == 0)
    {
        ESP_LOGI(TAG, "RTP: Sent %" PRIu32 " packets, %" PRIu32 " bytes",
                 session->packets_sent, session->bytes_sent);
    }

    return ESP_OK;
}

/**
 * @brief Receive RTP audio packet
 */
static esp_err_t rtp_receive_audio(int16_t *audio_buffer, size_t *buffer_len)
{
    if (!sip_client.rtp_session.session_active || !audio_buffer || !buffer_len)
    {
        return ESP_ERR_INVALID_ARG;
    }

    rtp_session_t *session = &sip_client.rtp_session;
    uint8_t packet_buffer[RTP_MAX_PACKET_SIZE];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    // Try to receive RTP packet (non-blocking)
    int received = recvfrom(session->recv_socket, packet_buffer, sizeof(packet_buffer), 0,
                            (struct sockaddr *)&from_addr, &from_len);

    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // No data available - fill buffer with silence
            memset(audio_buffer, 0, *buffer_len * sizeof(int16_t));
            return ESP_OK;
        }
        ESP_LOGW(TAG, "RTP receive failed: errno %d", errno);
        return ESP_FAIL;
    }

    if (received < sizeof(rtp_header_t))
    {
        ESP_LOGW(TAG, "RTP packet too small: %d bytes", received);
        return ESP_FAIL;
    }

    // Parse RTP header
    rtp_header_t *rtp_header = (rtp_header_t *)packet_buffer;
    uint8_t *payload = packet_buffer + sizeof(rtp_header_t);
    size_t payload_size = received - sizeof(rtp_header_t);

    // Validate RTP header
    if (rtp_header->version != 2)
    {
        ESP_LOGW(TAG, "Invalid RTP version: %d", rtp_header->version);
        return ESP_FAIL;
    }

    // Update session info on first packet
    if (session->recv_ssrc == 0)
    {
        session->recv_ssrc = ntohl(rtp_header->ssrc);
        ESP_LOGI(TAG, "RTP: Remote SSRC: 0x%08" PRIx32, session->recv_ssrc);
    }

    // Check sequence number for packet loss
    uint16_t sequence = ntohs(rtp_header->sequence);
    if (session->packets_received > 0)
    {
        uint16_t expected = session->recv_sequence + 1;
        if (sequence != expected)
        {
            session->packets_lost++;
            ESP_LOGD(TAG, "RTP: Packet loss detected. Expected: %d, Got: %d", expected, sequence);
        }
    }
    session->recv_sequence = sequence;

    // Decode audio based on payload type
    size_t samples_decoded = 0;
    uint8_t payload_type = rtp_header->pt;

    if (payload_type == RTP_PAYLOAD_TYPE_PCMU)
    {
        // G.711 μ-law decoding
        samples_decoded = payload_size;
        for (size_t i = 0; i < payload_size && i < *buffer_len; i++)
        {
            audio_buffer[i] = ulaw_to_linear(payload[i]);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Unsupported RTP payload type: %d", payload_type);
        // Fill with silence for unsupported codecs
        memset(audio_buffer, 0, *buffer_len * sizeof(int16_t));
        samples_decoded = *buffer_len;
    }

    // Update statistics
    session->packets_received++;
    session->bytes_received += received;
    session->recv_timestamp = ntohl(rtp_header->timestamp);

    *buffer_len = samples_decoded;

    // Log periodically
    if (session->packets_received % 100 == 0)
    {
        ESP_LOGI(TAG, "RTP: Received %" PRIu32 " packets, %" PRIu32 " bytes, %" PRIu32 " lost",
                 session->packets_received, session->bytes_received, session->packets_lost);
    }

    return ESP_OK;
}

/**
 * @brief RTP periodic send timer callback
 */
static void rtp_send_timer_callback(TimerHandle_t xTimer)
{
    
    // The actual audio data will come from the UAC microphone callback

    if (sip_client.rtp_session.session_active && sip_client.call_active)
    {
        // If we have buffered audio data, send it
        rtp_session_t *session = &sip_client.rtp_session;
        if (session->send_buffer_pos > 0)
        {
            rtp_send_audio(session->send_buffer, session->send_buffer_pos);
            session->send_buffer_pos = 0; // Reset buffer
        }
    }
}

/**
 * @brief Start RTP session for active call
 */
static esp_err_t rtp_start_session(const char *remote_ip, uint16_t remote_port)
{
    ESP_LOGI(TAG, "Starting RTP session to %s:%d", remote_ip, remote_port);

    esp_err_t ret = rtp_session_configure(&sip_client.rtp_session, remote_ip, remote_port);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Start RTP send timer
    if (rtp_send_timer == NULL)
    {
        rtp_send_timer = xTimerCreate("rtp_send",
                                      pdMS_TO_TICKS(RTP_SEND_INTERVAL_MS),
                                      pdTRUE, // Auto-reload
                                      NULL,   // Timer ID
                                      rtp_send_timer_callback);
        if (rtp_send_timer == NULL)
        {
            ESP_LOGE(TAG, "Failed to create RTP send timer");
            return ESP_FAIL;
        }
    }

    if (xTimerStart(rtp_send_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start RTP send timer");
        return ESP_FAIL;
    }

    sip_client.rtp_ready = true;
    ESP_LOGI(TAG, "RTP session started successfully");
    return ESP_OK;
}

/**
 * @brief Stop RTP session
 */
static void rtp_stop_session(void)
{
    ESP_LOGI(TAG, "Stopping RTP session");

    sip_client.rtp_session.session_active = false;
    sip_client.rtp_ready = false;

    if (rtp_send_timer)
    {
        xTimerStop(rtp_send_timer, 0);
    }

    // Reset session state
    sip_client.rtp_session.send_buffer_pos = 0;
    sip_client.rtp_session.recv_buffer_len = 0;

    ESP_LOGI(TAG, "RTP session stopped");
}

/**
 * @brief Cleanup RTP session
 */
__attribute__((unused)) static void rtp_session_cleanup(rtp_session_t *session)
{
    if (!session)
        return;

    ESP_LOGI(TAG, "Cleaning up RTP session");

    if (session->send_socket >= 0)
    {
        close(session->send_socket);
        session->send_socket = -1;
    }

    if (session->recv_socket >= 0)
    {
        close(session->recv_socket);
        session->recv_socket = -1;
    }

    session->session_active = false;

    ESP_LOGI(TAG, "RTP session cleanup completed");
}

static void calculate_md5(const char *input, char *output)
{
    unsigned char hash[16];
    mbedtls_md5_context ctx;

    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);
    mbedtls_md5_update(&ctx, (const unsigned char *)input, strlen(input));
    mbedtls_md5_finish(&ctx, hash);
    mbedtls_md5_free(&ctx);

    // Convert to hex string
    for (int i = 0; i < 16; i++)
    {
        sprintf(output + (i * 2), "%02x", hash[i]);
    }
    output[32] = '\0';
}

/**
 * @brief Parse WWW-Authenticate header and generate response
 */
static esp_err_t parse_auth_header(const char *auth_header, char *response_hash)
{
    char realm[128] = {0};
    char nonce[128] = {0};

    // Parse realm
    const char *realm_start = strstr(auth_header, "realm=\"");
    if (realm_start)
    {
        realm_start += 7; // Skip 'realm="'
        const char *realm_end = strchr(realm_start, '"');
        if (realm_end)
        {
            size_t realm_len = realm_end - realm_start;
            if (realm_len < sizeof(realm))
            {
                strncpy(realm, realm_start, realm_len);
            }
        }
    }

    // Parse nonce
    const char *nonce_start = strstr(auth_header, "nonce=\"");
    if (nonce_start)
    {
        nonce_start += 7; // Skip 'nonce="'
        const char *nonce_end = strchr(nonce_start, '"');
        if (nonce_end)
        {
            size_t nonce_len = nonce_end - nonce_start;
            if (nonce_len < sizeof(nonce))
            {
                strncpy(nonce, nonce_start, nonce_len);
            }
        }
    }

    if (strlen(realm) == 0 || strlen(nonce) == 0)
    {
        ESP_LOGE(TAG, "Failed to parse authentication header");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SIP Auth - Realm: %s, Nonce: %s", realm, nonce);

    // Calculate response hash
    char ha1_input[256];
    char ha1[33];
    char ha2_input[256];
    char ha2[33];
    char response_input[256];

    // HA1 = MD5(username:realm:password)
    snprintf(ha1_input, sizeof(ha1_input), "%s:%s:%s", SIP_USER, realm, SIP_PASSWORD);
    calculate_md5(ha1_input, ha1);

    // HA2 = MD5(method:uri)
    snprintf(ha2_input, sizeof(ha2_input), "REGISTER:sip:%s:%d", SIP_SERVER_HOST, SIP_SERVER_PORT);
    calculate_md5(ha2_input, ha2);

    // Response = MD5(HA1:nonce:HA2)
    snprintf(response_input, sizeof(response_input), "%s:%s:%s", ha1, nonce, ha2);
    calculate_md5(response_input, response_hash);

    ESP_LOGI(TAG, "SIP Auth response calculated");
    return ESP_OK;
}

/**
 * @brief Send SIP REGISTER with authentication
 */
static esp_err_t sip_send_register_with_auth(const char *auth_header)
{
    if (!wifi_state.connected || sip_client.socket_fd < 0)
    {
        return ESP_FAIL;
    }

    char response_hash[33];
    char realm[128] = {0};
    char nonce[128] = {0};

    if (parse_auth_header(auth_header, response_hash) != ESP_OK)
    {
        return ESP_FAIL;
    }

    // Extract realm and nonce again
    const char *realm_start = strstr(auth_header, "realm=\"");
    if (realm_start)
    {
        realm_start += 7;
        const char *realm_end = strchr(realm_start, '"');
        if (realm_end)
        {
            size_t realm_len = realm_end - realm_start;
            if (realm_len < sizeof(realm))
            {
                strncpy(realm, realm_start, realm_len);
            }
        }
    }

    const char *nonce_start = strstr(auth_header, "nonce=\"");
    if (nonce_start)
    {
        nonce_start += 7;
        const char *nonce_end = strchr(nonce_start, '"');
        if (nonce_end)
        {
            size_t nonce_len = nonce_end - nonce_start;
            if (nonce_len < sizeof(nonce))
            {
                strncpy(nonce, nonce_start, nonce_len);
            }
        }
    }

    static char register_msg[1500];
    sip_client.cseq++;

    snprintf(register_msg, sizeof(register_msg),
             "REGISTER sip:%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP " IPSTR ":%d;branch=z9hG4bK%08" PRIx32 "\r\n"
             "Max-Forwards: 70\r\n"
             "From: <sip:%s@%s:%d>;tag=%s\r\n"
             "To: <sip:%s@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %" PRIu32 " REGISTER\r\n"
             "Contact: <sip:%s@" IPSTR ":%d>\r\n"
             "Authorization: Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", "
             "uri=\"sip:%s:%d\", response=\"%s\"\r\n"
             "Expires: 3600\r\n"
             "User-Agent: ESP32-S3-UAC-SIP/1.0\r\n"
             "Content-Length: 0\r\n"
             "\r\n",
             SIP_SERVER_HOST, SIP_SERVER_PORT,
             IP2STR(&wifi_state.ip), SIP_SERVER_PORT, esp_random(),
             SIP_USER, SIP_SERVER_HOST, SIP_SERVER_PORT, sip_client.from_tag,
             SIP_USER, SIP_SERVER_HOST, SIP_SERVER_PORT,
             sip_client.call_id,
             sip_client.cseq,
             SIP_USER, IP2STR(&wifi_state.ip), SIP_SERVER_PORT,
             SIP_USER, realm, nonce,
             SIP_SERVER_HOST, SIP_SERVER_PORT, response_hash);

    ESP_LOGI(TAG, "Sending SIP REGISTER with authentication");

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SIP_SERVER_HOST);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SIP_SERVER_PORT);

    int sent = sendto(sip_client.socket_fd, register_msg, strlen(register_msg), 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (sent < 0)
    {
        ESP_LOGE(TAG, "Failed to send SIP REGISTER with auth: errno %d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SIP REGISTER with auth sent (%d bytes)", sent);
    return ESP_OK;
}

// ============================================================================
// WIFI AND SIP FUNCTIONS (Updated for RTP integration)
// ============================================================================

// Forward declarations
static void wifi_init_sta(void);
static void sip_client_init(void);
static void sip_client_task(void *param);
static esp_err_t sip_send_register(void);

/**
 * @brief Wi-Fi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_state.connected = false;

        // Stop RTP session on Wi-Fi disconnect
        if (sip_client.rtp_ready)
        {
            rtp_stop_session();
        }

        if (wifi_state.retry_count < WIFI_RETRY_MAX)
        {
            esp_wifi_connect();
            wifi_state.retry_count++;
            ESP_LOGI(TAG, "Retry connecting to Wi-Fi (%d/%d)", wifi_state.retry_count, WIFI_RETRY_MAX);
        }
        else
        {
            ESP_LOGE(TAG, "Wi-Fi connection failed after %d retries", WIFI_RETRY_MAX);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        wifi_state.ip = event->ip_info.ip;
        wifi_state.connected = true;
        wifi_state.retry_count = 0;

        ESP_LOGI(TAG, "Wi-Fi connected! IP: " IPSTR, IP2STR(&wifi_state.ip));
        ESP_LOGI(TAG, "Starting SIP client...");

        // Initialize SIP client after Wi-Fi connection
        sip_client_init();
    }
}

/**
 * @brief Initialize Wi-Fi station mode
 */
static void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi station initialized. Connecting to %s...", WIFI_SSID);
}

/**
 * @brief Generate SIP Call-ID
 */
static void generate_call_id(char *call_id, size_t len)
{
    uint32_t random = esp_random();
    snprintf(call_id, len, "%08" PRIx32 "@" IPSTR, random, IP2STR(&wifi_state.ip));
}

/**
 * @brief Generate SIP tag
 */
static void generate_tag(char *tag, size_t len)
{
    uint32_t random = esp_random();
    snprintf(tag, len, "%08" PRIx32, random);
}

// Add this enhanced SIP debugging to your existing code
// Replace the existing sip_send_register() function with this version:

/**
 * @brief Enhanced SIP REGISTER with detailed debugging
 */
static esp_err_t sip_send_register(void)
{
    if (!wifi_state.connected || sip_client.socket_fd < 0)
    {
        ESP_LOGE(TAG, "SIP: Wi-Fi not connected or socket invalid");
        return ESP_FAIL;
    }

    static char register_msg[1024];
    generate_call_id(sip_client.call_id, sizeof(sip_client.call_id));
    generate_tag(sip_client.from_tag, sizeof(sip_client.from_tag));
    sip_client.cseq++;

    snprintf(register_msg, sizeof(register_msg),
             "REGISTER sip:%s:%d SIP/2.0\r\n"
             "Via: SIP/2.0/UDP " IPSTR ":%d;branch=z9hG4bK%08" PRIx32 "\r\n"
             "Max-Forwards: 70\r\n"
             "From: <sip:%s@%s:%d>;tag=%s\r\n"
             "To: <sip:%s@%s:%d>\r\n"
             "Call-ID: %s\r\n"
             "CSeq: %" PRIu32 " REGISTER\r\n"
             "Contact: <sip:%s@" IPSTR ":%d>\r\n"
             "Expires: 3600\r\n"
             "User-Agent: ESP32-S3-UAC-SIP/1.0\r\n"
             "Content-Length: 0\r\n"
             "\r\n",
             SIP_SERVER_HOST, SIP_SERVER_PORT,
             IP2STR(&wifi_state.ip), SIP_SERVER_PORT, esp_random(),
             SIP_USER, SIP_SERVER_HOST, SIP_SERVER_PORT, sip_client.from_tag,
             SIP_USER, SIP_SERVER_HOST, SIP_SERVER_PORT,
             sip_client.call_id,
             sip_client.cseq,
             SIP_USER, IP2STR(&wifi_state.ip), SIP_SERVER_PORT);

    ESP_LOGI(TAG, "🔍 SIP REGISTER DEBUG:");
    ESP_LOGI(TAG, "   Target: %s:%d", SIP_SERVER_HOST, SIP_SERVER_PORT);
    ESP_LOGI(TAG, "   Local IP: " IPSTR, IP2STR(&wifi_state.ip));
    ESP_LOGI(TAG, "   User: %s", SIP_USER);
    ESP_LOGI(TAG, "   Call-ID: %s", sip_client.call_id);
    ESP_LOGI(TAG, "   CSeq: %" PRIu32, sip_client.cseq);
    
    // Print the actual SIP message for debugging
    ESP_LOGI(TAG, "📤 SENDING SIP MESSAGE:");
    ESP_LOGI(TAG, "%s", register_msg);

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SIP_SERVER_HOST);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SIP_SERVER_PORT);

    // Test network connectivity first
    ESP_LOGI(TAG, "🌐 Testing network connectivity...");
    
    int sent = sendto(sip_client.socket_fd, register_msg, strlen(register_msg), 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (sent < 0)
    {
        ESP_LOGE(TAG, "❌ SIP REGISTER send failed: errno %d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }

    sip_client.state = SIP_STATE_REGISTERING;
    ESP_LOGI(TAG, "✅ SIP REGISTER sent (%d bytes) - Waiting for response...", sent);
    return ESP_OK;
}

// Also add this network testing function:

/**
 * @brief Test basic network connectivity to SIP server
 */
static void test_sip_connectivity(void)
{
    ESP_LOGI(TAG, "🔍 Testing SIP server connectivity...");
    
    // Create a test socket
    int test_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (test_sock < 0) {
        ESP_LOGE(TAG, "❌ Cannot create test socket");
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SIP_SERVER_PORT);
    
    if (inet_aton(SIP_SERVER_HOST, &server_addr.sin_addr) == 0) {
        ESP_LOGE(TAG, "❌ Invalid SIP server IP: %s", SIP_SERVER_HOST);
        close(test_sock);
        return;
    }

    // Try to send a simple test packet
    const char *test_msg = "OPTIONS sip:" SIP_SERVER_HOST " SIP/2.0\r\n\r\n";
    int result = sendto(test_sock, test_msg, strlen(test_msg), 0, 
                       (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if (result > 0) {
        ESP_LOGI(TAG, "✅ Network path to SIP server seems OK");
    } else {
        ESP_LOGE(TAG, "❌ Cannot reach SIP server: errno %d (%s)", errno, strerror(errno));
    }
    
    close(test_sock);
}

// Add this call in your sip_client_init() function:
// test_sip_connectivity();

/**
 * @brief Parse SDP for RTP information
 */
static esp_err_t parse_sdp_for_rtp(const char *sdp, char *remote_ip, uint16_t *remote_port)
{
    if (!sdp || !remote_ip || !remote_port)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Parsing SDP:\n%s", sdp);

    // Find connection line (c=IN IP4 <ip>)
    const char *c_line = strstr(sdp, "c=IN IP4 ");
    if (c_line)
    {
        c_line += strlen("c=IN IP4 ");
        const char *end = strchr(c_line, '\r');
        if (!end)
            end = strchr(c_line, '\n');
        if (end)
        {
            size_t ip_len = end - c_line;
            if (ip_len < 16)
            { // Max IP length
                strncpy(remote_ip, c_line, ip_len);
                remote_ip[ip_len] = '\0';
            }
        }
    }

    // Find media line (m=audio <port> RTP/AVP <payload>)
    const char *m_line = strstr(sdp, "m=audio ");
    if (m_line)
    {
        m_line += strlen("m=audio ");
        *remote_port = (uint16_t)atoi(m_line);
    }

    if (strlen(remote_ip) > 0 && *remote_port > 0)
    {
        ESP_LOGI(TAG, "Parsed SDP: Remote RTP endpoint %s:%d", remote_ip, *remote_port);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Failed to parse SDP for RTP information");
    return ESP_FAIL;
}

/**
 * @brief Enhanced SIP response processing with authentication support
 */
static void process_sip_response(char *response, int len)
{
    ESP_LOGD(TAG, "SIP Response received:\n%s", response);

    // Extract status code
    int status_code = 0;
    if (sscanf(response, "SIP/2.0 %d", &status_code) == 1)
    {
        ESP_LOGI(TAG, "SIP Response: %d", status_code);

        switch (status_code)
        {
        case 200: // OK
            if (sip_client.state == SIP_STATE_REGISTERING)
            {
                sip_client.state = SIP_STATE_REGISTERED;
                sip_client.is_connected = true;
                sip_client.registration_retry_count = 0;
                ESP_LOGI(TAG, "SIP Registration successful!");
            }
            else if (sip_client.state == SIP_STATE_CALLING)
            {
                // Extract To tag from 200 OK response
                const char *to_tag_start = strstr(response, "To:");
                if (to_tag_start)
                {
                    const char *tag_start = strstr(to_tag_start, "tag=");
                    if (tag_start)
                    {
                        tag_start += 4; // Skip "tag="
                        const char *tag_end = strpbrk(tag_start, " \r\n;>");
                        if (tag_end)
                        {
                            size_t tag_len = tag_end - tag_start;
                            if (tag_len < sizeof(sip_client.to_tag))
                            {
                                strncpy(sip_client.to_tag, tag_start, tag_len);
                                sip_client.to_tag[tag_len] = '\0';
                            }
                        }
                    }
                }

                sip_client.state = SIP_STATE_IN_CALL;
                sip_client.call_active = true;

                // Parse SDP and start RTP session
                const char *sdp_start = strstr(response, "\r\n\r\n");
                if (sdp_start)
                {
                    sdp_start += 4;
                    char remote_ip[16] = {0};
                    uint16_t remote_port = 0;

                    if (parse_sdp_for_rtp(sdp_start, remote_ip, &remote_port) == ESP_OK)
                    {
                        if (rtp_start_session(remote_ip, remote_port) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "🔊 CALL ESTABLISHED! Audio bridge active!");
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to start RTP session");
                        }
                    }
                }
            }
            break;

        case 401: // Unauthorized - Authentication required
            if (sip_client.state == SIP_STATE_REGISTERING)
            {
                ESP_LOGI(TAG, "SIP Authentication required");

                // Find WWW-Authenticate header
                const char *auth_header = strstr(response, "WWW-Authenticate:");
                if (auth_header)
                {
                    auth_header += 17; // Skip "WWW-Authenticate:"
                    while (*auth_header == ' ')
                        auth_header++; // Skip spaces

                    const char *auth_end = strstr(auth_header, "\r\n");
                    if (auth_end)
                    {
                        char auth_line[512];
                        size_t auth_len = auth_end - auth_header;
                        if (auth_len < sizeof(auth_line))
                        {
                            strncpy(auth_line, auth_header, auth_len);
                            auth_line[auth_len] = '\0';

                            ESP_LOGI(TAG, "Sending authenticated REGISTER");
                            sip_send_register_with_auth(auth_line);
                        }
                    }
                }
            }
            break;

        case 403: // Forbidden
            ESP_LOGW(TAG, "SIP Authentication failed (403 Forbidden)");
            sip_client.state = SIP_STATE_ERROR;
            break;

        case 404: // Not Found
            ESP_LOGW(TAG, "SIP User/Extension not found (404)");
            break;

        case 486: // Busy Here
        case 487: // Request Terminated
            ESP_LOGW(TAG, "Call failed/declined (%d)", status_code);
            sip_client.state = SIP_STATE_REGISTERED;
            sip_client.call_active = false;

            if (sip_client.rtp_ready)
            {
                rtp_stop_session();
            }
            break;

        default:
            ESP_LOGW(TAG, "Unhandled SIP response: %d", status_code);
            break;
        }
    }

    // Handle BYE request (call termination)
    if (strstr(response, "BYE ") == response)
    {
        ESP_LOGI(TAG, "Call terminated by remote party");
        sip_client.call_active = false;
        sip_client.state = SIP_STATE_REGISTERED;

        if (sip_client.rtp_ready)
        {
            rtp_stop_session();
        }
    }
}

/**
 * @brief SIP client task with RTP integration
 */
static void sip_client_task(void *param)
{
    static char rx_buffer[2048];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGI(TAG, "SIP client task started");

    while (1)
    {
        // Wait for Wi-Fi connection
        if (!wifi_state.connected)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Create UDP socket for SIP
        if (sip_client.socket_fd < 0)
        {
            sip_client.socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sip_client.socket_fd < 0)
            {
                ESP_LOGE(TAG, "Unable to create SIP socket: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            // Set socket timeout
            struct timeval timeout;
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            setsockopt(sip_client.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

            ESP_LOGI(TAG, "SIP socket created");
        }

        // Handle SIP state machine
        switch (sip_client.state)
        {
        case SIP_STATE_DISCONNECTED:
            ESP_LOGI(TAG, "Attempting SIP registration...");
            if (sip_send_register() == ESP_OK)
            {
                sip_client.last_keepalive = xTaskGetTickCount();
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            break;

        case SIP_STATE_REGISTERING:
            // printf("Registering.......................\n\n"); 
            // Wait for registration response with timeout
            if ((xTaskGetTickCount() - sip_client.last_keepalive) > pdMS_TO_TICKS(10000))
            {
                ESP_LOGW(TAG, "SIP registration timeout, retrying...");
                sip_client.state = SIP_STATE_DISCONNECTED;
                sip_client.registration_retry_count++;
                if (sip_client.registration_retry_count > 5)
                {
                    ESP_LOGE(TAG, "SIP registration failed after 5 retries");
                    vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 30 seconds before retry
                    sip_client.registration_retry_count = 0;
                }
            }
            break;

        case SIP_STATE_REGISTERED:
            printf("Registered.......................\n\n"); 
            // Send keep-alive (re-register) every 30 minutes
            if ((xTaskGetTickCount() - sip_client.last_keepalive) > pdMS_TO_TICKS(1800000))
            {
                ESP_LOGI(TAG, "Sending SIP keep-alive registration...");
                sip_send_register();
            }
            break;

        case SIP_STATE_ERROR:
            ESP_LOGW(TAG, "SIP in error state, attempting recovery...");
            close(sip_client.socket_fd);
            sip_client.socket_fd = -1;
            sip_client.state = SIP_STATE_DISCONNECTED;

            // Clean up RTP session on error
            if (sip_client.rtp_ready)
            {
                rtp_stop_session();
            }

            vTaskDelay(pdMS_TO_TICKS(10000));
            break;

        default:
            break;
        }

        // Check for incoming SIP messages
        int len = recvfrom(sip_client.socket_fd, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0)
        {
            rx_buffer[len] = 0; // Null terminate
            ESP_LOGI(TAG, "SIP Response received (%d bytes):", len);
            ESP_LOGI(TAG, "Response: %s", rx_buffer); // This will show us what the server sends
            process_sip_response(rx_buffer, len);
        }
        else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            ESP_LOGE(TAG, "SIP socket error: errno %d", errno);
        }
        else
        {
            // No data received - this is normal for non-blocking socket
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

sudo iptables -F
sudo iptables -t nat -F
sudo iptables -A FORWARD -i wlp0s20f3 -o proton0 -j ACCEPT
sudo iptables -A FORWARD -i proton0 -o wlp0s20f3 -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o proton0 -j MASQUERADE




/**
 * @brief Initialize SIP client with RTP support
 */
static void sip_client_init(void)
{
    test_sip_connectivity();
    ESP_LOGI(TAG, "Initializing SIP client with RTP support...");

    memset(&sip_client, 0, sizeof(sip_client));
    sip_client.state = SIP_STATE_DISCONNECTED;
    sip_client.socket_fd = -1;
    sip_client.cseq = 1;
    sip_client.rtp_local_port = RTP_PORT_BASE;

    // Initialize RTP session
    if (rtp_session_init(&sip_client.rtp_session) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize RTP session");
        return;
    }

    // Create SIP client task
    xTaskCreate(sip_client_task, "sip_client", 12288, NULL, 5, NULL);

    ESP_LOGI(TAG, "SIP client with RTP support initialized");
}

// ============================================================================
// ORIGINAL UAC FUNCTIONS (Updated with RTP integration)
// ============================================================================

/**
 * @brief GPIO interrupt handler for mute button
 */
static void IRAM_ATTR mute_button_isr_handler(void *arg)
{
    uint32_t current_time = xTaskGetTickCount();
    int button_level = gpio_get_level(MUTE_BUTTON_GPIO);

    // Determine if button is currently pressed (considering active low)
    bool button_currently_pressed = MUTE_BUTTON_ACTIVE_LOW ? (button_level == 0) : (button_level == 1);

    if (button_currently_pressed && !mute_ctrl.button_pressed)
    {
        // Button just pressed
        mute_ctrl.button_pressed = true;
        mute_ctrl.press_start_time = current_time;
        mute_ctrl.state = BUTTON_STATE_PRESSED;
    }
    else if (!button_currently_pressed && mute_ctrl.button_pressed)
    {
        // Button just released
        mute_ctrl.button_pressed = false;
        uint32_t press_duration = current_time - mute_ctrl.press_start_time;

        // Debounce check
        if (press_duration > pdMS_TO_TICKS(DEBOUNCE_TIME_MS))
        {
            if (press_duration < pdMS_TO_TICKS(LONG_PRESS_TIME_MS))
            {
                // Short press - toggle mute
                if ((current_time - mute_ctrl.last_toggle_time) > pdMS_TO_TICKS(DEBOUNCE_TIME_MS))
                {
                    mute_ctrl.is_muted = !mute_ctrl.is_muted;
                    mute_ctrl.last_toggle_time = current_time;
                    mute_toggle_count++;
                }
            }
            else
            {
                // Long press - also toggle mute
                mute_ctrl.is_muted = !mute_ctrl.is_muted;
                mute_ctrl.last_toggle_time = current_time;
                mute_toggle_count++;
            }
        }
        mute_ctrl.state = BUTTON_STATE_RELEASED;
    }
}

/**
 * @brief Initialize mute button
 */
static esp_err_t init_mute_button(void)
{
    ESP_LOGI(TAG, "Initializing mute button on GPIO %d...", MUTE_BUTTON_GPIO);

    // Configure GPIO for button input
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << MUTE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = MUTE_BUTTON_ACTIVE_LOW ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = MUTE_BUTTON_ACTIVE_LOW ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE // Trigger on both rising and falling edges
    };

    esp_err_t ret = gpio_config(&btn_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure mute button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install GPIO interrupt service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handler for the button GPIO
    ret = gpio_isr_handler_add(MUTE_BUTTON_GPIO, mute_button_isr_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize mute state
    mute_ctrl.is_muted = false;
    mute_ctrl.button_pressed = false;
    mute_ctrl.state = BUTTON_STATE_RELEASED;

    ESP_LOGI(TAG, "Mute button initialized (GPIO %d, %s)",
             MUTE_BUTTON_GPIO,
             MUTE_BUTTON_ACTIVE_LOW ? "Active Low" : "Active High");
    ESP_LOGI(TAG, "   Press button to toggle mute/unmute");
    ESP_LOGI(TAG, "   Hold for 2+ seconds for long press action");

    return ESP_OK;
}

/**
 * @brief Get current mute status
 */
static bool is_microphone_muted(void)
{
    return mute_ctrl.is_muted;
}

/**
 * @brief Apply volume boost with clipping protection and startup fade
 */
static inline void apply_volume_boost(int16_t *samples, size_t num_samples)
{
#ifdef CONFIG_UAC_ENABLE_VOLUME_BOOST
    if (VOLUME_BOOST_PERCENT == 100 && spk_callback_count > STARTUP_FADE_CALLBACKS)
    {
        return; // No boost needed and startup period over
    }

    // Calculate fade multiplier for startup
    float fade_multiplier = 1.0f;
    if (spk_callback_count <= STARTUP_FADE_CALLBACKS)
    {
        fade_multiplier = (float)spk_callback_count / (float)STARTUP_FADE_CALLBACKS;
        if (fade_multiplier < 0.1f)
            fade_multiplier = 0.1f; // Minimum 10%
    }

    for (size_t i = 0; i < num_samples; i++)
    {
        int32_t amplified = (int32_t)samples[i] * VOLUME_BOOST_PERCENT / 100;

        // Apply startup fade
        amplified = (int32_t)((float)amplified * fade_multiplier);

        // Clipping protection
        if (amplified > 32767)
            amplified = 32767;
        if (amplified < -32768)
            amplified = -32768;

        samples[i] = (int16_t)amplified;
    }
#endif
}

/**
 * @brief Robot sound prevention - gentle buffer reset
 */
static void prevent_robot_sound(void)
{
#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
    if (BUFFER_RESET_INTERVAL_MS == 0)
    {
        return; // Disabled
    }

    uint32_t current_time = xTaskGetTickCount();

    if ((current_time - last_buffer_reset) > pdMS_TO_TICKS(BUFFER_RESET_INTERVAL_MS))
    {
        ESP_LOGI(TAG, "Preventive buffer reset (anti-robot sound)...");

        // Briefly reset I2S channels
#ifdef CONFIG_UAC_ENABLE_I2S_SPK
        if (i2s_spk_handle)
        {
            i2s_channel_disable(i2s_spk_handle);
            vTaskDelay(pdMS_TO_TICKS(10)); // Slightly longer delay
            i2s_channel_enable(i2s_spk_handle);
        }
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
        if (i2s_mic_handle)
        {
            i2s_channel_disable(i2s_mic_handle);
            vTaskDelay(pdMS_TO_TICKS(10));
            i2s_channel_enable(i2s_mic_handle);
        }
#endif

        last_buffer_reset = current_time;
        consecutive_errors = 0;

        ESP_LOGI(TAG, "Buffer reset completed");
    }
#endif
}

#ifdef CONFIG_UAC_ENABLE_I2S_SPK
/**
 * @brief Initialize I2S speaker with enhanced buffer settings
 */
static esp_err_t init_i2s_speaker(void)
{
    ESP_LOGI(TAG, "Initializing I2S speaker (MAX98357A)...");
    ESP_LOGI(TAG, "   BCLK: GPIO %d, WS: GPIO %d, DOUT: GPIO %d",
             I2S_SPK_BCLK, I2S_SPK_WS, I2S_SPK_DOUT);
    ESP_LOGI(TAG, "   Sample Rate: %d Hz, Channels: %d", SAMPLE_RATE, CHANNELS);
    ESP_LOGI(TAG, "   Enhanced buffers: %d x %d (startup protection)", DMA_BUF_COUNT, DMA_BUF_LEN);

    // Create I2S TX channel with enhanced settings
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = DMA_BUF_LEN;
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &i2s_spk_handle, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2S speaker channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure I2S standard mode
    i2s_slot_mode_t slot_mode = (CHANNELS == 2) ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SPK_BCLK,
            .ws = I2S_SPK_WS,
            .dout = I2S_SPK_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(i2s_spk_handle, &std_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init I2S speaker mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(i2s_spk_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable I2S speaker: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S speaker initialized with startup protection");
    return ESP_OK;
}
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
/**
 * @brief Initialize I2S microphone with enhanced settings
 */
static esp_err_t init_i2s_microphone(void)
{
    ESP_LOGI(TAG, "Initializing I2S microphone (INMP441)...");
    ESP_LOGI(TAG, "   BCLK: GPIO %d, WS: GPIO %d, DIN: GPIO %d",
             I2S_MIC_BCLK, I2S_MIC_WS, I2S_MIC_DIN);

    // Create I2S RX channel for microphone with enhanced settings
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 12; // Increased for stability
    chan_cfg.dma_frame_num = 256;
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &i2s_mic_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2S microphone channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure I2S standard mode for microphone
    i2s_slot_mode_t slot_mode = (CHANNELS == 2) ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_BCLK,
            .ws = I2S_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_MIC_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(i2s_mic_handle, &std_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init I2S microphone mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(i2s_mic_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable I2S microphone: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S microphone initialized");
    return ESP_OK;
}
#endif

/**
 * @brief Test headset hardware
 */
static void test_headset_once(void)
{
    if (initial_test_done)
    {
        return;
    }

    ESP_LOGI(TAG, "Testing headset hardware...");

    // Test mute button
    ESP_LOGI(TAG, "Testing mute button...");
    int button_level = gpio_get_level(MUTE_BUTTON_GPIO);
    bool button_pressed = MUTE_BUTTON_ACTIVE_LOW ? (button_level == 0) : (button_level == 1);
    ESP_LOGI(TAG, "   Button state: %s (level=%d)", button_pressed ? "PRESSED" : "RELEASED", button_level);

#ifdef CONFIG_UAC_ENABLE_I2S_SPK
    // Test speaker with tone
    if (i2s_spk_handle)
    {
        ESP_LOGI(TAG, "Testing speaker with 440Hz tone for 1 second...");

        int16_t test_buffer[512];
        size_t bytes_written;
        int total_samples = SAMPLE_RATE; // 1 second
        int samples_written = 0;

        while (samples_written < total_samples)
        {
            int samples_to_write = (total_samples - samples_written > 512) ? 512 : (total_samples - samples_written);

            // Generate 440Hz sine wave with gentle fade-in
            for (int i = 0; i < samples_to_write; i++)
            {
                float time = (float)(samples_written + i) / SAMPLE_RATE;
                float fade = (time < 0.1f) ? (time / 0.1f) : 1.0f;              // 100ms fade-in
                float sample = sinf(2.0f * M_PI * 440.0f * time) * 0.2f * fade; // Reduced volume
                int16_t sample_value = (int16_t)(sample * 16383);

                if (CHANNELS == 2)
                {
                    // Stereo: same signal to both channels
                    test_buffer[i * 2] = sample_value;
                    test_buffer[i * 2 + 1] = sample_value;
                }
                else
                {
                    // Mono
                    test_buffer[i] = sample_value;
                }
            }

            size_t buffer_size = samples_to_write * CHANNELS * sizeof(int16_t);
            esp_err_t ret = i2s_channel_write(i2s_spk_handle, test_buffer, buffer_size, &bytes_written, 100);

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Speaker test failed: %s", esp_err_to_name(ret));
                break;
            }

            samples_written += samples_to_write;
        }

        ESP_LOGI(TAG, "Speaker test completed");
    }
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
    // Test microphone
    if (i2s_mic_handle)
    {
        ESP_LOGI(TAG, "Testing microphone...");

        int16_t test_mic_buffer[256];
        size_t bytes_read;

        esp_err_t ret = i2s_channel_read(i2s_mic_handle, test_mic_buffer,
                                         sizeof(test_mic_buffer), &bytes_read, 1000);

        if (ret == ESP_OK && bytes_read > 0)
        {
            ESP_LOGI(TAG, "Microphone test completed - read %zu bytes", bytes_read);

            // Check for audio signal
            bool has_signal = false;
            for (int i = 0; i < bytes_read / 2; i++)
            {
                if (abs(test_mic_buffer[i]) > 100)
                {
                    has_signal = true;
                    break;
                }
            }

            ESP_LOGI(TAG, "Audio signal: %s", has_signal ? "DETECTED" : "Silent");
        }
        else
        {
            ESP_LOGW(TAG, "Microphone test failed: %s", esp_err_to_name(ret));
        }
    }
#endif

    initial_test_done = true;

#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
    last_buffer_reset = xTaskGetTickCount();
#endif

    ESP_LOGI(TAG, "Headset test completed");
}

// ============================================================================
// UAC CALLBACKS WITH RTP INTEGRATION (Phase 2A Implementation)
// ============================================================================

/**
 * @brief UAC microphone callback with RTP audio bridging
 */
static esp_err_t uac_microphone_cb(uint8_t *buffer, size_t len, size_t *bytes_written, void *arg)
{
    mic_callback_count++;

    // Check if microphone is muted
    if (is_microphone_muted())
    {
        memset(buffer, 0, len);
        *bytes_written = len;
        return ESP_OK;
    }

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
    // Read from I2S microphone
    if (i2s_mic_handle && len <= sizeof(mic_buffer))
    {
        size_t bytes_read;
        esp_err_t ret = i2s_channel_read(i2s_mic_handle, mic_buffer, len, &bytes_read, 5);

        if (ret == ESP_OK && bytes_read > 0)
        {
            memcpy(buffer, mic_buffer, bytes_read);
            *bytes_written = bytes_read;

            // 🎯 PHASE 2B: ROUTE USB MICROPHONE TO SIP
            if (sip_client.call_active && sip_client.rtp_ready)
            {
                int16_t *samples = (int16_t *)buffer;
                size_t num_samples = bytes_read / sizeof(int16_t);

                // Send audio to SIP via RTP
                esp_err_t rtp_result = rtp_send_audio(samples, num_samples);

                // Log successful audio routing periodically
                if (mic_callback_count % 2000 == 0)
                {
                    ESP_LOGI(TAG, " USB mic to SIP: %zu samples, RTP: %s",
                             num_samples, rtp_result == ESP_OK ? "OK" : "FAIL");
                }
            }

            // Standard logging
            if (mic_callback_count % 2000 == 0)
            {
                ESP_LOGI(TAG, "Microphone ACTIVE: %lu callbacks", mic_callback_count);
            }

            return ESP_OK;
        }
    }
#endif

    // Fallback: silence
    memset(buffer, 0, len);
    *bytes_written = len;
    return ESP_OK;
}

/**
 * @brief Enhanced UAC speaker callback with RTP audio bridging
 */
static esp_err_t uac_speaker_cb(uint8_t *buffer, size_t len, void *arg)
{
    spk_callback_count++;

    bool sip_audio_used = false;

    // 🎯 PHASE 2B: ROUTE SIP AUDIO TO USB SPEAKER
    if (sip_client.call_active && sip_client.rtp_ready)
    {
        int16_t sip_audio_buffer[480]; // 10ms at 48kHz
        size_t sip_audio_len = sizeof(sip_audio_buffer) / sizeof(int16_t);

        // Try to get SIP audio from RTP
        esp_err_t rtp_result = rtp_receive_audio(sip_audio_buffer, &sip_audio_len);

        if (rtp_result == ESP_OK && sip_audio_len > 0)
        {
            // Use SIP audio instead of USB audio
            size_t bytes_to_copy = sip_audio_len * sizeof(int16_t);
            if (bytes_to_copy > len)
                bytes_to_copy = len;

            memcpy(buffer, sip_audio_buffer, bytes_to_copy);

            // Fill remaining buffer with silence if needed
            if (bytes_to_copy < len)
            {
                memset((uint8_t *)buffer + bytes_to_copy, 0, len - bytes_to_copy);
            }

            sip_audio_used = true;

            // Log successful audio routing periodically
            if (spk_callback_count % 2000 == 0)
            {
                ESP_LOGI(TAG, "SIP to USB speaker: %zu samples", sip_audio_len);
            }
        }
    }

#ifdef CONFIG_UAC_ENABLE_I2S_SPK
    if (i2s_spk_handle == NULL)
    {
        return ESP_OK;
    }

    // Apply volume boost only if not using SIP audio
    if (!sip_audio_used)
    {
        int16_t *samples = (int16_t *)buffer;
        size_t num_samples = len / sizeof(int16_t);
        apply_volume_boost(samples, num_samples);
    }

    // Write to I2S speaker
    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(i2s_spk_handle, buffer, len, &bytes_written, 30);

    if (ret != ESP_OK || bytes_written != len)
    {
#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
        consecutive_errors++;
#endif
        if (spk_callback_count <= 10)
        {
            ESP_LOGW(TAG, "I2S write issue: %s, %d/%d bytes",
                     esp_err_to_name(ret), bytes_written, len);
        }
    }
    else
    {
#ifdef CONFIG_UAC_ENABLE_ROBOT_SOUND_FIX
        consecutive_errors = 0;
#endif
        // Enhanced logging with SIP status
        if (spk_callback_count % 2000 == 0)
        {
            ESP_LOGI(TAG, "Speaker: %lu callbacks, %s audio",
                     spk_callback_count,
                     sip_audio_used ? "SIP" : "USB");
        }
    }

    // Prevent robot sound
    if (spk_callback_count % 1000 == 0)
    {
        prevent_robot_sound();
    }
#endif

    return ESP_OK;
}

// ============================================================================
// DEDICATED RTP BRIDGE TASK (ADD THESE FUNCTIONS)
// ============================================================================

/**
 * @brief Dedicated RTP bridge task - handles all RTP processing
 */
static void rtp_bridge_task(void *param)
{
    ESP_LOGI(TAG, "RTP bridge task started");

    audio_frame_t usb_frame, sip_frame;
    TickType_t last_stats_time = 0;

    while (1)
    {
        // Process USB to SIP direction
        if (sip_client.call_active && sip_client.rtp_ready && usb_to_sip_queue)
        {
            if (xQueueReceive(usb_to_sip_queue, &usb_frame, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // Send USB audio to SIP via RTP
                rtp_send_audio(usb_frame.audio_data, usb_frame.sample_count);
            }
        }

        // Process SIP to USB direction
        if (sip_client.call_active && sip_client.rtp_ready && sip_to_usb_queue)
        {
            // Receive RTP audio
            size_t audio_len = AUDIO_FRAME_SIZE;
            if (rtp_receive_audio(sip_frame.audio_data, &audio_len) == ESP_OK && audio_len > 0)
            {
                sip_frame.sample_count = audio_len;
                sip_frame.timestamp = xTaskGetTickCount();

                // Send to USB speaker queue (non-blocking)
                xQueueSend(sip_to_usb_queue, &sip_frame, 0);
            }
        }

        // Periodic statistics (every 10 seconds)
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_stats_time) > pdMS_TO_TICKS(10000))
        {
            if (sip_client.call_active)
            {
                ESP_LOGI(TAG, "RTP Bridge: Sent %" PRIu32 ", Rcvd %" PRIu32 " packets",
                         sip_client.rtp_session.packets_sent,
                         sip_client.rtp_session.packets_received);
            }
            last_stats_time = current_time;
        }

        // Small delay
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Initialize lightweight audio bridge system
 */
static esp_err_t init_lightweight_audio_bridge(void)
{
    ESP_LOGI(TAG, "Initializing lightweight audio bridge...");

    // Create audio queues
    usb_to_sip_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_frame_t));
    if (!usb_to_sip_queue)
    {
        ESP_LOGE(TAG, "Failed to create USB→SIP queue");
        return ESP_FAIL;
    }

    sip_to_usb_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_frame_t));
    if (!sip_to_usb_queue)
    {
        ESP_LOGE(TAG, "Failed to create SIP→USB queue");
        vQueueDelete(usb_to_sip_queue);
        return ESP_FAIL;
    }

    // Create RTP bridge task with large stack
    BaseType_t task_result = xTaskCreate(
        rtp_bridge_task, // Task function
        "rtp_bridge",    // Task name
        12288,           // Large stack size (12KB)
        NULL,            // Parameters
        5,               // Priority (medium)
        NULL             // Task handle
    );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create RTP bridge task");
        vQueueDelete(usb_to_sip_queue);
        vQueueDelete(sip_to_usb_queue);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Lightweight audio bridge initialized");
    return ESP_OK;
}

// Add this function to your ESP32 code
static void send_wireshark_test_packet(void)
{
    int test_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (test_sock < 0) {
        ESP_LOGE(TAG, "Failed to create test socket");
        return;
    }
    
    struct sockaddr_in test_addr;
    test_addr.sin_family = AF_INET;
    test_addr.sin_port = htons(12345);
    test_addr.sin_addr.s_addr = inet_addr("192.168.18.255"); // Broadcast
    
    const char *test_msg = "🔍 WIRESHARK TEST FROM ESP32-S3 🔍";
    
    int sent = sendto(test_sock, test_msg, strlen(test_msg), 0,
                     (struct sockaddr*)&test_addr, sizeof(test_addr));
    
    ESP_LOGI(TAG, "📡 Wireshark test packet: %s (%d bytes)", 
             sent > 0 ? "SENT" : "FAILED", sent);
    
    close(test_sock);
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

/**
 * @brief Main application with Phase 2A RTP integration
 */
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " ESP32-S3 UAC2 Headset + SIP Client");
    ESP_LOGI(TAG, "         Phase 2A: RTP Audio Bridge");
    ESP_LOGI(TAG, "===========================================");

    // Initialize NVS (required for Wi-Fi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Display configuration
    ESP_LOGI(TAG, "Audio Configuration:");
    ESP_LOGI(TAG, "   Sample Rate: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "   Channels: %d (%s)", CHANNELS, (CHANNELS == 2) ? "stereo" : "mono");
    ESP_LOGI(TAG, "   Bits per Sample: %d", BITS_PER_SAMPLE);
    ESP_LOGI(TAG, "   Volume Boost: %d%%", VOLUME_BOOST_PERCENT);
    ESP_LOGI(TAG, "   Startup Fade: %d callbacks (~200ms)", STARTUP_FADE_CALLBACKS);

    ESP_LOGI(TAG, "Hardware Configuration:");
#ifdef CONFIG_UAC_ENABLE_I2S_SPK
    ESP_LOGI(TAG, "   Speaker: MAX98357A (GPIO %d,%d,%d)", I2S_SPK_BCLK, I2S_SPK_WS, I2S_SPK_DOUT);
#else
    ESP_LOGI(TAG, "   Speaker: DISABLED");
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
    ESP_LOGI(TAG, "   Microphone: INMP441 (GPIO %d,%d,%d)", I2S_MIC_BCLK, I2S_MIC_WS, I2S_MIC_DIN);
#else
    ESP_LOGI(TAG, "   Microphone: DISABLED");
#endif

    ESP_LOGI(TAG, "   Mute Button: GPIO %d (%s)", MUTE_BUTTON_GPIO,
             MUTE_BUTTON_ACTIVE_LOW ? "Active Low" : "Active High");

    ESP_LOGI(TAG, "SIP Configuration:");
    ESP_LOGI(TAG, "   Server: %s:%d", SIP_SERVER_HOST, SIP_SERVER_PORT);
    ESP_LOGI(TAG, "   User: %s", SIP_USER);
    ESP_LOGI(TAG, "   Wi-Fi: %s", WIFI_SSID);

    ESP_LOGI(TAG, "RTP Configuration:");
    ESP_LOGI(TAG, "   Local Port: %d", RTP_PORT_BASE);
    ESP_LOGI(TAG, "   Packet Interval: %d ms", RTP_SEND_INTERVAL_MS);
    ESP_LOGI(TAG, "   Audio Buffer: %d samples (%.1f ms)", RTP_AUDIO_BUFFER_SIZE,
             (float)RTP_AUDIO_BUFFER_SIZE / SAMPLE_RATE * 1000);
    ESP_LOGI(TAG, "   Codec: G.711 μ-law (Opus planned)");

    ESP_LOGI(TAG, "Device Identity:");
    ESP_LOGI(TAG, "   Device Name: %s", CONFIG_UAC_DEVICE_NAME);
    ESP_LOGI(TAG, "   Manufacturer: %s", CONFIG_UAC_MANUFACTURER_NAME);
    ESP_LOGI(TAG, "===========================================");

    // Initialize Wi-Fi first
    ESP_LOGI(TAG, "Starting Wi-Fi initialization...");
    wifi_init_sta();

    // Initialize mute button
    ESP_LOGI(TAG, "Initializing mute button...");
    ret = init_mute_button();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Mute button initialization failed!");
        return;
    }

    // Initialize hardware
    ESP_LOGI(TAG, "Initializing headset hardware...");

#ifdef CONFIG_UAC_ENABLE_I2S_SPK
    ret = init_i2s_speaker();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Speaker initialization failed!");
        return;
    }
#endif

#ifdef CONFIG_UAC_ENABLE_I2S_MIC
    ret = init_i2s_microphone();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Microphone initialization failed!");
        return;
    }
#endif

    // Initialize lightweight audio bridge
    ESP_LOGI(TAG, "Initializing audio bridge...");
    ret = init_lightweight_audio_bridge();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Audio bridge initialization failed!");
        return;
    }

    // Test hardware
    ESP_LOGI(TAG, "Hardware stabilization...");
    vTaskDelay(pdMS_TO_TICKS(1500));
    test_headset_once();
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Initialize UAC device
    ESP_LOGI(TAG, "Initializing UAC2 device...");

    uac_device_config_t uac_config = {
        .input_cb = uac_microphone_cb,
        .output_cb = uac_speaker_cb,
    };

    ret = uac_device_init(&uac_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UAC device initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  RTP AUDIO BRIDGE READY!");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✓ USB Audio Device: Ready for Android/PC");
    ESP_LOGI(TAG, "✓ Wi-Fi: Connecting to %s...", WIFI_SSID);
    ESP_LOGI(TAG, "✓ SIP Client: Will auto-register when Wi-Fi connects");
    ESP_LOGI(TAG, "✓ RTP Engine: G.711 μ-law codec ready");
    ESP_LOGI(TAG, "✓ Audio Bridge: USB ↔ SIP routing ACTIVE");
    ESP_LOGI(TAG, "✓ Mute Button: Press to toggle microphone");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Audio Flow:");
    ESP_LOGI(TAG, "   Android → USB → ESP32 → RTP → SIP Server");
    ESP_LOGI(TAG, "   Android ← USB ← ESP32 ← RTP ← SIP Server");
    ESP_LOGI(TAG, "===========================================");

    // Main monitoring loop
    uint32_t loop_count = 0;
    while (1)
    {


        vTaskDelay(pdMS_TO_TICKS(15000)); // Every 15 seconds
        loop_count++;

        ESP_LOGI(TAG, "System Status #%" PRIu32 ":", loop_count);

        send_wireshark_test_packet();
        // Wi-Fi Status
        ESP_LOGI(TAG, "Wi-Fi: %s%s",
                 wifi_state.connected ? "CONNECTED" : "DISCONNECTED",
                 wifi_state.connected ? " (" IPSTR ")" : "");

        // SIP Status
        const char *sip_state_str[] = {
            "DISCONNECTED", "REGISTERING", "REGISTERED", "CALLING", "IN_CALL", "ERROR"};
        ESP_LOGI(TAG, "SIP: %s%s",
                 sip_state_str[sip_client.state],
                 sip_client.call_active ? " (CALL ACTIVE)" : "");

        // RTP Status
        if (sip_client.rtp_ready && sip_client.rtp_session.session_active)
        {
            ESP_LOGI(TAG, "RTP: ACTIVE (Sent: %" PRIu32 ", Rcvd: %" PRIu32 ", Lost: %" PRIu32 ")",
                     sip_client.rtp_session.packets_sent,
                     sip_client.rtp_session.packets_received,
                     sip_client.rtp_session.packets_lost);
        }
        else
        {
            ESP_LOGI(TAG, "RTP: STANDBY");
        }

        // Audio Status
        ESP_LOGI(TAG, "Microphone: %" PRIu32 " callbacks (%s)",
                 mic_callback_count,
                 is_microphone_muted() ? "MUTED " : "ACTIVE ");
        ESP_LOGI(TAG, "Speaker: %" PRIu32 " callbacks", spk_callback_count);
        ESP_LOGI(TAG, "Mute toggles: %" PRIu32, mute_toggle_count);

        // Button Status
        int button_level = gpio_get_level(MUTE_BUTTON_GPIO);
        bool button_pressed = MUTE_BUTTON_ACTIVE_LOW ? (button_level == 0) : (button_level == 1);
        ESP_LOGI(TAG, "Button: %s", button_pressed ? "PRESSED" : "RELEASED");

        // System Health
        ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());

        if (spk_callback_count > 0 || mic_callback_count > 0)
        {
            ESP_LOGI(TAG, "USB Audio: ACTIVE - device connected!");
        }
        else if (loop_count > 2)
        {
            ESP_LOGI(TAG, "USB Audio: Waiting for connection...");
        }

        if (wifi_state.connected && sip_client.state == SIP_STATE_REGISTERED)
        {
            ESP_LOGI(TAG, "Status: READY FOR SIP CALLS! ");
            if (sip_client.call_active)
            {
                ESP_LOGI(TAG, " AUDIO BRIDGE ACTIVE: USB ↔ SIP routing!");
            }
        }
        else if (wifi_state.connected)
        {
            ESP_LOGI(TAG, "Status: SIP registration in progress...");
        }
        else
        {
            ESP_LOGI(TAG, "Status: Waiting for Wi-Fi connection...");
        }

        ESP_LOGI(TAG, "");
    }
}