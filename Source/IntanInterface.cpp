// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "IntanInterface.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <queue>
#include <deque>
#include <condition_variable>
#include <cerrno>
#include <array>
#include <algorithm>   // std::max/std::min (not transitively guaranteed on MSVC)
#include <cmath>

// Platform-specific networking
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socklen_t = int;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <pthread.h>       // recv-thread scheduling priority
    #include <sched.h>
    #ifdef __APPLE__
        #include <pthread/qos.h>
        #include <mach/mach.h>              // real-time (time-constraint) scheduling
        #include <mach/thread_policy.h>
        #include <mach/mach_time.h>
    #endif
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket close
    using SOCKET = int;
#endif

// M_PI is a POSIX extension, not standard C++ -- MSVC's <cmath> omits it unless
// _USE_MATH_DEFINES is set. Provide a portable fallback (used by the DSP filter
// design below).
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

namespace {
    constexpr uint32_t CMD_MAGIC = 0xDEADBEEF;   // command-protocol magic

    // ------------------------------------------------------------------------
    // UNIFIED single-port packet format (matches mz-unified-ports firmware,
    // branch claude/unified-ports; authoritative spec docs/unified-packet-
    // format.md, host reference remote/net.py UnifiedSink). EVERY PL->host
    // stream arrives on ONE UDP port (default 0x6800) and carries the same
    // 8-word little-endian common header; the host demuxes by stream_type.
    //   w0 MAGIC = 0xCAFEBABE
    //   w1 TYPE_VER = stream_type[7:0] | version[15:8] | flags[31:16]
    //   w2/w3 = 64-bit master timestamp
    //   w4 = per-stream SEQ (+1 per packet of that stream -- the loss check)
    //   w5 = AUX0  (stream-specific)
    //   w6 = AUX1  (stream-specific)
    //   w7 = RSVD
    constexpr uint32_t UNIFIED_MAGIC          = 0xCAFEBABE;
    constexpr uint8_t  UNIFIED_VERSION        = 1;
    constexpr size_t   COMMON_HEADER_WORDS    = 8;   // the 8 shared header words
    constexpr uint8_t  STREAM_TYPE_BROADBAND  = 1;
    constexpr uint8_t  STREAM_TYPE_LFP        = 2;
    constexpr uint8_t  STREAM_TYPE_IMU        = 3;   // BNO055 side channel
    // A fabric swap reads a ~4 MB bitstream off the SD card and programs the
    // PL through PCAP. net.py allows 15 s for the same command.
    constexpr uint32_t kFabricLoadTimeoutMs   = 20000;
    // IMU packet: 8-word common header + 5 payload words (10 LE int16).
    constexpr size_t   IMU_PACKET_BYTES       = 52;
    // BNO055 fixed-point scale factors (data sheet section 3.6.4).
    constexpr float    BNO055_QUAT_SCALE      = 1.0f / 16384.0f;
    constexpr float    BNO055_ACCEL_SCALE     = 1.0f / 100.0f;   // -> m/s^2
    constexpr float    BNO055_GYRO_SCALE      = 1.0f / 16.0f;    // -> deg/s
    constexpr uint8_t  STREAM_TYPE_WAVELET    = 3;   // reserved (follow-on branch)

    constexpr size_t CMD_PACKET_SIZE = 20;
    constexpr size_t ACK_PACKET_SIZE = 3;
    // Firmware status response tiers (we require the current full size):
    //   86  bytes  pre-aux firmware
    //   98  bytes  aux-seq-v2 (aux sequencer block)
    //  122  bytes  + DMA/perf instrumentation (fw 1.1.0.0)
    //  126  bytes  + aux_ctrl (CTRL_REG_22 readback)        -- 65d5fb5
    //  148  bytes  + rhd_reg[22] mirror (commanded chip cfg) -- 7fb41dc
    //  160  bytes  + LFP/DSP engine config + status (re-added with the on-PL engine)
    //  288  bytes  + chirp NCO, recv->transmit spike stats, TX drop diagnostics
    // 288 B is the current aux + LFP firmware (fw 2.1.0.0). Optional blocks are
    // gated by their offset threshold, so the parser accepts any response >= the
    // minimum (rhd_reg at 148) and decodes what the device actually sent. This
    // plugin reads through the LFP config block (160); the appended chirp / spike /
    // TX-drop tail (160..288) is left unread. The buffer is oversized so leftover
    // bytes can't sit in the TCP queue and corrupt the next command's ACK.
    constexpr size_t STATUS_RESPONSE_SIZE = 512;       // receive buffer, with room to grow
    constexpr size_t STATUS_RESPONSE_SIZE_LFP = 160;   // through the LFP engine config block
    constexpr size_t STATUS_RESPONSE_SIZE_RHD = 148;   // through rhd_reg[22] (minimum accepted)
    constexpr size_t STATUS_RESPONSE_SIZE_AUXCTRL = 126;
    constexpr size_t STATUS_RESPONSE_SIZE_PERF = 122;
    constexpr size_t STATUS_RESPONSE_SIZE_AUX = 98;

    // CTRL_REG_AUX_CTRL bit layout (mirrors firmware/include/main.h).
    constexpr uint32_t AUX_CTRL_SEQ_EN              = 1u << 0;
    constexpr uint32_t AUX_CTRL_FS_SW               = 1u << 4;
    constexpr uint32_t AUX_CTRL_FS_GPIO_EN          = 1u << 5;
    constexpr int      AUX_CTRL_FS_GPIO_SEL_SHIFT   = 6;
    constexpr uint32_t AUX_CTRL_DSP_SW              = 1u << 9;
    constexpr uint32_t AUX_CTRL_DSP_GPIO_EN         = 1u << 10;
    constexpr int      AUX_CTRL_DSP_GPIO_SEL_SHIFT  = 11;
    constexpr uint32_t AUX_CTRL_DIGOUT_SW           = 1u << 14;
    constexpr uint32_t AUX_CTRL_DIGOUT_GPIO_EN      = 1u << 15;
    constexpr int      AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT = 16;
    constexpr int      AUX_CTRL_REG3_STATIC_SHIFT   = 24;

    // Command IDs
    enum CommandId : uint32_t {
        CMD_START = 0x01,
        CMD_STOP = 0x02,
        CMD_RESET_TIMESTAMP = 0x03,
        CMD_SET_LOOP_COUNT = 0x10,
        CMD_SET_PHASE = 0x11,
        CMD_SET_DEBUG_MODE = 0x12,
        CMD_SET_CHANNEL_ENABLE = 0x13,
        CMD_SET_PHASE_B = 0x14,      // port B (second cable) CIPO phase (phase2, phase3)
        CMD_LOAD_CONVERT = 0x20,
        CMD_LOAD_INIT = 0x21,
        CMD_LOAD_CABLE_TEST = 0x22,
        CMD_FULL_CABLE_TEST = 0x30,
        CMD_GET_STATUS = 0x40,
        CMD_SET_UDP_DEST = 0x50,
        // Aux command sequencer / override layer (firmware aux-seq-v2;
        // mirrors firmware/src-core0/network.c + remote/net.py)
        CMD_AUX_WRITE_WORD = 0x70,   // p1 = slot | bank<<8 | is_len<<16; p2 = addr<<16 | data
        CMD_AUX_BANK_SELECT = 0x71,  // p1 = slot; p2 = bank (confirmed before ACK)
        // 0x72 retired (was CMD_AUX_SEQ_EN): the aux command engine is always on
        CMD_READ_REGISTER = 0x73,    // p1 = reg -> 4-byte {cipo1,cipo0} response
        CMD_WRITE_REGISTER = 0x74,   // p1 = reg; p2 = value -> 4-byte echo response
        CMD_SET_FAST_SETTLE = 0x75,  // p1 = amp: sw|gpio_en<<1|pin<<4; p2 = dsp: same layout
        CMD_SET_DIGOUT = 0x76,       // p1 = sw|gpio_en<<1|pin<<4; p2 = reg3_static byte
        // LFP/DSP engine (firmware 0e99881 / fw >= 1.2)
        CMD_LFP_ENABLE = 0x80,       // p1 = 0/1
        CMD_LFP_SET_PARAMS = 0x81,   // p1 ignored (decimation is structural), p2 = num_taps
        CMD_LFP_SET_CHANNELS = 0x82, // p1 = 8-bit lane mask
        CMD_LFP_WRITE_COEF = 0x83,   // p1 bit0 = clear-ptr-first, bit1 = stage (0=halfband,1=decimator); p2 = 18-bit signed coef
        CMD_DETECT_IMU = 0xB0,       // -> 12-byte {result_a, result_b, version}
        CMD_SET_CONFIG = 0xB4,       // p1 = fabric selector; PCAP-swaps the whole PL
        CMD_PL_STATUS  = 0xB5,       // -> {int32 config, uint32 flags}; works in ANY state
        CMD_IMU_STREAM = 0xB8        // p1 = port mask (absolute), p2 = period ms -> 12-byte reply
    };
    
    // ACK status codes
    constexpr uint8_t ACK_SUCCESS = 0x06;
    constexpr uint8_t ACK_ERROR = 0x15;
    
    // Broadband packet header (UNIFIED format): the 8-word common header PLUS a
    // 6-word broadband sub-block = 14 header words ahead of the data words (the
    // data words are unchanged by the header reformat). See net.py
    // BB_HEADER_WORDS = 14 and docs/unified-packet-format.md "As implemented".
    //   w0..w7  = common header (above)
    //   w8      = sub-block: prev-packet fs/inject-slot aux echoes (the sweep-slot
    //             echo is in w6[31:16], paired intra-packet with data word 34)
    //   w9..w12 = sub-block: 8 external-ADC breadcrumbs (currently 0)
    //   w13     = sub-block: reserved
    //   w14..   = DATA words
    constexpr size_t PACKET_HEADER_WORDS = 14;

    // Cable detection slices the data section at the broadband header boundary,
    // which MUST equal the real header size. The RTL always writes a 14-word
    // header (7 fixed FIFO header writes, mask 0x0F -> 14 BRAM words) regardless
    // of channel_enable, and the chip DATA words (the INTAN/chip-ID/MISO readback)
    // begin at word 14 -- confirmed in data_generator_core.sv. The old value 8
    // was a leftover from before the unified header grew from 10 to 14: it made
    // the scorer read 6 header/sub-block words (aux echoes, ADC breadcrumbs) as
    // data and mis-locate every lane, so RESCAN never scored above threshold.
    // (net.py's `packet[8:]` had the identical bug.) Derive it from the header
    // size so this can never silently drift again when the header changes.
    constexpr size_t CABLE_TEST_DATA_OFFSET_WORDS = PACKET_HEADER_WORDS;
    
    // Auto-detection constants
    constexpr uint16_t INTAN_PATTERN[] = {0x0049, 0x004E, 0x0054, 0x0041, 0x004E};  // 'I','N','T','A','N'
    constexpr size_t INTAN_PATTERN_SIZE = 5;
    
    constexpr uint16_t CHIP_ID_RHD2164 = 4;  // 64-channel with DDR
    constexpr uint16_t CHIP_ID_RHD2132 = 1;  // 32-channel without DDR
    constexpr uint16_t CHIP_ID_RHD2216 = 2;  // 16-channel
    
    constexpr uint16_t MISO_REG_DDR = 0x35;     // Regular word when DDR available
    constexpr uint16_t MISO_DDR_DDR = 0x3A;     // DDR word when DDR available
    constexpr uint16_t MISO_NO_DDR = 0x00;      // When no DDR
    
    // Reference only (detection sizes packets from the live channel_enable via
    // expectedPacketSizeWords_). With ce=0x0F this is 14-word unified header +
    // 70 data = 84 words (matches net.py CABLE_TEST_PACKET_SIZE_WORDS); ce=0xFF
    // makes it 154. scoreChannel slices from CABLE_TEST_DATA_OFFSET_WORDS, so it
    // is size-agnostic.
    constexpr size_t CABLE_TEST_PACKET_SIZE_WORDS = 84;
    constexpr double DETECTION_THRESHOLD = 60.0;
    constexpr int NUM_PHASES_TO_TEST = 16;

    // Firmware (ab14c19) holds a one-packet guard band between the PL write
    // frontier and the PS read frontier (cross-clock read-during-write fix).
    // Worse: handle_enable_streaming() resyncs ps_read_address to the MOST
    // RECENT magic in BRAM, so a finite loop_count that stops before that
    // resync leaves ps_read at the last packet -- packets_available = 0 and
    // no UDP is ever emitted. Use loop_count=0 (infinite); the PL keeps writing,
    // PS drains continuously, and CMD_STOP halts it after we've captured.
    // RTL: `loop_limit_reached <= (loop_count != 0) && (counter >= loop_count)`,
    // so 0 is the "no limit" sentinel.
    constexpr uint32_t DETECTION_LOOP_COUNT = 0;
    // Cable test uses ce=0xFF (both ports, all 8 streams) so ports A and B are
    // detected in PARALLEL: each SPI cycle emits 4 words, one per lane
    // (A-CIPO0, A-CIPO1, B-CIPO0, B-CIPO1) at stride 4 / offsets 0..3; 35 cycles
    // -> 140 data words, so the packet is 14-word unified header + 140 = 154 words.
    constexpr uint8_t  DETECTION_CHANNEL_ENABLE = 0xFF;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

namespace {
    std::string ipToString(uint32_t ip) {
        struct in_addr addr;
        addr.s_addr = ip;
        return inet_ntoa(addr);
    }
    
    uint32_t stringToIp(const std::string& ipStr) {
        struct in_addr addr;
        if (inet_pton(AF_INET, ipStr.c_str(), &addr) == 1) {
            return addr.s_addr;
        }
        return 0;
    }
    
    void packU32LE(uint8_t* buf, uint32_t val) {
        buf[0] = val & 0xFF;
        buf[1] = (val >> 8) & 0xFF;
        buf[2] = (val >> 16) & 0xFF;
        buf[3] = (val >> 24) & 0xFF;
    }
    
    uint32_t unpackU32LE(const uint8_t* buf) {
        return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    }
    
    uint16_t unpackU16BE(const uint8_t* buf) {
        return (buf[0] << 8) | buf[1];
    }

    // Signed 16-bit little-endian (the IMU payload's native form). Built up
    // through uint16_t so the sign conversion is implementation-defined
    // nowhere: shifting into the sign bit of a signed type is not portable.
    int16_t unpackI16LE(const uint8_t* buf) {
        return (int16_t)(uint16_t)(buf[0] | (buf[1] << 8));
    }
}

// ============================================================================
// IMPLEMENTATION CLASS
// ============================================================================

class IntanInterface::Impl {
public:
    Impl(const std::string& deviceIp, uint16_t tcpPort, uint16_t udpPort)
        : deviceIp_(deviceIp)
        , tcpPort_(tcpPort)
        , udpPort_(udpPort)
        , tcpSocket_(INVALID_SOCKET)
        , udpSocket_(INVALID_SOCKET)
        , running_(false)
        , connected_(false)
        , currentChannelEnable_(0x0F)
        // 0x0F (4 streams) = 14-word unified header + 70 data words = 84 words.
        , expectedPacketSizeWords_(84)
        , totalPackets_(0)
        , totalErrors_(0)
        , magicErrors_(0)
        , timestampErrors_(0)
        , sizeErrors_(0)
        , lastTimestamp_(0)
        , lastStatsTime_(std::chrono::steady_clock::now())
        , lastPacketCount_(0)
        , captureForDetection_(false)
        , maxDetectionQueueSize_(20)
    {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("Failed to initialize Winsock");
        }
#endif
        
        // Connect TCP
        if (!connectTcp()) {
            throw std::runtime_error("Failed to establish TCP connection to " + deviceIp_);
        }
        
        // Auto-configure UDP destination
        // Bring up an acquisition fabric FIRST. The board boots blank, and a
        // blank PL refuses every command that touches acquisition registers --
        // which used to make a fresh reboot look like a dead board: the UDP
        // destination was NACKed and the initial getStatus failed.
        ensureAcquisitionFabric();

        autoConfigureUdp();

        // Start the UNIFIED UDP listener: ONE socket on udpPort_ (0x6800) carries
        // ALL streams (broadband + LFP), demuxed by stream_type. Two threads:
        //   * recvThread_  -- the hot path: recvfrom -> ring, nothing else, so
        //                     broadband is NEVER blocked while a slow consumer
        //                     processes a packet (the no-loss drain rule).
        //   * demuxThread_ -- pops the ring, peeks stream_type, routes broadband
        //                     to the data callback and LFP to the LFP callback.
        running_ = true;
        recvThread_  = std::thread(&Impl::udpRecvThread, this);
        demuxThread_ = std::thread(&Impl::udpDemuxThread, this);

        // Give the listener a moment to bind so the device's reply about
        // udp_dest reflects post-autoConfigureUdp state.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get initial status to sync channel enable
        DeviceStatus status;
        if (getStatusInternal(status)) {
            updateChannelEnable(status.channelEnable);
            std::cout << "[GLANCE] firmware reports udp_dest "
                      << status.udpDestIp << ":" << status.udpDestPort
                      << ", channel_enable=0x" << std::hex
                      << static_cast<int>(status.channelEnable) << std::dec
                      << ", fw " << status.getFirmwareVersionString()
                      << std::endl;
        } else {
            std::cout << "[GLANCE] WARN: initial getStatus failed"
                      << std::endl;
        }
    }
    
    ~Impl() {
        shutdown();
    }
    
    void shutdown() {
        running_ = false;

        // Wake the demux thread if it's blocked on an empty ring so it can see
        // running_ == false and exit promptly.
        ringCv_.notify_all();

        if (recvThread_.joinable()) {
            recvThread_.join();
        }
        if (demuxThread_.joinable()) {
            demuxThread_.join();
        }

        if (tcpSocket_ != INVALID_SOCKET) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
        }

        if (udpSocket_ != INVALID_SOCKET) {
            closesocket(udpSocket_);
            udpSocket_ = INVALID_SOCKET;
        }

#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    bool isConnected() const {
        return connected_.load();
    }
    
    bool isReady() const {
        if (!connected_.load() || !running_.load()) {
            return false;
        }
        
        // Check device status
        DeviceStatus status;
        if (!getStatusInternal(status)) {
            return false;
        }
        
        // Device should not be transmitting
        return !status.transmissionActive;
    }
    
    // Raise the receive timeout for the life of one command, then put it back.
    // Almost every command answers in microseconds, so the socket's default is
    // deliberately short; a fabric swap is the exception -- it reads megabytes
    // off the SD card and programs the PL, taking seconds.
    struct ScopedRecvTimeout {
        SOCKET sock;
        bool applied = false;
        ScopedRecvTimeout(SOCKET s, uint32_t ms) : sock(s) {
            if (ms == 0 || s == INVALID_SOCKET) return;
#ifdef _WIN32
            DWORD tv = ms;
#else
            struct timeval tv;
            tv.tv_sec  = (long)(ms / 1000);
            tv.tv_usec = (long)((ms % 1000) * 1000);
#endif
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                       reinterpret_cast<const char*>(&tv), sizeof(tv));
            applied = true;
        }
        ~ScopedRecvTimeout() {
            if (!applied) return;
#ifdef _WIN32
            DWORD tv = 3000;
#else
            struct timeval tv;
            tv.tv_sec = 3; tv.tv_usec = 0;
#endif
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                       reinterpret_cast<const char*>(&tv), sizeof(tv));
        }
    };

    bool sendCommand(CommandId cmdId, uint32_t param1 = 0, uint32_t param2 = 0,
                     uint8_t* responseData = nullptr, size_t* responseLen = nullptr,
                     uint32_t timeoutMs = 0) {
        std::lock_guard<std::mutex> lock(tcpMutex_);

        if (tcpSocket_ == INVALID_SOCKET) {
            return false;
        }

        ScopedRecvTimeout recvTimeout(tcpSocket_, timeoutMs);

        // Build command packet
        uint8_t cmd[CMD_PACKET_SIZE];
        static uint32_t ackId = 1;
        
        packU32LE(&cmd[0], CMD_MAGIC);
        packU32LE(&cmd[4], cmdId);
        packU32LE(&cmd[8], ackId);
        packU32LE(&cmd[12], param1);
        packU32LE(&cmd[16], param2);
        
        // Send command
        if (send(tcpSocket_, reinterpret_cast<char*>(cmd), CMD_PACKET_SIZE, 0) != CMD_PACKET_SIZE) {
            reportError("Failed to send TCP command");
            markDisconnected();
            return false;
        }

        // Receive response. SO_RCVTIMEO on the socket means recv() returns
        // <0 with errno EAGAIN / EWOULDBLOCK if the board doesn't respond
        // within the timeout; recv() returns 0 if the connection was closed.
        // In either case we mark the link dead so subsequent commands fail
        // fast and the host can disconnect / reconnect cleanly.
        uint8_t response[5];
        int received = recv(tcpSocket_, reinterpret_cast<char*>(response), 5, 0);

        if (received < 3) {
            if (received < 0) {
                reportError("TCP command timed out (no response from board)");
            } else if (received == 0) {
                reportError("TCP connection closed by board");
            } else {
                reportError("Failed to receive command response");
            }
            markDisconnected();
            return false;
        }
        
        // Validate ACK
        uint16_t recvAckId = unpackU16BE(&response[0]);
        uint8_t status = response[2];
        
        if (recvAckId != ackId) {
            reportError("ACK ID mismatch");
            return false;
        }
        
        if (status != ACK_SUCCESS) {
            reportError("Command failed on device");
            return false;
        }
        
        ackId++;
        
        // Check for data response
        if (received == 5 && responseData && responseLen) {
            uint16_t dataLen = unpackU16BE(&response[3]);
            if (dataLen > 0 && dataLen <= *responseLen) {
                int dataReceived = recv(tcpSocket_, reinterpret_cast<char*>(responseData), dataLen, 0);
                if (dataReceived == dataLen) {
                    *responseLen = dataLen;
                    return true;
                }
                // Short read or recv error: connection is in a bad state.
                reportError("TCP data response truncated or timed out");
                markDisconnected();
                return false;
            } else if (dataLen > 0) {
                // Firmware is sending more bytes than the caller's buffer can
                // hold (newer firmware revision). Drain them so the next command's
                // ACK header isn't corrupted, then report failure for this call.
                uint8_t scratch[256];
                uint16_t remaining = dataLen;
                while (remaining > 0) {
                    int chunk = remaining > sizeof(scratch) ? (int)sizeof(scratch) : remaining;
                    int got = recv(tcpSocket_, reinterpret_cast<char*>(scratch), chunk, 0);
                    if (got <= 0) break;
                    remaining -= (uint16_t)got;
                }
                *responseLen = 0;
                return false;
            }
        }

        return true;
    }
    
    bool getStatusInternal(DeviceStatus& status) const {
        // Need to cast away const for mutex, but logically this is a const operation
        auto* self = const_cast<Impl*>(this);
        
        uint8_t data[STATUS_RESPONSE_SIZE];
        size_t dataLen = STATUS_RESPONSE_SIZE;

        if (!self->sendCommand(CMD_GET_STATUS, 0, 0, data, &dataLen)) {
            return false;
        }

        // Require the full current status (see STATUS_RESPONSE_SIZE comment). A
        // larger response from newer firmware still parses (optional blocks are
        // gated by length); anything smaller is rejected.
        if (dataLen < STATUS_RESPONSE_SIZE_RHD) {
            return false;
        }
        
        // Parse response structure
        const uint8_t* p = data;
        
        status.protocolVersion = unpackU32LE(p) & 0xFFFF; p += 2;
        status.deviceType = unpackU32LE(p) & 0xFFFF; p += 2;
        status.firmwareVersion = unpackU32LE(p); p += 4;
        
        status.timestamp = static_cast<uint64_t>(unpackU32LE(p)) | 
                          (static_cast<uint64_t>(unpackU32LE(p + 4)) << 32); p += 8;
        status.packetsSent = unpackU32LE(p); p += 4;
        status.bramWriteAddr = unpackU32LE(p); p += 4;
        status.fifoCount = unpackU32LE(p) & 0xFFFF; p += 2;
        status.stateCounter = *p++; 
        status.cycleCounter = *p++;
        uint8_t flagsPl = *p++;
        p++; // reserved
        
        status.transmissionActive = (flagsPl & 0x01) != 0;
        status.loopLimitReached = (flagsPl & 0x02) != 0;
        
        status.packetsReceived = unpackU32LE(p); p += 4;
        status.errorCount = unpackU32LE(p); p += 4;
        status.udpPacketsSent = unpackU32LE(p); p += 4;
        status.udpSendErrors = unpackU32LE(p); p += 4;
        status.psReadAddr = unpackU32LE(p); p += 4;
        status.packetSizeWords = unpackU32LE(p); p += 4;
        uint8_t flagsPs = *p++;
        p += 3; // reserved
        
        status.streamEnabled = (flagsPs & 0x01) != 0;
        
        status.loopCount = unpackU32LE(p); p += 4;
        status.phase0 = *p++;
        status.phase1 = *p++;
        status.channelEnable = *p++;
        status.debugMode = *p++;
        p += 8; // reserved
        
        uint32_t udpDestIp = unpackU32LE(p); p += 4;
        status.udpDestIp = ipToString(udpDestIp);
        status.udpDestPort = unpackU32LE(p) & 0xFFFF; p += 2;
        p += 2; // format
        status.udpBytesSent = unpackU32LE(p); p += 4;

        // Aux sequencer status block (aux-seq-v2 firmware and later)
        status.hasAuxStatus = (dataLen >= STATUS_RESPONSE_SIZE_AUX);
        status.auxSeqEnabled = false;
        status.fastSettleActive = false;
        status.digoutState = false;
        status.dspResetActive = false;
        status.auxBankActive = 0;
        status.auxIndex[0] = status.auxIndex[1] = status.auxIndex[2] = 0;
        status.auxReadResult = 0;
        if (status.hasAuxStatus) {
            status.auxReadResult = unpackU32LE(p); p += 4;
            status.auxBankActive = *p++;
            uint8_t auxFlags = *p++;
            status.auxSeqEnabled    = (auxFlags & 0x01) != 0;
            status.fastSettleActive = (auxFlags & 0x02) != 0;
            status.digoutState      = (auxFlags & 0x04) != 0;
            status.dspResetActive   = (auxFlags & 0x08) != 0;
            status.auxIndex[0] = *p++;
            status.auxIndex[1] = *p++;
            status.auxIndex[2] = *p++;
            p += 3;  // 3 reserved bytes
        }

        // DMA / perf instrumentation block (firmware 1.1.0.0+, status >= 122)
        // -- we don't expose the fields in DeviceStatus yet, just skip past.
        if (dataLen >= STATUS_RESPONSE_SIZE_PERF) {
            p += 24;
        }

        // CTRL_REG_22 (aux_ctrl) readback (firmware 65d5fb5+, status >= 126).
        // Decode the fast-settle / DSP / digout config so the editor can sync
        // its TTL combo to whatever the device is actually in.
        status.hasAuxCtrl = false;
        status.auxCtrlRaw = 0;
        status.fsSwLevel = status.fsGpioEn = false;     status.fsGpioPin = 0;
        status.dspSwLevel = status.dspGpioEn = false;   status.dspGpioPin = 0;
        status.digoutSwLevel = status.digoutGpioEn = false; status.digoutGpioPin = 0;
        status.reg3Static = 0;
        if (dataLen >= STATUS_RESPONSE_SIZE_AUXCTRL) {
            uint32_t auxCtrl = unpackU32LE(p); p += 4;
            status.hasAuxCtrl = true;
            status.auxCtrlRaw = auxCtrl;
            status.fsSwLevel     = (auxCtrl & AUX_CTRL_FS_SW) != 0;
            status.fsGpioEn      = (auxCtrl & AUX_CTRL_FS_GPIO_EN) != 0;
            status.fsGpioPin     = (auxCtrl >> AUX_CTRL_FS_GPIO_SEL_SHIFT) & 0x7;
            status.dspSwLevel    = (auxCtrl & AUX_CTRL_DSP_SW) != 0;
            status.dspGpioEn     = (auxCtrl & AUX_CTRL_DSP_GPIO_EN) != 0;
            status.dspGpioPin    = (auxCtrl >> AUX_CTRL_DSP_GPIO_SEL_SHIFT) & 0x7;
            status.digoutSwLevel = (auxCtrl & AUX_CTRL_DIGOUT_SW) != 0;
            status.digoutGpioEn  = (auxCtrl & AUX_CTRL_DIGOUT_GPIO_EN) != 0;
            status.digoutGpioPin = (auxCtrl >> AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT) & 0x7;
            status.reg3Static    = (auxCtrl >> AUX_CTRL_REG3_STATIC_SHIFT) & 0xFF;
        }

        // RHD chip register mirror (firmware 7fb41dc+, status >= 148).
        status.hasRhdRegMirror = false;
        std::memset(status.rhdReg, 0, sizeof(status.rhdReg));
        if (dataLen >= STATUS_RESPONSE_SIZE_RHD) {
            std::memcpy(status.rhdReg, p, 22);
            p += 22;
            status.hasRhdRegMirror = true;
        }

        // LFP/DSP engine config + status (firmware 0e99881+, status >= 160).
        status.hasLfpStatus = false;
        status.lfpEnabled = false;
        status.lfpLaneMask = 0;
        status.lfpDecimR = 0;
        status.lfpNumTaps = 0;
        status.lfpPacketsSent = 0;
        status.lfpOverrun = false;
        if (dataLen >= STATUS_RESPONSE_SIZE_LFP) {
            status.hasLfpStatus = true;
            status.lfpEnabled  = (*p++) != 0;
            status.lfpLaneMask = *p++;
            status.lfpDecimR   = *p++;
            status.lfpNumTaps  = *p++;
            status.lfpPacketsSent = unpackU32LE(p); p += 4;
            status.lfpOverrun  = (*p++) != 0;
            p += 3;  // reserved
        }

        return true;
    }

    // ========================================================================
    // AUX COMMAND SEQUENCER
    // ========================================================================

    bool auxUploadBank(int slot, int bank, const std::vector<uint16_t>& commands,
                       int loopIndex) {
        if (commands.empty() || commands.size() > 64 ||
            loopIndex < 0 || loopIndex >= (int)commands.size()) {
            reportError("auxUploadBank: bad program size/loop index");
            return false;
        }
        uint32_t p1 = (slot & 3) | ((bank & 1) << 8);
        for (size_t i = 0; i < commands.size(); ++i) {
            uint32_t p2 = ((uint32_t)i << 16) | commands[i];
            if (!sendCommand(CMD_AUX_WRITE_WORD, p1, p2)) {
                reportError("auxUploadBank: word upload failed");
                return false;
            }
        }
        // Length record: loop index in [5:0], end index in [13:8]
        uint32_t lengthData = (loopIndex & 0x3F) |
                              (((uint32_t)(commands.size() - 1) & 0x3F) << 8);
        return sendCommand(CMD_AUX_WRITE_WORD, p1 | (1u << 16), lengthData);
    }

    bool auxBankSelect(int slot, int bank) {
        // Firmware polls bank_active (up to ~50 ms) before ACKing
        return sendCommand(CMD_AUX_BANK_SELECT, slot & 3, bank & 1);
    }

    bool setFastSettle(bool softwareLevel, bool gpioEnable, uint8_t gpioPin,
                       bool dspReset) {
        uint32_t p1 = (softwareLevel ? 1 : 0) | (gpioEnable ? 2 : 0)
                    | ((uint32_t)(gpioPin & 7) << 4);
        // When dspReset is requested, mirror the *entire* fast-settle config
        // into the DSP fields (same layout: bit0 = SW, bit1 = GPIO_EN, [6:4] =
        // pin sel). That way the DSP reset (CONVERT bit-H force) follows the
        // exact same trigger as the amplifier fast settle -- both fire together
        // on the SW level or the selected digital_in pin.
        uint32_t p2 = dspReset ? p1 : 0;
        return sendCommand(CMD_SET_FAST_SETTLE, p1, p2);
    }

    bool readRegister(uint8_t reg, uint16_t& cipo0Value, uint16_t& cipo1Value) {
        uint8_t data[4];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_READ_REGISTER, reg & 0x3F, 0, data, &dataLen))
            return false;
        if (dataLen != 4)
            return false;
        uint32_t result = unpackU32LE(data);
        cipo0Value = result & 0xFFFF;
        cipo1Value = (result >> 16) & 0xFFFF;
        return true;
    }
    
    // LFP / DSP engine (firmware 0e99881+). All commands are direct passes
    // through to the firmware -- the host owns the design (mirrors net.py's
    // configure_lfp(); see remote/net.py).
    bool lfpEnable(bool on) {
        return sendCommand(CMD_LFP_ENABLE, on ? 1 : 0);
    }
    bool lfpSetChannels(uint8_t laneMask) {
        return sendCommand(CMD_LFP_SET_CHANNELS, laneMask);
    }
    bool lfpSetParams(uint8_t decimR, uint8_t numTaps) {
        return sendCommand(CMD_LFP_SET_PARAMS, decimR, numTaps);
    }
    bool lfpWriteCoef(bool clearFirst, bool decimatorStage, int32_t coef18) {
        // Param1 bit1 selects the filter and must be held for every write,
        // including the pointer clear, or the reset and the writes land in
        // different filters. Param2 carries the 18-bit signed coefficient.
        uint32_t p1 = (clearFirst ? 1u : 0u) | (decimatorStage ? 2u : 0u);
        return sendCommand(CMD_LFP_WRITE_COEF, p1, (uint32_t)(coef18) & 0x3FFFFu);
    }
    bool lfpUploadCoefs(bool decimatorStage, const std::vector<int32_t>& coefs) {
        bool first = true;
        for (int32_t c : coefs) {
            if (!lfpWriteCoef(first, decimatorStage, c)) return false;
            first = false;
        }
        return true;
    }

    void setDataCallback(DataCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        dataCallback_ = callback;
    }

    void setLfpDataCallback(LfpDataCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        lfpDataCallback_ = callback;
    }

    void setImuDataCallback(ImuDataCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        imuDataCallback_ = callback;
    }

    // --- PL fabric (deferred boot) ------------------------------------------
    // The board boots with a BLANK programmable logic and loads a fabric on
    // command, so "connected" and "able to acquire" are different states. Any
    // command that touches acquisition registers is refused until a fabric is
    // live, which is why a fresh reboot used to fail at getStatus.
    bool plStatus(int& config, bool& isAcq) {
        uint8_t data[8];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_PL_STATUS, 0, 0, data, &dataLen) || dataLen < 8)
            return false;
        config = (int)unpackU32LE(data);
        isAcq  = (unpackU32LE(data + 4) & 0x1) != 0;
        return true;
    }

    // PCAP-swap the fabric. Multi-megabyte SD read + programming, so this is
    // seconds, not milliseconds -- the command socket's normal timeout is far
    // too short and the caller must expect to wait.
    bool setConfig(FabricConfig cfg) {
        uint8_t data[12];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_SET_CONFIG, (uint32_t)cfg, 0, data, &dataLen,
                         kFabricLoadTimeoutMs))
            return false;
        if (dataLen < 12) return false;
        return (int32_t)unpackU32LE(data) == 0;      // rc == 0
    }

    // Make the board ready to acquire, loading a fabric if it has none.
    // Mirrors net.py's `rescan`: census the IMUs on the scan fabric, then pick
    // the acquisition variant that matches what is actually plugged in, so a
    // headstage with an IMU keeps its I2C and one without keeps all 128
    // channels. Returns false only if no fabric could be brought up.
    // Census the IMUs and load the acquisition fabric that matches what is
    // actually plugged in. Always does the full dance, whatever is loaded now --
    // this is what RESCAN means, and it is the only way a headstage swapped
    // since connect gets noticed.
    bool rescanFabric(ImuPorts& present) {
        present.portA = present.portB = false;

        // The census has to happen on acq_imu_both: it is the only fabric with
        // I2C on BOTH ports, so an acq_imu_port_a fabric could never see a
        // headstage newly plugged into port B. It is also an acquisition
        // fabric with the UART routed, so the board's console stays alive
        // through the census.
        if (setConfig(FabricConfig::AcqImuBoth) && detectImu(present)) {
            std::cout << "[GLANCE] IMU census: port A "
                      << (present.portA ? "yes" : "no") << ", port B "
                      << (present.portB ? "yes" : "no") << std::endl;
        } else {
            std::cout << "[GLANCE] IMU census unavailable; assuming no IMUs"
                      << std::endl;
        }

        FabricConfig target =
            (present.portA && present.portB) ? FabricConfig::AcqImuBoth :
            present.portA                    ? FabricConfig::AcqImuPortA :
            present.portB                    ? FabricConfig::AcqImuPortB :
                                               FabricConfig::Acquisition;
        // Already on it when both ports answered -- skip the redundant load.
        if (target != FabricConfig::AcqImuBoth && !setConfig(target)) {
            reportError("Could not load an acquisition fabric onto the board");
            return false;
        }
        std::cout << "[GLANCE] fabric " << (int)target
                  << " -- board ready to acquire" << std::endl;
        return true;
    }

    bool ensureAcquisitionFabric() {
        int config = -1;
        bool isAcq = false;
        if (!plStatus(config, isAcq)) {
            // No PL_STATUS at all: firmware without the deferred-boot loader.
            // Nothing to arrange -- let the caller proceed as it always did.
            return true;
        }
        if (isAcq)
            return true;

        std::cout << "[GLANCE] board has no acquisition fabric loaded (PL is "
                  << (config < 0 ? "blank" : "on a scan fabric")
                  << "); bringing one up..." << std::endl;
        ImuPorts present;
        return rescanFabric(present);
    }

    // IMU control. detectImu is a one-shot probe (refused by the firmware on a
    // fabric without the AXI IICs, and while the IMU stream owns a port); the
    // stream commands mirror net.py imu_stream().
    bool detectImu(ImuPorts& present) {
        uint8_t data[12];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_DETECT_IMU, 0, 0, data, &dataLen) || dataLen != 12)
            return false;
        // Result bit 0 = present (ACKed AND chip_id == 0xA0).
        present.portA = (unpackU32LE(data) & 0x1) != 0;
        present.portB = (unpackU32LE(data + 4) & 0x1) != 0;
        return true;
    }

    bool setImuStream(const ImuPorts& ports, uint32_t periodMs, ImuPorts& active) {
        uint32_t mask = (ports.portA ? 1u : 0u) | (ports.portB ? 2u : 0u);
        uint8_t data[12];
        size_t dataLen = sizeof(data);
        if (!sendCommand(CMD_IMU_STREAM, mask, periodMs, data, &dataLen) || dataLen != 12)
            return false;
        uint32_t activeMask = unpackU32LE(data);
        active.portA = (activeMask & 1) != 0;
        active.portB = (activeMask & 2) != 0;
        if (mask == 0) {   // a stop resets the loss check: SEQ restarts at 0
            std::lock_guard<std::mutex> lock(statsMutex_);
            imuHaveSeq_[0] = imuHaveSeq_[1] = false;
        }
        return true;
    }

    void getImuStats(ImuStats& stats) const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        for (int p = 0; p < 2; ++p) {
            stats.samples[p]     = imuSamples_[p];
            stats.seqGaps[p]     = imuSeqGaps_[p];
            stats.lostSamples[p] = imuLostSamples_[p];
        }
    }

    void setErrorCallback(ErrorCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        errorCallback_ = callback;
    }
    
    void getReceptionStats(ReceptionStats& stats) const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - startTime_).count();
        auto recentElapsed = std::chrono::duration<double>(now - lastStatsTime_).count();
        
        stats.totalPackets = totalPackets_;
        stats.totalErrors = totalErrors_;
        stats.magicErrors = magicErrors_;
        stats.timestampErrors = timestampErrors_;
        stats.sizeErrors = sizeErrors_;
        stats.lastTimestamp = lastTimestamp_;
        
        stats.averageRate = (elapsed > 0) ? (totalPackets_ / elapsed) : 0.0;
        stats.instantRate = (recentElapsed > 0) ? 
            ((totalPackets_ - lastPacketCount_) / recentElapsed) : 0.0;
        
        size_t packetBytes = expectedPacketSizeWords_ * 4;
        stats.dataRateMbps = stats.averageRate * packetBytes * 8.0 / 1e6;
    }
    
    void updateChannelEnable(uint8_t channelEnable) {
        currentChannelEnable_ = channelEnable;
        expectedPacketSizeWords_ = IntanInterface::calculatePacketSize(channelEnable);
    }
    
    std::string getDeviceIp() const { return deviceIp_; }
    uint16_t getTcpPort() const { return tcpPort_; }
    uint16_t getUdpPort() const { return udpPort_; }
    
    // ========================================================================
    // AUTO-DETECTION METHODS
    // ========================================================================

    // Per-CIPO-lane best across the sweep. Used by runAutoDetection and
    // testPhaseParallel; declared once at class scope so both can share it.
    struct LaneBest {
        const char* label;     // "A.CIPO0", "A.CIPO1", "B.CIPO0", "B.CIPO1"
        int      stride;       // lane stride in data-word units
        int      offset;       // lane offset within each cycle's words
        double   bestScore;
        ChipType bestType;
        bool     bestHasDdr;
        uint8_t  bestPhase;
        bool     detected;
    };

    // One sweep over 16 phase values, both port-A and port-B phases ganged to
    // the same value, channel_enable = 0xFF so every packet carries CIPO0/CIPO1
    // for BOTH cables in the same lane positions. Each phase yields a single
    // packet; we score all four CIPO lanes from it and track per-port best.
    bool runAutoDetection(AutoDetectionResult& result, bool verbose) {
        result = AutoDetectionResult{};
        result.cipo0ChipType = ChipType::NONE;
        result.cipo1ChipType = ChipType::NONE;
        result.portBCipo0ChipType = ChipType::NONE;
        result.portBCipo1ChipType = ChipType::NONE;

        if (verbose) {
            std::cout << "[Detection] Starting automated chip detection (port A + port B, parallel)..." << std::endl;
        }

        DeviceStatus status;
        if (!getStatusInternal(status)) {
            if (verbose) std::cout << "[Detection] ERROR: Cannot read device status" << std::endl;
            return false;
        }
        if (status.transmissionActive) {
            if (verbose) std::cout << "[Detection] ERROR: Device is transmitting. Stop acquisition first." << std::endl;
            return false;
        }

        // initializeForDetection sets channel_enable=0xFF and loop_count=DETECTION_LOOP_COUNT
        if (!initializeForDetection(verbose)) {
            return false;
        }

        // Per-lane bookkeeping. Same struct as the method-level LaneBest below
        // so testPhaseParallel can update bestScore/bestType in place.
        LaneBest lanes[4] = {
            { "A.CIPO0", 4, 0, 0.0, ChipType::NONE, false, 0, false },
            { "A.CIPO1", 4, 1, 0.0, ChipType::NONE, false, 0, false },
            { "B.CIPO0", 4, 2, 0.0, ChipType::NONE, false, 0, false },
            { "B.CIPO1", 4, 3, 0.0, ChipType::NONE, false, 0, false },
        };

        for (int phase = 0; phase < NUM_PHASES_TO_TEST; ++phase) {
            std::array<double, 4>   scores{0,0,0,0};
            std::array<ChipType, 4> types{ChipType::NONE, ChipType::NONE, ChipType::NONE, ChipType::NONE};

            bool ok = testPhaseParallel(static_cast<uint8_t>(phase), lanes,
                                        scores, types, verbose);
            if (!ok) {
                continue;  // already logged "No packet received" in testPhaseParallel
            }

            // Per-lane track the best phase (independent across lanes; same value
            // of phase, but different CIPOs can prefer different phases when
            // cables differ in length).
            for (int i = 0; i < 4; ++i) {
                if (scores[i] > DETECTION_THRESHOLD && scores[i] > lanes[i].bestScore) {
                    lanes[i].bestScore   = scores[i];
                    lanes[i].bestType    = types[i];
                    lanes[i].bestHasDdr  = (types[i] == ChipType::RHD2164);
                    lanes[i].bestPhase   = static_cast<uint8_t>(phase);
                    lanes[i].detected    = true;
                }
            }

            if (verbose && (scores[0] + scores[1] + scores[2] + scores[3]) > 0) {
                std::cout << "  Phase " << phase << ": "
                          << "A.CIPO0=" << scores[0]
                          << " A.CIPO1=" << scores[1]
                          << " B.CIPO0=" << scores[2]
                          << " B.CIPO1=" << scores[3] << std::endl;
            }

            // Record the phase result for callers that want the raw matrix.
            PhaseTestResult pr;
            pr.phase = static_cast<uint8_t>(phase);
            pr.cipo0Score = scores[0]; pr.cipo1Score = scores[1];
            pr.cipo0HasDdr = (types[0] == ChipType::RHD2164);
            pr.cipo1HasDdr = (types[1] == ChipType::RHD2164);
            pr.cipo0ChipType = types[0]; pr.cipo1ChipType = types[1];
            result.allPhaseResults.push_back(pr);
        }

        // Port A: cipo0 + cipo1
        result.cipo0Detected     = lanes[0].detected;
        result.cipo0ChipType     = lanes[0].bestType;
        result.cipo0HasDdr       = lanes[0].bestHasDdr;
        result.cipo0Score        = lanes[0].bestScore;
        result.cipo1Detected     = lanes[1].detected;
        result.cipo1ChipType     = lanes[1].bestType;
        result.cipo1HasDdr       = lanes[1].bestHasDdr;
        result.cipo1Score        = lanes[1].bestScore;

        // Port B: cipo0 + cipo1
        result.portBCipo0Detected = lanes[2].detected;
        result.portBCipo0ChipType = lanes[2].bestType;
        result.portBCipo0HasDdr   = lanes[2].bestHasDdr;
        result.portBCipo1Detected = lanes[3].detected;
        result.portBCipo1ChipType = lanes[3].bestType;
        result.portBCipo1HasDdr   = lanes[3].bestHasDdr;

        // Pick one phase per port (the better of the port's two CIPO lanes).
        auto pickPhase = [](const LaneBest& a, const LaneBest& b) -> uint8_t {
            if (a.detected && b.detected) {
                return (a.bestScore >= b.bestScore) ? a.bestPhase : b.bestPhase;
            }
            if (a.detected) return a.bestPhase;
            if (b.detected) return b.bestPhase;
            return 0;
        };
        result.bestPhase0 = pickPhase(lanes[0], lanes[1]);
        result.bestPhase1 = pickPhase(lanes[2], lanes[3]);

        result.chipsDetected = result.cipo0Detected || result.cipo1Detected ||
                               result.portBCipo0Detected || result.portBCipo1Detected;
        result.success = result.chipsDetected;

        result.optimalChannelMask = 0;
        if (result.cipo0Detected) { result.optimalChannelMask |= 0x01; if (result.cipo0HasDdr) result.optimalChannelMask |= 0x02; }
        if (result.cipo1Detected) { result.optimalChannelMask |= 0x04; if (result.cipo1HasDdr) result.optimalChannelMask |= 0x08; }
        if (result.portBCipo0Detected) { result.optimalChannelMask |= 0x10; if (result.portBCipo0HasDdr) result.optimalChannelMask |= 0x20; }
        if (result.portBCipo1Detected) { result.optimalChannelMask |= 0x40; if (result.portBCipo1HasDdr) result.optimalChannelMask |= 0x80; }

        if (verbose) {
            if (result.success) {
                std::cout << "[Detection] Complete. Port A phase=" << (int)result.bestPhase0
                          << " port B phase=" << (int)result.bestPhase1
                          << " mask=0x" << std::hex << (int)result.optimalChannelMask
                          << std::dec << std::endl;
                std::cout << "[Detection] " << result.getChannelSummary() << std::endl;
            } else {
                std::cout << "[Detection] No chips detected on either port" << std::endl;
            }
        }
        return true;
    }

    // Set both phase pairs (port A: CMD_SET_PHASE; port B: CMD_SET_PHASE_B) to
    // `phase`, briefly run the cable-test sequence with loop_count buffered, and
    // score all four CIPO lanes out of the captured packet.
    bool testPhaseParallel(uint8_t phase,
                           LaneBest (&lanes)[4],
                           std::array<double, 4>& scoresOut,
                           std::array<ChipType, 4>& typesOut,
                           bool verbose) {
        // Stop, set both ports' phases, start, capture one packet, stop.
        sendCommand(CMD_STOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        if (!sendCommand(CMD_SET_PHASE,   phase, phase) ||
            !sendCommand(CMD_SET_PHASE_B, phase, phase)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        RxSnapshot before = rxSnapshot();
        startDetectionCapture();

        if (!sendCommand(CMD_START)) {
            stopDetectionCapture();
            return false;
        }

        std::vector<uint32_t> packet;
        bool captured = capturePacketForDetection(packet, 1.0);

        sendCommand(CMD_STOP);
        stopDetectionCapture();

        if (!captured) {
            if (verbose) {
                RxSnapshot after = rxSnapshot();
                std::cout << "  Phase " << static_cast<int>(phase)
                          << ": No packet received"
                          << "  (rx delta total=" << (after.total - before.total)
                          << " magicErr=" << (after.magic - before.magic)
                          << " sizeErr=" << (after.size - before.size)
                          << ", expectedBytes=" << (expectedPacketSizeWords_ * 4)
                          << ")" << std::endl;
            }
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            auto pr = scoreChannel(packet, lanes[i].stride, lanes[i].offset,
                                   lanes[i].label, verbose);
            scoresOut[i] = pr.first;
            typesOut[i]  = pr.second;
        }
        return true;
    }
    
    private:
    bool connectTcp() {
        tcpSocket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcpSocket_ == INVALID_SOCKET) {
            return false;
        }

        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(tcpPort_);

        if (inet_pton(AF_INET, deviceIp_.c_str(), &serverAddr.sin_addr) != 1) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
            return false;
        }

        // Non-blocking connect + select() with a bounded timeout, so a not-
        // yet-ready board (still booting its network stack) fails fast
        // instead of stalling the JUCE UI thread on the kernel's default TCP
        // connect timeout (minutes). The 15 s bound is long enough to cover
        // the common cases where macOS pauses on first-time ARP resolution
        // for the board's IP, but short enough that the user isn't sitting
        // in front of a frozen UI. If a prior attempt POISONED macOS's ARP
        // cache with a negative entry (clicking CONNECT before the board's
        // lwIP was up), even a fresh connect from ANY tool -- including
        // net.py -- will fail with EHOSTUNREACH for ~20 s. Either wait for
        // the cache to age out or run `sudo arp -d <BOARD_IP>` to clear it.
        constexpr int CONNECT_TIMEOUT_SEC = 15;
#ifdef _WIN32
        u_long nb = 1;
        ioctlsocket(tcpSocket_, FIONBIO, &nb);
#else
        int flags = fcntl(tcpSocket_, F_GETFL, 0);
        if (flags != -1)
            fcntl(tcpSocket_, F_SETFL, flags | O_NONBLOCK);
#endif

        int cr = connect(tcpSocket_,
                         reinterpret_cast<struct sockaddr*>(&serverAddr),
                         sizeof(serverAddr));
        if (cr == SOCKET_ERROR) {
#ifdef _WIN32
            int err = WSAGetLastError();
            const bool pending = (err == WSAEWOULDBLOCK || err == WSAEINPROGRESS);
#else
            const int err = errno;
            const bool pending = (err == EINPROGRESS || err == EWOULDBLOCK);
#endif
            if (!pending) {
                std::cout << "[GLANCE] connectTcp: connect() failed err="
                          << err
#ifndef _WIN32
                          << " (" << std::strerror(err) << ")"
                          << (err == EHOSTUNREACH
                              ? "  -- macOS ARP cache holds a negative entry"
                                " for this IP. Wait ~20 s or run: sudo arp -d "
                              : "")
                          << (err == EHOSTUNREACH ? deviceIp_ : std::string(""))
#endif
                          << std::endl;
                closesocket(tcpSocket_);
                tcpSocket_ = INVALID_SOCKET;
                return false;
            }

            fd_set wfds, efds;
            FD_ZERO(&wfds); FD_SET(tcpSocket_, &wfds);
            FD_ZERO(&efds); FD_SET(tcpSocket_, &efds);
            struct timeval tvc;
            tvc.tv_sec  = CONNECT_TIMEOUT_SEC;
            tvc.tv_usec = 0;
            int sel = select((int)tcpSocket_ + 1, nullptr, &wfds, &efds, &tvc);
            if (sel < 0) {
                std::cout << "[GLANCE] connectTcp: select() error errno="
                          << errno << std::endl;
                closesocket(tcpSocket_);
                tcpSocket_ = INVALID_SOCKET;
                return false;
            }
            if (sel == 0) {
                // Timeout. The board isn't listening yet -- most common cause
                // is trying to CONNECT before the Zynq's lwIP stack is up
                // (~5-20 s after boot). Try again in a couple seconds.
                std::cout << "[GLANCE] connectTcp: connect timeout after "
                          << CONNECT_TIMEOUT_SEC << "s to " << deviceIp_
                          << ":" << tcpPort_
                          << " (board still booting? retry in a few seconds)"
                          << std::endl;
                closesocket(tcpSocket_);
                tcpSocket_ = INVALID_SOCKET;
                return false;
            }

            // Check SO_ERROR to confirm connect() actually succeeded (select
            // returns writable on refused connections too).
            int soerr = 0;
            socklen_t soerrlen = sizeof(soerr);
            int goe = getsockopt(tcpSocket_, SOL_SOCKET, SO_ERROR,
                                 reinterpret_cast<char*>(&soerr), &soerrlen);
            if (goe != 0 || soerr != 0) {
                std::cout << "[GLANCE] connectTcp: SO_ERROR="
                          << soerr
#ifndef _WIN32
                          << " (" << std::strerror(soerr) << ")"
                          << (soerr == EHOSTUNREACH
                              ? "  -- macOS ARP cache holds a negative entry"
                                " for this IP. Wait ~20 s or run: sudo arp -d "
                              : "")
                          << (soerr == EHOSTUNREACH ? deviceIp_ : std::string(""))
#endif
                          << " (getsockopt rc=" << goe << ")" << std::endl;
                closesocket(tcpSocket_);
                tcpSocket_ = INVALID_SOCKET;
                return false;
            }
        }

        // Restore blocking mode for the rest of the session (send/recv use
        // SO_SNDTIMEO/SO_RCVTIMEO for their bounds).
#ifdef _WIN32
        u_long nb0 = 0;
        ioctlsocket(tcpSocket_, FIONBIO, &nb0);
#else
        if (flags != -1)
            fcntl(tcpSocket_, F_SETFL, flags);
#endif

        // Bound how long a single command can wait for a response. Without
        // this, recv() blocks indefinitely if the board's TCP stack stalls
        // (we've observed this when the board boot races plugin-side
        // CMD_GET_STATUS during a fresh connect). Normal responses are sub-
        // millisecond; 3 seconds gives plenty of headroom for any legitimate
        // command and means a stalled connection is detected promptly.
        struct timeval tv;
        tv.tv_sec  = 3;
        tv.tv_usec = 0;
        setsockopt(tcpSocket_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&tv), sizeof(tv));

        // OS-level dead-connection detection (independent of the recv timeout).
        int keep = 1;
        setsockopt(tcpSocket_, SOL_SOCKET, SO_KEEPALIVE,
                   reinterpret_cast<char*>(&keep), sizeof(keep));

        connected_ = true;
        return true;
    }
    
    void autoConfigureUdp() {
        // Get local IP that can reach device
        SOCKET tempSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (tempSock == INVALID_SOCKET) {
            std::cout << "[GLANCE] autoConfigureUdp: socket() failed" << std::endl;
            return;
        }

        struct sockaddr_in serverAddr;
        std::memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(tcpPort_);
        inet_pton(AF_INET, deviceIp_.c_str(), &serverAddr.sin_addr);

        bool ok = false;
        if (connect(tempSock, reinterpret_cast<struct sockaddr*>(&serverAddr),
                   sizeof(serverAddr)) == 0) {
            struct sockaddr_in localAddr;
            socklen_t addrLen = sizeof(localAddr);
            if (getsockname(tempSock, reinterpret_cast<struct sockaddr*>(&localAddr),
                           &addrLen) == 0) {
                char localIp[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &localAddr.sin_addr, localIp, sizeof(localIp));

                // Configure device to send UDP to us
                uint32_t ipInt = stringToIp(localIp);
                ok = sendCommand(CMD_SET_UDP_DEST, ntohl(ipInt), udpPort_);
                std::cout << "[GLANCE] autoConfigureUdp: telling device to send UDP to "
                          << localIp << ":" << udpPort_
                          << (ok ? "  (ACKed)" : "  (NACKed!)") << std::endl;
            } else {
                std::cout << "[GLANCE] autoConfigureUdp: getsockname failed" << std::endl;
            }
        } else {
            std::cout << "[GLANCE] autoConfigureUdp: connect() to "
                      << deviceIp_ << ":" << tcpPort_ << " failed" << std::endl;
        }

        closesocket(tempSock);
    }
    
    // ------------------------------------------------------------------------
    // UNIFIED UDP listener (ONE socket on udpPort_ = 0x6800; ALL streams).
    //
    // recvThread_ binds the socket with a big SO_RCVBUF and does the absolute
    // minimum on the hot path -- recvfrom into a ring -- so broadband is NEVER
    // dropped while a slow downstream consumer is busy (the no-loss drain rule
    // from CLAUDE.md / docs/unified-packet-format.md, matching net.py's
    // UnifiedSink._recv_loop). demuxThread_ pops the ring, peeks the common
    // header's stream_type (word 1, low byte) and routes broadband and LFP to
    // their handlers, each of which tracks that stream's per-packet SEQ (word 4)
    // = the loss check. The board sends exactly one packet per datagram.
    // ------------------------------------------------------------------------

    // Raise the calling thread's scheduling priority so the OS keeps it running
    // even when the machine is saturated (OpenEphys at ~300% CPU while the user
    // adds GUI events). BOTH the recv and demux threads call this: the recv thread
    // drains the socket, but the DEMUX thread does the real per-packet work (unpack
    // + SEQ + route at ~28k pkts/s), so if demux is left at normal priority it gets
    // starved, the recv->demux ring overflows, and packets are dropped -> exactly
    // the host-side SEQ gaps seen in OpenEphys but never in net.py. Best-effort:
    // privileged RT classes fall back gracefully if not permitted.
    static void raiseThreadPriority() {
#if defined(__APPLE__)
        // Promote to REAL-TIME time-constraint scheduling (CoreAudio's render-thread
        // mechanism; works unprivileged) so OE's GUI/render threads can't preempt the
        // socket drain -> kernel UDP overflow -> broadband SEQ gaps that net.py (no GUI)
        // never shows. CRITICAL: a thread that has a QoS class assigned is system-managed
        // and thread_policy_set(TIME_CONSTRAINT) is REJECTED on it. std::thread threads
        // are QOS_CLASS_UNSPECIFIED, so apply time-constraint directly and only fall back
        // to a QoS class if the kernel refuses. (The prior version set QoS first and the
        // RT policy silently failed.)
        kern_return_t kr = KERN_FAILURE;
        mach_timebase_info_data_t tb;
        if (mach_timebase_info(&tb) == KERN_SUCCESS && tb.numer != 0) {
            auto ms_to_abs = [&](double ms) -> uint32_t {
                return (uint32_t)((ms * 1.0e6) * tb.denom / tb.numer);   // ns -> mach abs
            };
            thread_time_constraint_policy_data_t pol;
            pol.period      = ms_to_abs(1.0);   // eligible to run ~every 1 ms
            pol.computation = ms_to_abs(0.3);   // guaranteed up to 0.3 ms CPU per period
            pol.constraint  = ms_to_abs(1.0);   // ...within a 1 ms deadline
            pol.preemptible = 0;                // don't let a render thread preempt mid-drain
            kr = thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                   THREAD_TIME_CONSTRAINT_POLICY,
                                   (thread_policy_t)&pol,
                                   THREAD_TIME_CONSTRAINT_POLICY_COUNT);
        }
        if (kr != KERN_SUCCESS)
            pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);   // fallback
        {
            // Log ONCE per thread (recv + demux) so it's visible whether RT engaged.
            static std::atomic<int> rtLogged { 0 };
            if (rtLogged.fetch_add(1) < 2)
                std::cout << "[GLANCE] recv/demux real-time scheduling: "
                          << (kr == KERN_SUCCESS ? "ENABLED (time-constraint)"
                                                 : "REJECTED -> QoS fallback")
                          << " (kr=" << (int)kr << ")" << std::endl;
        }
#elif defined(__linux__)
        struct sched_param sp; std::memset(&sp, 0, sizeof(sp));
        sp.sched_priority = sched_get_priority_min(SCHED_FIFO) + 1;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);   // needs privilege; ignored otherwise
#endif
    }

    void udpRecvThread() {
        // Give the receive thread HIGH scheduling priority so it keeps draining the
        // socket even when the machine is saturated (e.g. an OpenEphys GUI event
        // storm at ~300% CPU). See raiseThreadPriority(). BOTH the recv and demux
        // threads need this -- if EITHER starves, the recv->demux ring backs up and
        // we drop (a host-side broadband/LFP SEQ gap that net.py never shows).
        raiseThreadPriority();
        udpSocket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udpSocket_ == INVALID_SOCKET) {
            std::cout << "[GLANCE] UDP listener: socket() failed" << std::endl;
            reportError("Failed to create UDP socket");
            return;
        }

        // Allow rebinding the port even if a previous instance left it in
        // TIME_WAIT or another local process (e.g. net.py) holds it.
        int reuse = 1;
        setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));
#ifdef SO_REUSEPORT
        setsockopt(udpSocket_, SOL_SOCKET, SO_REUSEPORT,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));
#endif

        // Big receive buffer so the kernel never drops a datagram while we are
        // briefly off the socket (matches net.py's 16 MB SO_RCVBUF). The OS may
        // clamp this to net.core.rmem_max; we log what we actually got.
        int rcvbuf = 16 * 1024 * 1024;
        setsockopt(udpSocket_, SOL_SOCKET, SO_RCVBUF,
                   reinterpret_cast<char*>(&rcvbuf), sizeof(rcvbuf));
        int gotbuf = 0;
        socklen_t gotlen = sizeof(gotbuf);
        if (getsockopt(udpSocket_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<char*>(&gotbuf), &gotlen) == 0) {
            // Linux reports double the value it actually reserves.
            std::cout << "[GLANCE] UDP SO_RCVBUF = " << (gotbuf / 1024)
                      << " KB (requested " << (rcvbuf / 1024) << " KB)" << std::endl;
            // If the OS clamped it well below the request, the kernel drops UDP
            // under any brief stall -> broadband SEQ gaps that net.py (on a box
            // with a larger granted buffer) never sees. Surface it loudly with
            // the exact fix so it isn't silently the cause.
            if (gotbuf < rcvbuf) {
                std::cout << "[GLANCE] WARNING: SO_RCVBUF was CLAMPED to "
                          << (gotbuf / 1024) << " KB (< requested " << (rcvbuf / 1024)
                          << " KB). Raise the OS limit to avoid UDP drops under load: "
                          << "macOS  'sudo sysctl -w kern.ipc.maxsockbuf=33554432' ; "
                          << "Linux  'sudo sysctl -w net.core.rmem_max=16777216'."
                          << std::endl;
            }
        }

        // Bind to the unified UDP port.
        struct sockaddr_in bindAddr;
        std::memset(&bindAddr, 0, sizeof(bindAddr));
        bindAddr.sin_family = AF_INET;
        bindAddr.sin_addr.s_addr = INADDR_ANY;
        bindAddr.sin_port = htons(udpPort_);

        if (bind(udpSocket_, reinterpret_cast<struct sockaddr*>(&bindAddr),
                sizeof(bindAddr)) == SOCKET_ERROR) {
            std::cout << "[GLANCE] UDP listener: bind to port "
                      << udpPort_ << " FAILED (errno=" << errno << ")" << std::endl;
            reportError("Failed to bind UDP socket");
            closesocket(udpSocket_);
            udpSocket_ = INVALID_SOCKET;
            return;
        }

        std::cout << "[GLANCE] Unified UDP listener bound on port "
                  << udpPort_ << " (demux by stream_type: broadband + LFP on one port)"
                  << std::endl;

        // Short timeout so the thread sees running_ == false promptly on stop.
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(udpSocket_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&tv), sizeof(tv));

        startTime_ = std::chrono::steady_clock::now();

        // Thread-local buffer pools so the recv hot path never mallocs and never locks
        // per datagram: `spare` = recycled empty buffers to fill, `batch` = filled
        // datagrams to publish. Refilled from the demux-returned free-list in bulk.
        std::vector<Datagram> spare;
        std::vector<Datagram> batch;
        spare.reserve(kMaxRecvBatch);
        batch.reserve(kMaxRecvBatch);
        // Hand back a buffer whose `bytes` is sized to kDatagramCap. Recycled buffers
        // are already that size, so this resize is a no-op (no zero-fill); only a
        // freshly default-constructed Datagram pays the one-time grow.
        auto takeBuf = [&]() -> Datagram {
            Datagram d;
            if (!spare.empty()) { d = std::move(spare.back()); spare.pop_back(); }
            if (d.bytes.size() != kDatagramCap) d.bytes.resize(kDatagramCap);
            d.len = 0;
            return d;
        };

        while (running_.load()) {
            struct sockaddr_in senderAddr;
            socklen_t senderLen = sizeof(senderAddr);

            // (1) BLOCKING recv for the first datagram of the chunk. SO_RCVTIMEO makes
            //     this return ~once/sec when idle so we can re-check running_. recvfrom
            //     into the fixed kDatagramCap buffer; record the length, never resize.
            Datagram buffer = takeBuf();
            int received = recvfrom(udpSocket_, reinterpret_cast<char*>(buffer.bytes.data()),
                                    kDatagramCap, 0,
                                    reinterpret_cast<struct sockaddr*>(&senderAddr),
                                    &senderLen);
            if (received <= 0) {          // timeout / error
                spare.push_back(std::move(buffer));
                continue;
            }
            buffer.len = (size_t)received;
            batch.push_back(std::move(buffer));

            // (2) Drain every OTHER datagram already queued in the socket WITHOUT
            //     blocking -- this is the chunk. After a scheduling gap the whole
            //     backlog comes out in one pass (catch-up), capped at kMaxRecvBatch.
            while (batch.size() < kMaxRecvBatch) {
                Datagram b = takeBuf();
                int m = recvfrom(udpSocket_, reinterpret_cast<char*>(b.bytes.data()),
                                 kDatagramCap, MSG_DONTWAIT,
                                 reinterpret_cast<struct sockaddr*>(&senderAddr),
                                 &senderLen);
                if (m <= 0) {             // EAGAIN/EWOULDBLOCK: nothing more queued now
                    spare.push_back(std::move(b));
                    break;
                }
                b.len = (size_t)m;
                batch.push_back(std::move(b));
            }

            // (3) Publish the whole chunk under ONE lock + ONE notify, then top up the
            //     thread-local spare pool from the demux-returned free-list in bulk.
            {
                std::lock_guard<std::mutex> lock(ringMutex_);
                for (auto& d : batch) {
                    if (ring_.size() >= kRingMax) {
                        ringDrops_++;                                    // host ring overflow
                        if (spare.size() < kMaxRecvBatch) spare.push_back(std::move(d));
                    } else {
                        ring_.push_back(std::move(d));
                    }
                }
                ringCv_.notify_one();
                while (spare.size() < kMaxRecvBatch && !recvFreeList_.empty()) {
                    spare.push_back(std::move(recvFreeList_.back()));
                    recvFreeList_.pop_back();
                }
            }
            batch.clear();
        }
    }

    void udpDemuxThread() {
        // HIGH priority too -- the demux thread is the real per-packet worker and,
        // if starved under OpenEphys CPU load, the ring overflows -> SEQ gaps.
        raiseThreadPriority();
        auto lastDropLog = std::chrono::steady_clock::now();
        uint64_t lastRingDrops = 0;
        std::vector<Datagram> localBatch;   // whole ring drained per wakeup
        localBatch.reserve(kMaxRecvBatch);
        while (running_.load()) {
            {
                std::unique_lock<std::mutex> lock(ringMutex_);
                ringCv_.wait_for(lock, std::chrono::milliseconds(200),
                                 [this] { return !ring_.empty() || !running_.load(); });
                // Swap out ALL queued datagrams in one shot -- one lock per chunk, not
                // per datagram.
                while (!ring_.empty()) {
                    localBatch.push_back(std::move(ring_.front()));
                    ring_.pop_front();
                }
            }
            if (!localBatch.empty()) {
                // Decode the chunk OUTSIDE the lock so the recv thread keeps publishing.
                for (auto& d : localBatch)
                    demuxDatagram(d.bytes.data(), d.len);
                // Return the buffers to the free-list in bulk (one lock).
                std::lock_guard<std::mutex> lock(ringMutex_);
                for (auto& d : localBatch)
                    if (recvFreeList_.size() < kFreeListMax)
                        recvFreeList_.push_back(std::move(d));
                localBatch.clear();
            }
            // Surface ring overflow so the drop STAGE is visible: ringDrops_ climbing
            // == recv->demux ring overflowed == the demux couldn't keep up (a SEQ-gap
            // cause). If SEQ gaps appear but ringDrops_ stays flat, the loss is UPSTREAM
            // (kernel UDP overflow) or downstream (dataQueueDrops_ / OE sourceBuffer).
            {
                auto now = std::chrono::steady_clock::now();
                if (now - lastDropLog > std::chrono::seconds(5)) {
                    lastDropLog = now;
                    uint64_t rd, ringSz;
                    { std::lock_guard<std::mutex> lock(ringMutex_);
                      rd = ringDrops_; ringSz = ring_.size(); }
                    if (rd != lastRingDrops) {
                        std::cout << "[GLANCE][DROP] recv->demux ring overflow: "
                                  << "ringDrops=" << rd << " (+" << (rd - lastRingDrops)
                                  << "/5s), ring depth=" << ringSz << "/" << kRingMax
                                  << " -- demux starved; SEQ gaps originate HERE" << std::endl;
                        lastRingDrops = rd;
                    }
                }
            }
        }
        // Drain anything still queued at shutdown so we don't silently lose it.
        for (;;) {
            Datagram datagram;
            {
                std::lock_guard<std::mutex> lock(ringMutex_);
                if (ring_.empty()) break;
                datagram = std::move(ring_.front());
                ring_.pop_front();
            }
            demuxDatagram(datagram.bytes.data(), datagram.len);
        }
    }

    // Peek the common header and route by stream_type.
    void demuxDatagram(const uint8_t* data, size_t len) {
        if (len < COMMON_HEADER_WORDS * 4)
            return;                       // shorter than the 32-byte common header

        uint32_t magic   = unpackU32LE(data);          // w0
        uint32_t typeVer = unpackU32LE(data + 4);       // w1
        if (magic != UNIFIED_MAGIC) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            magicErrors_++;
            totalErrors_++;
            return;
        }

        uint8_t streamType = (uint8_t)(typeVer & 0xFF);
        if (streamType == STREAM_TYPE_BROADBAND) {
            processBroadbandDatagram(data, len);
        } else if (streamType == STREAM_TYPE_LFP) {
            processLfpDatagram(data, len);
        } else if (streamType == STREAM_TYPE_IMU) {
            processImuDatagram(data, len);
        }
        // Other stream types (e.g. WAVELET=3) are silently ignored on this
        // branch -- they belong to the follow-on consumer.
    }

    // IMU sample (UNIFIED stream_type = 3). Unlike broadband/LFP this stream is
    // built by the PS (its source is I2C, not a PL BRAM), one datagram per fused
    // sample per port at 100 Hz. Layout (docs/unified-packet-format.md):
    //   w0 MAGIC | w1 TYPE_VER (stream_type=3 | version<<8 | port<<16)
    //   w2/w3 = 64-bit PL master timestamp (same clock as the neural data)
    //   w4 = SEQ (PER PORT -- each port is its own stream)
    //   w5 = AUX0 = period_ms | iic_errors<<16 | send_drops<<24
    //   w6 = AUX1 = calib_stat | opr_mode<<8 | temp_c<<16
    //   w8..w12 = 10 LE int16: quat w/x/y/z, acc x/y/z, gyr x/y/z
    void processImuDatagram(const uint8_t* data, size_t len) {
        if (len < IMU_PACKET_BYTES) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            sizeErrors_++; totalErrors_++;
            return;
        }
        static constexpr size_t HDR = COMMON_HEADER_WORDS * 4;

        uint32_t typeVer = unpackU32LE(data + 4);
        int port = (int)((typeVer >> 16) & 1);
        uint32_t seq  = unpackU32LE(data + 16);
        uint32_t aux0 = unpackU32LE(data + 20);
        uint32_t aux1 = unpackU32LE(data + 24);

        // Per-PORT SEQ continuity = the IMU loss check. The board never retries
        // a failed send (a fresher sample is 10 ms out), so a forward gap is
        // exactly that many lost samples. Same forward/backward discipline as
        // the LFP path: a backward jump is a stream restart, not loss.
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            if (imuHaveSeq_[port]) {
                uint32_t delta = seq - (imuLastSeq_[port] + 1);
                if (delta != 0 && delta < 0x80000000u) {
                    imuSeqGaps_[port]++;
                    imuLostSamples_[port] += delta;
                    static thread_local std::chrono::steady_clock::time_point lastImuLoss{};
                    auto nowTp = std::chrono::steady_clock::now();
                    if (nowTp - lastImuLoss > std::chrono::seconds(1)) {
                        lastImuLoss = nowTp;
                        std::cout << "[GLANCE][LOSS] IMU port "
                                  << (port ? 'B' : 'A') << " SEQ gap: +" << delta
                                  << " missing, gaps=" << imuSeqGaps_[port]
                                  << " (throttled)" << std::endl;
                    }
                }
            }
            imuLastSeq_[port] = seq;
            imuHaveSeq_[port] = true;
            imuSamples_[port]++;
        }

        ImuSample sample;
        sample.timestamp = (uint64_t)unpackU32LE(data + 8) |
                           ((uint64_t)unpackU32LE(data + 12) << 32);
        sample.sequence      = seq;
        sample.port          = port;
        sample.periodMs      = (uint16_t)(aux0 & 0xFFFF);
        sample.iicErrors     = (uint8_t)((aux0 >> 16) & 0xFF);
        sample.sendDrops     = (uint8_t)((aux0 >> 24) & 0xFF);
        sample.calibStatus   = (uint8_t)(aux1 & 0xFF);
        sample.operatingMode = (uint8_t)((aux1 >> 8) & 0xFF);
        sample.temperatureC  = (int8_t)((aux1 >> 16) & 0xFF);
        for (int i = 0; i < 4; ++i)
            sample.quat[i]  = (float)unpackI16LE(data + HDR + 2 * i) * BNO055_QUAT_SCALE;
        for (int i = 0; i < 3; ++i)
            sample.accel[i] = (float)unpackI16LE(data + HDR + 8 + 2 * i) * BNO055_ACCEL_SCALE;
        for (int i = 0; i < 3; ++i)
            sample.gyro[i]  = (float)unpackI16LE(data + HDR + 14 + 2 * i) * BNO055_GYRO_SCALE;

        ImuDataCallback cb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            cb = imuDataCallback_;
        }
        if (cb) cb(sample);
    }

    // LFP frame (UNIFIED stream_type = 2). The PL builds the whole wire packet
    // (common header + decimated samples) and the PS just sends it on UDP 0x6800.
    // Header layout (docs/unified-packet-format.md, net.py receive_lfp):
    //   w0 MAGIC=0xCAFEBABE | w1 TYPE_VER (stream_type=2 | version<<8)
    //   w2/w3 = 64-bit master timestamp (newest input sample of the window)
    //   w4 = SEQ (PL LFP frame sequence; +1/frame -- the loss check)
    //   w5 = AUX0 = lane_mask | decim_R<<8 | num_taps<<16 | overrun<<24
    //   w6 = AUX1 = num_samples (= popcount(lane_mask) * 32)
    //   w7 = RSVD
    //   w8.. = popcount(lane_mask)*32 offset-binary 16-bit samples, 2 per word.
    void processLfpDatagram(const uint8_t* data, size_t len) {
        static constexpr size_t HDR = COMMON_HEADER_WORDS * 4;  // 32-byte common header
        if (len < HDR) return;

        uint64_t timestamp = (uint64_t)unpackU32LE(data + 8) |
                             ((uint64_t)unpackU32LE(data + 12) << 32);   // w2/w3
        uint32_t seq       = unpackU32LE(data + 16);   // w4 = per-stream SEQ
        uint32_t aux0      = unpackU32LE(data + 20);   // w5 = AUX0
        uint32_t numSamples = unpackU32LE(data + 24);  // w6 = AUX1 = num_samples

        uint8_t laneMask = (uint8_t)(aux0 & 0xFF);
        uint8_t decimR   = (uint8_t)((aux0 >> 8) & 0xFF);
        uint8_t numTaps  = (uint8_t)((aux0 >> 16) & 0xFF);
        bool    overrun  = ((aux0 >> 24) & 0x1) != 0;

        // Payload size from the lane mask. Each enabled lane contributes 32
        // amplifier channels x 1 16-bit sample = 64 bytes per frame. Cross-check
        // against the AUX1 num_samples field the firmware sends.
        int popcount = 0;
        for (int b = 0; b < 8; ++b) popcount += ((laneMask >> b) & 1);
        size_t expectedSamples = (size_t)popcount * 32;
        if (numSamples != 0 && numSamples != expectedSamples)
            return;                                    // mask/cfg drift -- drop
        size_t expectedBytes   = expectedSamples * sizeof(uint16_t);
        if (len - HDR < expectedBytes) return;         // truncated / wrong mask

        // Per-stream LFP SEQ continuity = the LFP loss check. A gap means the
        // board emitted frames we never received. Count + log it (never hide).
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            if (lfpHaveSeq_) {
                uint32_t expectedSeq = lfpLastSeq_ + 1;  // wraps at 2^32 naturally
                uint32_t delta = seq - expectedSeq;      // mod 2^32
                if (delta == 0) {
                    // Contiguous (including the 32-bit wrap) -- no loss.
                } else if (delta < 0x80000000u) {
                    // FORWARD gap => genuinely dropped LFP frame(s). Count always,
                    // throttle the log (see the broadband path for why).
                    lfpSeqGaps_++;
                    lfpLostFrames_ += delta;
                    static thread_local std::chrono::steady_clock::time_point lastLfpLoss{};
                    auto nowTp = std::chrono::steady_clock::now();
                    if (nowTp - lastLfpLoss > std::chrono::seconds(1)) {
                        lastLfpLoss = nowTp;
                        std::cout << "[GLANCE][LOSS] LFP SEQ gap: expected "
                                  << expectedSeq << ", got " << seq << " (+" << delta
                                  << " missing). lfp_seq_gaps=" << lfpSeqGaps_
                                  << " (throttled)" << std::endl;
                    }
                } else {
                    // BACKWARD jump => stream restart (SEQ resets on START); resync
                    // silently, not archival loss.
                }
            }
            lfpLastSeq_ = seq;
            lfpHaveSeq_ = true;
        }

        // The payload is little-endian uint16, naturally aligned (the common
        // header is a whole number of 32-bit words). recvfrom hands us a
        // uint8_t buffer; reinterpret as uint16_t* is safe on our platforms.
        const uint16_t* samples = reinterpret_cast<const uint16_t*>(data + HDR);

        LfpFrame frame;
        frame.timestamp     = timestamp;
        frame.frameSequence = seq;
        frame.laneMask      = laneMask;
        frame.decimR        = decimR;
        frame.numTaps       = numTaps;
        frame.overrun       = overrun;
        frame.samples       = samples;
        frame.sampleCount   = expectedSamples;

        LfpDataCallback cb;
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            cb = lfpDataCallback_;
        }
        if (cb) cb(frame);
    }

    // A datagram is exactly one broadband packet (the board sends one packet
    // per datagram). We still re-chunk defensively in case several were
    // coalesced (matches net.py UnifiedSink._handle_broadband).
    void processBroadbandDatagram(const uint8_t* data, size_t len) {
        size_t expectedBytes = expectedPacketSizeWords_ * 4;
        if (expectedBytes == 0)
            return;

        if (len == expectedBytes) {
            validateAndDispatchPacket(data, expectedBytes);
            return;
        }
        for (size_t offset = 0; offset + expectedBytes <= len; offset += expectedBytes) {
            validateAndDispatchPacket(data + offset, expectedBytes);
        }
    }
    
    void validateAndDispatchPacket(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        
        totalPackets_++;
        
        size_t expectedBytes = expectedPacketSizeWords_ * 4;
        
        // Check size
        if (len != expectedBytes) {
            sizeErrors_++;
            totalErrors_++;
            return;
        }
        
        // Unpack as 32-bit words. Reuse a per-thread buffer instead of allocating
        // a fresh vector on every packet: at 30 kHz a per-packet heap alloc here
        // (plus the ones on the recv + queue paths) churns the global allocator
        // and its latency drifts up over minutes -> a stall eventually overflows
        // the socket buffer -> SEQ gap. net.py stays clean by never allocating on
        // the hot path. This function only ever runs on the demux thread, so a
        // thread_local buffer is safe. resize() to the same size is a no-op.
        static thread_local std::vector<uint32_t> words;
        if (words.size() != expectedPacketSizeWords_)
            words.resize(expectedPacketSizeWords_);
        for (size_t i = 0; i < expectedPacketSizeWords_; ++i) {
            words[i] = unpackU32LE(&data[i * 4]);
        }
        
        // UNIFIED header validation: w0 = MAGIC (0xCAFEBABE), w1 low byte =
        // stream_type (must be BROADBAND here -- the demux already routed by it,
        // but re-check defensively against a coalesced foreign packet).
        uint8_t streamType = (uint8_t)(words[1] & 0xFF);
        if (words[0] != UNIFIED_MAGIC || streamType != STREAM_TYPE_BROADBAND) {
            magicErrors_++;
            totalErrors_++;
            return;
        }

        // Extract timestamp (w2/w3) and the per-stream SEQ (w4).
        uint64_t timestamp = (static_cast<uint64_t>(words[3]) << 32) | words[2];
        uint32_t seq       = words[4];

        // Per-stream SEQ continuity = the BROADBAND loss check (the no-loss
        // assertion). Each broadband packet's SEQ must be exactly +1 (mod 2^32)
        // from the previous; a gap means lost broadband packet(s). Broadband is
        // archival -- surface any gap loudly, never hide it. (Mirrors net.py
        // DataValidator's SEQ check; replaces the old timestamp-delta heuristic
        // as the authoritative loss signal.)
        if (haveSeq_) {
            uint32_t expectedSeq = lastSeq_ + 1;      // wraps at 2^32 naturally
            uint32_t delta = seq - expectedSeq;       // mod 2^32
            if (delta == 0) {
                // Contiguous (including the natural 32-bit wrap) -- no loss.
            } else if (delta < 0x80000000u) {
                // FORWARD gap => genuinely lost broadband packet(s). Always COUNT
                // it (the counters are the truth), but THROTTLE the log to at most
                // once/second: printing a blocking std::cout under statsMutex_ on
                // the demux thread for every gap slows the demux thread, backs up
                // the ring, and causes MORE drops -> a self-amplifying cascade.
                seqGaps_++;
                seqLostPackets_ += delta;
                timestampErrors_++;   // keep the timestamp-based loss counter moving too
                totalErrors_++;
                static thread_local std::chrono::steady_clock::time_point lastBbLoss{};
                auto nowTp = std::chrono::steady_clock::now();
                if (nowTp - lastBbLoss > std::chrono::seconds(1)) {
                    lastBbLoss = nowTp;
                    std::cout << "[GLANCE][LOSS] Broadband SEQ gap: expected "
                              << expectedSeq << ", got " << seq << " (+" << delta
                              << " missing). bb_seq_gaps=" << seqGaps_
                              << " (throttled)" << std::endl;
                }
            } else {
                // seq < expectedSeq => a BACKWARD jump: the broadband stream
                // RESTARTED (firmware resets/resyncs the SEQ on START -- e.g. the
                // 16 START/STOP cycles of cable detection). This is NOT archival
                // loss; resync silently rather than reporting ~2^32 "missing".
            }
        }
        lastSeq_  = seq;
        haveSeq_  = true;

        lastTimestamp_ = timestamp;
        
        // Update stats periodically
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - lastStatsTime_).count() >= 5.0) {
            lastStatsTime_ = now;
            lastPacketCount_ = totalPackets_;
        }
        
        // If capturing for detection, save packet
        if (captureForDetection_.load()) {
            std::lock_guard<std::mutex> detLock(detectionMutex_);
            if (detectionPacketQueue_.size() < maxDetectionQueueSize_) {
                detectionPacketQueue_.push(words);
            }
        }

        // Dispatch to callback (without holding stats lock)
        DataCallback callback;
        {
            std::lock_guard<std::mutex> cbLock(callbackMutex_);
            callback = dataCallback_;
        }
        
        if (callback) {
            // Don't hold statsMutex while calling user callback
            statsMutex_.unlock();
            callback(words.data(), words.size(), timestamp);
            statsMutex_.lock();
        }
    }
    
    void reportError(const std::string& message) const {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (errorCallback_) {
            errorCallback_(message);
        }
    }

    // Mark the link dead. Subsequent sendCommand calls fail fast because
    // tcpSocket_ is INVALID_SOCKET and isReady()/foundInputSource() return
    // false, so the host can disconnect / reconnect cleanly rather than
    // hanging on every subsequent recv().
    void markDisconnected() {
        connected_ = false;
        if (tcpSocket_ != INVALID_SOCKET) {
            closesocket(tcpSocket_);
            tcpSocket_ = INVALID_SOCKET;
        }
    }
    
    // ========================================================================
    // AUTO-DETECTION HELPER METHODS
    // ========================================================================
    
    bool initializeForDetection(bool verbose) {
        // Disable debug mode to read real chip data
        if (!sendCommand(CMD_STOP)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to disable debug mode" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        if (!sendCommand(CMD_SET_DEBUG_MODE, 0)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to disable debug mode" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Loop count >=2 is required: firmware ab14c19 holds a one-packet guard
        // band in packets_available(), so loop_count==1 -> zero UDP emitted.
        if (!sendCommand(CMD_SET_LOOP_COUNT, DETECTION_LOOP_COUNT)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set loop count" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Channel enable = 0xFF: both ports' CIPO0+CIPO1 land in one packet, so
        // a single phase sweep tests A and B in parallel.
        if (!sendCommand(CMD_SET_CHANNEL_ENABLE, DETECTION_CHANNEL_ENABLE)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to set channel enable" << std::endl;
            }
            return false;
        }
        updateChannelEnable(DETECTION_CHANNEL_ENABLE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Load and run initialization sequence
        if (verbose) {
            std::cout << "[Detection] Running initialization sequence..." << std::endl;
        }
        
        if (!sendCommand(CMD_LOAD_INIT)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to load init sequence" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Run initialization (acquire 1 packet)
        if (!sendCommand(CMD_START)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!sendCommand(CMD_STOP)) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Load cable test sequence
        if (verbose) {
            std::cout << "[Detection] Loading cable test sequence..." << std::endl;
        }
        
        if (!sendCommand(CMD_LOAD_CABLE_TEST)) {
            if (verbose) {
                std::cout << "[Detection] ERROR: Failed to load cable test sequence" << std::endl;
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return true;
    }
    
    // Snapshot reception counters for diagnostic comparisons.
    struct RxSnapshot {
        uint64_t total;
        uint64_t magic;
        uint64_t size;
    };
    RxSnapshot rxSnapshot() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return RxSnapshot{ totalPackets_, magicErrors_, sizeErrors_ };
    }

    // Score one CIPO lane out of a cable-test packet.
    //   stride / offset = pick lane out of the packed data: one 32-bit word per
    //                     enabled stream-pair, per cycle, in bit order.
    //                     ce=0x0F (port A only):  stride=2  offset=0(A0) 1(A1)
    //                     ce=0xF0 (port B only):  stride=2  offset=0(B0) 1(B1)
    //                     ce=0xFF (both ports):   stride=4
    //                                              offset=0(A0) 1(A1) 2(B0) 3(B1)
    //   label is used only for verbose output (e.g. "A.CIPO0").
    std::pair<double, ChipType> scoreChannel(
        const std::vector<uint32_t>& packet, int stride, int offset,
        const std::string& label, bool verbose) {

        if (packet.size() < CABLE_TEST_DATA_OFFSET_WORDS + 9 * static_cast<size_t>(stride)) {
            return {0.0, ChipType::NONE};
        }

        double score = 0.0;
        ChipType chipType = ChipType::NONE;

        // Extract data words from the calibrated cable-test offset (8 words),
        // matching net.py _score_channel exactly. NOT PACKET_HEADER_WORDS.
        std::vector<uint32_t> dataWords(packet.begin() + CABLE_TEST_DATA_OFFSET_WORDS, packet.end());

        if (dataWords.size() < 35) {
            return {0.0, ChipType::NONE};
        }

        // Extract this lane's words from the tightly-packed data block.
        std::vector<uint32_t> channelWords;
        for (size_t i = offset; i < dataWords.size(); i += stride) {
            channelWords.push_back(dataWords[i]);
        }

        if (channelWords.size() < 9) {
            return {0.0, ChipType::NONE};
        }
        
        // Extract regular and DDR streams from 32-bit words
        std::vector<uint16_t> regular, ddr;
        for (uint32_t word : channelWords) {
            regular.push_back(word & 0xFFFF);           // Lower 16 bits
            ddr.push_back((word >> 16) & 0xFFFF);       // Upper 16 bits
        }
        
        // Score INTAN pattern (indices 2-6 due to 2-cycle pipeline delay)
        std::vector<uint16_t> intanFound;
        for (size_t i = 0; i < INTAN_PATTERN_SIZE; ++i) {
            size_t idx = i + 2;  // Pipeline delay
            if (idx < regular.size()) {
                intanFound.push_back(regular[idx]);
                if (regular[idx] == INTAN_PATTERN[i]) {
                    score += 10.0;
                }
            }
        }
        
        // Check chip ID at index 7
        if (regular.size() > 7 && ddr.size() > 7) {
            uint16_t chipIdReg = regular[7];
            uint16_t chipIdDdr = ddr[7];
            
            // Both regular and DDR should read same chip ID
            if (chipIdReg == CHIP_ID_RHD2164 && chipIdDdr == CHIP_ID_RHD2164) {
                chipType = ChipType::RHD2164;
                score += 10.0;
            } else if (chipIdReg == CHIP_ID_RHD2132) {
                chipType = ChipType::RHD2132;
                score += 10.0;
            } else if (chipIdReg == CHIP_ID_RHD2216) {
                chipType = ChipType::RHD2132;  // Treat as RHD2132 (no DDR)
                score += 10.0;
            }
        }
        
        // Check MISO register at index 8
        if (regular.size() > 8 && ddr.size() > 8) {
            uint16_t misoReg = regular[8];
            uint16_t misoDdr = ddr[8];
            
            if (chipType == ChipType::RHD2164) {
                // RHD2164 should have specific MISO values
                if (misoReg == MISO_REG_DDR && misoDdr == MISO_DDR_DDR) {
                    score += 10.0;
                }
            } else if (chipType == ChipType::RHD2132) {
                // RHD2132 should have zero MISO
                if (misoReg == MISO_NO_DDR) {
                    score += 10.0;
                }
            }
        }
        
        // Verbose output
        if (verbose && score > DETECTION_THRESHOLD) {
            std::string patternStr;
            for (uint16_t val : intanFound) {
                if (val >= 0x20 && val <= 0x7E) {
                    patternStr += static_cast<char>(val);
                } else {
                    patternStr += '?';
                }
            }
            
            std::string chipStr = (chipType == ChipType::RHD2164) ? "RHD2164" : 
                                 (chipType == ChipType::RHD2132) ? "RHD2132" : "Unknown";
            
            std::cout << "    " << label << ": '" << patternStr
                     << "' (" << chipStr << ")" << std::endl;
        }
        
        return {score, chipType};
    }
    
    void startDetectionCapture() {
        std::lock_guard<std::mutex> lock(detectionMutex_);
        // Clear queue
        while (!detectionPacketQueue_.empty()) {
            detectionPacketQueue_.pop();
        }
        captureForDetection_ = true;
    }
    
    void stopDetectionCapture() {
        captureForDetection_ = false;
    }
    
    bool capturePacketForDetection(
        std::vector<uint32_t>& packet, double timeoutSec) {
        
        auto deadline = std::chrono::steady_clock::now() + 
                       std::chrono::milliseconds(static_cast<int>(timeoutSec * 1000));
        
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(detectionMutex_);
                if (!detectionPacketQueue_.empty()) {
                    packet = detectionPacketQueue_.front();
                    detectionPacketQueue_.pop();
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }
    
    // Configuration
    std::string deviceIp_;
    uint16_t tcpPort_;
    uint16_t udpPort_;
    
    // Sockets -- ONE UDP socket now (unified port; LFP socket removed).
    SOCKET tcpSocket_;
    SOCKET udpSocket_;

    // Threading: recvThread_ drains the socket -> ring; demuxThread_ routes by
    // stream_type. (Replaces the old per-stream udpThread_/lfpThread_ pair.)
    std::thread recvThread_;
    std::thread demuxThread_;
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    mutable std::mutex tcpMutex_;
    mutable std::mutex callbackMutex_;
    mutable std::mutex statsMutex_;

    // Promiscuous-drain ring between recvThread_ and demuxThread_. Big enough
    // to ride out a long downstream stall without dropping a datagram; an
    // overflow is counted (ringDrops_) and surfaced, never silently hidden.
    static constexpr size_t kRingMax = 200000;
    // Max datagrams the recv thread drains per wakeup into ONE locked hand-off, and the
    // max the demux swaps out per wakeup. macOS has no recvmmsg(), so this MSG_DONTWAIT
    // drain-into-chunk is the portable analogue of the acq-board's block reads: it
    // amortizes the per-datagram lock/notify overhead ~256x and lets the recv thread
    // catch up after a scheduling gap (bigger backlog -> bigger chunk). Tunable.
    static constexpr size_t kMaxRecvBatch = 256;
    // One received datagram. `bytes` is kept PERMANENTLY at kDatagramCap so the recv
    // hot path never re-grows/zero-fills it: the old code did buffer.resize(4096) then
    // buffer.resize(received) per datagram, so every recycled buffer was re-zeroed
    // (~3.5 KB) and destruct-walked back down 30k times/s -- a `sample` profile showed
    // that churn burning ~42% of a core ON THE RECV THREAD, stealing exactly the time
    // it needed to drain SO_RCVBUF (the kernel UDP overflow that caused the SEQ gaps,
    // and independent of any GUI load). `len` carries the real payload length that
    // recvfrom returned, so consumers pass (bytes.data(), len) and never touch size().
    static constexpr size_t kDatagramCap = 4096;
    struct Datagram {
        std::vector<uint8_t> bytes;   // size() stays == kDatagramCap after first use
        size_t len = 0;               // actual payload length from recvfrom
    };
    std::deque<Datagram> ring_;
    std::mutex ringMutex_;
    std::condition_variable ringCv_;
    uint64_t ringDrops_ = 0;
    // Recycled recv buffers so the hot recv path never mallocs per packet (30k/s
    // under load add latency + heap fragmentation, slowing the drain right when it
    // matters). demuxThread_ returns each consumed buffer here; recvThread_ reuses
    // it. Guarded by ringMutex_; capped so it can't grow with the ring.
    static constexpr size_t kFreeListMax = 1024;   // ~4 MB of recycled 4 KB buffers
    std::deque<Datagram> recvFreeList_;

    // State
    uint8_t currentChannelEnable_;
    size_t expectedPacketSizeWords_;

    // Statistics
    uint64_t totalPackets_;
    uint64_t totalErrors_;
    uint64_t magicErrors_;
    uint64_t timestampErrors_;
    uint64_t sizeErrors_;
    uint64_t lastTimestamp_;
    std::chrono::steady_clock::time_point startTime_;
    std::chrono::steady_clock::time_point lastStatsTime_;
    uint64_t lastPacketCount_;

    // Per-stream SEQ loss tracking (the no-loss assertion; header word 4).
    // Broadband must stay at zero gaps; LFP gaps are logged independently.
    bool     haveSeq_        = false;
    uint32_t lastSeq_        = 0;
    uint64_t seqGaps_        = 0;   // broadband gap events
    uint64_t seqLostPackets_ = 0;   // total broadband packets implied missing
    bool     lfpHaveSeq_     = false;
    uint32_t lfpLastSeq_     = 0;
    uint64_t lfpSeqGaps_     = 0;   // LFP gap events
    uint64_t lfpLostFrames_  = 0;   // total LFP frames implied missing
    
    // Callbacks
    DataCallback dataCallback_;
    LfpDataCallback lfpDataCallback_;
    ImuDataCallback imuDataCallback_;
    // IMU reception state, per port (0 = A, 1 = B). Guarded by statsMutex_.
    uint64_t imuSamples_[2]     = {0, 0};
    uint64_t imuSeqGaps_[2]     = {0, 0};
    uint64_t imuLostSamples_[2] = {0, 0};
    uint32_t imuLastSeq_[2]     = {0, 0};
    bool     imuHaveSeq_[2]     = {false, false};
    ErrorCallback errorCallback_;
    
    // Auto-detection
    std::mutex detectionMutex_;
    std::queue<std::vector<uint32_t>> detectionPacketQueue_;
    std::atomic<bool> captureForDetection_;
    size_t maxDetectionQueueSize_;
};

// ============================================================================
// PUBLIC INTERFACE IMPLEMENTATION
// ============================================================================

IntanInterface::IntanInterface(const std::string& deviceIp, uint16_t tcpPort, uint16_t udpPort)
    : pImpl_(std::make_unique<Impl>(deviceIp, tcpPort, udpPort))
{
}

IntanInterface::~IntanInterface() = default;

bool IntanInterface::foundInputSource() const {
    return pImpl_->isConnected();
}

bool IntanInterface::isReady() const {
    return pImpl_->isReady();
}

bool IntanInterface::getStatus(DeviceStatus& status) const {
    return pImpl_->getStatusInternal(status);
}

void IntanInterface::getReceptionStats(ReceptionStats& stats) const {
    pImpl_->getReceptionStats(stats);
}

bool IntanInterface::startAcquisition() {
    return pImpl_->sendCommand(CMD_START);
}

bool IntanInterface::stopAcquisition() {
    return pImpl_->sendCommand(CMD_STOP);
}

bool IntanInterface::resetTimestamp() {
    return pImpl_->sendCommand(CMD_RESET_TIMESTAMP);
}

bool IntanInterface::setLoopCount(uint32_t count) {
    return pImpl_->sendCommand(CMD_SET_LOOP_COUNT, count);
}

bool IntanInterface::setPhaseSelect(uint8_t phase0, uint8_t phase1) {
    return pImpl_->sendCommand(CMD_SET_PHASE, phase0, phase1);
}

bool IntanInterface::setPhaseSelectB(uint8_t phase2, uint8_t phase3) {
    return pImpl_->sendCommand(CMD_SET_PHASE_B, phase2, phase3);
}

bool IntanInterface::setDebugMode(bool enable) {
    return pImpl_->sendCommand(CMD_SET_DEBUG_MODE, enable ? 1 : 0);
}

bool IntanInterface::setChannelEnable(uint8_t channelMask) {
    if (pImpl_->sendCommand(CMD_SET_CHANNEL_ENABLE, channelMask)) {
        pImpl_->updateChannelEnable(channelMask);
        return true;
    }
    return false;
}

bool IntanInterface::setUdpDestination(const std::string& ipAddress, uint16_t port) {
    uint32_t ipInt = stringToIp(ipAddress);
    if (ipInt == 0) {
        return false;
    }
    return pImpl_->sendCommand(CMD_SET_UDP_DEST, ntohl(ipInt), port);
}

bool IntanInterface::loadConvertSequence() {
    return pImpl_->sendCommand(CMD_LOAD_CONVERT);
}

bool IntanInterface::loadInitSequence() {
    return pImpl_->sendCommand(CMD_LOAD_INIT);
}

bool IntanInterface::loadCableTestSequence() {
    return pImpl_->sendCommand(CMD_LOAD_CABLE_TEST);
}

bool IntanInterface::runFullCableTest() {
    return pImpl_->sendCommand(CMD_FULL_CABLE_TEST);
}

bool IntanInterface::auxUploadBank(int slot, int bank,
                                   const std::vector<uint16_t>& commands,
                                   int loopIndex) {
    return pImpl_->auxUploadBank(slot, bank, commands, loopIndex);
}

bool IntanInterface::auxBankSelect(int slot, int bank) {
    return pImpl_->auxBankSelect(slot, bank);
}

bool IntanInterface::setFastSettle(bool softwareLevel, bool gpioEnable,
                                   uint8_t gpioPin, bool dspReset) {
    return pImpl_->setFastSettle(softwareLevel, gpioEnable, gpioPin, dspReset);
}

bool IntanInterface::readRegister(uint8_t reg, uint16_t& cipo0Value,
                                  uint16_t& cipo1Value) {
    return pImpl_->readRegister(reg, cipo0Value, cipo1Value);
}

bool IntanInterface::lfpEnable(bool on) {
    return pImpl_->lfpEnable(on);
}

bool IntanInterface::lfpSetChannels(uint8_t laneMask) {
    return pImpl_->lfpSetChannels(laneMask);
}

bool IntanInterface::lfpSetParams(uint8_t decimR, uint8_t numTaps) {
    return pImpl_->lfpSetParams(decimR, numTaps);
}

bool IntanInterface::lfpWriteCoef(bool clearFirst, LfpStage stage, int32_t coef18) {
    return pImpl_->lfpWriteCoef(clearFirst, stage == LfpStage::Decimator, coef18);
}

bool IntanInterface::lfpUploadCoefs(LfpStage stage, const std::vector<int32_t>& coefs) {
    return pImpl_->lfpUploadCoefs(stage == LfpStage::Decimator, coefs);
}



void IntanInterface::setDataCallback(DataCallback callback) {
    pImpl_->setDataCallback(callback);
}

void IntanInterface::setLfpDataCallback(LfpDataCallback callback) {
    pImpl_->setLfpDataCallback(callback);
}

void IntanInterface::setImuDataCallback(ImuDataCallback callback) {
    pImpl_->setImuDataCallback(callback);
}

bool IntanInterface::detectImu(ImuPorts& present) {
    return pImpl_->detectImu(present);
}

bool IntanInterface::plStatus(int& config, bool& isAcq) {
    return pImpl_->plStatus(config, isAcq);
}

bool IntanInterface::setConfig(FabricConfig config) {
    return pImpl_->setConfig(config);
}

bool IntanInterface::ensureAcquisitionFabric() {
    return pImpl_->ensureAcquisitionFabric();
}

bool IntanInterface::rescanFabric(ImuPorts& present) {
    return pImpl_->rescanFabric(present);
}

bool IntanInterface::setImuStream(const ImuPorts& ports, uint32_t periodMs,
                                  ImuPorts& active) {
    return pImpl_->setImuStream(ports, periodMs, active);
}

void IntanInterface::getImuStats(ImuStats& stats) const {
    pImpl_->getImuStats(stats);
}

void IntanInterface::setErrorCallback(ErrorCallback callback) {
    pImpl_->setErrorCallback(callback);
}

bool IntanInterface::runAutoDetection(AutoDetectionResult& result, bool verbose) {
    return pImpl_->runAutoDetection(result, verbose);
}

bool IntanInterface::applyDetectionConfig(const AutoDetectionResult& result) {
    if (!result.success) {
        return false;
    }

    // Port A phase: bestPhase0 applies to both port-A CIPO lines (cipo0/cipo1).
    if (!setPhaseSelect(result.bestPhase0, result.bestPhase0)) {
        return false;
    }

    // Port B phase: bestPhase1 applies to both port-B CIPO lines (phase2/phase3).
    // setPhaseSelectB is a no-op on firmware without dual-port support; harmless
    // if no chips were found there.
    if (!setPhaseSelectB(result.bestPhase1, result.bestPhase1)) {
        return false;
    }

    // Combined 8-bit channel enable mask (port A in [3:0], port B in [7:4]).
    if (!setChannelEnable(result.optimalChannelMask)) {
        return false;
    }

    // Load normal conversion sequence (not cable test)
    if (!loadConvertSequence()) {
        return false;
    }

    return true;
}

uint32_t IntanInterface::calculatePacketSize(uint8_t channelMask) {
    int numChannels = 0;
    for (int i = 0; i < 8; ++i) {
        if (channelMask & (1 << i)) {
            numChannels++;
        }
    }
    
    if (numChannels == 0) {
        // Mask 0 -> 70 data words (matches net.py calculate_data_words), so the
        // 14-word unified header gives 84 words total.
        return PACKET_HEADER_WORDS + 70;
    }

    uint32_t total16bitWords = 35 * numChannels;
    uint32_t data32bitWords = (total16bitWords + 1) / 2;

    return PACKET_HEADER_WORDS + data32bitWords;
}

std::string IntanInterface::getDeviceIp() const {
    return pImpl_->getDeviceIp();
}

uint16_t IntanInterface::getTcpPort() const {
    return pImpl_->getTcpPort();
}

uint16_t IntanInterface::getUdpPort() const {
    return pImpl_->getUdpPort();
}

// ============================================================================
// HELPER METHODS FOR STATUS
// ============================================================================

std::string IntanInterface::DeviceStatus::getFirmwareVersionString() const {
    std::ostringstream oss;
    oss << ((firmwareVersion >> 24) & 0xFF) << "."
        << ((firmwareVersion >> 16) & 0xFF) << "."
        << ((firmwareVersion >> 8) & 0xFF) << "."
        << (firmwareVersion & 0xFF);
    return oss.str();
}

std::string IntanInterface::DeviceStatus::getChannelEnableString() const {
    std::ostringstream oss;
    bool first = true;
    
    if (channelEnable & CHANNEL_CIPO0_REGULAR) {
        if (!first) oss << ", ";
        oss << "CIPO0_REG";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO0_DDR) {
        if (!first) oss << ", ";
        oss << "CIPO0_DDR";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO1_REGULAR) {
        if (!first) oss << ", ";
        oss << "CIPO1_REG";
        first = false;
    }
    if (channelEnable & CHANNEL_CIPO1_DDR) {
        if (!first) oss << ", ";
        oss << "CIPO1_DDR";
        first = false;
    }
    
    return first ? "NONE" : oss.str();
}

std::string IntanInterface::DeviceStatus::getSummary() const {
    std::ostringstream oss;
    oss << "=== INTAN DEVICE STATUS ===\n";
    oss << "Firmware: v" << getFirmwareVersionString()
        << "  (protocol " << protocolVersion << ", device 0x"
        << std::hex << deviceType << std::dec << ")\n";
    oss << "--- PL Hardware ---\n";
    oss << "Timestamp: " << timestamp
        << "  Packets sent: " << packetsSent << "\n";
    oss << "Transmission: " << (transmissionActive ? "ACTIVE" : "stopped")
        << "  Loop limit: " << (loopLimitReached ? "reached" : "no")
        << "  State/Cycle: " << (int)stateCounter << "/" << (int)cycleCounter << "\n";
    oss << "BRAM write addr: " << bramWriteAddr
        << "  FIFO count: " << fifoCount << "\n";
    oss << "--- PS Software ---\n";
    oss << "Packets received: " << packetsReceived
        << "  Errors: " << errorCount << "\n";
    oss << "UDP sent: " << udpPacketsSent
        << "  UDP errors: " << udpSendErrors
        << "  Packet size: " << packetSizeWords << " words\n";
    oss << "--- Configuration ---\n";
    oss << "Channels: 0x" << std::hex << (int)channelEnable << std::dec
        << " (" << getChannelEnableString() << ")"
        << "  Phase: " << (int)phase0 << "/" << (int)phase1
        << "  Debug: " << (debugMode ? "ON" : "off") << "\n";
    oss << "UDP dest: " << udpDestIp << ":" << udpDestPort << "\n";
    oss << "--- Aux Sequencer ---\n";
    if (!hasAuxStatus) {
        oss << "(not supported by this firmware -- 86-byte status)\n";
    } else {
        oss << "Engine: " << (auxSeqEnabled ? "on" : "OFF?")
            << "  Fast settle: " << (fastSettleActive ? "ACTIVE" : "off")
            << "  Digout: " << (digoutState ? "1" : "0")
            << "  DSP reset: " << (dspResetActive ? "ACTIVE" : "off") << "\n";
        oss << "Active banks: slot0=" << ((auxBankActive >> 0) & 1)
            << " slot1=" << ((auxBankActive >> 1) & 1)
            << " slot2=" << ((auxBankActive >> 2) & 1)
            << "  Indices: " << (int)auxIndex[0] << "/"
            << (int)auxIndex[1] << "/" << (int)auxIndex[2] << "\n";
        oss << "Last inject result: 0x" << std::hex << std::setw(8)
            << std::setfill('0') << auxReadResult << std::dec << "\n";
    }
    oss << "--- LFP Engine ---\n";
    if (!hasLfpStatus) {
        oss << "(not supported by this firmware -- status < 160 bytes)\n";
    } else if (!lfpEnabled) {
        oss << "Disabled  (configure + enable via remote/net.py configure_lfp)\n";
    } else {
        int popcount = 0;
        for (int b = 0; b < 8; ++b) popcount += ((lfpLaneMask >> b) & 1);
        int outRate = (lfpDecimR > 0) ? (30000 / lfpDecimR) : 0;
        oss << "ENABLED  mask=0x" << std::hex << (int)lfpLaneMask << std::dec
            << " (" << popcount << " streams / " << (popcount * 32) << " ch)"
            << "  decim=" << (int)lfpDecimR
            << " (" << outRate << " Hz)"
            << "  taps=" << (int)lfpNumTaps << "\n";
        oss << "Packets sent: " << lfpPacketsSent
            << "  Overrun: " << (lfpOverrun ? "YES" : "no") << "\n";
    }
    oss << "===========================";
    return oss.str();
}

// ============================================================================
// HELPER METHODS FOR AUTO-DETECTION RESULT
// ============================================================================

std::string IntanInterface::AutoDetectionResult::chipTypeToString(ChipType type) {
    switch (type) {
        case ChipType::RHD2132: return "RHD2132";
        case ChipType::RHD2164: return "RHD2164";
        case ChipType::NONE:
        default: return "None";
    }
}

std::string IntanInterface::AutoDetectionResult::getSummary() const {
    if (!success) {
        return "Detection failed. Check connections and try manual configuration.";
    }

    if (!chipsDetected) {
        return "No Intan chips detected. Verify:\n"
               "  - SPI cable connections\n"
               "  - Chip power supply\n"
               "  - Cable integrity";
    }

    std::ostringstream oss;
    oss << "Chips detected!\n";
    oss << "  Port A phase: " << static_cast<int>(bestPhase0) << "\n";
    oss << "  Port B phase: " << static_cast<int>(bestPhase1) << "\n";
    oss << "  Channel Mask: 0x" << std::hex << std::uppercase
        << static_cast<int>(optimalChannelMask) << std::dec << "\n";

    if (cipo0Detected) {
        oss << "  A.CIPO0: " << chipTypeToString(cipo0ChipType) << "\n";
    }
    if (cipo1Detected) {
        oss << "  A.CIPO1: " << chipTypeToString(cipo1ChipType) << "\n";
    }
    if (portBCipo0Detected) {
        oss << "  B.CIPO0: " << chipTypeToString(portBCipo0ChipType) << "\n";
    }
    if (portBCipo1Detected) {
        oss << "  B.CIPO1: " << chipTypeToString(portBCipo1ChipType) << "\n";
    }

    double bestScore = cipo0Score + cipo1Score;
    std::string confidence = (bestScore > 100) ? "High" : "Medium";
    oss << "  Detection Confidence: " << confidence;

    return oss.str();
}

std::string IntanInterface::AutoDetectionResult::getChannelSummary() const {
    if (!chipsDetected) {
        return "No chips detected";
    }

    std::vector<std::string> channels;
    if (cipo0Detected) {
        channels.push_back("A.CIPO0 (" + chipTypeToString(cipo0ChipType) + ")");
    }
    if (cipo1Detected) {
        channels.push_back("A.CIPO1 (" + chipTypeToString(cipo1ChipType) + ")");
    }
    if (portBCipo0Detected) {
        channels.push_back("B.CIPO0 (" + chipTypeToString(portBCipo0ChipType) + ")");
    }
    if (portBCipo1Detected) {
        channels.push_back("B.CIPO1 (" + chipTypeToString(portBCipo1ChipType) + ")");
    }

    if (channels.empty()) {
        return "No active channels";
    }

    std::ostringstream oss;
    oss << "Active channels: ";
    for (size_t i = 0; i < channels.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << channels[i];
    }
    return oss.str();
}

