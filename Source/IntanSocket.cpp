// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "IntanSocket.h"
#include "IntanSocketEditor.h"

#include <sstream>

#ifdef __APPLE__
// Disable macOS App Nap for this (plugin-container) process so the OS doesn't throttle
// it a few seconds into acquisition and starve the UDP recv thread (packet loss that
// starts ~10 s in). Done through the Obj-C runtime directly -- no .mm file, no
// Foundation link -- so it can't affect whether the bundle loads. Equivalent to:
//   token = [[[NSProcessInfo processInfo] beginActivityWithOptions:opts reason:@"…"] retain];
#include <objc/objc.h>
#include <objc/runtime.h>
#include <objc/message.h>
static void disableAppNap()
{
    static id token = nullptr;
    if (token != nullptr)
        return;                                  // already active for this process
    Class piClass  = objc_getClass("NSProcessInfo");
    Class strClass = objc_getClass("NSString");
    if (piClass == nullptr || strClass == nullptr)
        return;
    auto msgClsSel    = reinterpret_cast<id(*)(Class, SEL)>(objc_msgSend);
    auto msgClsSelStr = reinterpret_cast<id(*)(Class, SEL, const char*)>(objc_msgSend);
    auto msgObjSel    = reinterpret_cast<id(*)(id, SEL)>(objc_msgSend);
    auto msgActivity  = reinterpret_cast<id(*)(id, SEL, unsigned long long, id)>(objc_msgSend);
    id reason = msgClsSelStr(strClass, sel_registerName("stringWithUTF8String:"),
                             "ephys-socket real-time UDP acquisition");
    id pi = msgClsSel(piClass, sel_registerName("processInfo"));
    // NSActivityUserInitiated (0x00FFFFFF) | NSActivityLatencyCritical (0xFF00000000).
    const unsigned long long opts = 0x00FFFFFFULL | 0xFF00000000ULL;
    id activity = msgActivity(pi, sel_registerName("beginActivityWithOptions:reason:"), opts, reason);
    token = msgObjSel(activity, sel_registerName("retain"));   // keep the activity alive
    std::cout << "[ephys-socket] App Nap disabled for acquisition (NSActivityLatencyCritical)"
              << std::endl;
}
#endif

using namespace IntanSocketNode;

DataThread* IntanSocket::createDataThread(SourceNode* sn)
{
    return new IntanSocket(sn);
}

IntanSocket::IntanSocket(SourceNode* sn) 
    : DataThread(sn)
{
    device_ip = DEFAULT_DEVICE_IP;
    tcp_port = DEFAULT_TCP_PORT;
    udp_port = DEFAULT_UDP_PORT;
    data_scale = DEFAULT_DATA_SCALE;
    aux_data_scale = DEFAULT_AUX_DATA_SCALE;
    channel_enable_mask = 0x0F;  // All channels enabled by default
    totalSamples = 0;
    eventState = 0;
    hasError = false;
    debugMode = false;

    // Create IntanInterface (will throw if can't connect, so wrap in try)
    intanInterface = nullptr;
    
    // Pre-allocate buffers for 4 channels (maximum)
    sourceBuffers.add(new DataBuffer(4, SAMPLE_RATE * bufferSizeInSeconds));
}

std::unique_ptr<GenericEditor> IntanSocket::createEditor(SourceNode* sn)
{
    std::unique_ptr<IntanSocketEditor> editor = std::make_unique<IntanSocketEditor>(sn, this);
    return editor;
}

IntanSocket::~IntanSocket()
{
    if (intanInterface && intanInterface->foundInputSource())
    {
        intanInterface->stopAcquisition();
    }
}

void IntanSocket::registerParameters()
{
    addStringParameter(Parameter::PROCESSOR_SCOPE, 
                      "device_ip", 
                      "Device IP", 
                      "IP address of Intan device", 
                      DEFAULT_DEVICE_IP,
                      true);
    
    addIntParameter(Parameter::PROCESSOR_SCOPE,
                   "tcp_port",
                   "TCP Port",
                   "TCP command port",
                   DEFAULT_TCP_PORT,
                   MIN_PORT,
                   MAX_PORT);
    
    addIntParameter(Parameter::PROCESSOR_SCOPE,
                   "udp_port",
                   "UDP Port",
                   "UDP data port",
                   DEFAULT_UDP_PORT,
                   MIN_PORT,
                   MAX_PORT);
    
    addFloatParameter(Parameter::PROCESSOR_SCOPE, 
                     "data_scale", 
                     "Scale", 
                     "Data scale (µV per bit)", 
                     "", 
                     DEFAULT_DATA_SCALE, 
                     MIN_DATA_SCALE, 
                     MAX_DATA_SCALE, 
                     0.01f);
    addFloatParameter(Parameter::PROCESSOR_SCOPE, 
                     "aux_data_scale",
                     "Scale",
                     "Aux data scale (uV per bit)",
                     "", 
                     DEFAULT_AUX_DATA_SCALE, 
                     MIN_DATA_SCALE, 
                     MAX_DATA_SCALE, 
                     0.01f);
}

void IntanSocket::disconnectDevice()
{
    if (intanInterface)
    {
        intanInterface->stopAcquisition();
        stopImuStreamQuietly();
        intanInterface.reset();
    }
    
    getParameter("device_ip")->setEnabled(true);

    if (sn->getEditor() != nullptr)
        static_cast<IntanSocketEditor*>(sn->getEditor())->disconnected();
}

bool IntanSocket::connectDevice(bool printOutput)
{
    try
    {
        // Create IntanInterface with configured ports
        intanInterface = std::make_unique<IntanInterface>(
            device_ip.toStdString(),
            tcp_port,   // Use parameter value
            udp_port    // Use parameter value
        );
        
        if (!intanInterface->foundInputSource())
        {
            if (printOutput)
            {
                LOGE("Failed to connect to Intan device at ", device_ip);
                CoreServices::sendStatusMessage("GLANCE: Connection failed.");
            }
            intanInterface.reset();
            return false;
        }
        
        if (!intanInterface->isReady())
        {
            if (printOutput)
            {
                LOGE("GLANCE: device not ready");
                CoreServices::sendStatusMessage("GLANCE: Device not ready.");
            }
            intanInterface.reset();
            return false;
        }

        // Set up data callback
        intanInterface->setDataCallback(
            [this](const uint32_t* data, size_t words, uint64_t timestamp) {
                processDataPacket(data, words, timestamp);
            }
        );

        // LFP callback: each frame is one decimated sample across all enabled
        // LFP channels. Always wired -- silently does nothing until the LFP
        // engine is enabled in the firmware.
        intanInterface->setLfpDataCallback(
            [this](const IntanInterface::LfpFrame& f) {
                processLfpFrame(f);
            }
        );

        // IMU callback: one fused BNO055 sample per streaming port at 100 Hz.
        // Always wired -- silently does nothing until a stream is started.
        intanInterface->setImuDataCallback(
            [this](const IntanInterface::ImuSample& s) {
                processImuSample(s);
            }
        );

        // Set up error callback
        intanInterface->setErrorCallback(
            [this, printOutput](const std::string& error) {
                if (printOutput)
                {
                    LOGE("GLANCE error: ", error.c_str());
                }
                hasError = true;
            }
        );

        // Pull authoritative state from the device. The firmware retains the
        // channel-enable mask, debug mode, phase delays, and aux-sequencer
        // state across plugin disconnects (they live in PL registers, no NVM
        // but they survive the lifetime of the firmware). Adopt whatever the
        // device is in and let the editor mirror it. After any successful
        // RESCAN, reconnecting restores the prior chip indicators and channel
        // count without re-running detection.
        //
        // Exception: if the firmware just booted, channel_enable=0 (no streams
        // configured yet). Publishing a 0-channel DataStream crashes downstream
        // plugins on the next updateSignalChain (LFP Viewer dereferences a
        // null stream in saveParameters). Seed the firmware with 0x0F so the
        // signal chain has *something* valid -- the chip indicators still stay
        // dark because no real RESCAN has happened, so the user is prompted to
        // RESCAN in the obvious way.
        IntanInterface::DeviceStatus status;
        if (!intanInterface->getStatus(status))
        {
            // The board responded to the constructor's getStatus but not this
            // one -- likely a half-up TCP stack right after boot. With the
            // recv timeout in place we don't hang, but we DO need to refuse
            // the connection cleanly so the user can retry.
            if (printOutput)
            {
                LOGE("GLANCE: status read failed -- board may be still booting. "
                     "Wait until the ethernet activity LED is steady and try "
                     "CONNECT again.");
                CoreServices::sendStatusMessage("GLANCE: not ready, retry CONNECT");
            }
            intanInterface.reset();
            return false;
        }

        if (status.channelEnable == 0)
        {
            if (!intanInterface->setChannelEnable(0x0F))
            {
                if (printOutput)
                    LOGE("GLANCE: setChannelEnable failed during initial seed");
                intanInterface.reset();
                return false;
            }
            Thread::sleep(10);
            if (!intanInterface->getStatus(status))
            {
                if (printOutput)
                    LOGE("GLANCE: status re-read failed after channel-enable seed");
                intanInterface.reset();
                return false;
            }
        }

        channel_enable_mask = status.channelEnable;
        num_channels = calculateNumChannels(channel_enable_mask);
        debugMode = (status.debugMode != 0);

        // The board boots into the accel sweep (slot 0 cycles CONVERT 32->33->34) and
        // the de-interleave always runs in sweep form -- there is no alternate mode.
        // auxSeqMode is just a local UI-state flag; it stays true from connect on.

        // LFP/DSP engine state -- mirror whatever the device reports. There is
        // nothing to validate or repair here: the filters are loaded from the
        // bitstream at boot and the decimation is structural, so any state the
        // firmware reports is a state it can actually stream in.
        applyLfpStatus(status);

        // IMU: probe for a BNO055 on each port. This only succeeds on a fabric
        // that carries the AXI IICs (acq_imu_*/scan) -- on a plain acquisition
        // fabric the firmware refuses the command, which is a normal "no IMU
        // here" answer, not an error. Nothing is started yet: arming a port
        // does a blocking NDOF entry on the board, so it happens in
        // startAcquisition() before the neural stream runs.
        refreshImuState();

        // Fast-settle / TTL state: prefer the new aux_ctrl readback
        // (firmware 65d5fb5+) which surfaces the actual SW level and TTL
        // pin select. On older firmware, fall back to the live fs_active
        // bit (SW state isn't directly observable) and assume no TTL pin.
        if (status.hasAuxCtrl)
        {
            fastSettleSw  = status.fsSwLevel;
            fastSettleTTL = status.fsGpioEn ? (int)status.fsGpioPin : -1;
        }
        else
        {
            fastSettleSw  = status.hasAuxStatus && status.fastSettleActive;
            fastSettleTTL = -1;
        }

        if (printOutput)
        {
            LOGC("Connected to Intan device - mask 0x",
                 String::toHexString((int)channel_enable_mask),
                 " (", num_channels, " channels), debug=",
                 debugMode ? "ON" : "OFF");
            LOGC("Firmware: ", status.getFirmwareVersionString().c_str());
            CoreServices::sendStatusMessage("GLANCE: Connected successfully.");
        }

        getParameter("device_ip")->setEnabled(false);

        if (sn->getEditor() != nullptr)
        {
            auto* editor = static_cast<IntanSocketEditor*>(sn->getEditor());
            editor->connected();
            // Mirror the device's persisted state into the UI (chip indicators
            // from channel_enable, DBG button label from debug_mode). On a fresh
            // boot mask=0 -> no chips shown -> user clicks RESCAN. On reconnect
            // after a previous RESCAN, the prior indicators come back for free.
            editor->syncFromDeviceState(channel_enable_mask, debugMode);
        }

        return true;
    }
    catch (const std::exception& e)
    {
        if (printOutput)
        {
            LOGE("Exception connecting to Intan: ", e.what());
            CoreServices::sendStatusMessage("GLANCE: Connection error.");
        }
        intanInterface.reset();
        return false;
    }
}

bool IntanSocket::errorFlag()
{
    return hasError.load();
}

void IntanSocket::updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
                                 OwnedArray<EventChannel>* eventChannels,
                                 OwnedArray<SpikeChannel>* spikeChannels,
                                 OwnedArray<DataStream>* sourceStreams,
                                 OwnedArray<DeviceInfo>* devices,
                                 OwnedArray<ConfigurationObject>* configurationObjects)
{
    continuousChannels->clear();
    eventChannels->clear();
    devices->clear();
    spikeChannels->clear();
    configurationObjects->clear();
    sourceStreams->clear();

    bool generatesTimestamps = true;

    DataStream::Settings dataStreamSettings{
        // Open Ephys appends its own "-A"/"-B" suffix per stream, so keep the
        // base name descriptive of the CONTENT rather than the device.
        "BroadbandStream",
        "Broadband 30 kHz amplifier data",
        // Stream / channel identifiers stay `intan.*`: these land in recordings
        // and saved signal chains, so they are not free to change -- they are
        // not a display name. The product brand is GLANCE (see OpenEphysLib.cpp);
        // these name the Intan RHD data they carry.
        "intan.data",
        SAMPLE_RATE,
        generatesTimestamps
    };

    DataStream* stream = new DataStream (dataStreamSettings);

    sourceStreams->add(stream);
    
    // ------------------------------------------------------------------
    // Build the channel layout from the active stream mask (8-bit, dual-port).
    //
    // Bits 0-3 = port A: bit0=A_CIPO0_REG, bit1=A_CIPO0_DDR,
    //                    bit2=A_CIPO1_REG, bit3=A_CIPO1_DDR
    // Bits 4-7 = port B: bit4=B_CIPO0_REG, bit5=B_CIPO0_DDR,
    //                    bit6=B_CIPO1_REG, bit7=B_CIPO1_DDR
    //
    // The PL emits per acquisition cycle the enabled 16-bit segments in
    // bit order (0 to 7). Each stream carries 32 amplifier channels; only
    // the four "regular" streams additionally carry 3 aux inputs (the DDR
    // streams just resample the same aux, so their aux samples are dropped).
    //
    // Channel order here MUST match the fill order in updateBuffer():
    //   [stream0 CH1..32][stream1 CH1..32]...  then  [aux per regular stream]
    //
    // For single-port 0x0F this is byte-identical to the previous layout:
    //   A_CH1..A_CH128  then  A_AUX0_1..A_AUX0_3, A_AUX1_1..A_AUX1_3
    // (prefix "A_" added to names, but count and order unchanged)
    // ------------------------------------------------------------------
    int n_streams = countStreams(channel_enable_mask);
    int n_aux_banks = countAuxBanks(channel_enable_mask);

    if (n_streams == 0)
        LOGE("No channels enabled!");

    int n_neural_channels = n_streams * 32;
    num_channels = n_neural_channels + n_aux_banks * 3;

    // Resize buffer to exactly the number of channels we publish
    sourceBuffers[0]->resize(num_channels > 0 ? num_channels : 1,
                             SAMPLE_RATE * bufferSizeInSeconds);

    // Neural channels, grouped per stream (de-interleaved).
    // Streams for bits 0-3 get port prefix "A_", bits 4-7 get "B_".
    // Numbering is sequential within each port (CH1..CH32 per stream,
    // continuing across streams of the same port).
    int portA_neuralIdx = 0;  // running counter for port A channels
    int portB_neuralIdx = 0;  // running counter for port B channels
    for (int b = 0; b < 8; ++b)
    {
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;

        String portPrefix = (b < 4) ? "A_" : "B_";

        for (int k = 0; k < 32; ++k)
        {
            int chanNum;
            if (b < 4)
                chanNum = ++portA_neuralIdx;
            else
                chanNum = ++portB_neuralIdx;

            ContinuousChannel::Settings chanSettings{
                ContinuousChannel::Type::ELECTRODE,
                portPrefix + "CH" + String(chanNum),
                "Intan neural data channel",
                "intan.continuous.ephys",
                data_scale,
                stream
            };

            continuousChannels->add(new ContinuousChannel(chanSettings));
            continuousChannels->getLast()->setUnits("uV");
        }
    }

    // Aux channels: only for the "regular" streams, 3 per bank.
    // Bank mapping:
    //   bit 0 (A_CIPO0_REG) -> bank 0 -> A_AUX0_1..A_AUX0_3
    //   bit 2 (A_CIPO1_REG) -> bank 1 -> A_AUX1_1..A_AUX1_3
    //   bit 4 (B_CIPO0_REG) -> bank 2 -> B_AUX0_1..B_AUX0_3
    //   bit 6 (B_CIPO1_REG) -> bank 3 -> B_AUX1_1..B_AUX1_3
    // Bit positions of the four regular streams: 0, 2, 4, 6
    static const int auxRegularBits[4] = {0, 2, 4, 6};
    static const char* auxBankNames[4] = {"A_AUX0", "A_AUX1", "B_AUX0", "B_AUX1"};

    for (int bankIdx = 0; bankIdx < 4; ++bankIdx)
    {
        int b = auxRegularBits[bankIdx];
        if ((channel_enable_mask & (1 << b)) == 0)
            continue;

        for (int a = 0; a < 3; ++a)
        {
            ContinuousChannel::Settings channelSettings {
                ContinuousChannel::AUX,
                String(auxBankNames[bankIdx]) + "_" + String(a + 1),
                "Intan aux input channel",
                "intan.continuous.aux",
                aux_data_scale,
                stream
            };

            continuousChannels->add (new ContinuousChannel (channelSettings));
            // Aux samples are published as raw signed ADC counts (bitVolts=1.0);
            // expose them in arbitrary units with a range that covers the full
            // signed-16-bit window. Accelerometer signals sit well inside this,
            // and the LFP viewer's range selector zooms in from there.
            continuousChannels->getLast()->setUnits ("a.u.");
            continuousChannels->getLast()->inputRange = {-32768.0f, 32767.0f};
        }
    }

    // ============================================================
    // TTL EVENT CHANNELS
    // ============================================================
    EventChannel::Settings settings {
        EventChannel::Type::TTL,
        "Acquisition Board TTL Input",
        "Events on digital input lines of an Open Ephys Acquisition Board",
        "acq-board.rhythm.events",
        stream,
        8
    };

    eventChannels->add (new EventChannel (settings));

    LOGC("Configured ", n_neural_channels, " channels");

    // ------------------------------------------------------------------
    // SECOND DATASTREAM: decimated LFP band (firmware LFP engine).
    // Created only when the engine is enabled in the firmware at connect-
    // time. Sample rate = SAMPLE_RATE / lfp_decim_R; channel count = popcount
    // of the LFP lane mask * 32 (amplifier channels only -- no aux). The
    // user configures + enables the engine via an out-of-band tool (e.g.
    // net.py configure_lfp), then reconnects the plugin to publish the
    // stream into Open Ephys.
    //
    // (Pattern follows the Neuropixels plugin's AP / LFP stream pairing:
    //  one DataStream per band, parallel sourceBuffers index.)
    // ------------------------------------------------------------------
    if (lfp_enabled && lfp_num_channels > 0 && lfp_decim_R > 0)
    {
        float lfpSampleRate = SAMPLE_RATE / (float)lfp_decim_R;

        DataStream::Settings lfpSettings{
            "LFPStream",
            "Decimated LFP band",
            "intan.data.lfp",
            lfpSampleRate,
            generatesTimestamps
        };
        DataStream* lfpStream = new DataStream(lfpSettings);
        sourceStreams->add(lfpStream);

        // sourceBuffers is owned by the plugin (not auto-managed by OE) and
        // the constructor only creates [0] for the broadband stream. Add the
        // LFP buffer on first connect with LFP enabled; resize on subsequent
        // reconnects when the channel count / rate might have changed.
        int lfpBufferSamples = (int)(lfpSampleRate * bufferSizeInSeconds);
        if (sourceBuffers.size() < 2) {
            sourceBuffers.add(new DataBuffer(lfp_num_channels, lfpBufferSamples));
        } else {
            sourceBuffers[1]->resize(lfp_num_channels, lfpBufferSamples);
        }

        // Channel naming mirrors the broadband layout but with an LFP_
        // prefix: LFP_A_CH1.., LFP_B_CH1.. Lane order follows the same
        // bit-order packing the firmware uses (low->high bit, A then B).
        int portA_idx = 0;
        int portB_idx = 0;
        for (int b = 0; b < 8; ++b)
        {
            if ((lfp_lane_mask & (1 << b)) == 0)
                continue;
            String portPrefix = (b < 4) ? "LFP_A_" : "LFP_B_";
            for (int k = 0; k < 32; ++k)
            {
                int chanNum = (b < 4) ? ++portA_idx : ++portB_idx;
                ContinuousChannel::Settings ls{
                    ContinuousChannel::Type::ELECTRODE,
                    portPrefix + "CH" + String(chanNum),
                    "Intan LFP-band neural data channel",
                    "intan.continuous.lfp",
                    data_scale,                  // same 0.195 µV/LSB as broadband
                    lfpStream
                };
                continuousChannels->add(new ContinuousChannel(ls));
                continuousChannels->getLast()->setUnits("uV");
            }
        }

        LOGC("Configured LFP stream: ", lfp_num_channels, " channels @ ",
             (int)lfpSampleRate, " Hz (mask=0x",
             String::toHexString((int)lfp_lane_mask),
             ", decim=", (int)lfp_decim_R,
             ", taps=", (int)lfp_num_taps, ")");
    }

    // ------------------------------------------------------------------
    // THIRD stream: IMU (stream_type = 3), published only when a BNO055
    // actually answered at connect time. 10 channels per streaming port --
    // quaternion w/x/y/z (unitless), accel x/y/z (m/s^2), gyro x/y/z (deg/s)
    // -- at the BNO055's 100 Hz fusion rate. This is a low-rate side channel;
    // it shares the unified UDP port but never the 30 kHz path.
    // ------------------------------------------------------------------
    imu_buffer_index = -1;
    if (imu_enabled && imu_num_channels > 0)
    {
        DataStream::Settings imuSettings{
            "IMUStream",
            "Headstage IMU (BNO055 fusion)",
            "intan.data.imu",
            IMU_SAMPLE_RATE,
            generatesTimestamps
        };
        DataStream* imuStream = new DataStream(imuSettings);
        sourceStreams->add(imuStream);

        // Build the buffer BEFORE publishing the index that names it, and do
        // the whole thing under imuMutex. Publishing first let the demux thread
        // enter addToBuffer() on a slot that DataBuffer::resize() was about to
        // free and reallocate -- a segfault inside memcpy, reachable whenever
        // the board's IMU stream is live while Open Ephys is stopped.
        {
            std::lock_guard<std::mutex> lock(imuMutex);
            imu_buffer_index = -1;          // demux thread stands down first
            int idx = sourceStreams->size() - 1;
            int imuBufferSamples = (int)(IMU_SAMPLE_RATE * bufferSizeInSeconds);
            if (imuBufferSamples < 1000) imuBufferSamples = 1000;  // 100 Hz: keep depth generous
            while (sourceBuffers.size() <= idx)
                sourceBuffers.add(new DataBuffer(imu_num_channels, imuBufferSamples));
            sourceBuffers[idx]->resize(imu_num_channels, imuBufferSamples);
            imuConvBuf.assign((size_t)imu_num_channels, 0.0f);
            imu_buffer_index = idx;         // ... and only now may it resume
        }

        // Channel names carry the physical quantity and unit, so the value is
        // readable in the GUI without consulting this source.
        static const char* kAxisNames[IMU_CHANS_PER_PORT] = {
            "QUAT_W", "QUAT_X", "QUAT_Y", "QUAT_Z",
            "ACC_X", "ACC_Y", "ACC_Z", "GYR_X", "GYR_Y", "GYR_Z"
        };
        static const char* kAxisUnits[IMU_CHANS_PER_PORT] = {
            "", "", "", "", "m/s^2", "m/s^2", "m/s^2", "deg/s", "deg/s", "deg/s"
        };
        // bitVolts is NOT cosmetic here: the record engine stores
        // int16 = sample / bitVolts (BinaryRecording.cpp scales by
        // 1/(0x7fff*bitVolts) then re-multiplies by 0x7fff), so the effective
        // full scale is 32767 x bitVolts. Publishing engineering units with
        // bitVolts = 1.0 quantised every channel to whole units -- which
        // ANNIHILATES a quaternion (|q| <= 1 -> 0 or +-1), left accel in 1 m/s^2
        // steps, and let only gyro survive because its numbers happen to be
        // large. Setting bitVolts to the BNO055's native LSB makes the stored
        // int16 exactly the raw sensor count: lossless, and the full int16 range
        // is used.
        //
        // inputRange is what the LFP viewer's per-AUX auto-scale reads
        // (LfpDisplay::updateRange -> channelMetadata inputRangeMin/Max), so
        // each quantity gets a sensible display range instead of sharing the
        // ±5000 default that made a ±1 quaternion invisible.
        static const float kAxisBitVolts[IMU_CHANS_PER_PORT] = {
            1.0f / 16384.0f, 1.0f / 16384.0f, 1.0f / 16384.0f, 1.0f / 16384.0f, // quat: 1 = 2^14
            0.01f, 0.01f, 0.01f,                                                // accel: 1 LSB = 0.01 m/s^2
            1.0f / 16.0f, 1.0f / 16.0f, 1.0f / 16.0f                            // gyro: 1 LSB = 1/16 deg/s
        };
        // Full scale of the sensor as NDOF configures it: quaternion is a unit
        // quaternion, accel is +-4 g, gyro is +-2000 deg/s.
        static const float kAxisRange[IMU_CHANS_PER_PORT] = {
            1.0f, 1.0f, 1.0f, 1.0f,
            39.24f, 39.24f, 39.24f,
            2000.0f, 2000.0f, 2000.0f
        };
        for (int p = 0; p < 2; ++p)
        {
            if (p == 0 && !imu_port_a) continue;
            if (p == 1 && !imu_port_b) continue;
            String portPrefix = (p == 0) ? "IMU_A_" : "IMU_B_";
            for (int k = 0; k < IMU_CHANS_PER_PORT; ++k)
            {
                ContinuousChannel::Settings is{
                    ContinuousChannel::Type::AUX,
                    portPrefix + kAxisNames[k],
                    "Headstage IMU (BNO055 NDOF fusion)",
                    "intan.continuous.imu",
                    kAxisBitVolts[k],   // -> stored int16 is the raw BNO055 count
                    imuStream
                };
                continuousChannels->add(new ContinuousChannel(is));
                continuousChannels->getLast()->setUnits(kAxisUnits[k]);
                continuousChannels->getLast()->inputRange.min = -kAxisRange[k];
                continuousChannels->getLast()->inputRange.max = +kAxisRange[k];
            }
        }

        LOGC("Configured IMU stream: ", imu_num_channels, " channels @ 100 Hz (port",
             imu_port_a && imu_port_b ? "s A+B)" : (imu_port_a ? " A)" : " B)"));
    }
}

bool IntanSocket::foundInputSource()
{
    return intanInterface && intanInterface->foundInputSource();
}

bool IntanSocket::isReady()
{
    return intanInterface && intanInterface->isReady();
}

void IntanSocket::parameterValueChanged(Parameter* parameter)
{
    if (parameter->getName() == "device_ip")
    {
        device_ip = parameter->getValueAsString();
    }
    else if (parameter->getName() == "tcp_port")
    {
        tcp_port = (int)parameter->getValue();
    }
    else if (parameter->getName() == "udp_port")
    {
        udp_port = (int)parameter->getValue();
    }
    else if (parameter->getName() == "data_scale")
    {
        data_scale = (float)parameter->getValue();
    }
    else if (parameter->getName() == "aux_data_scale")
    {
        aux_data_scale = (float)parameter->getValue();
    }
}

bool IntanSocket::startAcquisition()
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("Cannot start acquisition - device not ready");
        return false;
    }

#ifdef __APPLE__
    // Keep macOS from App-Napping the plugin-container process a few seconds in and
    // starving the UDP recv thread (loss that starts ~10 s into acquisition).
    disableAppNap();
#endif
    
    // Resize buffers - ONE time sample per packet across all channels
    convbuf.resize(num_channels);      // one time sample per channel

    totalSamples = 0;
    eventState = 0;
    hasError = false;
    
    // Clear any old data
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!dataQueue.empty())
            dataQueue.pop();
    }
    
    // Initialize device (sends INIT sequence, then CONVERT sequence)
    if (!intanInterface->loadInitSequence())
    {
        LOGE("Failed to load init sequence");
        return false;
    }
    Thread::sleep(100);  // Wait for initialization
    
    if (!intanInterface->loadConvertSequence())
    {
        LOGE("Failed to load convert sequence");
        return false;
    }
    
    // Thread::sleep(100);  // Wait for initialization
    
    // if (!intanInterface->setDebugMode(true))
    // {
    //     LOGE("Failed to set debug mode.");
    //     return false;
    // }
 

    if (!intanInterface->setLoopCount(0)) // Loop count 0 for infinite streaming
    {
        LOGE("Failed to set loop count to infinite");
        return false;
    }
    Thread::sleep(10);

    // Arm the IMU stream BEFORE the neural stream. Starting a port does a
    // blocking ~50 ms NDOF entry on the board, which the firmware refuses once
    // acquisition is running -- so this ordering is required, not incidental.
    // A failure here is not fatal to the recording: log it and stream neural
    // data anyway (the IMU channels stay at their last values).
    if (imu_enabled)
    {
        // imuConvBuf is deliberately NOT touched here -- the demux thread owns
        // it (see the header). Samples may already be in flight because the
        // board's IMU stream outlives a neural stop.
        imuSampleCounter.store(0, std::memory_order_relaxed);
        IntanInterface::ImuPorts want, active;
        want.portA = imu_port_a;
        want.portB = imu_port_b;
        if (!intanInterface->setImuStream(want, 0 /* default 100 Hz */, active))
        {
            LOGE("GLANCE: could not start the IMU stream (continuing without it)");
        }
        else if (active.portA != want.portA || active.portB != want.portB)
        {
            LOGE("GLANCE: IMU stream started on fewer ports than expected "
                 "(A=", (int)active.portA, " B=", (int)active.portB, ")");
        }
    }

    // Start acquisition on device
    if (!intanInterface->startAcquisition())
    {
        LOGE("Failed to start acquisition on device");
        // The IMU was armed a few lines up. Unwind it, or it streams into a
        // session that never started -- and the board keeps sending after the
        // user has been told acquisition failed.
        if (imu_enabled) stopImuStreamQuietly();
        return false;
    }
    
    LOGC("GLANCE acquisition started");
    startThread();
    
    return true;
}

bool IntanSocket::stopAcquisition()
{
    if (isThreadRunning())
    {
        signalThreadShouldExit();
        queueCv_.notify_all();   // wake updateBuffer() if it's blocked so run() exits now
    }

    if (intanInterface)
    {
        intanInterface->stopAcquisition();

        // Stop the IMU stream too: it has an independent lifecycle on the
        // board (it survives a neural stop), but leaving it running would keep
        // pushing samples into a stream Open Ephys has stopped reading.
        // Stopping never blocks, so it is safe here.
        if (imu_enabled)
        {
            stopImuStreamQuietly();

            IntanInterface::ImuStats istats;
            intanInterface->getImuStats(istats);
            if (istats.samples[0] || istats.samples[1])
                LOGC("GLANCE IMU: ", (int)istats.samples[0], " samples port A / ",
                     (int)istats.samples[1], " port B, SEQ gaps A=",
                     (int)istats.seqGaps[0], " B=", (int)istats.seqGaps[1]);
        }
    }

    sourceBuffers[0]->clear();
    if (imu_buffer_index > 0 && sourceBuffers.size() > imu_buffer_index)
        sourceBuffers[imu_buffer_index]->clear();

    LOGC("GLANCE acquisition stopped");
    return true;
}

void IntanSocket::processDataPacket(const uint32_t* data, size_t wordCount, uint64_t timestamp)
{
    // Called from IntanInterface's UDP thread
    // Queue the packet for processing in updateBuffer()

    std::unique_lock<std::mutex> lock(queueMutex);

    if (dataQueue.size() >= kMaxDataQueue) {
        // Consumer fell behind -> drop the OLDEST and count it, rather than grow
        // the queue without bound (see kMaxDataQueue note). A counted, bounded
        // drop here is far better than the uncounted allocator-stall spiral.
        // Recycle the dropped buffer instead of freeing it.
        if (bufferPool_.size() < kBufferPoolMax)
            bufferPool_.push_back(std::move(dataQueue.front().data));
        dataQueue.pop();
        dataQueueDrops_.fetch_add(1, std::memory_order_relaxed);
    }

    // Reuse a pooled buffer so this hot path (running ON the demux thread) does NO
    // per-packet heap alloc -- assign() into an already-sized buffer reuses its
    // capacity. A fresh alloc here is what churns the allocator and stalls the demux
    // -> ring backup -> SEQ gaps (net.py avoids exactly this by never allocating).
    DataPacket packet;
    if (!bufferPool_.empty()) {
        packet.data = std::move(bufferPool_.back());
        bufferPool_.pop_back();
    }
    packet.data.assign(data, data + wordCount);   // reuses capacity: no realloc
    packet.timestamp = timestamp;

    dataQueue.push(std::move(packet));   // move, not copy
    lock.unlock();
    queueCv_.notify_one();               // wake the (blocked) DataThread
}

void IntanSocket::processLfpFrame(const IntanInterface::LfpFrame& frame)
{
    // Called from IntanInterface's LFP listener thread. If no second
    // DataStream was published (LFP wasn't enabled at connect time), there's
    // no sourceBuffers[1] to push into -- silently drop.
    if (!lfp_enabled || lfp_num_channels <= 0) return;
    if (sourceBuffers.size() < 2) return;
    if ((int)frame.sampleCount != lfp_num_channels) return;  // mask/cfg drift

    // Convert offset-binary uint16 -> signed float in uV, matching broadband
    // scaling. One time sample across all channels per frame.
    if ((int)lfpConvBuf.size() != lfp_num_channels)
        lfpConvBuf.resize(lfp_num_channels);

    for (int ch = 0; ch < lfp_num_channels; ++ch)
        lfpConvBuf[ch] = (float)((int)frame.samples[ch] - 32768) * data_scale;

    // Use the frame's timestamp (= frame_seq * decim_R, in broadband ticks --
    // aligns with the broadband stream). One TTL event word per sample;
    // we don't have a per-frame digital_in latch on the LFP path, so keep
    // it constant at eventState (no transitions on this stream).
    int64 lfpSampleNumber = (int64)frame.frameSequence;
    double lfpTimestamp = (double)frame.timestamp;

    sourceBuffers[1]->addToBuffer(lfpConvBuf.data(),
                                  &lfpSampleNumber,
                                  &lfpTimestamp,
                                  &eventState,
                                  1);  // ONE time sample
}

void IntanSocket::processImuSample(const IntanInterface::ImuSample& sample)
{
    // Called from IntanInterface's demux thread. No stream published (no IMU
    // at connect time, or the board started streaming a port we didn't
    // publish) -> nothing to push into.
    // Everything below reads geometry the GUI thread can rewrite (updateSettings,
    // refreshImuState), so it is all inside one lock. 100 Hz -- affordable.
    std::lock_guard<std::mutex> lock(imuMutex);

    if (imu_buffer_index < 0 || imu_num_channels <= 0) return;
    if (sourceBuffers.size() <= imu_buffer_index) return;
    if (sample.port == 0 && !imu_port_a) return;
    if (sample.port == 1 && !imu_port_b) return;

    if ((int)imuConvBuf.size() != imu_num_channels)
        imuConvBuf.assign((size_t)imu_num_channels, 0.0f);

    // Each port owns a fixed 10-channel block; port A first when both stream.
    // The two ports sample independently (staggered on the board), so a packet
    // updates only its own block and the other port's last values are carried
    // forward -- a sample-and-hold, which is the honest representation of two
    // asynchronous sensors on one DataStream.
    int base = (sample.port == 1 && imu_port_a) ? IMU_CHANS_PER_PORT : 0;
    if (base + IMU_CHANS_PER_PORT > (int)imuConvBuf.size())
        return;                    // geometry says this port has no block here
    for (int i = 0; i < 4; ++i) imuConvBuf[base + i]     = sample.quat[i];
    for (int i = 0; i < 3; ++i) imuConvBuf[base + 4 + i] = sample.accel[i];
    for (int i = 0; i < 3; ++i) imuConvBuf[base + 7 + i] = sample.gyro[i];

    // Sample numbers must be monotonic across the stream; the per-port SEQ is
    // not (two ports interleave), so count pushes locally. The board's PL
    // timestamp still rides along, which is what aligns IMU with neural data.
    int64 imuSampleNumber = imuSampleCounter.fetch_add(1, std::memory_order_relaxed);
    double imuTimestamp = (double)sample.timestamp;

    sourceBuffers[imu_buffer_index]->addToBuffer(imuConvBuf.data(),
                                                 &imuSampleNumber,
                                                 &imuTimestamp,
                                                 &eventState,
                                                 1);  // ONE time sample
}

bool IntanSocket::updateBuffer()
{
    if (hasError)
    {
        return false;
    }

    // Periodic visibility into the DOWNSTREAM (post-SEQ-check) drop stage. If
    // dataQueueDrops_ climbs, OpenEphys's own consumer (this DataThread ->
    // sourceBuffer -> processing graph/rendering) can't keep up at ~28k pkts/s.
    // That loss is SILENT -- it happens AFTER the demux SEQ check, so it does NOT
    // appear as a SEQ gap. Together with the [GLANCE][DROP] ring log this
    // pins the stage: ringDrops => demux starved (upstream); dataQueueDrops => OE
    // too slow (here). If BOTH are flat but OE still loses, it's the OE sourceBuffer.
    {
        static auto lastLog = std::chrono::steady_clock::now();
        static uint64_t lastDrops = 0;
        auto now = std::chrono::steady_clock::now();
        if (now - lastLog > std::chrono::seconds(5)) {
            lastLog = now;
            uint64_t d = dataQueueDrops_.load(std::memory_order_relaxed);
            size_t qsz;
            { std::lock_guard<std::mutex> lock(queueMutex); qsz = dataQueue.size(); }
            int sb = sourceBuffers[0]->getNumSamples();
            // Standing pipeline latency per stage -- should hover near ZERO (the big
            // buffers are for burst absorption, not steady occupancy). sourceBuffer
            // fill / SAMPLE_RATE is the display latency you feel; dataQueue should stay
            // ~empty now that updateBuffer drains to empty. If sourceBuffer stays deep
            // while dataQueue is ~0, the standing depth is in OE's own buffer (a paced
            // consumer), not ours.
            std::cout << "[IntanSocket][LATENCY] sourceBuffer=" << sb << " samp ("
                      << (sb * 1000.0 / SAMPLE_RATE) << " ms), dataQueue=" << qsz
                      << "/" << kMaxDataQueue;
            if (d != lastDrops)
                std::cout << "  DROPS +" << (d - lastDrops) << "/5s (total " << d << ")";
            std::cout << std::endl;
            lastDrops = d;
        }
    }

    // BLOCK (bounded) until a packet is queued instead of returning immediately. The OE
    // DataThread::run() loop calls updateBuffer() with NO sleep, so a fast return on an
    // empty queue spins a whole CPU core -- self-inflicted load that starves the recv/
    // kernel path (net.py blocks on recv and uses ~1/4 the CPU). The 100 ms cap lets
    // run() still poll threadShouldExit() promptly.
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        if (dataQueue.empty() && !hasError)
            queueCv_.wait_for(lock, std::chrono::milliseconds(100),
                              [this] { return !dataQueue.empty() || hasError || threadShouldExit(); });
    }

    // Drain EVERY queued packet this call so the dataQueue can never build a standing
    // backlog -> latency (the "big buffer, kept empty" rule: the buffer is for burst
    // absorption, not steady occupancy). OE calls updateBuffer in a tight loop, but its
    // per-call overhead can drop the effective rate below the ~30 kHz arrival rate;
    // one-packet-per-call then leaves a PERMANENT queue that shows up as display lag.
    // Looping to empty decouples our drain from OE's call cadence. Nothing is dropped.
    std::vector<uint32_t> recycle;   // previous packet's buffer, returned to the pool
    while (true)
    {
    DataPacket packet;
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        // Return the previous iteration's buffer to the pool under the SAME lock as
        // the next pop -- recycling costs no extra lock. The producer then reuses it
        // (no per-packet alloc). capacity()>0 distinguishes a real buffer from a
        // moved-from empty one.
        if (recycle.capacity() > 0 && bufferPool_.size() < kBufferPoolMax)
            bufferPool_.push_back(std::move(recycle));
        recycle = std::vector<uint32_t>{};   // moved-from -> guaranteed empty/no capacity
        if (dataQueue.empty())
            break;

        packet = std::move(dataQueue.front());   // move out, no copy
        dataQueue.pop();
    }

    // UNIFIED broadband header: 8-word common header + 6-word sub-block = 14
    // header words ahead of the data (docs/unified-packet-format.md). Timestamp
    // is the common header's w2/w3 (unchanged offset).
    static constexpr size_t kBroadbandHeaderWords = 14;

    int64 timestamp = (static_cast<uint64_t>(packet.data.data()[3]) << 32) | packet.data.data()[2];

    // Skip the full 14-word unified broadband header; the data words that
    // follow are unchanged by the header reformat.
    const uint32_t* dataWords = packet.data.data() + kBroadbandHeaderWords;
    size_t numDataWords = packet.data.size() - kBroadbandHeaderWords;

    // (Per-second PACKET DEBUG dump removed -- it was log spam; totalSamples
    // is still incremented at the end of this function as a sample counter,
    // preserving the upstream fix that made it actually count.)

    // Each packet contains ONE time sample for every channel.
    //
    // The data is a tight stream of 16-bit samples (two per 32-bit word, low
    // half first). For each acquisition cycle c (0..34) the PL appends the
    // enabled streams in bit order: bit0=CIPO0 reg, bit1=CIPO0 DDR,
    // bit2=CIPO1 reg, bit3=CIPO1 DDR. So a stream's sample for cycle c sits at
    // flat index  c * nStreams + slot,  where slot is its rank among the
    // enabled streams (== its bit-order rank).
    //
    // Two corrections vs. a naive copy:
    //   1. De-interleave: group each stream's 35 cycles back together.
    //   2. Undo the 2-cycle SPI pipeline delay: the result of COPI command i
    //      arrives in cycle (i+2) mod 35. The convert sequence issues
    //      amplifier ch 0..31 then 3 aux reads, so amplifier ch k lands in
    //      cycle (k+2), and aux 0/1/2 land in cycles 34/0/1.
    // Amplifier output is offset binary (baseline 0x8000), converted to signed
    // counts. Aux is physically unsigned but is centered the same way for
    // display (see the aux loop below).

    convbuf.resize(num_channels);

    // Scan all 8 bits: bits 0-3 = port A, bits 4-7 = port B.
    int streamBits[8];
    int nStreams = 0;
    for (int b = 0; b < 8; ++b)
    {
        if (channel_enable_mask & (1 << b))
            streamBits[nStreams++] = b;
    }

    // One-shot diagnostic: print the actual runtime sizes the first packet of
    // each acquisition. If numDataWords < 140 or nStreams != 8 at 0xFF, the
    // packet size pipeline is broken somewhere — do NOT remove until verified.
    {
        static uint32_t lastReportedMask = 0;
        if (channel_enable_mask != lastReportedMask)
        {
            lastReportedMask = channel_enable_mask;
            LOGC("[ephys-socket diag] mask=0x", String::toHexString((int)channel_enable_mask),
                 " nStreams=", nStreams,
                 " packet.data.size=", (int)packet.data.size(),
                 " numDataWords=", (int)numDataWords,
                 " num_channels=", num_channels);
        }
    }

    // Re-scan (the loop above was moved up so we can report nStreams);
    // this second pass is a no-op but kept for clarity of the original flow.
    nStreams = 0;
    for (int b = 0; b < 8; ++b)
        if (channel_enable_mask & (1 << b))
            streamBits[nStreams++] = b;

    const int nDataSamples = 35 * nStreams;

    // Fetch one 16-bit sample by flat index into the de-interleaved data.
    auto sampleAt = [&](int flatIdx) -> uint16_t {
        if (flatIdx < 0 || flatIdx >= nDataSamples)
            return 0x8000;  // midscale -> 0 after offset-binary conversion
        int wordIdx = flatIdx / 2;
        if ((size_t)wordIdx >= numDataWords)
            return 0x8000;
        if ((flatIdx & 1) == 0)
            return (uint16_t)(dataWords[wordIdx] & 0xFFFF);       // low 16 bits
        return (uint16_t)((dataWords[wordIdx] >> 16) & 0xFFFF);   // high 16 bits
    };

    int outCh = 0;

    // Neural channels: de-interleaved, de-skewed, and converted exactly as the
    // acquisition-board plugin does -- (raw_offset_binary - 32768) * 0.195 --
    // so the buffer carries true microvolts. With bitVolts = data_scale = 0.195
    // the record node stores (raw - 32768): the exact signed ADC count,
    // full-range and lossless (see storage note in README.md).
    for (int s = 0; s < nStreams; ++s)
    {
        for (int k = 0; k < 32; ++k)
        {
            int cycle = (k + 2) % 35;
            int flat = cycle * nStreams + s;
            convbuf[outCh++] = (float)((int)sampleAt(flat) - 32768) * data_scale;
        }
    }

    // Aux channels: only the "regular" streams carry aux inputs.
    // Regular stream bit positions and their aux bank indices:
    //   bit 0 (A_CIPO0_REG) -> aux bank 0
    //   bit 2 (A_CIPO1_REG) -> aux bank 1
    //   bit 4 (B_CIPO0_REG) -> aux bank 2
    //   bit 6 (B_CIPO1_REG) -> aux bank 3
    // (DDR streams at bits 1, 3, 5, 7 just resample the same aux -- drop them.)
    //
    // Converted exactly as the acquisition-board plugin does:
    //   (raw - 32768) * 0.0000374
    // The record node therefore stores (raw - 32768): the exact signed ADC
    // count, lossless. The midscale subtraction is a constant, reversible
    // representation choice, not a baseline/detrend.
    //
    // The accel sweep lives on aux slot 0 (cycle 32), so its command echo AND its
    // reply ride the SAME packet: header word 6 [31:16] carries the CONVERT(32|33|34)
    // command -- the axis label -- and the +2 SPI readback puts that axis's sample in
    // data word 34 of this very packet. De-interleave by echo with sample-and-hold so
    // the 3 output channels stay at the full 30 kHz buffer rate (each axis refreshes
    // every 3rd packet). Because label and sample never cross a packet boundary, a
    // dropped packet can't mislabel an axis.
    //   (Data words 0/1 = slots 1/2's replies -- the fs 'I' register read and the
    //    inject register / injected-read result -- neither is accelerometer data.)
    //
    // The aux engine is ALWAYS ON (aux_flags bit0 is hardwired 1), so there is no
    // "plain per-axis" fallback to select -- every packet is in sweep form.
    //
    // UNIFIED header field mapping (docs/unified-packet-format.md, net.py
    // print_aux_info):
    //   AUX1 = common-header word 6 = {echo_sweep[31:16], aux_flags[15:8], digital_in[7:0]}
    //   sub-block word 8           = {echo_slot2_prev[31:16], echo_slot1_prev[15:0]}
    // The accel command echo that drives the de-interleave is in the HIGH 16 bits of
    // word 6 (THIS packet). Port B uses the SAME header echo as port A.
    {
        uint32_t auxWord = packet.data.data()[6];   // AUX1 (flags + digital_in + sweep echo)

        // Mapping from stream bit position to aux bank index.
        // Regular streams are at even bit positions 0, 2, 4, 6.
        //   bit 0 -> bank 0, bit 2 -> bank 1, bit 4 -> bank 2, bit 6 -> bank 3
        // DDR streams (odd bits) do not carry independent aux data.
        auto auxBankForBit = [](int b) -> int {
            // b must be 0, 2, 4, or 6
            return b / 2;
        };

        uint16_t echo0 = (auxWord >> 16) & 0xFFFF;   // slot-0 (accel) cmd, answered @ data word 34
        bool isConvert = (echo0 & 0xC000) == 0;
        int convCh = (echo0 >> 8) & 0x3F;

        for (int s = 0; s < nStreams; ++s)
        {
            int b = streamBits[s];
            if ((b & 1) != 0)
                continue;  // skip DDR streams (odd bits)
            int bank = auxBankForBit(b);

            if (isConvert && convCh >= 32 && convCh <= 34)
                lastAccel[bank][convCh - 32] = sampleAt(34 * nStreams + s);

            for (int a = 0; a < 3; ++a)
                convbuf[outCh++] = (float)((int)lastAccel[bank][a] - 32768) * aux_data_scale;
        }
    }

    // Safety net: zero any channels we somehow didn't fill.
    for (; outCh < num_channels; ++outCh)
        convbuf[outCh] = 0.0f;
    
    // digital_in[7:0] now lives in the LOW byte of AUX1 (common header word 6).
    uint64 ttlEventWord = packet.data.data()[6] & 0xFFu; // digital input = low 8 bits of AUX1


    double ts;
    
    sourceBuffers[0]->addToBuffer(convbuf.data(),
                                   &timestamp,
                                   &ts,
                                   &ttlEventWord,
                                   1);  // ONE time sample

    totalSamples++;
    recycle = std::move(packet.data);   // hold this buffer for the next iteration's lock
    }  // end while: drain the next queued packet (keep the dataQueue empty)

    return true;
}

// Re-read which ports carry a BNO055 and size the IMU stream from it.
//
// EVERY path that can change IMU geometry must come through here, for the same
// reason applyLfpStatus() exists for the LFP band (CLAUDE.md hard rule 1): the
// stream is published from these fields at updateSettings() time, so a stale
// value silently yields no IMU channels -- or channels that never fill -- with
// nothing in the log. Today that is connect and RESCAN; a third caller means
// calling this, not copying it.
// Stop the board's IMU stream, tolerating failure. Called from the paths that
// tear a session down, because the board's IMU stream does NOT stop by itself:
// it survives a neural stop, a disconnect, and an Open Ephys crash. Leaving it
// running is not cosmetic -- the firmware refuses DETECT_IMU while a port is
// streaming, so the NEXT connect censuses nothing and publishes no IMU stream
// even though the IMU is sitting there transmitting.
void IntanSocket::stopImuStreamQuietly()
{
    if (!intanInterface) return;
    IntanInterface::ImuPorts none, active;
    if (!intanInterface->setImuStream(none, 0, active))
    {
        LOGE("GLANCE: could not stop the board's IMU stream -- it will keep "
             "sending; a RESCAN or reconnect will clear it");
    }
    else if (active.portA || active.portB)
    {
        LOGE("GLANCE: board still reports IMU ports streaming after stop");
    }
}

void IntanSocket::refreshImuState()
{
    // The census is a blocking TCP round trip, so it happens OUTSIDE the lock.
    // Holding imuMutex across it would park the demux thread for the command
    // timeout, and the recv->demux ring would back up behind it -- trading an
    // IMU race for broadband loss, which is a strictly worse bargain.
    IntanInterface::ImuPorts present;
    bool censused = intanInterface && intanInterface->detectImu(present);

    // Publish the new geometry as ONE atomic step. The demux thread reads these
    // four fields together; a torn view (new channel count, stale port flag)
    // indexes imuConvBuf past its end -- a 40-byte heap overflow on exactly the
    // A+B -> A-only rescan this function exists to handle.
    {
        std::lock_guard<std::mutex> lock(imuMutex);
        imu_port_a = censused && present.portA;
        imu_port_b = censused && present.portB;
        imu_enabled = imu_port_a || imu_port_b;
        imu_num_channels = IMU_CHANS_PER_PORT *
                           ((imu_port_a ? 1 : 0) + (imu_port_b ? 1 : 0));
        imuConvBuf.clear();        // re-sized to the new geometry on next sample
    }

    if (!censused)
    {
        LOGC("GLANCE: no IMU census available on this fabric -- no IMU stream");
    }
    else
    {
        LOGC("GLANCE: IMU census -- port A ", imu_port_a ? "yes" : "no",
             ", port B ", imu_port_b ? "yes" : "no",
             imu_enabled ? " -> publishing IMU stream" : " -> no IMU stream");
    }
}

// What the RESCAN button means: work out what is plugged in NOW. That is a
// fabric decision before it is a phase decision -- a headstage with an IMU
// needs an acq_imu_* fabric, and the phase sweep can only run once the right
// fabric is live. Chip detection alone (the old behaviour) could never notice
// an IMU, so the button appeared to ignore them.
bool IntanSocket::rescanDevice(IntanInterface::AutoDetectionResult& result)
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("GLANCE: cannot rescan - device not ready");
        return false;
    }

    IntanInterface::ImuPorts present;
    if (!intanInterface->rescanFabric(present))
        return false;          // could not load a fabric; nothing else is valid

    // A fabric swap resets PL state, so EVERY geometry the plugin caches is now
    // stale -- not just the IMU's. Hard rule 1 applies here as much as anywhere:
    // pl_config_apply() calls pl_lfp_set_config(enable = 0), so the swap turns
    // the LFP engine OFF, and the PCAP reprogram resets the channel-enable
    // register. Re-read the device instead of trusting what we held before.
    // Without this, a RESCAN that finds no chips leaves the plugin believing
    // LFP is on and publishing a stream that records pure zeros, silently --
    // which is the exact failure that rule exists to prevent.
    IntanInterface::DeviceStatus status;
    if (intanInterface->getStatus(status))
    {
        channel_enable_mask = status.channelEnable;
        num_channels = calculateNumChannels(channel_enable_mask);
        applyLfpStatus(status);
    }
    else
    {
        LOGE("GLANCE: could not re-read status after the fabric swap -- "
             "LFP/channel geometry may be stale; reconnect before recording");
    }

    refreshImuState();         // geometry for the stream published below
    if (!runAutoDetection(result, true))
        return false;

    // Seating check. An IMU answering on a cable's I2C bus proves the headstage
    // is plugged in and powered, so if that same cable's CIPO0 scores no chip,
    // the likely cause is a partly-seated connector rather than an empty port --
    // the two lanes come through the same Omnetics shell. Worth saying out loud:
    // the recording would otherwise start with that port silently contributing
    // nothing, and the operator finds out afterwards.
    //
    // net.py deliberately does not special-case this (it already prints the
    // census and the per-lane scores for a human to read); the notification
    // belongs here, where someone is actually looking while setting up.
    const bool aFault = present.portA && !result.cipo0Detected;
    const bool bFault = present.portB && !result.portBCipo0Detected;
    if (aFault || bFault)
    {
        const char* which = aFault && bFault ? "A and B" : (aFault ? "A" : "B");
        LOGE("GLANCE: port ", which, " has an IMU but no chip on CIPO0 -- "
             "check the headstage is fully seated");
        CoreServices::sendStatusMessage(
            juce::String("GLANCE: port ") + which +
            " IMU found but no chip - check headstage seating");
    }
    return true;
}

bool IntanSocket::runAutoDetection(IntanInterface::AutoDetectionResult& result, bool verbose)
{
    if (!intanInterface || !intanInterface->isReady())
    {
        LOGE("Cannot run auto-detection - device not ready");
        return false;
    }
    
    // Run detection
    bool success = intanInterface->runAutoDetection(result, verbose);
    
    if (success && result.success)
    {
        LOGC("Auto-detection complete: ", result.getChannelSummary().c_str());
    }
    
    return success;
}

// Adopt the firmware's LFP geometry. The frame consumer drops any frame whose
// sampleCount disagrees with lfp_num_channels, so every path that can change the
// board's lane mask must come through here -- otherwise the stale count silently
// discards the entire stream and the LFP band reads as zeros rather than as an
// error. The lane mask mirrors the broadband channel-enable on the board, which
// is why auto-detection changes it too.
void IntanSocket::applyLfpStatus(const IntanInterface::DeviceStatus& s)
{
    if (s.hasLfpStatus && s.lfpEnabled && s.lfpLaneMask != 0 && s.lfpDecimR != 0)
    {
        lfp_enabled   = true;
        lfp_lane_mask = s.lfpLaneMask;
        lfp_decim_R   = s.lfpDecimR;
        lfp_num_taps  = s.lfpNumTaps;
        int popcount = 0;
        for (int b = 0; b < 8; ++b)
            popcount += ((lfp_lane_mask >> b) & 1);
        lfp_num_channels = popcount * 32;
    }
    else
    {
        lfp_enabled      = false;
        lfp_lane_mask    = 0;
        lfp_decim_R      = 0;
        lfp_num_taps     = 0;
        lfp_num_channels = 0;
    }
}

bool IntanSocket::applyDetectionConfig(const IntanInterface::AutoDetectionResult& result)
{
    if (!intanInterface || !result.success)
    {
        return false;
    }
    
    // Apply configuration from detection
    if (!intanInterface->applyDetectionConfig(result))
    {
        LOGE("Failed to apply detection configuration");
        return false;
    }
    
    // Update local channel enable state
    channel_enable_mask = result.optimalChannelMask;
    num_channels = calculateNumChannels(channel_enable_mask);

    // The board derives the LFP lane mask from the broadband channel-enable we
    // just changed, so the LFP frames are now a different size. Re-read the
    // geometry before the signal chain is rebuilt: leaving it stale makes the
    // frame consumer reject every frame as drift, and the LFP band goes silently
    // to zero until the engine is toggled off and on.
    IntanInterface::DeviceStatus s;
    if (intanInterface->getStatus(s))
        applyLfpStatus(s);

    LOGC("Applied detection config - ", num_channels, " channels enabled",
         lfp_enabled ? String(", LFP ") + String(lfp_num_channels) + " ch"
                     : String(", LFP off"));

    return true;
}
void IntanSocket::setDebugMode(bool enable, uint8_t mask)
{
    debugMode = enable;

    if (debugMode)
    {
        // ====================================================================
        // ENABLE DEBUG MODE
        // ====================================================================

        bool dualPort = (mask & 0xF0) != 0;
        LOGC("Enabling debug mode - mask 0x", String::toHexString((int)mask),
             dualPort ? " (dual-port 268ch)" : " (single-port 134ch)");

        // Check if we have a connection to the hardware
        if (!intanInterface || !intanInterface->isReady())
        {
            LOGE("Cannot enable debug mode - device not ready");
            CoreServices::sendStatusMessage("GLANCE: Debug mode failed - device not connected");
            debugMode = false;
            return;
        }

        // Step 1: Send hardware debug mode enable command (0x12 SET_DEBUG_MODE)
        if (!intanInterface->setDebugMode(true))
        {
            LOGE("Failed to send debug mode enable command to hardware");
            CoreServices::sendStatusMessage("GLANCE: Failed to enable hardware debug mode");
            debugMode = false;
            return;
        }

        Thread::sleep(50);  // Let hardware switch to debug mode

        // Step 2: Apply the requested channel-enable mask.
        if (!intanInterface->setChannelEnable(mask))
        {
            LOGE("Failed to set channel enable for debug mode");
            CoreServices::sendStatusMessage("GLANCE: Failed to configure channels");
            debugMode = false;
            return;
        }

        Thread::sleep(50);  // Let channel config take effect

        // Step 5: Update local configuration
        channel_enable_mask = mask;
        num_channels = calculateNumChannels(channel_enable_mask);

        LOGC("Debug mode enabled - mask 0x", String::toHexString((int)mask),
             ", channels=", num_channels);

        // Step 6: Update the chip display in the editor
        if (sn->getEditor() != nullptr)
        {
            IntanSocketEditor* editor = static_cast<IntanSocketEditor*>(sn->getEditor());

            // Fake detection result mirroring the requested mask. Port A is
            // lit whenever any of the low-nibble bits are on; port B only
            // when the high nibble is on.
            IntanInterface::AutoDetectionResult debugResult;
            debugResult.success = true;
            debugResult.chipsDetected = true;
            debugResult.cipo0Detected = (mask & 0x01) != 0;
            debugResult.cipo1Detected = (mask & 0x04) != 0;
            debugResult.cipo0ChipType = debugResult.cipo0Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.cipo1ChipType = debugResult.cipo1Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.cipo0HasDdr = false;
            debugResult.cipo1HasDdr = false;
            debugResult.portBCipo0Detected = (mask & 0x10) != 0;
            debugResult.portBCipo1Detected = (mask & 0x40) != 0;
            debugResult.portBCipo0ChipType = debugResult.portBCipo0Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.portBCipo1ChipType = debugResult.portBCipo1Detected ? IntanInterface::ChipType::RHD2164 : IntanInterface::ChipType::NONE;
            debugResult.portBCipo0HasDdr = false;
            debugResult.portBCipo1HasDdr = false;
            debugResult.optimalChannelMask = mask;
            debugResult.bestPhase0 = 0;
            debugResult.bestPhase1 = 0;

            editor->updateChipDetection(debugResult);
        }

        // Step 7: Update the signal chain to reflect new channel count
        CoreServices::updateSignalChain(sn->getEditor());
        CoreServices::sendStatusMessage(dualPort
            ? "GLANCE: Debug mode enabled (dual-port, 268 channels)"
            : "GLANCE: Debug mode enabled (single-port, 134 channels)");
    }
    else
    {
        // ====================================================================
        // DISABLE DEBUG MODE
        // ====================================================================
        
        LOGC("Disabling debug mode");
        
        // Check if we have a connection to the hardware
        if (!intanInterface || !intanInterface->isReady())
        {
            LOGD("Debug mode disabled (device not connected)");
            debugMode = false;
            return;
        }
        
        // Step 1: Send hardware debug mode disable command (0x12 SET_DEBUG_MODE with param1=0)
        if (!intanInterface->setDebugMode(false))
        {
            LOGE("Failed to send debug mode disable command to hardware");
            // Continue anyway - we want to update the UI
        }
        
        Thread::sleep(50);
        
        // Step 2: Reset to default channel configuration
        // Note: You may want to restore the previous channel enable state
        // For now, we'll set it to a reasonable default (all channels)
        if (!intanInterface->setChannelEnable(0x0F))
        {
            LOGE("Failed to reset channel enable");
        }
        
        Thread::sleep(50);
        
        // Step 4: Clear the chip displays
        if (sn->getEditor() != nullptr)
        {
            IntanSocketEditor* editor = static_cast<IntanSocketEditor*>(sn->getEditor());
            
            // Clear the chip displays - user should run RESCAN to detect real chips
            IntanInterface::AutoDetectionResult clearResult;
            clearResult.success = false;
            clearResult.chipsDetected = false;
            clearResult.cipo0Detected = false;
            clearResult.cipo1Detected = false;
            clearResult.cipo0ChipType = IntanInterface::ChipType::NONE;
            clearResult.cipo1ChipType = IntanInterface::ChipType::NONE;
            
            editor->updateChipDetection(clearResult);
        }
        
        LOGC("Debug mode disabled - use RESCAN to detect actual chips");
        CoreServices::sendStatusMessage("GLANCE: Debug mode disabled - run RESCAN");
    }
}
// ============================================================================
// AUX COMMAND SEQUENCER TOOLING (firmware aux-seq-v2)
// ============================================================================

void IntanSocket::printDeviceStatus()
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Cannot print status - device not connected");
        CoreServices::sendStatusMessage("GLANCE: not connected");
        return;
    }

    IntanInterface::DeviceStatus status;
    if (!intanInterface->getStatus(status))
    {
        LOGE("Failed to read device status");
        CoreServices::sendStatusMessage("GLANCE: status read failed");
        return;
    }

    // Emit line-by-line so each line gets the console prefix
    std::istringstream iss(status.getSummary());
    std::string line;
    while (std::getline(iss, line))
        LOGC(line);

    // Plugin-side reception stats. These tell us whether UDP packets are being
    // lost (timestampErrors), arriving with wrong size (sizeErrors), or with
    // a bad magic header (magicErrors). A growing timestampErrors with a stable
    // packetsReceived from the device == packet drops between firmware and host
    // (network or BRAM ring overrun). Growing PL fifoCount or PS errorCount in
    // the device status == firmware can't keep up with PL writes.
    IntanInterface::ReceptionStats rx;
    intanInterface->getReceptionStats(rx);
    LOGC("--- Plugin Reception ---");
    LOGC("Packets recv: ", (int64)rx.totalPackets,
         "  totalErr: ", (int64)rx.totalErrors,
         "  magicErr: ", (int64)rx.magicErrors,
         "  tsErr: ", (int64)rx.timestampErrors,
         "  sizeErr: ", (int64)rx.sizeErrors);
    LOGC("Rate: ", rx.instantRate, " pkt/s (", rx.dataRateMbps, " Mbps)");

    CoreServices::sendStatusMessage("GLANCE: status printed to console");
}

bool IntanSocket::pushFastSettleConfig()
{
    if (!intanInterface)
        return false;
    bool gpioEn = fastSettleTTL >= 0;
    // Default: amplifier fast settle (RHD Reg-0 D5) and DSP reset (CONVERT
    // bit-H) follow the SAME trigger -- both the SETTLE button's software
    // level and the TTL Settle pin. Configurability for independent DSP
    // triggering can be added later.
    return intanInterface->setFastSettle(fastSettleSw, gpioEn,
                                         gpioEn ? (uint8_t)fastSettleTTL : 0,
                                         true /* DSP follows fast settle */);
}

void IntanSocket::setManualFastSettle(bool active)
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Fast settle: device not connected");
        return;
    }

    fastSettleSw = active;
    if (pushFastSettleConfig())
    {
        LOGC("Fast settle ", active ? "ON" : "OFF",
             " (RHD Reg-0 D5 via the override whole-replacing the fs slot)");
        CoreServices::sendStatusMessage(active ? "GLANCE: FAST SETTLE ON"
                                               : "GLANCE: fast settle off");
    }
    else
    {
        LOGE("Fast settle command failed");
    }
}

void IntanSocket::setFastSettleTTLPin(int pin)
{
    if (!intanInterface || !intanInterface->foundInputSource())
        return;

    fastSettleTTL = (pin >= 0 && pin <= 7) ? pin : -1;
    if (pushFastSettleConfig())
    {
        // (LOGC expands to a statement with its own ';' -- keep braces)
        if (fastSettleTTL >= 0)
        {
            LOGC("Fast settle following digital_in[", fastSettleTTL, "]");
        }
        else
        {
            LOGC("TTL fast settle disabled");
        }
    }
}

bool IntanSocket::setAuxSequencerMode(bool enable)
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("Aux sequencer: device not connected");
        return false;
    }

    // The accelerometer sweep is the one and only aux configuration -- the board
    // boots into it (slot 0 cycles CONVERT 32->33->34) and the de-interleave always
    // runs in sweep form. This call (re)asserts that config, uploading into the
    // STANDBY bank and swapping it live so it doubles as the double-buffer test.
    // `enable` is retained for API/UI compatibility; there is no alternate mode.
    (void)enable;

    IntanInterface::DeviceStatus status;
    if (!intanInterface->getStatus(status))
    {
        LOGE("Aux sequencer: status read failed");
        return false;
    }
    if (!status.hasAuxStatus)
    {
        LOGE("This firmware predates the aux sequencer (86-byte status) - "
             "update BOOT.bin to the aux-seq-v2 build");
        CoreServices::sendStatusMessage("GLANCE: firmware lacks aux sequencer");
        return false;
    }

    // Target bank for slot 0: if the sweep is already running, write the STANDBY
    // bank and swap - this exercises the live double-buffer + atomic packet-boundary
    // swap. Otherwise start on bank 0. Only slot 0 (the program) has a bank.
    int progBank = status.auxSeqEnabled ? ((status.auxBankActive & 1) ^ 1) : 0;

    // The standard aux config (mirrors remote/net.py aux_demo_setup):
    //   slot 0 (cycle 32): the accel sweep, one axis per packet -- the ONLY cycling
    //           slot; its reply pairs intra-packet at data word 34.
    //   slot 1 (cycle 33): the fs register -- reads the INTAN ROM 'I' (register 40);
    //           the override whole-replaces it on a fast-settle edge.
    //   slot 2 (cycle 34): the inject register -- reads the temperature channel;
    //           injection whole-replaces it on demand.
    std::vector<uint16_t> sweep  = { IntanInterface::rhdConvert(32),
                                     IntanInterface::rhdConvert(33),
                                     IntanInterface::rhdConvert(34) };
    std::vector<uint16_t> fsReg  = { IntanInterface::rhdRead(40) };    // 'I' of INTAN
    std::vector<uint16_t> injReg = { IntanInterface::rhdConvert(49) }; // temperature channel

    if (!intanInterface->auxUploadBank(0, progBank, sweep, 0) ||
        !intanInterface->auxUploadBank(1, 0, fsReg, 0) ||
        !intanInterface->auxUploadBank(2, 0, injReg, 0))
    {
        LOGE("Aux bank upload failed");
        return false;
    }

    // Only slot 0 (the program) has a bank to swap; slots 1 and 2 are registers.
    if (!intanInterface->auxBankSelect(0, progBank))
    {
        LOGE("Aux bank select failed (slot 0, bank ", progBank, ")");
        return false;
    }

    // (No enable step: the aux engine is always on -- uploading + selecting the
    // bank is all that's needed.)

    // Reset the de-interleave state for all 4 aux banks to midscale
    for (int b = 0; b < 4; ++b)
        for (int a = 0; a < 3; ++a)
            lastAccel[b][a] = 0x8000;

    auxSeqMode = true;
    if (status.auxSeqEnabled)
    {
        LOGC("Aux accel sweep reloaded LIVE via standby-bank swap (slot 0 now on bank ",
             progBank, ")");
    }
    else
    {
        LOGC("Aux accel sweep active - intra-packet de-interleave (10 kHz/axis)");
    }
    CoreServices::sendStatusMessage("GLANCE: aux sweep active");
    return true;
}


bool IntanSocket::setLfpEnabled(bool enable)
{
    if (!intanInterface || !intanInterface->foundInputSource())
    {
        LOGE("LFP: device not connected");
        return false;
    }

    // Enabling is all that is needed: the board boots with both filters of the
    // decimation cascade already loaded, so there is nothing to configure. The
    // plugin must NOT push coefficients here -- the filters are generated by
    // programmable_logic/sim/design_lfp_filters.py, and re-deriving them in the
    // plugin would let the two drift apart. Changing the filter (linear vs
    // minimum phase) is done from remote/net.py. See docs/lfp.md.
    if (enable)
    {
        IntanInterface::DeviceStatus s;
        if (!intanInterface->getStatus(s))
        {
            LOGE("LFP: status read failed before enable");
            return false;
        }
        if (!s.hasLfpStatus)
        {
            LOGE("LFP: this firmware does not expose the LFP engine");
            CoreServices::sendStatusMessage("GLANCE: firmware lacks LFP engine");
            return false;
        }
    }

    if (!intanInterface->lfpEnable(enable))
    {
        LOGE("LFP: ", enable ? "enable" : "disable", " command failed");
        return false;
    }

    // Re-read status so our local LFP state (which gates the second
    // DataStream in updateSettings) reflects the new firmware state.
    IntanInterface::DeviceStatus s;
    if (intanInterface->getStatus(s))
    {
        applyLfpStatus(s);
        if (lfp_enabled)
        {
            LOGC("LFP enabled - ", lfp_num_channels, " channels @ ",
                 (int)(SAMPLE_RATE / lfp_decim_R), " Hz");
            CoreServices::sendStatusMessage("GLANCE: LFP stream ON");
        }
        else
        {
            LOGC("LFP disabled");
            CoreServices::sendStatusMessage("GLANCE: LFP stream off");
        }
    }
    return true;
}
