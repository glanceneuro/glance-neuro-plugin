// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "IntanSocketEditor.h"
#include "IntanSocket.h"

using namespace IntanSocketNode;

// ============================================================================
// PortInterface Implementation
// ============================================================================

PortInterface::PortInterface(IntanSocket* node_, const String& portName_)
    : node(node_), portName(portName_)
{
}

void PortInterface::updateCipo0Status(bool detected, IntanInterface::ChipType type)
{
    cipo0Detected = detected;
    cipo0Type = type;
    repaint();
}

void PortInterface::updateCipo1Status(bool detected, IntanInterface::ChipType type)
{
    cipo1Detected = detected;
    cipo1Type = type;
    repaint();
}

void PortInterface::reset()
{
    cipo0Detected = cipo1Detected = false;
    cipo0Type = cipo1Type = IntanInterface::ChipType::NONE;
    repaint();
}

void PortInterface::paintChipBox(Graphics& g, int x, bool detected, IntanInterface::ChipType type)
{
    if (detected)
    {
        g.setColour(Colour(255, 145, 0));
        g.fillRoundedRectangle(x, 1, 23, 15, 3.0f);
        g.setColour(Colours::black);
        g.setFont(FontOptions("Inter", "Bold", 10.0f));
        String chipLabel = (type == IntanInterface::ChipType::RHD2164) ? "64"
                         : (type == IntanInterface::ChipType::RHD2132) ? "32" : "??";
        g.drawText(chipLabel, x, 1, 23, 15, Justification::centred, false);
    }
    else
    {
        g.setColour(Colour(80, 80, 80));
        g.fillRoundedRectangle(x, 1, 23, 15, 3.0f);
    }
}

void PortInterface::paint(Graphics& g)
{
    // Background
    g.setColour(findColour(ThemeColours::componentBackground).darker(0.2f));
    g.fillRoundedRectangle(5, 0, getWidth() - 10, getHeight(), 4.0f);

    // Port letter
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Bold", 13.0f));
    g.drawText(portName, 9, 1, 14, 15, Justification::centred, false);

    // Two chip boxes: CIPO0 at x=25, CIPO1 at x=50
    paintChipBox(g, 25, cipo0Detected, cipo0Type);
    paintChipBox(g, 50, cipo1Detected, cipo1Type);
}

// ============================================================================
// SampleRateInterface Implementation
// ============================================================================

SampleRateInterface::SampleRateInterface(IntanSocket* node_)
    : node(node_)
{
    rateLabel = std::make_unique<Label>("SampleRate", "30.0 kS/s");
    rateLabel->setBounds(0, 14, 80, 20);
    rateLabel->setJustificationType(Justification::centred);
    rateLabel->setColour(Label::backgroundColourId, Colour(60, 60, 60));
    rateLabel->setColour(Label::textColourId, Colours::white);
    addAndMakeVisible(rateLabel.get());
}

void SampleRateInterface::paint(Graphics& g)
{
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Regular", 10.0f));
    g.drawText("Sample Rate", 0, 0, 80, 15, Justification::left, false);
}

// ============================================================================
// BandwidthInterface Implementation
// ============================================================================

BandwidthInterface::BandwidthInterface(IntanSocket* node_)
    : node(node_)
    , lowerBandwidth(1.0)
    , upperBandwidth(7500.0)
{
    lowerBandwidthLabel = std::make_unique<Label>("LowerBW", "1");
    lowerBandwidthLabel->setEditable(true, false, false);
    lowerBandwidthLabel->addListener(this);
    lowerBandwidthLabel->setBounds(25, 10, 50, 20);
    addAndMakeVisible(lowerBandwidthLabel.get());
    
    upperBandwidthLabel = std::make_unique<Label>("UpperBW", "7500");
    upperBandwidthLabel->setEditable(true, false, false);
    upperBandwidthLabel->addListener(this);
    upperBandwidthLabel->setBounds(25, 25, 50, 20);
    addAndMakeVisible(upperBandwidthLabel.get());
}

void BandwidthInterface::labelTextChanged(Label* label)
{
    // Placeholder - will be implemented when hardware supports bandwidth control
    if (label == lowerBandwidthLabel.get())
    {
        double value = label->getText().getDoubleValue();
        if (value >= 0.1 && value <= 500.0)
        {
            lowerBandwidth = value;
        }
        else
        {
            label->setText(String(lowerBandwidth), dontSendNotification);
            CoreServices::sendStatusMessage("Lower bandwidth must be between 0.1 and 500 Hz");
        }
    }
    else if (label == upperBandwidthLabel.get())
    {
        double value = label->getText().getDoubleValue();
        if (value >= 100.0 && value <= 20000.0)
        {
            upperBandwidth = value;
        }
        else
        {
            label->setText(String(upperBandwidth), dontSendNotification);
            CoreServices::sendStatusMessage("Upper bandwidth must be between 100 and 20000 Hz");
        }
    }
}

void BandwidthInterface::setLowerBandwidth(double value)
{
    lowerBandwidth = value;
    lowerBandwidthLabel->setText(String(value), dontSendNotification);
}

void BandwidthInterface::setUpperBandwidth(double value)
{
    upperBandwidth = value;
    upperBandwidthLabel->setText(String(value), dontSendNotification);
}

void BandwidthInterface::paint(Graphics& g)
{
    g.setColour(findColour(ThemeColours::defaultText));
    g.setFont(FontOptions("Inter", "Regular", 10.0f));
    g.drawText("Bandwidth", 0, 0, 200, 11, Justification::left, false);
    g.drawText("Low:", 0, 11, 200, 15, Justification::left, false);
    g.drawText("High:", 0, 26, 200, 15, Justification::left, false);
}

// ============================================================================
// IntanSocketEditor Implementation
// ============================================================================

IntanSocketEditor::IntanSocketEditor(GenericProcessor* parentNode, IntanSocket* socket)
    : GenericEditor(parentNode)
{
    node = socket;
    desiredWidth = 425;

    // Port A and B chip detection (each row: port letter + two CIPO boxes)
    portAInterface = std::make_unique<PortInterface>(node, "A");
    portAInterface->setBounds(3, 28, 76, 18);
    addAndMakeVisible(portAInterface.get());

    portBInterface = std::make_unique<PortInterface>(node, "B");
    portBInterface->setBounds(3, 48, 76, 18);
    addAndMakeVisible(portBInterface.get());

    // Connect/Disconnect buttons
    connectButton = std::make_unique<UtilityButton>(stringConnect);
    connectButton->setFont(FontOptions("Small Text", 12, Font::bold));
    connectButton->setRadius(3.0f);
    connectButton->setBounds(6, 75, 65, 18);
    connectButton->addListener(this);
    addAndMakeVisible(connectButton.get());

    disconnectButton = std::make_unique<UtilityButton>(stringDisconnect);
    disconnectButton->setFont(FontOptions("Small Text", 12, Font::bold));
    disconnectButton->setRadius(3.0f);
    disconnectButton->setBounds(6, 75, 65, 18);
    disconnectButton->addListener(this);
    addAndMakeVisible(disconnectButton.get());
    disconnectButton->setVisible(false);
    
    // Rescan button
    rescanButton = std::make_unique<UtilityButton>("RESCAN");
    rescanButton->setRadius(3.0f);
    rescanButton->setBounds(6, 96, 65, 18);
    rescanButton->addListener(this);
    rescanButton->setTooltip("Auto-detect connected chips");
    addAndMakeVisible(rescanButton.get());
    rescanButton->setVisible(false);

    // Debug mode buttons: single-port (0x0F → 134 ch) vs dual-port (0xFF → 268 ch)
    debugMode1PButton = std::make_unique<UtilityButton>("DBG 1P");
    debugMode1PButton->setBounds(6, 117, 32, 18);
    debugMode1PButton->addListener(this);
    debugMode1PButton->setTooltip("Single-port debug: synthetic sine on port A only (mask 0x0F, 128 neural + 6 aux)");
    addAndMakeVisible(debugMode1PButton.get());
    debugMode1PButton->setVisible(false);

    debugMode2PButton = std::make_unique<UtilityButton>("DBG 2P");
    debugMode2PButton->setBounds(40, 117, 32, 18);
    debugMode2PButton->addListener(this);
    debugMode2PButton->setTooltip("Dual-port debug: synthetic sine on both ports (mask 0xFF, 256 neural + 12 aux)");
    addAndMakeVisible(debugMode2PButton.get());
    debugMode2PButton->setVisible(false);

    debugModeState = DebugMode::Off;

    // ------------------------------------------------------------------
    // Aux sequencer test tooling (firmware aux-seq-v2). These three work
    // DURING acquisition -- that is their purpose.
    // ------------------------------------------------------------------
    // Always visible; greyed out until a device is connected.
    statusButton = std::make_unique<UtilityButton>("STATUS");
    statusButton->setFont(FontOptions("Small Text", 12, Font::bold));
    statusButton->setRadius(3.0f);
    statusButton->setBounds(345, 28, 72, 18);
    statusButton->addListener(this);
    statusButton->setTooltip("Print full device status (incl. aux sequencer) to the console");
    addAndMakeVisible(statusButton.get());
    statusButton->setEnabledState(false);

    fastSettleButton = std::make_unique<UtilityButton>("SETTLE");
    fastSettleButton->setFont(FontOptions("Small Text", 12, Font::bold));
    fastSettleButton->setRadius(3.0f);
    fastSettleButton->setBounds(345, 53, 72, 18);
    fastSettleButton->addListener(this);
    fastSettleButton->setTooltip("Toggle amplifier fast settle (RHD Reg-0 D5) - hold for ~250 us per datasheet");
    addAndMakeVisible(fastSettleButton.get());
    fastSettleButton->setEnabledState(false);

    fastSettleActive = false;
    // The accelerometer sweep is always on -- the board boots into it and the
    // plugin de-interleaves it unconditionally -- so there is no aux-mode toggle.

    lfpEnableButton = std::make_unique<UtilityButton>("LFP");
    lfpEnableButton->setFont(FontOptions("Small Text", 12, Font::bold));
    lfpEnableButton->setRadius(3.0f);
    lfpEnableButton->setBounds(345, 103, 72, 18);
    lfpEnableButton->addListener(this);
    lfpEnableButton->setTooltip("Toggle the firmware LFP/DSP engine. LFP frames arrive on "
                                "the SAME unified UDP port as broadband (default 0x6800), "
                                "tagged stream_type=2. Filter design and channel mask are "
                                "set out-of-band -- run remote/net.py configure_lfp(...) "
                                "FIRST, then enable here. Toggling re-runs updateSettings.");
    addAndMakeVisible(lfpEnableButton.get());
    lfpEnableButton->setEnabledState(false);

    lfpActive = false;

    // Sample rate interface
    sampleRateInterface = std::make_unique<SampleRateInterface>(node);
    sampleRateInterface->setBounds(80, 22, 80, 50);
    addAndMakeVisible(sampleRateInterface.get());
    
    // Bandwidth interface
    bandwidthInterface = std::make_unique<BandwidthInterface>(node);
    bandwidthInterface->setBounds(80, 60, 80, 45);
    addAndMakeVisible(bandwidthInterface.get());
    
    // TTL Settle dropdown
    ttlSettleLabel = std::make_unique<Label>("TTL Settle", "TTL Settle");
    ttlSettleLabel->setFont(FontOptions("Inter", "Regular", 10.0f));
    ttlSettleLabel->setBounds(80, 100, 70, 20);
    addAndMakeVisible(ttlSettleLabel.get());
    
    ttlSettleCombo = std::make_unique<ComboBox>("TTLSettleCombo");
    ttlSettleCombo->setBounds(80, 115, 60, 18);
    ttlSettleCombo->addListener(this);
    ttlSettleCombo->addItem("-", 1);
    for (int k = 0; k < 8; k++)
    {
        ttlSettleCombo->addItem("TTL" + String(1 + k), 2 + k);
    }
    ttlSettleCombo->setSelectedId(1, dontSendNotification);
    ttlSettleCombo->setEnabled(false);   // enabled only when connected
    addAndMakeVisible(ttlSettleCombo.get());
    
    // Add parameter editors for IP/ports on the right side
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "device_ip", 170, 25);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "tcp_port", 170, 60);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "udp_port", 255, 25);
    addTextBoxParameterEditor(Parameter::PROCESSOR_SCOPE, "data_scale", 255, 60);

    for (auto& ed : parameterEditors)
    {
        ed->setLayout(ParameterEditor::Layout::nameOnTop);
        ed->setBounds(ed->getX(), ed->getY(), 75, 30);
    }
}

void IntanSocketEditor::startAcquisition()
{
    rescanButton->setEnabledState(false);
    disconnectButton->setEnabled(false);
    disconnectButton->setAlpha(0.2f);
    debugMode1PButton->setEnabledState(false);
    debugMode2PButton->setEnabledState(false);
    // LFP toggle would change the source-stream count -> unsafe mid-acquisition
    lfpEnableButton->setEnabledState(false);
}

void IntanSocketEditor::stopAcquisition()
{
    if (node->errorFlag())
    {
        node->disconnectDevice();
    }

    rescanButton->setEnabledState(true);
    disconnectButton->setEnabled(true);
    disconnectButton->setAlpha(1.0f);
    debugMode1PButton->setEnabledState(true);
    debugMode2PButton->setEnabledState(true);
    lfpEnableButton->setEnabledState(true);
}

void IntanSocketEditor::buttonClicked(Button* button)
{
    if (button == connectButton.get() && !acquisitionIsActive)
    {
        node->connectDevice();
        CoreServices::updateSignalChain(this);
    }
    else if (button == disconnectButton.get() && !acquisitionIsActive)
    {
        node->disconnectDevice();
    }
    else if (button == rescanButton.get() && !acquisitionIsActive)
    {
        // Run auto-detection
        IntanInterface::AutoDetectionResult result;
        if (node->runAutoDetection(result, true))
        {
            updateChipDetection(result);
            
            if (result.success)
            {
                // Apply the configuration
                node->applyDetectionConfig(result);
                CoreServices::sendStatusMessage("Intan: Auto-detection successful");
                CoreServices::updateSignalChain(this);
            }
            else
            {
                CoreServices::sendStatusMessage("Intan: No chips detected");
            }
        }
    }
    else if (button == statusButton.get())
    {
        // Works during acquisition by design
        node->printDeviceStatus();
    }
    else if (button == fastSettleButton.get())
    {
        // Works during acquisition by design (that is the point of settle)
        fastSettleActive = !fastSettleActive;
        node->setManualFastSettle(fastSettleActive);
        refreshAuxButtons();
    }
    else if (button == lfpEnableButton.get() && !acquisitionIsActive)
    {
        // Stream count changes -> needs an updateSignalChain afterwards so OE
        // rebuilds the signal chain with / without the second DataStream.
        // Restricted to !acquisition because adding a stream mid-acquisition
        // would invalidate downstream node settings.
        bool target = !lfpActive;
        if (node->setLfpEnabled(target))
        {
            lfpActive = target;
            CoreServices::updateSignalChain(this);
        }
        refreshAuxButtons();
    }
    else if ((button == debugMode1PButton.get() || button == debugMode2PButton.get())
             && !acquisitionIsActive)
    {
        DebugMode target;
        if (button == debugMode1PButton.get())
            target = (debugModeState == DebugMode::SinglePort) ? DebugMode::Off : DebugMode::SinglePort;
        else
            target = (debugModeState == DebugMode::DualPort) ? DebugMode::Off : DebugMode::DualPort;

        if (target == DebugMode::Off)
        {
            node->setDebugMode(false);
        }
        else
        {
            uint8_t mask = (target == DebugMode::SinglePort) ? 0x0F : 0xFF;
            node->setDebugMode(true, mask);
        }
        debugModeState = target;
        refreshDebugButtons();
    }

}

void IntanSocketEditor::comboBoxChanged(ComboBox* comboBox)
{
    if (comboBox == ttlSettleCombo.get())
    {
        // Id 1 = "-" (off); ids 2..9 = TTL1..TTL8 -> digital_in pins 0..7.
        // Fast settle then follows the selected pin level (sampled once per
        // packet in the PL; edge -> one injection packet each way).
        int selectedId = ttlSettleCombo->getSelectedId();
        node->setFastSettleTTLPin(selectedId >= 2 ? (selectedId - 2) : -1);
        refreshAuxButtons();
    }
}

void IntanSocketEditor::refreshDebugButtons()
{
    auto reset = [](UtilityButton* b, const String& lbl) {
        b->setLabel(lbl);
        b->setColour(TextButton::buttonColourId, b->findColour(TextButton::buttonColourId));
    };
    auto active = [](UtilityButton* b, const String& lbl) {
        b->setLabel(lbl);
        b->setColour(TextButton::buttonColourId, Colours::orange.darker(0.3f));
    };

    switch (debugModeState)
    {
        case DebugMode::Off:
            reset(debugMode1PButton.get(), "DBG 1P");
            reset(debugMode2PButton.get(), "DBG 2P");
            break;
        case DebugMode::SinglePort:
            active(debugMode1PButton.get(), "1P: ON");
            reset(debugMode2PButton.get(), "DBG 2P");
            break;
        case DebugMode::DualPort:
            reset(debugMode1PButton.get(), "DBG 1P");
            active(debugMode2PButton.get(), "2P: ON");
            break;
    }
}

void IntanSocketEditor::refreshAuxButtons()
{
    // Pull authoritative state from the node (it may have auto-enabled the
    // sequencer for fast settle, or synced state on reconnect)
    fastSettleActive = node->isFastSettleOn();

    if (fastSettleActive)
    {
        fastSettleButton->setLabel("SETTLE: ON");
        fastSettleButton->setColour(TextButton::buttonColourId, Colours::red.darker(0.2f));
    }
    else
    {
        fastSettleButton->setLabel("SETTLE");
        fastSettleButton->setColour(TextButton::buttonColourId,
                                    findColour(TextButton::buttonColourId));
    }

    lfpActive = node->isLfpEnabled();
    if (lfpActive)
    {
        lfpEnableButton->setLabel("LFP: ON");
        lfpEnableButton->setColour(TextButton::buttonColourId,
                                   Colours::green.darker(0.3f));
    }
    else
    {
        lfpEnableButton->setLabel("LFP");
        lfpEnableButton->setColour(TextButton::buttonColourId,
                                   findColour(TextButton::buttonColourId));
    }
}

void IntanSocketEditor::connected()
{
    connectButton->setVisible(false);
    disconnectButton->setVisible(true);
    rescanButton->setVisible(true);
    debugMode1PButton->setVisible(true);
    debugMode2PButton->setVisible(true);
    statusButton->setEnabledState(true);
    fastSettleButton->setEnabledState(true);
    lfpEnableButton->setEnabledState(true);
    refreshAuxButtons();   // sync with device state (persists across reconnect)

    // Pull the TTL fast-settle pin from the device (aux_ctrl readback,
    // firmware 65d5fb5+) so the combo reflects whatever is actually live --
    // a reconnect after a previous configuration restores the prior pin for
    // free, no spurious write needed. On older firmware (no aux_ctrl field),
    // the node falls back to pushing the combo's current selection.
    ttlSettleCombo->setEnabled(true);
    int devicePin = node->getDeviceTTLFastSettlePin();   // -1 = off, 0..7 = pin
    if (devicePin >= 0)
        ttlSettleCombo->setSelectedId(devicePin + 2, dontSendNotification);
    else
        ttlSettleCombo->setSelectedId(1, dontSendNotification);  // "-"
}

void IntanSocketEditor::disconnected()
{
    connectButton->setVisible(true);
    disconnectButton->setVisible(false);
    rescanButton->setVisible(false);
    debugMode1PButton->setVisible(true);
    debugMode2PButton->setVisible(true);
    statusButton->setEnabledState(false);
    fastSettleButton->setEnabledState(false);
    lfpEnableButton->setEnabledState(false);

    // TTL Settle combo: disable while disconnected and reset to "-" so a
    // future reconnect can't silently re-enable the TTL trigger on a freshly
    // booted board.
    ttlSettleCombo->setEnabled(false);
    ttlSettleCombo->setSelectedId(1, dontSendNotification);

    // Reset chip displays
    portAInterface->reset();
    portBInterface->reset();
}

void IntanSocketEditor::updateChipDetection(const IntanInterface::AutoDetectionResult& result)
{
    portAInterface->updateCipo0Status(result.cipo0Detected, result.cipo0ChipType);
    portAInterface->updateCipo1Status(result.cipo1Detected, result.cipo1ChipType);
    portBInterface->updateCipo0Status(result.portBCipo0Detected, result.portBCipo0ChipType);
    portBInterface->updateCipo1Status(result.portBCipo1Detected, result.portBCipo1ChipType);
}

void IntanSocketEditor::syncFromDeviceState(uint8_t mask, bool debugOn)
{
    // Channel-enable bits, by lane:
    //   port A: bit0=A.CIPO0_REG, bit1=A.CIPO0_DDR, bit2=A.CIPO1_REG, bit3=A.CIPO1_DDR
    //   port B: bit4..bit7 same order
    // A lane is "detected" if its REG bit is set. RHD2164 has DDR (REG+DDR),
    // RHD2132 does not (REG only). This is the same shape a successful RESCAN
    // leaves the firmware in, so adopting it at reconnect is lossless.
    auto laneFromBits = [](uint8_t m, int regBit, int ddrBit)
        -> std::pair<bool, IntanInterface::ChipType>
    {
        bool reg = (m & (1 << regBit)) != 0;
        bool ddr = (m & (1 << ddrBit)) != 0;
        if (!reg) return { false, IntanInterface::ChipType::NONE };
        return { true, ddr ? IntanInterface::ChipType::RHD2164
                           : IntanInterface::ChipType::RHD2132 };
    };

    auto a0 = laneFromBits(mask, 0, 1);
    auto a1 = laneFromBits(mask, 2, 3);
    auto b0 = laneFromBits(mask, 4, 5);
    auto b1 = laneFromBits(mask, 6, 7);

    portAInterface->updateCipo0Status(a0.first, a0.second);
    portAInterface->updateCipo1Status(a1.first, a1.second);
    portBInterface->updateCipo0Status(b0.first, b0.second);
    portBInterface->updateCipo1Status(b1.first, b1.second);

    // Debug-button state. Pick the closer of single-port / dual-port based on
    // whether the high nibble is set; that's enough to label DBG 1P / DBG 2P
    // correctly.
    if (!debugOn)                       debugModeState = DebugMode::Off;
    else if ((mask & 0xF0) != 0)        debugModeState = DebugMode::DualPort;
    else                                debugModeState = DebugMode::SinglePort;
    refreshDebugButtons();
}
