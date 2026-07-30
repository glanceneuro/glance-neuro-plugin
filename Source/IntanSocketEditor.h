// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#ifndef __IntanSocketEditorH__
#define __IntanSocketEditorH__

#ifdef _WIN32
#include <Windows.h>
#endif

#include <VisualizerEditorHeaders.h>
#include "IntanSocket.h"
#include "IntanInterface.h"

namespace IntanSocketNode
{
class IntanSocket;

/** UI component for bandwidth filter settings */
class BandwidthInterface : public Component,
                          public Label::Listener
{
public:
    BandwidthInterface(IntanSocket* node);
    ~BandwidthInterface() {}
    
    void paint(Graphics& g) override;
    void labelTextChanged(Label* label) override;
    
    void setLowerBandwidth(double value);
    void setUpperBandwidth(double value);
    double getLowerBandwidth() { return lowerBandwidth; }
    double getUpperBandwidth() { return upperBandwidth; }
    
private:
    IntanSocket* node;
    std::unique_ptr<Label> lowerBandwidthLabel;
    std::unique_ptr<Label> upperBandwidthLabel;
    double lowerBandwidth;
    double upperBandwidth;
};

/** UI component for sample rate selection */
class SampleRateInterface : public Component
{
public:
    SampleRateInterface(IntanSocket* node);
    ~SampleRateInterface() {}
    
    void paint(Graphics& g) override;
    
private:
    IntanSocket* node;
    std::unique_ptr<Label> rateLabel;
};

/** UI component showing both CIPO chip slots for one port (A or B). */
class PortInterface : public Component
{
public:
    PortInterface(IntanSocket* node, const String& portName);
    ~PortInterface() {}

    void paint(Graphics& g) override;
    void updateCipo0Status(bool detected, IntanInterface::ChipType chipType);
    void updateCipo1Status(bool detected, IntanInterface::ChipType chipType);
    void reset();

private:
    IntanSocket* node;
    String portName;
    bool cipo0Detected = false;
    bool cipo1Detected = false;
    IntanInterface::ChipType cipo0Type = IntanInterface::ChipType::NONE;
    IntanInterface::ChipType cipo1Type = IntanInterface::ChipType::NONE;

    void paintChipBox(Graphics& g, int x, bool detected, IntanInterface::ChipType type);
};

class IntanSocketEditor : public GenericEditor,
                          public Button::Listener,
                          public ComboBox::Listener
{
public:
    /** Constructor */
    IntanSocketEditor(GenericProcessor* parentNode, IntanSocket* node);

    /** Button listener callback */
    void buttonClicked(Button* button) override;
    
    /** ComboBox listener callback */
    void comboBoxChanged(ComboBox* comboBox) override;

    /** Called at start of acquisition */
    void startAcquisition() override;

    /** Called at end of acquisition */
    void stopAcquisition() override;

    /** Called when socket connects */
    void connected();

    /** Called when socket disconnects */
    void disconnected();
    
    /** Update chip detection display */
    void updateChipDetection(const IntanInterface::AutoDetectionResult& result);

    /** Sync editor state from the device's status response. Called on connect
        so the UI snaps to whatever the firmware is already configured for
        (channel-enable mask + debug mode), without forcing a RESCAN. */
    void syncFromDeviceState(uint8_t channelMask, bool debugOn);

private:
    // Buttons
    std::unique_ptr<UtilityButton> connectButton;
    std::unique_ptr<UtilityButton> disconnectButton;
    std::unique_ptr<UtilityButton> rescanButton;
    std::unique_ptr<UtilityButton> debugMode1PButton;   // single-port (0x0F)
    std::unique_ptr<UtilityButton> debugMode2PButton;   // dual-port (0xFF)
    enum class DebugMode { Off, SinglePort, DualPort };
    DebugMode debugModeState = DebugMode::Off;
    void refreshDebugButtons();

    // Aux sequencer test tooling (all usable DURING acquisition)
    std::unique_ptr<UtilityButton> statusButton;      // print device status to console
    std::unique_ptr<UtilityButton> fastSettleButton;  // toggle software fast settle
    std::unique_ptr<UtilityButton> lfpEnableButton;   // toggle LFP/DSP engine + 2nd stream
    bool fastSettleActive;
    bool lfpActive;

    /** Refresh the fast-settle button label/color from node state. The accel
        sweep is now always-on (the board boots into it) so it has no toggle. */
    void refreshAuxButtons();
    
    // UI components
    std::unique_ptr<PortInterface> portAInterface;
    std::unique_ptr<PortInterface> portBInterface;
    std::unique_ptr<SampleRateInterface> sampleRateInterface;
    std::unique_ptr<BandwidthInterface> bandwidthInterface;
    
    // Dropdowns
    std::unique_ptr<ComboBox> ttlSettleCombo;
    std::unique_ptr<Label> ttlSettleLabel;

    String stringConnect = "CONNECT";
    String stringDisconnect = "DISCONNECT";

    // Parent node
    IntanSocket* node;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IntanSocketEditor);
};

} // namespace IntanSocketNode

#endif
