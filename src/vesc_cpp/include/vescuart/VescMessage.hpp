#ifndef VESCMESSAGE_HPP
#define VESCMESSAGE_HPP

#include <optional>
#include <vector>
#include <stdint.h>

namespace vescuart
{

enum class VescMessageType
{
    version,
    jumpToBootloader,
    eraseNewApp,
    writeNewAppData,
    getValues,
    setDuty,
    setCurrent,
    setCurrentBrake,
    setRpm,
    setPos,
    setHandbrake,
    setDetect,
    setServoPos,
    setMotorConfig,
    getMotorConfig,
    getMotorConfigDefault,
    setAppConfig,
    getAppConfig,
    getAppConfigDefault,
    samplePrint,
    terminalCommand,
    print,
    rotorPosition,
    eperimentSample,
    detectMotorParam,
    detectMotorRightLeft,
    detectMotorFluxLinkage,
    detectEncoder,
    detectHallFoc,
    reboot,
    alive,
    getDecodedPpm,
    getDecodedAdc,
    getDecodedChuk,
    forwardCan,
    setChuckData,
    customAppData,
    nrfStartPairing,
    gpdSetFsw,
    gpdBufferNotify,
    gpdBufferSizeLeft,
    gpdFillBuffer,
    gpdOutputSample,
    gpdSetMode,
    gpdFillBuffer8Int,
    gpdFillBuffer16Int,
    gpdSetBufferIntScale,
    getValuesSetup,
    setMotorConfigTemp,
    setMotorConfigTempSetup,
    getValuesSelective,
    getValuesSetupSelective,
    extNrfPresent,
    extNrfEsbSetChAddr,
    extNrfEsbSendData,
    extNrfEsbRxData,
    extNrfSetEnabled,
    detectMotorFluxLinkageOpenloop,
    detectApplyAllFoc,
    jumpToBootloaderAllCan,
    eraseNewAppAllCan,
    writeNewAppDataAllCan,
    pingCan,
    appDisableOutput,
    terminalCmdSync,
    getImuData,
    bmConnect,
    bmEraseFlashAll,
    bmWriteFlash,
    bmReboot,
    bmDisconnect,
    bmMapPinsDefault,
    bmMapPinsNrf5x,
    eraseBootloader,
    eraseBootloaderAllCan,
    pilotInit,
    pilotData,
    pilotAddGraph,
    pilotSetGraph,
    getDecodedBalance,
    bmMemRead,
};

class VescMessage : public std::vector<uint8_t>
{
  public:
    VescMessage() {}

    /**
     * @param type type of message that will be used as the first byte of the message
     */
    VescMessage(VescMessageType type, std::optional<int> canId);

    /**
     * @brief adds a datatype to this message
     *
     * @param data data to be added
     */
    template <typename T>
    void add(T data);

    template <typename T, typename U>
    void add(T data, float scale);

    template <typename T>
    T get();

    template <typename T, typename U>
    T get(float scale);

    /**
     * @brief appends a message type to the message
     *
     * @param type message type to be added
     */
    void addMessageType(VescMessageType type);

    /**
     * @brief adds crc to message
     */
    void addCRC();

    bool checkCRC();

    /*
     * @brief adds postfix to forward all can messages to VESC with given can id if id is given
     *
     * @param canId can id of receiving VESC
     */
    void addForwardCan(std::optional<int> canId);

    VescMessage finalize() const;
};

#include "vescuart/VescMessage.tcc"

} // namespace vescuart

#endif
