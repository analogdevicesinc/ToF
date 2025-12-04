# TOF Camera API Documentation

Complete API reference for the TOF Camera library.
Based on the bindings in aditofpython.cpython-310-x86_64-linux-gnu, compatible with Python 3.10
Note that this reference has been auto-generated based on the datatypes and descriptions received from the python bindings. No detailed descriptions are available for most objects.


## Table of Contents

- [Camera](#camera)
- [Camera Details API Documentation](#camera-details-api-documentation)
- [Enum ConnectionType](#enum-connectiontype)
- [Depth Sensor Interface API Documentation](#depth-sensor-interface-api-documentation)
- [Depth Sensor Mode Details API Documentation](#depth-sensor-mode-details-api-documentation)
- [Frame API Documentation](#frame-api-documentation)
- [Frame Data Details API Documentation](#frame-data-details-api-documentation)
- [Frame Details API Documentation](#frame-details-api-documentation)
- [Frame Handler API Documentation](#frame-handler-api-documentation)
- [Intrinsic Parameters API Documentation](#intrinsic-parameters-api-documentation)
- [Metadata API Documentation](#metadata-api-documentation)
- [Sensor Details API Documentation](#sensor-details-api-documentation)
- [Enum Status](#enum-status)

---


---

# Camera
## Methods

### `adsd3500DisableCCBM`

adsd3500DisableCCBM(self: aditofpython.Camera, value: bool) -> aditofpython.Status


---

### `adsd3500GetABinvalidationThreshold`

adsd3500GetABinvalidationThreshold(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetConfidenceThreshold`

adsd3500GetConfidenceThreshold(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetEnableMetadatainAB`

adsd3500GetEnableMetadatainAB(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetFirmwareVersion`

adsd3500GetFirmwareVersion(self: aditofpython.Camera, fwVersion: str, fwHash: str) -> Tuple[aditofpython.Status, str, str]


---

### `adsd3500GetFrameRate`

adsd3500GetFrameRate(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetGenericTemplate`

adsd3500GetGenericTemplate(self: aditofpython.Camera, arg0: int) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetImagerErrorCode`

adsd3500GetImagerErrorCode(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFExponentialTerm`

adsd3500GetJBLFExponentialTerm(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFGaussianSigma`

adsd3500GetJBLFGaussianSigma(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFfilterEnableState`

adsd3500GetJBLFfilterEnableState(self: aditofpython.Camera) -> Tuple[aditofpython.Status, bool]


---

### `adsd3500GetJBLFfilterSize`

adsd3500GetJBLFfilterSize(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetLaserTemperature`

adsd3500GetLaserTemperature(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetMIPIOutputSpeed`

adsd3500GetMIPIOutputSpeed(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetRadialThresholdMax`

adsd3500GetRadialThresholdMax(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetRadialThresholdMin`

adsd3500GetRadialThresholdMin(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetSensorTemperature`

adsd3500GetSensorTemperature(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetStatus`

adsd3500GetStatus(self: aditofpython.Camera, chipStatus: int, imagerStatus: int) -> Tuple[aditofpython.Status, int, int]


---

### `adsd3500GetTemperatureCompensationStatus`

adsd3500GetTemperatureCompensationStatus(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetVCSELDelay`

adsd3500GetVCSELDelay(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500IsCCBMsupported`

adsd3500IsCCBMsupported(self: aditofpython.Camera) -> Tuple[aditofpython.Status, bool]


---

### `adsd3500ResetIniParamsForMode`

adsd3500ResetIniParamsForMode(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetABinvalidationThreshold`

adsd3500SetABinvalidationThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetConfidenceThreshold`

adsd3500SetConfidenceThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetEnableEdgeConfidence`

adsd3500SetEnableEdgeConfidence(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnableMetadatainAB`

adsd3500SetEnableMetadatainAB(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnablePhaseInvalidation`

adsd3500SetEnablePhaseInvalidation(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnableTemperatureCompensation`

adsd3500SetEnableTemperatureCompensation(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetFrameRate`

adsd3500SetFrameRate(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetGenericTemplate`

adsd3500SetGenericTemplate(self: aditofpython.Camera, reg: int, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFABThreshold`

adsd3500SetJBLFABThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetJBLFExponentialTerm`

adsd3500SetJBLFExponentialTerm(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFGaussianSigma`

adsd3500SetJBLFGaussianSigma(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFMaxEdgeThreshold`

adsd3500SetJBLFMaxEdgeThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetJBLFfilterEnableState`

adsd3500SetJBLFfilterEnableState(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `adsd3500SetJBLFfilterSize`

adsd3500SetJBLFfilterSize(self: aditofpython.Camera, size: int) -> aditofpython.Status


---

### `adsd3500SetMIPIOutputSpeed`

adsd3500SetMIPIOutputSpeed(self: aditofpython.Camera, speed: int) -> aditofpython.Status


---

### `adsd3500SetRadialThresholdMax`

adsd3500SetRadialThresholdMax(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetRadialThresholdMin`

adsd3500SetRadialThresholdMin(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetToggleMode`

adsd3500SetToggleMode(self: aditofpython.Camera, mode: int) -> aditofpython.Status


---

### `adsd3500SetVCSELDelay`

adsd3500SetVCSELDelay(self: aditofpython.Camera, delay: int) -> aditofpython.Status


---

### `adsd3500ToggleFsync`

adsd3500ToggleFsync(self: aditofpython.Camera) -> aditofpython.Status


---

### `adsd3500UpdateFirmware`

adsd3500UpdateFirmware(self: aditofpython.Camera, filePath: str) -> aditofpython.Status


---

### `adsd3500setEnableDynamicModeSwitching`

adsd3500setEnableDynamicModeSwitching(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `adsds3500setDynamicModeSwitchingSequence`

adsds3500setDynamicModeSwitchingSequence(self: aditofpython.Camera, sequence: List[Tuple[int, int]]) -> aditofpython.Status


---

### `enableDepthCompute`

enableDepthCompute(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `enableXYZframe`

enableXYZframe(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `getAvailableControls`

getAvailableControls(*args, **kwargs)
Overloaded function.

1. getAvailableControls(self: aditofpython.Camera, availableModes: list) -> aditofpython.Status

2. getAvailableControls(self: aditofpython.Camera, controls: list) -> aditofpython.Status


---

### `getAvailableModes`

getAvailableModes(self: aditofpython.Camera, availableModes: list) -> aditofpython.Status


---

### `getControl`

getControl(self: aditofpython.Camera, control: str, value: str) -> aditofpython.Status


---

### `getDetails`

getDetails(self: aditofpython.Camera, details: aditofpython.CameraDetails) -> aditofpython.Status


---

### `getFrameProcessParams`

getFrameProcessParams(self: aditofpython.Camera) -> Tuple[aditofpython.Status, dict]


---

### `getSensor`

getSensor(self: aditofpython.Camera) -> aditof::DepthSensorInterface


---

### `initialize`

initialize(self: aditofpython.Camera, configFilepath: str = '') -> aditofpython.Status


---

### `loadDepthParamsFromJsonFile`

loadDepthParamsFromJsonFile(self: aditofpython.Camera, loadPathFile: str, mode: int) -> aditofpython.Status


---

### `readSerialNumber`

readSerialNumber(self: aditofpython.Camera, serialNumber: str, useCacheValue: bool) -> Tuple[aditofpython.Status, str]


---

### `requestFrame`

requestFrame(self: aditofpython.Camera, frame: aditof::Frame) -> aditofpython.Status


---

### `saveDepthParamsToJsonFile`

saveDepthParamsToJsonFile(self: aditofpython.Camera, savePathFile: str) -> aditofpython.Status


---

### `saveModuleCCB`

saveModuleCCB(self: aditofpython.Camera, filepath: str) -> aditofpython.Status


---

### `saveModuleCFG`

saveModuleCFG(self: aditofpython.Camera, filepath: str) -> aditofpython.Status


---

### `setControl`

setControl(self: aditofpython.Camera, control: str, value: str) -> aditofpython.Status


---

### `setFrameProcessParams`

setFrameProcessParams(self: aditofpython.Camera, params: dict) -> aditofpython.Status


---

### `setMode`

setMode(self: aditofpython.Camera, mode: int) -> aditofpython.Status


---

### `setSensorConfiguration`

setSensorConfiguration(self: aditofpython.Camera, sensorConf: str) -> aditofpython.Status


---

### `start`

start(self: aditofpython.Camera) -> aditofpython.Status


---

### `stop`

stop(self: aditofpython.Camera) -> aditofpython.Status


---




---

# Camera Details API Documentation
## Methods

### `adsd3500DisableCCBM`

adsd3500DisableCCBM(self: aditofpython.Camera, value: bool) -> aditofpython.Status


---

### `adsd3500GetABinvalidationThreshold`

adsd3500GetABinvalidationThreshold(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetConfidenceThreshold`

adsd3500GetConfidenceThreshold(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetEnableMetadatainAB`

adsd3500GetEnableMetadatainAB(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetFirmwareVersion`

adsd3500GetFirmwareVersion(self: aditofpython.Camera, fwVersion: str, fwHash: str) -> Tuple[aditofpython.Status, str, str]


---

### `adsd3500GetFrameRate`

adsd3500GetFrameRate(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetGenericTemplate`

adsd3500GetGenericTemplate(self: aditofpython.Camera, arg0: int) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetImagerErrorCode`

adsd3500GetImagerErrorCode(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFExponentialTerm`

adsd3500GetJBLFExponentialTerm(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFGaussianSigma`

adsd3500GetJBLFGaussianSigma(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetJBLFfilterEnableState`

adsd3500GetJBLFfilterEnableState(self: aditofpython.Camera) -> Tuple[aditofpython.Status, bool]


---

### `adsd3500GetJBLFfilterSize`

adsd3500GetJBLFfilterSize(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetLaserTemperature`

adsd3500GetLaserTemperature(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetMIPIOutputSpeed`

adsd3500GetMIPIOutputSpeed(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetRadialThresholdMax`

adsd3500GetRadialThresholdMax(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetRadialThresholdMin`

adsd3500GetRadialThresholdMin(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetSensorTemperature`

adsd3500GetSensorTemperature(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetStatus`

adsd3500GetStatus(self: aditofpython.Camera, chipStatus: int, imagerStatus: int) -> Tuple[aditofpython.Status, int, int]


---

### `adsd3500GetTemperatureCompensationStatus`

adsd3500GetTemperatureCompensationStatus(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500GetVCSELDelay`

adsd3500GetVCSELDelay(self: aditofpython.Camera) -> Tuple[aditofpython.Status, int]


---

### `adsd3500IsCCBMsupported`

adsd3500IsCCBMsupported(self: aditofpython.Camera) -> Tuple[aditofpython.Status, bool]


---

### `adsd3500ResetIniParamsForMode`

adsd3500ResetIniParamsForMode(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetABinvalidationThreshold`

adsd3500SetABinvalidationThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetConfidenceThreshold`

adsd3500SetConfidenceThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetEnableEdgeConfidence`

adsd3500SetEnableEdgeConfidence(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnableMetadatainAB`

adsd3500SetEnableMetadatainAB(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnablePhaseInvalidation`

adsd3500SetEnablePhaseInvalidation(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetEnableTemperatureCompensation`

adsd3500SetEnableTemperatureCompensation(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetFrameRate`

adsd3500SetFrameRate(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetGenericTemplate`

adsd3500SetGenericTemplate(self: aditofpython.Camera, reg: int, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFABThreshold`

adsd3500SetJBLFABThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetJBLFExponentialTerm`

adsd3500SetJBLFExponentialTerm(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFGaussianSigma`

adsd3500SetJBLFGaussianSigma(self: aditofpython.Camera, value: int) -> aditofpython.Status


---

### `adsd3500SetJBLFMaxEdgeThreshold`

adsd3500SetJBLFMaxEdgeThreshold(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetJBLFfilterEnableState`

adsd3500SetJBLFfilterEnableState(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `adsd3500SetJBLFfilterSize`

adsd3500SetJBLFfilterSize(self: aditofpython.Camera, size: int) -> aditofpython.Status


---

### `adsd3500SetMIPIOutputSpeed`

adsd3500SetMIPIOutputSpeed(self: aditofpython.Camera, speed: int) -> aditofpython.Status


---

### `adsd3500SetRadialThresholdMax`

adsd3500SetRadialThresholdMax(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetRadialThresholdMin`

adsd3500SetRadialThresholdMin(self: aditofpython.Camera, threshold: int) -> aditofpython.Status


---

### `adsd3500SetToggleMode`

adsd3500SetToggleMode(self: aditofpython.Camera, mode: int) -> aditofpython.Status


---

### `adsd3500SetVCSELDelay`

adsd3500SetVCSELDelay(self: aditofpython.Camera, delay: int) -> aditofpython.Status


---

### `adsd3500ToggleFsync`

adsd3500ToggleFsync(self: aditofpython.Camera) -> aditofpython.Status


---

### `adsd3500UpdateFirmware`

adsd3500UpdateFirmware(self: aditofpython.Camera, filePath: str) -> aditofpython.Status


---

### `adsd3500setEnableDynamicModeSwitching`

adsd3500setEnableDynamicModeSwitching(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `adsds3500setDynamicModeSwitchingSequence`

adsds3500setDynamicModeSwitchingSequence(self: aditofpython.Camera, sequence: List[Tuple[int, int]]) -> aditofpython.Status


---

### `enableDepthCompute`

enableDepthCompute(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `enableXYZframe`

enableXYZframe(self: aditofpython.Camera, enable: bool) -> aditofpython.Status


---

### `getAvailableControls`

getAvailableControls(*args, **kwargs)
Overloaded function.

1. getAvailableControls(self: aditofpython.Camera, availableModes: list) -> aditofpython.Status

2. getAvailableControls(self: aditofpython.Camera, controls: list) -> aditofpython.Status


---

### `getAvailableModes`

getAvailableModes(self: aditofpython.Camera, availableModes: list) -> aditofpython.Status


---

### `getControl`

getControl(self: aditofpython.Camera, control: str, value: str) -> aditofpython.Status


---

### `getDetails`

getDetails(self: aditofpython.Camera, details: aditofpython.CameraDetails) -> aditofpython.Status


---

### `getFrameProcessParams`

getFrameProcessParams(self: aditofpython.Camera) -> Tuple[aditofpython.Status, dict]


---

### `getSensor`

getSensor(self: aditofpython.Camera) -> aditof::DepthSensorInterface


---

### `initialize`

initialize(self: aditofpython.Camera, configFilepath: str = '') -> aditofpython.Status


---

### `loadDepthParamsFromJsonFile`

loadDepthParamsFromJsonFile(self: aditofpython.Camera, loadPathFile: str, mode: int) -> aditofpython.Status


---

### `readSerialNumber`

readSerialNumber(self: aditofpython.Camera, serialNumber: str, useCacheValue: bool) -> Tuple[aditofpython.Status, str]


---

### `requestFrame`

requestFrame(self: aditofpython.Camera, frame: aditof::Frame) -> aditofpython.Status


---

### `saveDepthParamsToJsonFile`

saveDepthParamsToJsonFile(self: aditofpython.Camera, savePathFile: str) -> aditofpython.Status


---

### `saveModuleCCB`

saveModuleCCB(self: aditofpython.Camera, filepath: str) -> aditofpython.Status


---

### `saveModuleCFG`

saveModuleCFG(self: aditofpython.Camera, filepath: str) -> aditofpython.Status


---

### `setControl`

setControl(self: aditofpython.Camera, control: str, value: str) -> aditofpython.Status


---

### `setFrameProcessParams`

setFrameProcessParams(self: aditofpython.Camera, params: dict) -> aditofpython.Status


---

### `setMode`

setMode(self: aditofpython.Camera, mode: int) -> aditofpython.Status


---

### `setSensorConfiguration`

setSensorConfiguration(self: aditofpython.Camera, sensorConf: str) -> aditofpython.Status


---

### `start`

start(self: aditofpython.Camera) -> aditofpython.Status


---

### `stop`

stop(self: aditofpython.Camera) -> aditofpython.Status


---




---

# Enum ConnectionType

Generated automatically from Python bindings.

### `Usb` = `1`
Description: _TODO_

### `Network` = `2`
Description: _TODO_

### `OnTarget` = `0`
Description: _TODO_

### `Offline` = `3`
Description: _TODO_



---

# Depth Sensor Interface API Documentation
## Methods

### `adsd3500_read_cmd`

adsd3500_read_cmd(self: aditofpython.DepthSensorInterface, cmd: int, data: numpy.ndarray[numpy.uint16], usDelay: int) -> Tuple[aditofpython.Status, int]


---

### `adsd3500_read_payload`

adsd3500_read_payload(self: aditofpython.DepthSensorInterface, payload: numpy.ndarray[numpy.uint8], payload_len: int) -> Tuple[aditofpython.Status, int]


---

### `adsd3500_read_payload_cmd`

adsd3500_read_payload_cmd(self: aditofpython.DepthSensorInterface, cmd: int, readback_data: numpy.ndarray[numpy.uint8], payload_len: int) -> Tuple[aditofpython.Status, int]


---

### `adsd3500_register_interrupt_callback`

adsd3500_register_interrupt_callback(self: aditofpython.DepthSensorInterface, cb: Callable[[aditofpython.Adsd3500Status], None]) -> aditofpython.Status


---

### `adsd3500_reset`

adsd3500_reset(self: aditofpython.DepthSensorInterface) -> aditofpython.Status


---

### `adsd3500_unregister_interrupt_callback`

adsd3500_unregister_interrupt_callback(self: aditofpython.DepthSensorInterface, cb: Callable[[aditofpython.Adsd3500Status], None]) -> aditofpython.Status


---

### `adsd3500_write_cmd`

adsd3500_write_cmd(self: aditofpython.DepthSensorInterface, cmd: int, data: int) -> aditofpython.Status


---

### `adsd3500_write_payload`

adsd3500_write_payload(self: aditofpython.DepthSensorInterface, payload: numpy.ndarray[numpy.uint8], payload_len: int) -> aditofpython.Status


---

### `adsd3500_write_payload_cmd`

adsd3500_write_payload_cmd(self: aditofpython.DepthSensorInterface, cmd: numpy.ndarray[numpy.uint8], payload: int, payload_len: int) -> aditofpython.Status


---

### `getAvailableControls`

getAvailableControls(self: aditofpython.DepthSensorInterface, controls: list) -> aditofpython.Status


---

### `getAvailableModes`

getAvailableModes(self: aditofpython.DepthSensorInterface, modes: list) -> aditofpython.Status


---

### `getControl`

getControl(self: aditofpython.DepthSensorInterface, control: str, value: str) -> aditofpython.Status


---

### `getDetails`

getDetails(self: aditofpython.DepthSensorInterface, details: aditofpython.SensorDetails) -> aditofpython.Status


---

### `getFrame`

getFrame(self: aditofpython.DepthSensorInterface, buffer: numpy.ndarray[numpy.uint16]) -> aditofpython.Status


---

### `getHandle`

getHandle(self: aditofpython.DepthSensorInterface, handle: capsule) -> Tuple[aditofpython.Status, capsule]


---

### `getModeDetails`

getModeDetails(self: aditofpython.DepthSensorInterface, mode: int, details: aditofpython.DepthSensorModeDetails) -> aditofpython.Status


---

### `getName`

getName(self: aditofpython.DepthSensorInterface, name: str) -> aditofpython.Status


---

### `initTargetDepthCompute`

initTargetDepthCompute(self: aditofpython.DepthSensorInterface, iniFile: numpy.ndarray[numpy.uint8], iniFileLength: int, calData: numpy.ndarray[numpy.uint8], calDataLength: int) -> Tuple[aditofpython.Status, int, int]


---

### `open`

open(self: aditofpython.DepthSensorInterface) -> aditofpython.Status


---

### `setControl`

setControl(self: aditofpython.DepthSensorInterface, control: str, value: str) -> aditofpython.Status


---

### `setHostConnectionType`

setHostConnectionType(self: aditofpython.DepthSensorInterface, connectionType: str) -> aditofpython.Status


---

### `setMode`

setMode(*args, **kwargs)
Overloaded function.

1. setMode(self: aditofpython.DepthSensorInterface, mode: aditofpython.DepthSensorModeDetails) -> aditofpython.Status

2. setMode(self: aditofpython.DepthSensorInterface, mode: int) -> aditofpython.Status


---

### `start`

start(self: aditofpython.DepthSensorInterface) -> aditofpython.Status


---

### `stop`

stop(self: aditofpython.DepthSensorInterface) -> aditofpython.Status


---




---

# Depth Sensor Mode Details API Documentation
## Attributes

### `baseResolutionHeight`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `baseResolutionWidth`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `driverConfiguration`

Type: `DriverConfiguration`

---

### `frameContent`

Type: `list`

Built-in mutable sequence.

If no argument is given, the constructor creates a new empty list.
The argument must be an iterable if specified.

---

### `frameHeightInBytes`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `frameWidthInBytes`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `isPCM`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `metadataSize`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `modeNumber`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `numberOfPhases`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `pixelFormatIndex`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---




---

# Frame API Documentation
## Methods

### `getData`

getData(self: aditofpython.Frame, dataType: str) -> aditofpython.frameData


---

### `getDataDetails`

getDataDetails(self: aditofpython.Frame, dataType: str, dataDetails: aditofpython.FrameDataDetails) -> aditofpython.Status


---

### `getDetails`

getDetails(self: aditofpython.Frame, details: aditofpython.FrameDetails) -> aditofpython.Status


---

### `getMetadataStruct`

getMetadataStruct(self: aditofpython.Frame) -> Tuple[aditofpython.Status, aditofpython.Metadata]


---

### `setDetails`

setDetails(self: aditofpython.Frame, details: aditofpython.FrameDetails) -> aditofpython.Status


---




---

# Frame Data Details API Documentation
## Attributes

### `bytesCount`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `height`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `subelementSize`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `subelementsPerElement`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `type`

Type: `str`

str(object='') -> str
str(bytes_or_buffer[, encoding[, errors]]) -> str

Create a new string object from the given object. If encoding or
errors is specified, then the object must expose a data buffer
that will be decoded using the given encoding and error handler.
Otherwise, returns the result of object.__str__() (if defined)
or repr(object).
encoding defaults to sys.getdefaultencoding().
errors defaults to 'strict'.

---

### `width`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---




---

# Frame Details API Documentation
## Attributes

### `cameraMode`

Type: `str`

str(object='') -> str
str(bytes_or_buffer[, encoding[, errors]]) -> str

Create a new string object from the given object. If encoding or
errors is specified, then the object must expose a data buffer
that will be decoded using the given encoding and error handler.
Otherwise, returns the result of object.__str__() (if defined)
or repr(object).
encoding defaults to sys.getdefaultencoding().
errors defaults to 'strict'.

---

### `dataDetails`

Type: `list`

Built-in mutable sequence.

If no argument is given, the constructor creates a new empty list.
The argument must be an iterable if specified.

---

### `height`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `passiveIRCaptured`

Type: `bool`

bool(x) -> bool

Returns True when the argument x is true, False otherwise.
The builtins True and False are the only two instances of the class bool.
The class bool is a subclass of the class int, and cannot be subclassed.

---

### `totalCaptures`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `type`

Type: `str`

str(object='') -> str
str(bytes_or_buffer[, encoding[, errors]]) -> str

Create a new string object from the given object. If encoding or
errors is specified, then the object must expose a data buffer
that will be decoded using the given encoding and error handler.
Otherwise, returns the result of object.__str__() (if defined)
or repr(object).
encoding defaults to sys.getdefaultencoding().
errors defaults to 'strict'.

---

### `width`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---




---

# Frame Handler API Documentation
## Methods

### `readNextFrame`

readNextFrame(self: aditofpython.FrameHandler, frame: aditofpython.Frame, fullFileName: str) -> aditofpython.Status


---

### `saveFrameToFile`

saveFrameToFile(self: aditofpython.FrameHandler, frame: aditofpython.Frame, fileName: str = '') -> aditofpython.Status


---

### `saveFrameToFileMultithread`

saveFrameToFileMultithread(self: aditofpython.FrameHandler, frame: aditofpython.Frame, fileName: str = '') -> aditofpython.Status


---

### `setCustomFormat`

setCustomFormat(self: aditofpython.FrameHandler, format: str) -> aditofpython.Status


---

### `setFrameContent`

setFrameContent(self: aditofpython.FrameHandler, frameContent: str) -> aditofpython.Status


---

### `setInputFileName`

setInputFileName(self: aditofpython.FrameHandler, fullFileName: str) -> aditofpython.Status


---

### `setOutputFilePath`

setOutputFilePath(self: aditofpython.FrameHandler, filePath: str) -> aditofpython.Status


---

### `storeFramesToSingleFile`

storeFramesToSingleFile(self: aditofpython.FrameHandler, enable: bool) -> aditofpython.Status


---




---

# Intrinsic Parameters API Documentation
## Attributes

### `codx`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `cody`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `cx`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `cy`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `fx`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `fy`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k1`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k2`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k3`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k4`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k5`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `k6`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `p1`

Type: `float`

Convert a string or number to a floating point number, if possible.

---

### `p2`

Type: `float`

Convert a string or number to a floating point number, if possible.

---




---

# Metadata API Documentation
## Attributes

### `abFrequencyIndex`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `bitsInAb`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `bitsInConfidence`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `bitsInDepth`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `elapsedTimeFractionalValue`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `elapsedTimeSecondsValue`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `frameNumber`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `frequencyIndex`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `height`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `imagerMode`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `invalidPhaseValue`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `laserTemperature`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `numberOfFrequencies`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `numberOfPhases`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `outputConfiguration`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `sensorTemperature`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `width`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---

### `xyzEnabled`

Type: `int`

int([x]) -> integer
int(x, base=10) -> integer

Convert a number or string to an integer, or return 0 if no arguments
are given.  If x is a number, return x.__int__().  For floating point
numbers, this truncates towards zero.

If x is not a number or if base is given, then x must be a string,
bytes, or bytearray instance representing an integer literal in the
given base.  The literal can be preceded by '+' or '-' and be surrounded
by whitespace.  The base defaults to 10.  Valid bases are 0 and 2-36.
Base 0 means to interpret the base from the string as an integer literal.
>>> int('0b100', base=0)
4

---




---

# Sensor Details API Documentation
## Attributes

### `connectionType`

Type: `ConnectionType`

Members:

  Usb

  Network

  OnTarget

  Offline

---

### `id`

Type: `str`

str(object='') -> str
str(bytes_or_buffer[, encoding[, errors]]) -> str

Create a new string object from the given object. If encoding or
errors is specified, then the object must expose a data buffer
that will be decoded using the given encoding and error handler.
Otherwise, returns the result of object.__str__() (if defined)
or repr(object).
encoding defaults to sys.getdefaultencoding().
errors defaults to 'strict'.

---




---

# Enum Status

Generated automatically from Python bindings.

### `Ok` = `0`
Description: _TODO_

### `Busy` = `1`
Description: _TODO_

### `Unreachable` = `2`
Description: _TODO_

### `InvalidArgument` = `3`
Description: _TODO_

### `Unavailable` = `4`
Description: _TODO_

### `GenericError` = `5`
Description: _TODO_


