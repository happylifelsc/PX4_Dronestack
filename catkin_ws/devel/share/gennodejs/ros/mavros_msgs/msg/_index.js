
"use strict";

let AttitudeTarget = require('./AttitudeTarget.js');
let LandingTarget = require('./LandingTarget.js');
let ESCStatus = require('./ESCStatus.js');
let Altitude = require('./Altitude.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let WaypointReached = require('./WaypointReached.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let HomePosition = require('./HomePosition.js');
let HilSensor = require('./HilSensor.js');
let ManualControl = require('./ManualControl.js');
let WaypointList = require('./WaypointList.js');
let CommandCode = require('./CommandCode.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let Tunnel = require('./Tunnel.js');
let FileEntry = require('./FileEntry.js');
let Mavlink = require('./Mavlink.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let RCOut = require('./RCOut.js');
let HilControls = require('./HilControls.js');
let ActuatorControl = require('./ActuatorControl.js');
let MountControl = require('./MountControl.js');
let VehicleInfo = require('./VehicleInfo.js');
let RTKBaseline = require('./RTKBaseline.js');
let GPSINPUT = require('./GPSINPUT.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let Waypoint = require('./Waypoint.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let StatusText = require('./StatusText.js');
let VFR_HUD = require('./VFR_HUD.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ESCInfo = require('./ESCInfo.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ParamValue = require('./ParamValue.js');
let PositionTarget = require('./PositionTarget.js');
let Trajectory = require('./Trajectory.js');
let Vibration = require('./Vibration.js');
let HilGPS = require('./HilGPS.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let RTCM = require('./RTCM.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let SysStatus = require('./SysStatus.js');
let DebugValue = require('./DebugValue.js');
let RadioStatus = require('./RadioStatus.js');
let Thrust = require('./Thrust.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let LogEntry = require('./LogEntry.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let GPSRTK = require('./GPSRTK.js');
let State = require('./State.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let LogData = require('./LogData.js');
let Param = require('./Param.js');
let CellularStatus = require('./CellularStatus.js');
let ExtendedState = require('./ExtendedState.js');
let GPSRAW = require('./GPSRAW.js');
let RCIn = require('./RCIn.js');
let TerrainReport = require('./TerrainReport.js');

module.exports = {
  AttitudeTarget: AttitudeTarget,
  LandingTarget: LandingTarget,
  ESCStatus: ESCStatus,
  Altitude: Altitude,
  OverrideRCIn: OverrideRCIn,
  WaypointReached: WaypointReached,
  ESCTelemetryItem: ESCTelemetryItem,
  HomePosition: HomePosition,
  HilSensor: HilSensor,
  ManualControl: ManualControl,
  WaypointList: WaypointList,
  CommandCode: CommandCode,
  ESCTelemetry: ESCTelemetry,
  Tunnel: Tunnel,
  FileEntry: FileEntry,
  Mavlink: Mavlink,
  NavControllerOutput: NavControllerOutput,
  RCOut: RCOut,
  HilControls: HilControls,
  ActuatorControl: ActuatorControl,
  MountControl: MountControl,
  VehicleInfo: VehicleInfo,
  RTKBaseline: RTKBaseline,
  GPSINPUT: GPSINPUT,
  ADSBVehicle: ADSBVehicle,
  Waypoint: Waypoint,
  CompanionProcessStatus: CompanionProcessStatus,
  StatusText: StatusText,
  VFR_HUD: VFR_HUD,
  HilStateQuaternion: HilStateQuaternion,
  OpticalFlowRad: OpticalFlowRad,
  ESCInfo: ESCInfo,
  MagnetometerReporter: MagnetometerReporter,
  PlayTuneV2: PlayTuneV2,
  ParamValue: ParamValue,
  PositionTarget: PositionTarget,
  Trajectory: Trajectory,
  Vibration: Vibration,
  HilGPS: HilGPS,
  EstimatorStatus: EstimatorStatus,
  RTCM: RTCM,
  OnboardComputerStatus: OnboardComputerStatus,
  BatteryStatus: BatteryStatus,
  ESCStatusItem: ESCStatusItem,
  TimesyncStatus: TimesyncStatus,
  SysStatus: SysStatus,
  DebugValue: DebugValue,
  RadioStatus: RadioStatus,
  Thrust: Thrust,
  CamIMUStamp: CamIMUStamp,
  LogEntry: LogEntry,
  WheelOdomStamped: WheelOdomStamped,
  GPSRTK: GPSRTK,
  State: State,
  GlobalPositionTarget: GlobalPositionTarget,
  HilActuatorControls: HilActuatorControls,
  CameraImageCaptured: CameraImageCaptured,
  ESCInfoItem: ESCInfoItem,
  LogData: LogData,
  Param: Param,
  CellularStatus: CellularStatus,
  ExtendedState: ExtendedState,
  GPSRAW: GPSRAW,
  RCIn: RCIn,
  TerrainReport: TerrainReport,
};
