
"use strict";

let LogRequestEnd = require('./LogRequestEnd.js')
let CommandAck = require('./CommandAck.js')
let CommandHome = require('./CommandHome.js')
let FileRemove = require('./FileRemove.js')
let CommandBool = require('./CommandBool.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandInt = require('./CommandInt.js')
let ParamPush = require('./ParamPush.js')
let LogRequestList = require('./LogRequestList.js')
let ParamGet = require('./ParamGet.js')
let ParamSet = require('./ParamSet.js')
let CommandTOL = require('./CommandTOL.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileTruncate = require('./FileTruncate.js')
let LogRequestData = require('./LogRequestData.js')
let CommandLong = require('./CommandLong.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let SetMode = require('./SetMode.js')
let WaypointPush = require('./WaypointPush.js')
let MessageInterval = require('./MessageInterval.js')
let FileList = require('./FileList.js')
let FileChecksum = require('./FileChecksum.js')
let FileClose = require('./FileClose.js')
let WaypointPull = require('./WaypointPull.js')
let MountConfigure = require('./MountConfigure.js')
let FileRead = require('./FileRead.js')
let StreamRate = require('./StreamRate.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileRename = require('./FileRename.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let ParamPull = require('./ParamPull.js')
let WaypointClear = require('./WaypointClear.js')
let FileOpen = require('./FileOpen.js')
let FileWrite = require('./FileWrite.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')

module.exports = {
  LogRequestEnd: LogRequestEnd,
  CommandAck: CommandAck,
  CommandHome: CommandHome,
  FileRemove: FileRemove,
  CommandBool: CommandBool,
  FileMakeDir: FileMakeDir,
  CommandInt: CommandInt,
  ParamPush: ParamPush,
  LogRequestList: LogRequestList,
  ParamGet: ParamGet,
  ParamSet: ParamSet,
  CommandTOL: CommandTOL,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandVtolTransition: CommandVtolTransition,
  FileTruncate: FileTruncate,
  LogRequestData: LogRequestData,
  CommandLong: CommandLong,
  FileRemoveDir: FileRemoveDir,
  CommandTriggerInterval: CommandTriggerInterval,
  SetMode: SetMode,
  WaypointPush: WaypointPush,
  MessageInterval: MessageInterval,
  FileList: FileList,
  FileChecksum: FileChecksum,
  FileClose: FileClose,
  WaypointPull: WaypointPull,
  MountConfigure: MountConfigure,
  FileRead: FileRead,
  StreamRate: StreamRate,
  SetMavFrame: SetMavFrame,
  FileRename: FileRename,
  CommandTriggerControl: CommandTriggerControl,
  ParamPull: ParamPull,
  WaypointClear: WaypointClear,
  FileOpen: FileOpen,
  FileWrite: FileWrite,
  VehicleInfoGet: VehicleInfoGet,
};
