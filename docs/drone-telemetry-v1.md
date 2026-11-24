# 阶段一设备遥测协议 v1

适用范围：单后端进程、已登录用户、明确标识来源的设备。阶段四已增加 `source=dji` 的只读遥测；模拟设备可使用 [模拟任务与执行记录扩展](./simulation-missions.md)，仍不提供真实飞行控制。

## 连接入口与身份

- `GET /api/drones`：当前用户的设备快照。
- `GET /api/drones/{deviceId}/status`：指定设备状态；无权限或不存在时返回 404。
- `WS /api/gateway/session`：网关登记、心跳和遥测。
- `WS /api/drones/stream`：网页设备事件订阅，包含初始快照。

HTTP 使用现有 `Authorization: Bearer ...` 登录令牌。两个 WebSocket 均须在连接后 5 秒内发送第一条认证消息；认证成功前不登记设备或发送状态。

```json
{"schemaVersion":1,"type":"authenticate","token":"<平台登录令牌>"}
```

成功回复包含 `schemaVersion:1`、`type:authenticated` 和 `serverEpochSeconds`。令牌不放进 URL，不写入脚本、截图或日志。身份过期关闭码为 4401。网关上传消息须为 JSON 文本、最大 16384 个字符；后端下发完整任务可更大。

设备以“当前用户 ID + deviceId”隔离，用户归属从登录身份确定。`source` 允许 `simulated` 或 `dji`；它是网关声明的来源，不是对硬件身份的密码学证明。DJI 注册必须使用 `model=DJI Mini 3` 且不得声明模拟任务能力；其遥测不得带模拟来源、模拟定位或任务执行编号。

## 网关登记

```json
{"schemaVersion":1,"type":"register","deviceId":"sim-mini3-001","name":"Mini 3 接入模拟器","source":"simulated"}
```

`deviceId` 为 1–64 个英文字母、数字、下划线或连字符。设备名为 1–64 字符，模拟型号为“Mini 3 接入模拟器”。后端回复 `registered`，包含新生成的 `gatewaySessionId` 和 `staleAfterMs`。心跳收到 `heartbeat_ack`，便于安卓本地发现反向链路失联。

DJI 遥测用 `positioningQuality=gnss`、实际飞行模式和 `heightSource=dji_relative`；相对高度填入 `heightRelativeTakeoffMeters`，未经基准核验的 `heightEllipsoidMeters` 必须为空。未知高度使用 `heightSource=unavailable` 且两个高度均为空。缺少椭球高时保留状态读数，不生成猜测的地图三维位置。

每次重新登记生成新会话。相同用户和设备 ID 替换原会话，不新增重复设备。旧会话的数据、心跳和关闭操作均不能影响新会话。同一用户最多保留 32 台设备、同时 12 个监控订阅。

## 心跳与遥测

心跳和遥测共用一个单调递增的 `sequence`，新会话可重新从 0 开始。心跳默认每秒发送一次。

```json
{"schemaVersion":1,"type":"heartbeat","gatewaySessionId":"<登记返回的会话 ID>","sequence":1,"aircraftConnected":true}
```

遥测默认 5 Hz，消息示例：

```json
{
  "schemaVersion":1,
  "type":"telemetry",
  "gatewaySessionId":"<登记返回的会话 ID>",
  "sequence":2,
  "capturedAt":"2026-11-12T16:00:00+00:00",
  "sourceAgeMs":0,
  "horizontalDatum":"WGS84",
  "positionValid":true,
  "lat":34.21400484,
  "lng":117.13414207,
  "heightEllipsoidMeters":80,
  "heightRelativeTakeoffMeters":80,
  "heightSource":"simulation",
  "speedMps":3.49,
  "headingDegrees":0,
  "batteryPercent":100,
  "positioningQuality":"simulated",
  "flightMode":"SIMULATED"
}
```

`capturedAt` 必须包含时区，同一会话不能回退或重复。数据时效根据网关报告的 `sourceAgeMs` 与服务器本地单调时钟计算，不依赖电脑和网关的墙上时钟完全同步。

经纬度使用角度，高度与距离使用米，速度使用米/秒。航向为北向 0°、顺时针增加，范围 `[0,360)`；电量范围 `[0,100]`。所有数值须有限，未知的可空字段使用 `null`。有效位置必须包含经纬度，地图定位还要求已知椭球高；相对起飞高度不能替代椭球高绘图。高度不可用时使用 `heightSource: "unavailable"` 并将两个高度字段置空。

无效位置、越界值、乱序、重复采集时间及已过期的数据不更新最后有效位置，不延长该位置的新鲜度。`sourceAgeMs >= 3000` 的消息在默认配置下被拒绝。拒绝回复 `type: "error"` 和原因代码，不回显令牌或完整输入。

DJI 飞机尚无有效定位时，网关仍可发送 `positionValid: false` 的新鲜遥测，并将未知经纬度置为空。后端使用该消息更新电量和飞行模式，保留最后有效位置并将其标记为过期。网关首个心跳应明确报告飞机连接状态。

## 设备快照与网页订阅

认证后，网页先收到 `snapshot`，之后收到 `device_updated`。空闲时每秒发送一次 `server_heartbeat`，用于判断监控链路是否仍工作。

快照包含 `schemaVersion`、`revision`、`serverTime`、`staleAfterMs`、`offlineAfterMs` 和 `devices` 数组。设备状态包含：

| 字段 | 含义 |
| --- | --- |
| `deviceId`, `name`, `model`, `source` | 设备身份与模拟来源。 |
| `gatewaySessionId` | 当前有效网关会话。 |
| `gatewayConnected`, `aircraftConnected` | 网关连接与模拟设备连接状态。 |
| `telemetryStale`, `telemetryAgeMs` | 最近有效位置是否过期及其年龄。无本会话有效遥测时年龄为空。 |
| `receivedAt` | 后端最后接收有效遥测的 UTC 时间。 |
| `lastHeartbeatAgeMs` | 最近心跳年龄。 |
| `telemetry` | 最近有效遥测；尚无数据时为空。 |
| `aircraftStatus`, `aircraftStatusAgeMs` | DJI 最近新鲜状态与其年龄。状态包含 `capturedAt`、`batteryPercent`、`flightMode`。会话切换、断开或过期时清空。 |
| `revision` | 后端状态版本，过滤重复或乱序状态事件。 |

订阅连接的队列有上限。慢客户端积压时收到最新完整快照，避免不断累积旧数据。重连先同步快照；后端重启后版本号可能归零，新连接快照为当前依据。

## 状态和生命周期

- 连续 3 秒无新有效位置，位置标为过期，心跳可以仍在线。
- 正常关闭网关立即标为离线；连续 6 秒无心跳也标为离线。后台每 0.25 秒检查一次。
- 网关重新登记时，原位置可以继续保留显示，但标为过期，直到新会话提供有效数据。
- 网页监控链路断开后，不继续显示设备在线；缺少服务端消息 5 秒会触发重连。
- 前端每台设备最多保留 3000 个有效位置，退出登录后清除设备、航迹和连接。
- 设备状态与航迹仅存在内存；后端重启后重新登记，网页刷新后从服务器恢复最新状态，历史航迹不承诺保留。

实现入口：`drone_schemas.py`、`services/drone_registry.py`、`routers/drones.py`。依赖接口参考 [FastAPI WebSocket 文档](https://fastapi.tiangolo.com/advanced/websockets/) 与 [websockets asyncio 客户端文档](https://websockets.readthedocs.io/en/stable/reference/asyncio/client.html)。
