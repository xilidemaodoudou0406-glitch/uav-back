# 模拟任务扩展协议

只用于 `simulation_only` 范围，不接入真实飞行控制。沿用设备遥测协议 v1 的认证、用户隔离、会话和递增序号。

## 能力与任务快照

网关注册时可增加 `capabilities: ["mission_simulation"]`。没有这一能力的旧网关仍可上报阶段一遥测，但不能接收任务。

创建任务使用 `POST /api/missions`，数据包括 `name`、`deviceId`、`executionScope: "simulation_only"`、`heightReference: "model_ellipsoid_display"`、`trajectorySamples`。每个采样至少有 `lat`、`lng`、`altitude`、`timeSeconds`；可带 `speedMetersPerSecond` 和 `phase`。

时间从 0 开始并严格递增，2–20000 点、最长 3600 秒。服务端加入 `version: 1`、`calibrationStatus: "unverified"` 后，对标准化 JSON（键排序、无额外空格、UTF-8、禁止 NaN）计算 SHA-256。保存完整快照，不提供覆盖接口。

## HTTP 接口

| 接口 | 含义 |
| --- | --- |
| `POST /api/missions` | 创建不可变任务，返回 201。 |
| `GET /api/missions` | 当前用户最近 100 个任务的摘要。 |
| `GET /api/missions/{id}` | 完整快照、摘要、最近执行架次。 |
| `POST /api/missions/{id}/prepare` | 请求体含唯一 `commandId`，创建执行架次并发送准备指令，返回 202。 |
| `POST /api/missions/{id}/commands` | 请求体含 `commandId`、`runId` 和 `action`；动作包括 START、PAUSE、RESUME、ABORT。 |
| `GET /api/missions/{id}/runs/{runId}` | 指令、事件与位置记录。 |

所有接口使用现有登录鉴权。错误用户不可读写任务。同一设备只允许一个 preparing/ready/executing/paused 架次；准备必须等待网关确认。相同指令编号及相同内容重试返回原结果，不重新发送；相同编号配不同内容被拒绝。

## WebSocket 下发与回执

后端通过已认证的网关连接发送 `mission_command`：

```json
{
  "schemaVersion":1,
  "type":"mission_command",
  "commandId":"唯一指令编号",
  "missionId":"任务编号",
  "runId":"本次执行架次",
  "action":"PREPARE",
  "gatewaySessionId":"当前网关会话",
  "checksum":"64 位 SHA-256",
  "expiresAt":1794500000,
  "snapshot":{}
}
```

`snapshot` 仅在 PREPARE 中携带，示例中的空对象和时间值为占位符。第三、第四阶段同时增加 `canonicalSnapshot`：按后端排序规则生成的原始 UTF-8 JSON 字符串。安卓对该字符串核对 SHA-256 后，解析该字符串作为执行数据，不重新用 Java 数字格式化生成摘要；Python 模拟器也优先使用它。后续指令仍绑定架次、会话和摘要。认证回复增加 `serverEpochSeconds`，安卓换算指令的服务端时间；心跳回复 `heartbeat_ack` 用于本地失联判断。

PREPARE 现在强制重新运行 [坐标与任务约束预检](./navigation-assurance.md)，失败时拒绝下发。DJI 来源只允许遥测，不接收模拟任务。

网关校验后回复 `command_result`，包含公共 `schemaVersion`、`gatewaySessionId`、递增 `sequence`，以及 `commandId`、`runId`、`checksum`、`accepted`、`state`、`reason`、`elapsedSeconds`。回执与心跳/遥测共用发送锁和序号，保证实际发送顺序。回执中的任务时间用于同步暂停、继续等状态的进度。

遥测可增加 `missionRunId`、`missionState` 和 `missionElapsedSeconds`。后端仅接受当前用户、设备、会话及对应执行架次的任务记录，完成状态还须符合任务时间。

## 状态与中断

准备：preparing → ready。开始：ready → executing。暂停：executing → paused。继续：paused → executing。终止：ready/executing/paused → aborted。到达末点：executing → completed。

网关断开、更换会话、确认超时或后端重启会使未结束架次进入 failed，并记录原因。不会通过重连重放开始指令。确认超时还会关闭对应网关会话，避免服务端继续假定该执行可控。

HTTP 202 仅代表受理。任务摘要中的 `pendingCommand` 表示待确认指令，`lastCommand` 包含最近指令及结果；状态与进度以网关回执/遥测为依据。

历史记录按架次保存在现有 SQLite 的新增表中，与页面内存航迹独立。回放只读取样本，不发送设备操作。
