<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'
import mqtt from 'mqtt'
import * as echarts from 'echarts'

/*
 * 本文件是整个网页前端的主控组件，承担了以下职责：
 * 1. 管理登录、角色权限和界面交互。
 * 2. 直连 MQTT Broker 接收实时数据、发送控制命令。
 * 3. 通过本地 Node API 读取 SQLite 历史数据并绘图。
 * 4. 管理 ACK、超时、重试、动作记录和阈值编辑状态。
 *
 * 由于逻辑很多，这里按“常量 -> 状态 -> 计算属性 -> 工具函数 ->
 * MQTT 通信 -> 历史查询 -> 生命周期 -> 模板”顺序组织。
 */

/* ======================== 通信与行为常量 ======================== */
const DEFAULT_BROKER_URL = 'wss://zn2333.top/mqtt'
const MAX_LOGS = 60
const ACK_TIMEOUT_MS = 30000
const CMD_QOS = 1
const FAST_QUERY_QOS = 1
const MANUAL_RETRY_DELAY_MS = 4200
const MANUAL_STATUS_SYNC_DELAY_MS = 1400
const MANUAL_ACK_TIMEOUT_MS = 35000
const THRESHOLD_RETRY_DELAY_MS = 5200
const TELE_STALE_RECOVER_MS = 30000
const QUERY_MIN_GAP_MS = 900
const THRESHOLD_REPAIR_INTERVAL_MS = 8000
const HISTORY_REFRESH_MS = 5000
const WEB_ALIVE_HEARTBEAT_MS = 8000
const MQTT_RECONNECT_PERIOD_MS = 1000
const MQTT_CONNECT_TIMEOUT_MS = 10000
const MQTT_KEEPALIVE_S = 30
const ADMIN_LOGIN_PASSWORD = '666'
const ACTION_DUP_SUPPRESS_MS = 15000
const CMD_RATE_LIMIT_MS = 3000
const GATEWAY_OFFLINE_TIMEOUT_MS = 30000

/* 手动控制状态码对应的界面文本。 */
const MANUAL_STATE_TEXT = {
  0: '手动关',
  1: '手动开',
  2: '自动关',
  3: '自动开',
  4: '手动关(执行中)',
  5: '手动开(执行中)'
}

/* ACK 返回的简短 reason 码转中文。 */
const ACK_REASON_TEXT = {
  missing: '缺少参数',
  json: 'JSON 格式错误',
  range: '参数越界',
  busy: '设备忙'
}

/* 连接状态监测卡片的展示项定义。 */
const CONNECTION_ITEMS = [
  { key: 'th', label: '温湿度传感器' },
  { key: 'pm', label: '粉尘传感器' },
  { key: 'gas', label: '气体传感器' },
  { key: 'human', label: '远程毫米波雷达' },
  { key: 'light', label: '远程光照传感器' },
  { key: 'door', label: '远程门磁开关' }
]

/* 设备状态区和手动控制区共用的设备清单。 */
const DEVICE_ITEMS = [
  { dev: 'heater', label: '制热器', kind: 'binary' },
  { dev: 'cooler', label: '制冷器', kind: 'binary' },
  { dev: 'humidifier', label: '加湿器', kind: 'binary' },
  { dev: 'dehumidifier', label: '除湿器', kind: 'binary' },
  { dev: 'alarm', label: '声光报警器', kind: 'binary' },
  { dev: 'light', label: 'LED 灯', kind: 'binary' },
  { dev: 'fan', label: '风扇', kind: 'gear' }
]

/*
 * 阈值元数据：
 * - id 对应 STM32 侧阈值编号。
 * - key 是前端内部统一字段名。
 * - 其余字段用于输入校验、显示单位和标题。
 */
const THRESHOLD_ITEMS = [
  { id: 0, key: 'temp_low', label: '加热器阈值', type: 'float', unit: '°C', min: -50, max: 100 },
  { id: 1, key: 'temp_high', label: '制冷器阈值', type: 'float', unit: '°C', min: -50, max: 100 },
  { id: 2, key: 'humidity_low', label: '加湿器阈值', type: 'float', unit: '%RH', min: 0, max: 100 },
  { id: 3, key: 'humidity_fan_high', label: '除湿/湿度风扇阈值', type: 'float', unit: '%RH', min: 0, max: 100 },
  { id: 4, key: 'pm25_alarm_delay_s', label: '粉尘报警延时', type: 'float', unit: 's', min: 0, max: 86400 },
  { id: 5, key: 'gas_alarm_delay_s', label: '气体报警延时', type: 'float', unit: 's', min: 0, max: 86400 },
  { id: 6, key: 'pm25_alarm_threshold', label: '粉尘报警阈值', type: 'float', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 7, key: 'gas_alarm_threshold', label: '气体报警阈值', type: 'float', unit: 'PPM', min: 0, max: 9999 },
  { id: 8, key: 'door_open_alarm_s', label: '门未关报警延时', type: 'float', unit: 's', min: 0, max: 86400 },
  { id: 9, key: 'light_on_threshold', label: '光感开灯阈值', type: 'float', unit: 'lux', min: 0, max: 9999 },
  { id: 10, key: 'human_light_distance', label: '人感距离阈值', type: 'float', unit: 'cm', min: 0, max: 999.9 },
  { id: 11, key: 'fan_speed_l1', label: '风扇1档速度', type: 'float', unit: '%', min: 0, max: 100 },
  { id: 12, key: 'fan_speed_l2', label: '风扇2档速度', type: 'float', unit: '%', min: 0, max: 100 },
  { id: 13, key: 'fan_speed_l3', label: '风扇3档速度', type: 'float', unit: '%', min: 0, max: 100 },
  { id: 14, key: 'fan_speed_l4', label: '风扇4档速度', type: 'float', unit: '%', min: 0, max: 100 },
  { id: 15, key: 'fan_speed_l5', label: '风扇5档速度', type: 'float', unit: '%', min: 0, max: 100 },
  { id: 17, key: 'pm_l1_low', label: '粉尘1档下限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 18, key: 'pm_l1_high', label: '粉尘1档上限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 19, key: 'pm_l2_low', label: '粉尘2档下限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 20, key: 'pm_l2_high', label: '粉尘2档上限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 21, key: 'pm_l3_low', label: '粉尘3档下限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 22, key: 'pm_l3_high', label: '粉尘3档上限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 23, key: 'pm_l4_low', label: '粉尘4档下限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 24, key: 'pm_l4_high', label: '粉尘4档上限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 25, key: 'pm_l5_low', label: '粉尘5档下限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 26, key: 'pm_l5_high', label: '粉尘5档上限', type: 'int', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 33, key: 'gas_l1_low', label: '气体1档下限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 34, key: 'gas_l1_high', label: '气体1档上限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 35, key: 'gas_l2_low', label: '气体2档下限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 36, key: 'gas_l2_high', label: '气体2档上限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 37, key: 'gas_l3_low', label: '气体3档下限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 38, key: 'gas_l3_high', label: '气体3档上限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 39, key: 'gas_l4_low', label: '气体4档下限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 40, key: 'gas_l4_high', label: '气体4档上限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 41, key: 'gas_l5_low', label: '气体5档下限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 42, key: 'gas_l5_high', label: '气体5档上限', type: 'int', unit: 'PPM', min: 0, max: 9999 },
  { id: 48, key: 'hyst_temp', label: '温度迟滞', type: 'float', unit: '°C', min: 0, max: 50 },
  { id: 49, key: 'hyst_humi', label: '湿度迟滞', type: 'float', unit: '%RH', min: 0, max: 50 },
  { id: 50, key: 'hyst_light', label: '光照迟滞', type: 'float', unit: 'lux', min: 0, max: 9999 },
  { id: 51, key: 'hyst_gas', label: '气体报警迟滞', type: 'float', unit: 'PPM', min: 0, max: 9999 },
  { id: 52, key: 'hyst_pm25', label: '粉尘报警迟滞', type: 'float', unit: 'ug/m3', min: 0, max: 9999 },
  { id: 53, key: 'min_switch_s', label: '最小切换间隔', type: 'float', unit: 's', min: 0, max: 3600 }
]

/* 便于通过 id / key 快速查找阈值定义，避免反复遍历。 */
const thresholdById = new Map(THRESHOLD_ITEMS.map((item) => [String(item.id), item]))
const thresholdByKey = new Map(THRESHOLD_ITEMS.map((item) => [item.key, item]))

/* ======================== 基础连接与登录状态 ======================== */
const brokerUrl = ref(DEFAULT_BROKER_URL)
const gatewayId = ref('gw001')
const clientId = ref(makeClientId())
const connectionState = ref('未连接')
const isConnected = ref(false)
const lastError = ref('')
const mqttClient = ref(null)
const isLoggedIn = ref(false)
const loginRole = ref('normal')
const userRole = ref('')
const adminPasswordInput = ref('')
const loginError = ref('')

/* ======================== 实时 MQTT 数据缓存 ======================== */
const sensorsData = ref(null)
const statusData = ref(null)
const thresholdsData = ref(null)

/* 记录各类实时数据最近一次收到的时间。 */
const lastSensorsAt = ref('')
const lastStatusAt = ref('')
const lastThresholdsAt = ref('')
const lastAckAt = ref('')

/* ======================== 命令日志与提示 ======================== */
const commandLogs = ref([])
let commandSeq = 0
let requestSeq = 0

const toastMessage = ref('')
const toastVisible = ref(false)
let toastTimer = null
let lastCmdSentAt = 0

/* ======================== 阈值、历史查询表单状态 ======================== */
const thresholdDrafts = reactive({})
const thresholdKeyword = ref('')
const thresholdOnlyChanged = ref(false)
const thresholdSavingAll = ref(false)
const historyHours = ref(1)
const historyRangeMode = ref('hours')
const historyFromLocal = ref('')
const historyToLocal = ref('')
const historyIntervalSec = ref(0)
const actionHours = ref(1)
const actionRangeMode = ref('hours')
const actionFromLocal = ref('')
const actionToLocal = ref('')

/* 历史图表中可切换的传感器序列定义。 */
const HISTORY_SERIES_META = [
  { key: 'temp', name: '温度' },
  { key: 'humi', name: '湿度' },
  { key: 'pm25', name: 'PM2.5' },
  { key: 'gas', name: '气体' },
  { key: 'pwm', name: 'PWM' },
  { key: 'dist', name: '距离' },
  { key: 'light', name: '光照' },
  { key: 'human', name: '人员(0/1)' },
  { key: 'door', name: '门状态(0/1)' }
]

/* 各条折线的固定配色，保持刷新后颜色不跳动。 */
const HISTORY_SERIES_COLORS = {
  temp: '#1f77b4',
  humi: '#2ca02c',
  pm25: '#d62728',
  gas: '#ff7f0e',
  pwm: '#9467bd',
  dist: '#17becf',
  light: '#bcbd22',
  human: '#e377c2',
  door: '#7f7f7f'
}

/* 图表勾选框状态，决定哪些序列参与绘制。 */
const historySeriesSelected = reactive({
  temp: true,
  humi: true,
  pm25: true,
  gas: true,
  pwm: true,
  dist: true,
  light: true,
  human: true,
  door: true
})

/* 记录 4 张图的 dataZoom 区间，避免刷新数据时丢失当前缩放位置。 */
const historyZoom = reactive({
  th: { start: 0, end: 100 },
  pm_light: { start: 0, end: 100 },
  gas_pwm: { start: 0, end: 100 },
  human_dist_door: { start: 0, end: 100 }
})

/* ======================== 历史数据请求与结果状态 ======================== */
const historyLoading = ref(false)
const historyError = ref('')
const historyNotice = ref('')
const historyDeleteBusy = ref(false)
const actionLoading = ref(false)
const actionError = ref('')
const actionNotice = ref('')
const actionDeleteBusy = ref(false)
const sensorHistory = ref([])
const actionHistory = ref([])

/* 统一保证 actionRows 一定是数组，方便下游直接 map。 */
const actionRows = computed(() => {
  return Array.isArray(actionHistory.value) ? actionHistory.value : []
})
/*
 * 将动作记录转换成表格展示结构：
 * - 统一设备中文名。
 * - 统一动作文本。
 * - 优先使用 detail 中已经格式化好的日期时间。
 */
const actionDisplayRows = computed(() => {
  const mapDeviceName = (dev) => {
    const key = String(dev || '').toLowerCase()
    if (key === 'heater') return '制热器'
    if (key === 'cooler') return '制冷器'
    if (key === 'humidifier') return '加湿器'
    if (key === 'dehumidifier') return '除湿器'
    if (key === 'alarm') return '声光报警器'
    if (key === 'light') return 'LED灯'
    if (key === 'fan') return '风扇'
    if (key.startsWith('relay_ch')) {
      const ch = key.replace('relay_ch', '')
      return `继电器CH${ch}`
    }
    if (!key || key === 'unknown' || key === '-') {
      return '-'
    }
    return String(dev)
  }

  const mapActionText = (row) => {
    const mode = String(row?.mode || '').toLowerCase() === 'manual' ? '手动' : '自动'
    const raw = String(row?.action || '').toLowerCase()

    if ((String(row?.device || '').toLowerCase() === 'fan') && (raw.startsWith('gear_') || raw.startsWith('value_'))) {
      const gear = raw.split('_')[1]
      return `${mode}${gear || '-'}档`
    }
    if (raw === 'on' || raw === 'value_1') {
      return `${mode}开`
    }
    if (raw === 'off' || raw === 'value_0') {
      return `${mode}关`
    }

    if (raw.startsWith('pwm_')) {
      return `手动PWM${raw.split('_')[1] || ''}`
    }
    if (!raw || raw === 'unknown' || raw === 'pwm_unknown' || raw === '-') {
      return '-'
    }
    return row?.action || '-'
  }

  const formatDateFromMs = (ms) => {
    const n = Number(ms)
    if (!Number.isFinite(n) || n <= 0) return '-'
    const d = new Date(n)
    return `${d.getFullYear()}/${d.getMonth() + 1}/${d.getDate()}`
  }
  const formatTimeFromMs = (ms) => {
    const n = Number(ms)
    if (!Number.isFinite(n) || n <= 0) return '-'
    const d = new Date(n)
    const hh = String(d.getHours()).padStart(2, '0')
    const mm = String(d.getMinutes()).padStart(2, '0')
    return `${hh}:${mm}`
  }

  return actionRows.value.map((row) => {
    const detail = row?.detail || {}
    const dateText = (typeof detail.date === 'string' && detail.date) ? detail.date : formatDateFromMs(row?.ts)
    const timeText = (typeof detail.time === 'string' && detail.time) ? detail.time : formatTimeFromMs(row?.ts)
    return {
      ...row,
      dateText,
      timeText,
      deviceText: mapDeviceName(row?.device || detail?.dev),
      actionText: mapActionText(row)
    }
  })
})
/* 快速取动作表第一行，供“最新上次动作”摘要区域显示。 */
const latestAction = computed(() => {
  if (!actionDisplayRows.value.length) {
    return {
      dateText: '-',
      timeText: '-',
      deviceText: '-',
      actionText: '-'
    }
  }
  return actionDisplayRows.value[0]
})
/* ======================== ECharts DOM 引用与实例句柄 ======================== */
const historyChartElTh = ref(null)
const historyChartElPmLight = ref(null)
const historyChartElGasPwm = ref(null)
const historyChartElHumanDistDoor = ref(null)
let historyChartTh = null
let historyChartPmLight = null
let historyChartGasPwm = null
let historyChartHumanDistDoor = null
const historyZoomHandlers = {
  th: null,
  pm_light: null,
  gas_pwm: null,
  human_dist_door: null
}
let historyTimer = null
let actionTimer = null
let resizeHandler = null
let webAliveTimer = null
let bootstrapQueryTimer = null
let manualStatusQueryTimer = null
let telemetryRecoverTimer = null
let thresholdRepairTimer = null
let thresholdRepairRound = 0
let gatewayOfflineTimer = null

/* 记录各类 query 最近一次发送时间，用于节流。 */
const queryLastSendAt = new Map()

/* 手动控制区默认表单值。 */
const manualForms = reactive({
  heater: { sec: 30, value: 1 },
  cooler: { sec: 30, value: 1 },
  humidifier: { sec: 30, value: 1 },
  dehumidifier: { sec: 30, value: 1 },
  alarm: { sec: 30, value: 1 },
  light: { sec: 30, value: 1 },
  fan: { sec: 30, value: 2 }
})

/* ======================== 计算属性：主题、权限、展示卡片 ======================== */
const cmdBaseTopic = computed(() => `ws/${gatewayId.value}/cmd`)
const isAdmin = computed(() => userRole.value === 'admin')
const roleText = computed(() => (isAdmin.value ? '管理员' : (userRole.value === 'normal' ? '普通用户' : '-')))

const sensorCards = computed(() => {
  const s = sensorsData.value ?? {}
  const remote = statusData.value?.sensor ?? {
    human: s.human,
    dist: s.dist,
    light: s.light,
    door: s.door
  }
  const conn = statusData.value?.conn ?? {}
  const connHuman = (conn.human !== undefined) ? conn.human : ((s.human !== undefined || s.dist !== undefined) ? 1 : undefined)
  const connLight = (conn.light !== undefined) ? conn.light : ((s.light !== undefined) ? 1 : undefined)
  const connDoor = (conn.door !== undefined) ? conn.door : ((s.door !== undefined) ? 1 : undefined)

  const valueOrDisconnected = (connected, valueText) => {
    if (connected === 0) {
      return '未连接'
    }
    return valueText
  }

  return [
    { label: '温度', value: valueOrDisconnected(conn.th, formatValue(s.temp, '°C')) },
    { label: '湿度', value: valueOrDisconnected(conn.th, formatValue(s.humi, '%RH')) },
    { label: 'PM2.5', value: valueOrDisconnected(conn.pm, formatValue(s.pm25, '')) },
    { label: '气体', value: valueOrDisconnected(conn.gas, formatValue(s.gas, '')) },
    { label: '风扇PWM', value: formatValue(s.pwm, '%') },
    {
      label: '人员监测',
      value: valueOrDisconnected(connHuman, remote.human === 1 ? '有人' : remote.human === 0 ? '无人' : '-')
    },
    { label: '距离', value: valueOrDisconnected(connHuman, formatValue(remote.dist, '')) },
    { label: '光照', value: valueOrDisconnected(connLight, formatValue(remote.light, '')) },
    {
      label: '门状态',
      value: valueOrDisconnected(connDoor, remote.door === 1 ? '关闭' : remote.door === 0 ? '打开' : '-')
    }
  ]
})

/* 将 manual 状态映射成设备状态卡片文本。 */
const deviceStatusCards = computed(() => {
  const manual = statusData.value?.manual ?? sensorsData.value?.manual ?? {}

  return DEVICE_ITEMS.map((item) => {
    if (item.dev === 'fan') {
      const mode = Number(manual.fan_mode)
      const gear = Number(manual.fan_gear)
      if (!Number.isFinite(mode) || !Number.isFinite(gear)) {
        return { ...item, text: '-' }
      }
      return { ...item, text: `${mode === 1 ? '手动' : '自动'}${gear}档` }
    }

    const code = Number(manual[item.dev])
    return { ...item, text: manualStateText(code) }
  })
})

/* 汇总 WiFi、继电器链路和各个传感器在线状态。 */
const connectionCards = computed(() => {
  const conn = statusData.value?.conn ?? {}
  const wifi = statusData.value?.wifi
  const relayLink = statusData.value?.relay_link

  const base = CONNECTION_ITEMS.map((item) => ({
    label: item.label,
    value:
      conn[item.key] === 1
        ? '已连接'
        : conn[item.key] === 0
          ? '未连接'
          : '-'
  }))

  base.unshift({
    label: 'WiFi',
    value: wifi === 1 ? '已连接' : wifi === 0 ? '未连接' : '-'
  })
  base.push({ label: '继电器', value: relayLink === 1 ? '已连接' : relayLink === 0 ? '未连接' : '-' })

  return base
})

/* 判断阈值草稿是否相对当前值发生变化，用于高亮和批量保存。 */
function isThresholdDraftChanged(item, currentValue, draftValue) {
  const cur = Number(currentValue)
  const draft = Number(draftValue)

  if (!Number.isFinite(cur) || !Number.isFinite(draft)) {
    return false
  }

  if (item.type === 'int') {
    return Math.trunc(cur) !== Math.trunc(draft)
  }

  return Math.abs(cur - draft) >= 0.0001
}

/*
 * 从 thresholdsData 中提取统一的“id -> value”映射。
 * 兼容 MCU 返回扁平对象和 { v: {...} } 两种结构。
 */
function getThresholdMapFromState() {
  const state = thresholdsData.value
  if (!state || typeof state !== 'object') {
    return {}
  }

  const source = (
    state.v &&
    typeof state.v === 'object' &&
    !Array.isArray(state.v)
  ) ? state.v : state

  const out = {}
  THRESHOLD_ITEMS.forEach((item) => {
    const idKey = String(item.id)
    let raw

    if (Object.prototype.hasOwnProperty.call(source, idKey)) {
      raw = source[idKey]
    } else if (Object.prototype.hasOwnProperty.call(source, item.key)) {
      raw = source[item.key]
    } else {
      return
    }

    const normalized = normalizeThresholdValue(item, raw)
    if (normalized === null) {
      return
    }
    out[idKey] = normalized
  })

  return out
}

/* 将阈值当前值、草稿值、改动状态整理成列表。 */
const thresholdRows = computed(() => {
  const map = getThresholdMapFromState()

  return THRESHOLD_ITEMS.map((item) => {
    const key = String(item.id)
    const current = Object.prototype.hasOwnProperty.call(map, key) ? map[key] : null
    const draft = thresholdDrafts[key]
    const changed = isThresholdDraftChanged(item, current, draft)

    return {
      ...item,
      current,
      draft,
      changed
    }
  })
})

/* 搜索与“只看已改动”筛选后的阈值列表。 */
const thresholdRowsVisible = computed(() => {
  const keyword = String(thresholdKeyword.value || '').trim().toLowerCase()

  return thresholdRows.value.filter((row) => {
    if (thresholdOnlyChanged.value && !row.changed) {
      return false
    }

    if (!keyword) {
      return true
    }

    return (
      String(row.label || '').toLowerCase().includes(keyword) ||
      String(row.id).includes(keyword)
    )
  })
})

/* 当前已改动的阈值数量，供工具栏摘要显示。 */
const thresholdChangedCount = computed(() => thresholdRows.value.filter((row) => row.changed).length)

/* 生成 MQTT clientId，避免多个浏览器客户端互相踢下线。 */
function makeClientId() {
  const rand = Math.random().toString(16).slice(2, 10)
  const stamp = Date.now().toString(36)
  return `web_${rand}_${stamp}`
}

/* 手动刷新 clientId，便于用户重新登录或切换连接。 */
function refreshClientId() {
  clientId.value = makeClientId()
}

/* 在登录弹窗中切换普通用户 / 管理员角色。 */
function loginAs(role) {
  loginRole.value = role === 'admin' ? 'admin' : 'normal'
  loginError.value = ''
}

/* 提交登录：管理员校验密码，成功后进入系统并触发连接。 */
function submitLogin() {
  loginError.value = ''

  if (loginRole.value === 'admin') {
    if (!adminPasswordInput.value) {
      loginError.value = '请输入管理员密码'
      return
    }
    if (adminPasswordInput.value !== ADMIN_LOGIN_PASSWORD) {
      loginError.value = '管理员密码错误'
      return
    }
  }

  userRole.value = loginRole.value
  isLoggedIn.value = true
  adminPasswordInput.value = ''

  if (!isConnected.value) {
    connectMqtt()
  }
  refreshHistoryData()
}

/* 登出后重置角色状态，并主动断开 MQTT。 */
function logoutCurrentUser() {
  isLoggedIn.value = false
  userRole.value = ''
  loginRole.value = 'normal'
  adminPasswordInput.value = ''
  loginError.value = ''
  disconnectMqtt()
}

/* 所有写操作统一走这里校验管理员权限。 */
function ensureAdminPermission() {
  if (isAdmin.value) {
    return true
  }
  lastError.value = '普通用户无权限执行该操作'
  return false
}

/* 页面顶部临时浮层提示。 */
function showToast(msg, durationMs = 2000) {
  toastMessage.value = msg
  toastVisible.value = true
  if (toastTimer) clearTimeout(toastTimer)
  toastTimer = setTimeout(() => { toastVisible.value = false }, durationMs)
}

/* 对用户主动写命令做最小发送间隔限制，防止误触连续发送。 */
function checkRateLimit() {
  const now = Date.now()
  if (now - lastCmdSentAt < CMD_RATE_LIMIT_MS) {
    const remain = Math.ceil((CMD_RATE_LIMIT_MS - (now - lastCmdSentAt)) / 1000)
    showToast(`请勿频繁发送，请等待 ${remain} 秒后重试`, 2000)
    return false
  }
  lastCmdSentAt = now
  return true
}

/*
 * 建立 MQTT 连接，并在 connect/offline/close/error/message 中维护整页状态。
 * 连接成功后会自动补查询基础数据、启动保活和恢复机制。
 */
function connectMqtt() {
  if (mqttClient.value && isConnected.value) {
    return
  }
  if (connectionState.value === '连接中' || connectionState.value === '重连中') {
    return
  }

  if (mqttClient.value) {
    mqttClient.value.end(true)
    mqttClient.value = null
  }

  lastError.value = ''
  connectionState.value = '连接中'
  sensorsData.value = null
  statusData.value = null
  thresholdsData.value = null
  lastSensorsAt.value = ''
  lastStatusAt.value = ''
  lastThresholdsAt.value = ''
  stopManualQueue()
  queryLastSendAt.clear()

  const client = mqtt.connect(brokerUrl.value, {
    clientId: clientId.value,
    clean: true,
    reconnectPeriod: MQTT_RECONNECT_PERIOD_MS,
    connectTimeout: MQTT_CONNECT_TIMEOUT_MS,
    keepalive: MQTT_KEEPALIVE_S,
    resubscribe: true
  })

  mqttClient.value = client

  client.on('connect', () => {
    isConnected.value = true
    connectionState.value = '已连接'
    sensorsData.value = null
    statusData.value = null
    thresholdsData.value = null
    lastSensorsAt.value = ''
    lastStatusAt.value = ''
    lastThresholdsAt.value = ''
    subscribeTopics(() => {
      startBootstrapQuery()
      startTelemetryRecover()
      startThresholdRepair()
    })

    startWebAliveHeartbeat()
    resetGatewayOfflineTimer()
  })

  client.on('reconnect', () => {
    connectionState.value = '重连中'
  })

  client.on('offline', () => {
    isConnected.value = false
    connectionState.value = '离线'
    stopWebAliveHeartbeat()
    stopBootstrapQuery()
    stopManualQueue()
    stopTelemetryRecover()
    stopThresholdRepair()
    stopGatewayOfflineTimer()
  })

  client.on('close', () => {
    isConnected.value = false
    if (connectionState.value !== '未连接') {
      connectionState.value = '连接关闭'
    }
    stopWebAliveHeartbeat()
    stopBootstrapQuery()
    stopManualQueue()
    stopTelemetryRecover()
    stopThresholdRepair()
    stopGatewayOfflineTimer()
  })

  client.on('error', (err) => {
    lastError.value = err?.message ?? '连接错误'
    connectionState.value = '连接错误'
  })

  client.on('message', (topic, payload) => {
    handleMessage(topic, payload)
  })
}

/* 主动断开 MQTT，并统一清理所有相关定时器和瞬时状态。 */
function disconnectMqtt() {
  if (!mqttClient.value) {
    stopWebAliveHeartbeat()
    stopBootstrapQuery()
    stopManualQueue()
    stopTelemetryRecover()
    stopThresholdRepair()
    stopGatewayOfflineTimer()
    queryLastSendAt.clear()
    return
  }
  mqttClient.value.end(true)
  mqttClient.value = null
  isConnected.value = false
  connectionState.value = '未连接'
  queryLastSendAt.clear()
  stopWebAliveHeartbeat()
  stopBootstrapQuery()
  stopManualQueue()
  stopTelemetryRecover()
  stopThresholdRepair()
  stopGatewayOfflineTimer()
}

/* 停止网页端心跳上报。 */
function stopWebAliveHeartbeat() {
  if (webAliveTimer) {
    clearInterval(webAliveTimer)
    webAliveTimer = null
  }
}

/* 停止启动阶段的补查询流程。 */
function stopBootstrapQuery() {
  if (bootstrapQueryTimer) {
    clearTimeout(bootstrapQueryTimer)
    bootstrapQueryTimer = null
  }
}

/* 停止阈值补全轮询。 */
function stopThresholdRepair() {
  if (thresholdRepairTimer) {
    clearInterval(thresholdRepairTimer)
    thresholdRepairTimer = null
  }
}

/*
 * 阈值补全：
 * 页面刚连上时可能只拿到部分阈值，定时发 query 直到阈值数据完整。
 */
function startThresholdRepair() {
  stopThresholdRepair()
  thresholdRepairRound = 0

  const tick = () => {
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    if (hasPendingWriteCommands()) {
      return
    }
    if (hasCompleteThresholdsData()) {
      return
    }
    const plan = ['thresholds']
    const what = plan[thresholdRepairRound % plan.length]
    thresholdRepairRound += 1
    sendQuery(what, false, FAST_QUERY_QOS)
  }

  tick()
  thresholdRepairTimer = setInterval(tick, THRESHOLD_REPAIR_INTERVAL_MS)
}

/* ISO 时间字符串转毫秒时间戳，失败则返回 0。 */
function parseIsoMs(iso) {
  if (!iso) {
    return 0
  }
  const ms = Date.parse(iso)
  return Number.isFinite(ms) ? ms : 0
}

/* 取最近一次实时消息到达时间，用于判断实时数据是否陈旧。 */
function getLatestRealtimeMs() {
  return Math.max(
    parseIsoMs(lastSensorsAt.value),
    parseIsoMs(lastStatusAt.value),
    parseIsoMs(lastThresholdsAt.value),
    parseIsoMs(lastAckAt.value)
  )
}

/* 停止“网关长时间无实时消息”检测。 */
function stopGatewayOfflineTimer() {
  if (gatewayOfflineTimer) {
    clearTimeout(gatewayOfflineTimer)
    gatewayOfflineTimer = null
  }
}

/* 只要有新实时消息到来，就重置一次网关掉线计时器。 */
function resetGatewayOfflineTimer() {
  stopGatewayOfflineTimer()
  if (!mqttClient.value || !isConnected.value) {
    return
  }
  gatewayOfflineTimer = setTimeout(() => {
    gatewayOfflineTimer = null
    sensorsData.value = null
    statusData.value = null
  }, GATEWAY_OFFLINE_TIMEOUT_MS)
}

/* 停止实时数据恢复轮询。 */
function stopTelemetryRecover() {
  if (telemetryRecoverTimer) {
    clearInterval(telemetryRecoverTimer)
    telemetryRecoverTimer = null
  }
}

/*
 * 当 sensors/status 长时间没有新消息时，主动 query 一次，拉回实时状态。
 */
function runTelemetryRecoverNow() {
  if (!mqttClient.value || !isConnected.value) {
    return
  }
  if (hasPendingWriteCommands()) {
    return
  }
  subscribeTopics(() => {
    sendQuery('all', false, FAST_QUERY_QOS)
  })
}

/* 启动实时状态恢复定时器。 */
function startTelemetryRecover() {
  stopTelemetryRecover()

  telemetryRecoverTimer = setInterval(() => {
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    const latestMs = getLatestRealtimeMs()
    const now = Date.now()
    if ((latestMs > 0) && ((now - latestMs) < TELE_STALE_RECOVER_MS)) {
      return
    }
    runTelemetryRecoverNow()
  }, 15000)
}

/* 当前是否还有等待 ACK 的手动控制命令。 */
function hasPendingManualCommands() {
  return commandLogs.value.some((entry) => entry?.status === 'pending' && entry?.cmd === 'manual')
}

/* 当前是否还有等待 ACK 的阈值写命令。 */
function hasPendingThresholdCommands() {
  return commandLogs.value.some((entry) => entry?.status === 'pending' && entry?.cmd === 'threshold')
}

/* 手动控制和阈值修改都属于写命令，补查询时应尽量避开。 */
function hasPendingWriteCommands() {
  return hasPendingManualCommands() || hasPendingThresholdCommands()
}

/* 停止手动命令后的补状态查询定时器。 */
function stopManualStatusQueryTimer() {
  if (manualStatusQueryTimer) {
    clearTimeout(manualStatusQueryTimer)
    manualStatusQueryTimer = null
  }
}

/* 停止所有与手动控制命令补偿相关的定时器。 */
function stopManualQueue() {
  stopManualStatusQueryTimer()
}

/*
 * 手动命令发出后，延迟查询一次 status。
 * 目的不是替代 ACK，而是在 ACK 丢失时用状态回传反推命令是否执行成功。
 */
function scheduleManualStatusQuery(matchKey = '', delayMs = MANUAL_STATUS_SYNC_DELAY_MS, reqId = '') {
  stopManualStatusQueryTimer()
  manualStatusQueryTimer = setTimeout(() => {
    manualStatusQueryTimer = null
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    if (reqId && !findPendingByReq(reqId)) {
      return
    }
    if (!reqId && matchKey && !findPendingByMatchKeyOldest(matchKey)) {
      return
    }
    sendQuery('status', false, FAST_QUERY_QOS, 900)
  }, Math.max(0, Math.trunc(delayMs)))
}

/*
 * 页面首次连上时补拉基础数据。
 * 如果仅靠被动订阅，用户刚打开页面时可能需要等待设备下一次上报才能看到完整信息。
 */
function startBootstrapQuery() {
  stopBootstrapQuery()
  if (!mqttClient.value || !isConnected.value) {
    return
  }

  sendQuery('all', false, FAST_QUERY_QOS)

  bootstrapQueryTimer = setTimeout(() => {
    bootstrapQueryTimer = null
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    if (!statusData.value) {
      sendQuery('status', false, FAST_QUERY_QOS)
    }
    if (!sensorsData.value) {
      sendQuery('sensors', false, FAST_QUERY_QOS)
    }
    if (!hasCompleteThresholdsData()) {
      sendQuery('thresholds', false, FAST_QUERY_QOS)
    }
  }, 1200)
}

/* 定时上报一个轻量 query，用于维持网页端活跃感知并尽快发现链路异常。 */
function startWebAliveHeartbeat() {
  stopWebAliveHeartbeat()
  webAliveTimer = setInterval(() => {
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    mqttClient.value.publish(
      `${cmdBaseTopic.value}/ping`,
      JSON.stringify({ ts: Date.now() }),
      { qos: 0 }
    )
  }, WEB_ALIVE_HEARTBEAT_MS)
}

/* 精确订阅列表。通配订阅失败时回退使用。 */
function getTeleTopics() {
  const root = `ws/${gatewayId.value}/tele`
  return [
    `${root}/status`,
    `${root}/sensors`,
    `${root}/thresholds`,
    `${root}/thresholds/change1`,
    `${root}/thresholds/change2`,
    `${root}/thresholds/change3`,
    `${root}/thresholds/change4`,
    `${root}/thresholds/change5`,
    `${root}/thresholds/change6`,
    `${root}/ack`,
    `${root}/action`
  ]
}

/* 统计订阅结果中成功 / 拒绝数量，便于统一判断。 */
function countSubscribeGranted(granted) {
  if (!Array.isArray(granted) || granted.length <= 0) {
    return { ok: 1, denied: 0, total: 1 }
  }

  let ok = 0
  let denied = 0
  granted.forEach((item) => {
    const qos = Number(item?.qos)
    if (Number.isFinite(qos) && qos >= 0 && qos < 128) {
      ok += 1
    } else {
      denied += 1
    }
  })
  return { ok, denied, total: granted.length }
}

/*
 * 优先尝试订阅 tele/#，失败后降级为逐个 topic 精确订阅。
 * 这样既兼容权限更严格的 Broker，也能减少初始化失败率。
 */
function subscribeTopics(onDone) {
  if (!mqttClient.value || !isConnected.value) {
    return
  }

  const wildcard = `ws/${gatewayId.value}/tele/#`
  mqttClient.value.subscribe(wildcard, { qos: 0 }, (wildErr, wildGranted) => {
    const wildResult = countSubscribeGranted(wildGranted)
    if (!wildErr && wildResult.ok > 0 && wildResult.denied <= 0) {
      if (typeof onDone === 'function') {
        onDone()
      }
      return
    }

    const fallbackReason = wildErr
      ? (wildErr.message || '未知错误')
      : `授权拒绝(${wildResult.denied}/${wildResult.total})`
    lastError.value = `通配订阅失败，降级精确订阅: ${fallbackReason}`

    const topics = getTeleTopics()
    mqttClient.value.subscribe(topics, { qos: 0 }, (err, granted) => {
      const result = countSubscribeGranted(granted)
      if (err) {
        lastError.value = `订阅失败: ${err.message || '未知错误'}`
      } else if (result.denied > 0) {
        lastError.value = `订阅被拒绝 ${result.denied}/${result.total}`
      }

      if (typeof onDone === 'function') {
        onDone()
      }
    })
  })
}

/* MQTT 收消息总入口：按 topic 类型拆分到不同状态处理逻辑。 */
function handleMessage(topic, payloadBuffer) {
  const payloadText = payloadBuffer.toString()
  let payload

  try {
    payload = JSON.parse(payloadText)
  } catch (err) {
    lastError.value = `消息解析失败(${topic}): ${err.message}`
    return
  }

  const now = new Date().toISOString()

  if (topic.endsWith('/sensors')) {
    sensorsData.value = payload
    lastSensorsAt.value = now
    resetGatewayOfflineTimer()
    stopBootstrapIfReady()
    return
  }

  if (topic.endsWith('/status')) {
    statusData.value = payload
    applyManualStatusAsAckFallback(payload)
    lastStatusAt.value = now
    resetGatewayOfflineTimer()
    stopBootstrapIfReady()
    return
  }

  if (topic.endsWith('/thresholds')) {
    const normalized = extractThresholdMapFromPayload(payload)
    if (normalized) {
      mergeNormalizedThresholdMap(normalized, payload)
    } else {
      thresholdsData.value = payload
      syncThresholdDrafts()
    }
    lastThresholdsAt.value = now
    if (hasCompleteThresholdsData()) {
      stopThresholdRepair()
    }
    stopBootstrapIfReady()
    return
  }

  if (topic.includes('/thresholds/change')) {
    if (mergeThresholdGroupPayload(topic, payload)) {
      lastThresholdsAt.value = now
      if (hasCompleteThresholdsData()) {
        stopThresholdRepair()
      }
      stopBootstrapIfReady()
    }
    return
  }

  if (topic.endsWith('/ack')) {
    lastAckAt.value = now
    applyAck(payload)
    return
  }

  if (topic.endsWith('/action')) {
    applyActionAsAckFallback(payload)
    upsertRealtimeAction(payload, 'action')
    return
  }
}

/* 将首次拿到的阈值同步到输入草稿，但不覆盖用户已修改未提交的值。 */
function syncThresholdDrafts() {
  const map = getThresholdMapFromState()

  THRESHOLD_ITEMS.forEach((item) => {
    const key = String(item.id)
    if (thresholdDrafts[key] !== undefined) {
      return
    }

    if (Object.prototype.hasOwnProperty.call(map, key)) {
      thresholdDrafts[key] = map[key]
    }
  })
}

/* 检查阈值是否已全部就绪。 */
function hasCompleteThresholdsData() {
  const map = getThresholdMapFromState()
  return THRESHOLD_ITEMS.every((item) => Object.prototype.hasOwnProperty.call(map, String(item.id)))
}

/* 启动补查询阶段要求 sensors/status/thresholds 三块数据都准备好。 */
function hasBootstrapDataReady() {
  return Boolean(sensorsData.value && statusData.value && hasCompleteThresholdsData())
}

/* 一旦基础数据齐全，就停止启动补查询，避免无意义 query。 */
function stopBootstrapIfReady() {
  if (hasBootstrapDataReady()) {
    stopBootstrapQuery()
  }
}

/* 任意输入转数字，失败统一返回 null。 */
function toNumberOrNull(v) {
  const n = Number(v)
  return Number.isFinite(n) ? n : null
}

/*
 * 将 STM32 原始阈值值转换为前端显示/编辑值。
 * 当前阈值在前后端保持同一量纲，直接原样返回。
 */
function mapThresholdRawToUi(item, value) {
  return value
}

/*
 * 将前端编辑值转换回 STM32 期望的原始值。
 * 当前阈值在前后端保持同一量纲，直接原样返回。
 */
function mapThresholdUiToRaw(item, value) {
  return value
}

/* 根据阈值类型将值标准化为 int / 1 位小数 float。 */
function normalizeThresholdValue(item, rawValue) {
  const n = toNumberOrNull(mapThresholdRawToUi(item, rawValue))
  if (n === null) {
    return null
  }

  if (item.type === 'int') {
    return Math.trunc(n)
  }

  return Number(n.toFixed(1))
}

/* 从对象型 payload 中抽取阈值映射。 */
function extractThresholdMapFromObject(raw) {
  if (!raw || typeof raw !== 'object' || Array.isArray(raw)) {
    return null
  }

  const out = {}
  let count = 0

  THRESHOLD_ITEMS.forEach((item) => {
    const idKey = String(item.id)
    let value

    if (Object.prototype.hasOwnProperty.call(raw, idKey)) {
      value = raw[idKey]
    } else if (Object.prototype.hasOwnProperty.call(raw, item.key)) {
      value = raw[item.key]
    } else {
      return
    }

    const normalized = normalizeThresholdValue(item, value)
    if (normalized === null) {
      return
    }

    out[idKey] = normalized
    count += 1
  })

  return count > 0 ? out : null
}

/* 从数组型 payload 中抽取阈值映射，兼容 [v1,v2...] 和 [{id,value}] 两种格式。 */
function extractThresholdMapFromArray(raw) {
  if (!Array.isArray(raw)) {
    return null
  }

  const out = {}
  let count = 0

  raw.forEach((entry, index) => {
    let item = thresholdById.get(String(index))
    let value = entry

    if (entry && typeof entry === 'object' && !Array.isArray(entry)) {
      const entryId = entry.id ?? entry.key
      if (entryId !== undefined && entryId !== null) {
        if (typeof entryId === 'string' && thresholdByKey.has(entryId)) {
          item = thresholdByKey.get(entryId)
        } else {
          item = thresholdById.get(String(entryId))
        }
      }

      if (Object.prototype.hasOwnProperty.call(entry, 'value')) {
        value = entry.value
      } else if (Object.prototype.hasOwnProperty.call(entry, 'v')) {
        value = entry.v
      }
    }

    if (!item) {
      return
    }

    const normalized = normalizeThresholdValue(item, value)
    if (normalized === null) {
      return
    }

    out[String(item.id)] = normalized
    count += 1
  })

  return count > 0 ? out : null
}

/* 从各种可能的 payload 结构中尽量解析出统一阈值映射。 */
function extractThresholdMapFromPayload(payload) {
  if (!payload || typeof payload !== 'object') {
    return null
  }

  const candidates = [
    payload?.v,
    payload?.values,
    payload?.thresholds,
    payload?.threshold,
    payload?.data?.v,
    payload?.data,
    payload
  ]

  for (let i = 0; i < candidates.length; i += 1) {
    const candidate = candidates[i]
    const normalized = Array.isArray(candidate)
      ? extractThresholdMapFromArray(candidate)
      : extractThresholdMapFromObject(candidate)
    if (normalized) {
      return normalized
    }
  }

  return null
}

/* 把部分阈值合并到现有 thresholdsData 中，并同步草稿。 */
function mergeNormalizedThresholdMap(partialMap, payload = null) {
  if (!partialMap || typeof partialMap !== 'object') {
    return false
  }

  const current = { ...getThresholdMapFromState() }

  let count = 0
  Object.keys(partialMap).forEach((idKey) => {
    if (!thresholdById.has(String(idKey))) {
      return
    }
    current[String(idKey)] = partialMap[idKey]
    count += 1
  })

  if (count <= 0) {
    return false
  }

  thresholdsData.value = {
    ...(thresholdsData.value ?? {}),
    id: payload?.id ?? thresholdsData.value?.id ?? gatewayId.value,
    seq: payload?.seq ?? thresholdsData.value?.seq ?? 0,
    v: current
  }
  syncThresholdDrafts()
  return true
}

/* 从 /thresholds/changeX topic 中解析出分组编号。 */
function parseThresholdGroupFromTopic(topic) {
  if (typeof topic !== 'string') {
    return 0
  }
  const matched = /\/thresholds\/change(\d+)$/.exec(topic.trim())
  if (!matched) {
    return 0
  }
  const group = Number(matched[1])
  return Number.isFinite(group) ? group : 0
}

/* 根据 change1~change6 对应关系，返回该分组涉及的阈值 id 集合。 */
function getThresholdIdSetByGroup(group) {
  if (group >= 1 && group <= 4) {
    const base = (group - 1) * 4
    return new Set([String(base), String(base + 1), String(base + 2), String(base + 3)])
  }
  if (group === 5) {
    return new Set(['17', '18', '19', '20', '21', '22', '23', '24', '25', '26'])
  }
  if (group === 6) {
    return new Set(['33', '34', '35', '36', '37', '38', '39', '40', '41', '42'])
  }
  return new Set()
}

/* 将 MCU 发来的 change 分组阈值回包合并进全量阈值状态。 */
function mergeThresholdGroupPayload(topic, payload) {
  const payloadGroup = Number(payload?.group)
  const group = (Number.isFinite(payloadGroup) && payloadGroup > 0)
    ? payloadGroup
    : parseThresholdGroupFromTopic(topic)
  const partialMap = {}

  if (group >= 1 && group <= 4) {
    const arr = Array.isArray(payload?.v) ? payload.v : null
    if (arr && arr.length >= 4) {
      const base = (group - 1) * 4
      for (let i = 0; i < 4; i += 1) {
        const item = thresholdById.get(String(base + i))
        if (!item) {
          return false
        }
        const normalized = normalizeThresholdValue(item, arr[i])
        if (normalized === null) {
          return false
        }
        partialMap[String(base + i)] = normalized
      }
    }
  } else if (group === 5 || group === 6) {
    const low = Array.isArray(payload?.low) ? payload.low : null
    const high = Array.isArray(payload?.high) ? payload.high : null
    if (low && high && low.length >= 5 && high.length >= 5) {
      const idBase = group === 5 ? 17 : 33
      for (let i = 0; i < 5; i += 1) {
        const itemLow = thresholdById.get(String(idBase + i * 2))
        const itemHigh = thresholdById.get(String(idBase + i * 2 + 1))
        if (!itemLow || !itemHigh) {
          return false
        }
        const lo = normalizeThresholdValue(itemLow, low[i])
        const hi = normalizeThresholdValue(itemHigh, high[i])
        if (lo === null || hi === null) {
          return false
        }
        partialMap[String(idBase + i * 2)] = lo
        partialMap[String(idBase + i * 2 + 1)] = hi
      }
    }
  } else {
    return false
  }

  if (Object.keys(partialMap).length <= 0) {
    const normalized = extractThresholdMapFromPayload(payload)
    if (!normalized) {
      return false
    }
    const allowSet = getThresholdIdSetByGroup(group)
    if (allowSet.size <= 0) {
      return false
    }
    Object.keys(normalized).forEach((idKey) => {
      if (allowSet.has(String(idKey))) {
        partialMap[String(idKey)] = normalized[idKey]
      }
    })
    if (Object.keys(partialMap).length <= 0) {
      return false
    }
  }

  return mergeNormalizedThresholdMap(partialMap, payload)
}

/* 当 threshold ACK 已明确返回新值时，可提前把界面状态同步过去。 */
function applyThresholdAckAsFallback(ackPayload) {
  if (Number(ackPayload?.ok) !== 1) {
    return
  }

  const idKey = String(ackPayload?.id ?? '')
  const item = thresholdById.get(idKey)
  if (!item) {
    return
  }

  const normalized = normalizeThresholdValue(item, ackPayload?.value)
  if (normalized === null) {
    return
  }

  mergeNormalizedThresholdMap({ [idKey]: normalized }, ackPayload)
  lastThresholdsAt.value = new Date().toISOString()
}

/* 将所有阈值草稿恢复为设备当前值。 */
function resetAllThresholdDrafts() {
  if (!ensureAdminPermission()) {
    return
  }

  const map = getThresholdMapFromState()

  THRESHOLD_ITEMS.forEach((item) => {
    const key = String(item.id)
    if (Object.prototype.hasOwnProperty.call(map, key)) {
      thresholdDrafts[key] = map[key]
    }
  })
}

/* 生成请求唯一 req_id，用于命令日志和 ACK 精确匹配。 */
function nextReqId(cmd) {
  requestSeq = (requestSeq + 1) & 0xffff
  return `${cmd}_${Date.now().toString(36)}_${requestSeq.toString(36)}`
}

/* 清理某条命令日志的超时定时器。 */
function clearLogTimer(entry) {
  if (entry?.timeoutId) {
    clearTimeout(entry.timeoutId)
    entry.timeoutId = null
  }
}

/* 按 req_id 找待确认命令。 */
function findPendingByReq(reqId) {
  return commandLogs.value.find((entry) => entry.status === 'pending' && entry.reqId === reqId) ?? null
}

/* 按业务 matchKey 找最早一条待确认命令，便于“新命令覆盖旧命令”。 */
function findPendingByMatchKeyOldest(matchKey) {
  if (!matchKey) {
    return null
  }
  for (let i = commandLogs.value.length - 1; i >= 0; i -= 1) {
    const entry = commandLogs.value[i]
    if (entry.status === 'pending' && entry.matchKey === matchKey) {
      return entry
    }
  }
  return null
}

/* 按命令类型找最早一条待确认命令。 */
function findPendingByCmdOldest(cmd) {
  for (let i = commandLogs.value.length - 1; i >= 0; i -= 1) {
    const entry = commandLogs.value[i]
    if (entry.status === 'pending' && entry.cmd === cmd) {
      return entry
    }
  }
  return null
}

/*
 * 为命令构造“业务等价键”。
 * 例如同一个设备的两次手动命令，topic/payload 变化不大时可视为同类命令。
 */
function buildCommandMatchKey(cmd, topic, payload) {
  const p = (payload && typeof payload === 'object' && !Array.isArray(payload)) ? payload : {}

  if (cmd === 'manual') {
    const devFromPayload = typeof p.dev === 'string' ? String(p.dev).toLowerCase() : ''
    const devFromTopic = parseManualDeviceFromTopic(topic)
    const dev = devFromPayload || devFromTopic
    return dev ? `manual:${dev}` : `manual:${topic}`
  }
  if (cmd === 'threshold') {
    if (p.id !== undefined && p.id !== null) {
      return `threshold:id:${p.id}`
    }
    return 'threshold'
  }
  if (cmd === 'relay') {
    if (p.ch !== undefined && p.ch !== null) {
      return `relay:ch:${p.ch}`
    }
    return 'relay'
  }
  if (cmd === 'fan') {
    return 'fan'
  }

  return `${cmd}:${topic}`
}

/* 为 ACK 构造业务匹配键，在 req_id 缺失时仍有机会匹配到命令。 */
function buildAckMatchKey(ackPayload) {
  const cmd = typeof ackPayload?.cmd === 'string' ? ackPayload.cmd : ''
  if (!cmd) {
    return ''
  }

  if (cmd === 'manual') {
    const dev = ackPayload?.dev ?? ackPayload?.device
    if (dev) {
      return `manual:${String(dev).toLowerCase()}`
    }
    return 'manual'
  }
  if (cmd === 'threshold') {
    if (ackPayload?.id !== undefined && ackPayload?.id !== null) {
      return `threshold:id:${ackPayload.id}`
    }
    return 'threshold'
  }
  if (cmd === 'relay') {
    if (ackPayload?.ch !== undefined && ackPayload?.ch !== null) {
      return `relay:ch:${ackPayload.ch}`
    }
    return 'relay'
  }
  if (cmd === 'fan') {
    return 'fan'
  }

  return ''
}

/*
 * 通用 MQTT 发送入口：
 * - 自动附加 req_id。
 * - 记录命令日志。
 * - 设置 ACK 超时。
 * - 必要时覆盖旧的同类待确认命令。
 */
function publishCommand(cmd, topic, payload, options = {}) {
  const trackAck = options.trackAck !== false
  const supersedeSameCmd = options.supersedeSameCmd !== false
  const qos = Number.isInteger(options.qos) ? options.qos : CMD_QOS
  const timeoutMsRaw = Number(options.timeoutMs)
  const ackTimeoutMs = Number.isFinite(timeoutMsRaw)
    ? Math.max(1000, Math.trunc(timeoutMsRaw))
    : ACK_TIMEOUT_MS
  const matchKey = buildCommandMatchKey(cmd, topic, payload)

  if (!mqttClient.value || !isConnected.value) {
    lastError.value = 'MQTT 未连接'
    return
  }

  let reqId = null
  let payloadObj = payload

  if (trackAck && payload && typeof payload === 'object' && !Array.isArray(payload)) {
    const existingReqId = (typeof payload.req_id === 'string' && payload.req_id)
      ? payload.req_id
      : ''
    reqId = existingReqId || nextReqId(cmd)
    payloadObj = { ...payload, req_id: reqId }
  }

  let payloadText
  try {
    payloadText = JSON.stringify(payloadObj)
  } catch {
    lastError.value = '命令 JSON 序列化失败'
    return
  }

  let logItem = null

  if (trackAck) {
    if (supersedeSameCmd) {
      const oldPending = findPendingByMatchKeyOldest(matchKey)
      if (oldPending) {
        clearLogTimer(oldPending)
        oldPending.status = 'failed'
        oldPending.reason = '被同类新命令覆盖'
      }
    }

    logItem = {
      localSeq: ++commandSeq,
      createdAt: new Date().toISOString(),
      cmd,
      topic,
      payloadText,
      reqId,
      matchKey,
      status: 'pending',
      reason: '',
      ack: null,
      timeoutId: null
    }

    addLog(logItem)

    logItem.timeoutId = setTimeout(() => {
      if (logItem.status === 'pending') {
        logItem.status = 'timeout'
        logItem.reason = 'ACK 超时'
      }
    }, ackTimeoutMs)
  }

  mqttClient.value.publish(topic, payloadText, { qos }, (err) => {
    if (!err) {
      return
    }

    if (logItem) {
      clearLogTimer(logItem)
      logItem.status = 'failed'
      logItem.reason = `发送失败: ${err.message}`
      return
    }

    lastError.value = `发送失败: ${err.message}`
  })
}

/* 将命令日志插到顶部，并裁剪总长度。 */
function addLog(item) {
  commandLogs.value.unshift(item)

  if (commandLogs.value.length > MAX_LOGS) {
    const dropped = commandLogs.value.splice(MAX_LOGS)
    dropped.forEach((entry) => {
      clearLogTimer(entry)
    })
  }
}

/*
 * 处理 tele/ack：
 * 优先按 req_id 精确匹配，缺失时再回退到业务匹配键。
 */
function applyAck(ackPayload) {
  const ackCmd = typeof ackPayload?.cmd === 'string' ? ackPayload.cmd : null
  const ackReqId = typeof ackPayload?.req_id === 'string' ? ackPayload.req_id : null
  const ackMatchKey = buildAckMatchKey(ackPayload)
  const strictReqOnly = (ackCmd === 'manual' || ackCmd === 'threshold')

  if (ackCmd === 'threshold') {
    applyThresholdAckAsFallback(ackPayload)
  }

  let matched = null
  if (ackReqId) {
    matched = findPendingByReq(ackReqId)
  }
  if (!matched && ackMatchKey && !strictReqOnly) {
    matched = findPendingByMatchKeyOldest(ackMatchKey)
  }
  if (!matched && ackCmd && !strictReqOnly) {
    matched = findPendingByCmdOldest(ackCmd)
  }

  if (matched) {
    clearLogTimer(matched)
    matched.status = ackPayload?.ok === 1 ? 'success' : 'failed'
    matched.reason = ackPayload?.ok === 1 ? '' : mapAckReason(ackPayload?.reason)
    matched.ack = ackPayload
    if (matched.cmd === 'manual') {
      if (!hasPendingManualCommands()) {
        stopManualStatusQueryTimer()
      }
    }
    return
  }

  if (ackReqId) {
    const sameReq = commandLogs.value.find((entry) =>
      entry?.reqId === ackReqId &&
      (ackCmd ? entry?.cmd === ackCmd : true)
    )
    if (sameReq) {
      clearLogTimer(sameReq)
      if (sameReq.status === 'pending' || sameReq.status === 'timeout' || sameReq.status === 'failed') {
        sameReq.status = ackPayload?.ok === 1 ? 'success' : 'failed'
        sameReq.reason = ackPayload?.ok === 1 ? '' : mapAckReason(ackPayload?.reason)
      }
      sameReq.ack = ackPayload
      return
    }
  }

  addLog({
    localSeq: ++commandSeq,
    createdAt: new Date().toISOString(),
    cmd: ackCmd ?? 'unknown',
    topic: `ws/${gatewayId.value}/tele/ack`,
    payloadText: '-',
    reqId: ackReqId ?? '',
    matchKey: ackMatchKey || '',
    status: ackPayload?.ok === 1 ? 'success' : 'failed',
    reason: ackPayload?.ok === 1 ? '收到未匹配 ACK' : mapAckReason(ackPayload?.reason),
    ack: ackPayload,
    timeoutId: null
  })
}

/* 从 manual topic 路径中解析出设备名。 */
function parseManualDeviceFromTopic(topic) {
  if (typeof topic !== 'string') {
    return ''
  }
  const m = /\/manual\/([^/?#]+)$/.exec(topic.trim())
  return m ? String(m[1] || '').toLowerCase() : ''
}

/* 判断设备文本是否可视为“未知值”。 */
function isUnknownDeviceText(v) {
  const s = String(v || '').trim().toLowerCase()
  return !s || s === 'unknown' || s === '-'
}

/* 判断动作文本是否可视为“未知值”。 */
function isUnknownActionText(v) {
  const s = String(v || '').trim().toLowerCase()
  return !s || s === 'unknown' || s === 'pwm_unknown' || s === '-'
}

/* 两个候选文本里优先保留信息量更高的那个。 */
function pickBetterText(currentValue, nextValue, isUnknownFn) {
  if (isUnknownFn(nextValue)) {
    return currentValue
  }
  if (isUnknownFn(currentValue)) {
    return nextValue
  }
  return nextValue
}

/* 从动作记录的不同字段兼容性地提取 req_id。 */
function extractActionReqId(rowLike) {
  const direct = typeof rowLike?.reqId === 'string' ? rowLike.reqId : ''
  if (direct) {
    return direct
  }
  return typeof rowLike?.detail?.req_id === 'string' ? rowLike.detail.req_id : ''
}

/* 从 action payload 中提取设备名。 */
function extractActionDevice(payload) {
  const raw = typeof payload?.dev === 'string'
    ? payload.dev
    : (typeof payload?.device === 'string' ? payload.device : '')
  if (raw) {
    return String(raw).toLowerCase()
  }
  return parseManualDeviceFromTopic(payload?.topic)
}

/* 将 ack/status/action 各种来源的动作描述统一折叠成标准结构。 */
function summarizeRealtimeAction(payload, source) {
  const cmd = typeof payload?.cmd === 'string' ? payload.cmd : source

  if (cmd === 'relay') {
    const ch = Number(payload?.ch)
    const on = Number(payload?.on)
    return {
      cmd,
      device: Number.isFinite(ch) ? `relay_ch${Math.trunc(ch)}` : 'relay',
      mode: 'manual',
      action: on === 1 ? 'on' : on === 0 ? 'off' : 'unknown'
    }
  }

  if (cmd === 'fan') {
    const pwm = Number(payload?.pwm)
    return {
      cmd,
      device: 'fan',
      mode: 'manual',
      action: Number.isFinite(pwm) ? `pwm_${Math.trunc(pwm)}` : 'pwm_unknown'
    }
  }

  if (cmd === 'manual') {
    const dev = extractActionDevice(payload) || 'unknown'
    const mode = typeof payload?.mode === 'string' ? payload.mode : 'manual'
    const value = Number(payload?.value)
    let action = ''
    if (Number.isFinite(value)) {
      action = `value_${Math.trunc(value)}`
    } else if (mode === 'auto') {
      action = 'auto'
    } else if (typeof payload?.action === 'string' && payload.action) {
      action = payload.action
    } else {
      action = mode || 'unknown'
    }
    return {
      cmd,
      device: dev,
      mode,
      action
    }
  }

  const fallbackDevice = extractActionDevice(payload)
  const on = Number(payload?.on)
  const pwm = Number(payload?.pwm)
  let action = typeof payload?.action === 'string' ? payload.action : ''
  if (!action) {
    if (on === 1 || on === 0) {
      action = on === 1 ? 'on' : 'off'
    } else if (Number.isFinite(pwm)) {
      action = `pwm_${Math.trunc(pwm)}`
    }
  }

  return {
    cmd,
    device: fallbackDevice || 'unknown',
    mode: typeof payload?.mode === 'string' ? payload.mode : '',
    action
  }
}

/* 安全 JSON.parse，失败返回 null。 */
function parseJsonSafely(text) {
  if (typeof text !== 'string' || !text) {
    return null
  }
  try {
    return JSON.parse(text)
  } catch {
    return null
  }
}

/* 判断某条待确认手动命令是否与某条 action 回传相匹配。 */
function manualPendingMatchesAction(logItem, actionSummary) {
  const payloadObj = parseJsonSafely(logItem?.payloadText)
  if (!payloadObj || typeof payloadObj !== 'object') {
    return false
  }

  const devFromTopic = parseManualDeviceFromTopic(logItem?.topic)
  const devFromPayload = (typeof payloadObj.dev === 'string' ? payloadObj.dev : '')
  const pendingDev = String(devFromPayload || devFromTopic || '').toLowerCase()
  const actionDev = String(actionSummary?.device || '').toLowerCase()
  if (!pendingDev || !actionDev || pendingDev !== actionDev) {
    return false
  }

  const pendingMode = String(payloadObj.mode || 'manual').toLowerCase()
  const actionMode = String(actionSummary?.mode || '').toLowerCase()
  const actionText = String(actionSummary?.action || '').toLowerCase()

  if (pendingMode === 'auto') {
    if (actionMode && actionMode !== 'auto') {
      return false
    }
    return true
  }

  const valueNum = Number(payloadObj.value)
  if (pendingDev === 'fan') {
    if (!Number.isFinite(valueNum)) {
      return false
    }
    const gear = Math.max(1, Math.min(5, Math.trunc(valueNum)))
    return actionText === `gear_${gear}` || actionText === `value_${gear}` || actionText === `pwm_${gear}`
  }

  if (!Number.isFinite(valueNum)) {
    return actionText === 'on' || actionText === 'off' || actionText === 'value_1' || actionText === 'value_0'
  }

  if (Math.trunc(valueNum) > 0) {
    return actionText === 'on' || actionText === 'value_1'
  }
  return actionText === 'off' || actionText === 'value_0'
}

/* 判断某条待确认手动命令是否已经在 status 中体现出来。 */
function manualPendingMatchesStatus(logItem, manualStatus) {
  const payloadObj = parseJsonSafely(logItem?.payloadText)
  if (!payloadObj || typeof payloadObj !== 'object' || !manualStatus || typeof manualStatus !== 'object') {
    return false
  }

  const devFromTopic = parseManualDeviceFromTopic(logItem?.topic)
  const devFromPayload = typeof payloadObj.dev === 'string' ? payloadObj.dev : ''
  const dev = String(devFromPayload || devFromTopic || '').toLowerCase()
  if (!dev) {
    return false
  }

  const mode = String(payloadObj.mode || 'manual').toLowerCase()
  if (dev === 'fan') {
    const fanMode = Number(manualStatus.fan_mode)
    const fanGear = Number(manualStatus.fan_gear)

    if (mode === 'auto') {
      return fanMode === 0
    }

    const valueNum = Number(payloadObj.value)
    if (!Number.isFinite(valueNum)) {
      return fanMode === 1
    }
    const expectedGear = Math.max(1, Math.min(5, Math.trunc(valueNum)))
    return fanMode === 1 && fanGear === expectedGear
  }

  const code = Number(manualStatus[dev])
  if (!Number.isFinite(code)) {
    return false
  }

  if (mode === 'auto') {
    return code === 2 || code === 3
  }

  const valueNum = Number(payloadObj.value)
  if (Number.isFinite(valueNum) && Math.trunc(valueNum) > 0) {
    return code === 1 || code === 5
  }
  return code === 0 || code === 4
}

/* 如果手动命令的 ACK 丢失，尝试使用 status 回传把日志补记为成功。 */
function applyManualStatusAsAckFallback(statusPayload) {
  const manual = statusPayload?.manual
  if (!manual || typeof manual !== 'object') {
    return
  }

  let hit = false
  commandLogs.value.forEach((entry) => {
    if (entry?.status !== 'pending' || entry?.cmd !== 'manual') {
      return
    }
    if (!manualPendingMatchesStatus(entry, manual)) {
      return
    }

    clearLogTimer(entry)
    entry.status = 'success'
    entry.reason = ''
    entry.ack = {
      ok: 1,
      cmd: 'manual',
      source: 'status_fallback',
      manual
    }
    hit = true
  })

  if (hit && !hasPendingManualCommands()) {
    stopManualStatusQueryTimer()
  }
}

/* 如果 ACK 缺失但 action 已经上报，也可反向确认命令成功。 */
function applyActionAsAckFallback(payload) {
  if (!payload || typeof payload !== 'object') {
    return
  }

  const actionSummary = summarizeRealtimeAction(payload, 'action')
  const actionDev = String(actionSummary?.device || '').toLowerCase()
  if (!actionDev || actionDev === 'unknown') {
    return
  }

  const exactManualKey = `manual:${actionDev}`
  let matched = findPendingByMatchKeyOldest(exactManualKey)
  if (matched && !manualPendingMatchesAction(matched, actionSummary)) {
    matched = null
  }

  if (!matched) {
    matched = commandLogs.value.find((item) =>
      item?.status === 'pending' &&
      item?.cmd === 'manual' &&
      manualPendingMatchesAction(item, actionSummary)
    )
  }

  if (!matched) {
    return
  }

  clearLogTimer(matched)
  matched.status = 'success'
  matched.reason = ''
  matched.ack = {
    ok: 1,
    cmd: 'manual',
    source: 'action_fallback',
    device: actionDev,
    mode: actionSummary?.mode || '',
    action: actionSummary?.action || ''
  }
}

/* 合并两条疑似重复的动作记录，保留信息量更完整的一条。 */
function mergeActionRow(currentRow, nextRow) {
  const currentDetail = (currentRow?.detail && typeof currentRow.detail === 'object') ? currentRow.detail : {}
  const nextDetail = (nextRow?.detail && typeof nextRow.detail === 'object') ? nextRow.detail : {}
  const currentDetailSize = Object.keys(currentDetail).length
  const nextDetailSize = Object.keys(nextDetail).length

  const merged = {
    ...currentRow,
    ...nextRow,
    ts: Math.max(Number(currentRow?.ts || 0), Number(nextRow?.ts || 0)),
    seq: Number.isFinite(Number(nextRow?.seq)) ? Number(nextRow.seq) : currentRow?.seq,
    reqId: nextRow?.reqId || currentRow?.reqId || '',
    device: pickBetterText(currentRow?.device, nextRow?.device, isUnknownDeviceText),
    mode: pickBetterText(currentRow?.mode, nextRow?.mode, (v) => !String(v || '').trim()),
    action: pickBetterText(currentRow?.action, nextRow?.action, isUnknownActionText),
    ok: Number.isFinite(Number(nextRow?.ok)) ? Number(nextRow.ok) : currentRow?.ok,
    reason: nextRow?.reason || currentRow?.reason || null
  }

  if (nextDetailSize >= currentDetailSize) {
    merged.detail = nextDetail
  } else {
    merged.detail = currentDetail
  }
  return merged
}

/*
 * 将历史动作列表标准化、去重、压缩：
 * 既保留重要信息，又抑制短时间重复上报造成的噪声。
 */
function normalizeActionHistoryRows(rows) {
  const srcRows = Array.isArray(rows) ? rows : []
  const out = []
  const keyToIndex = new Map()

  srcRows.forEach((item) => {
    const row = {
      ...item,
      ts: Number(item?.ts) || Date.now(),
      seq: Number.isFinite(Number(item?.seq)) ? Math.trunc(Number(item.seq)) : null,
      reqId: extractActionReqId(item),
      cmd: typeof item?.cmd === 'string' ? item.cmd : '',
      device: typeof item?.device === 'string' ? item.device : (typeof item?.detail?.dev === 'string' ? item.detail.dev : 'unknown'),
      mode: typeof item?.mode === 'string' ? item.mode : '',
      action: typeof item?.action === 'string' ? item.action : '',
      source: typeof item?.source === 'string' ? item.source : '',
      detail: (item?.detail && typeof item.detail === 'object') ? item.detail : {}
    }

    const seq = Number(row.seq)
    let dedupKey = ''
    if (row.reqId) {
      dedupKey = `req:${row.reqId}`
    } else if (Number.isFinite(seq)) {
      dedupKey = `seq:${Math.trunc(seq)}:${String(row.cmd || '')}`
    } else if (!isUnknownDeviceText(row.device) && !isUnknownActionText(row.action) && row.ts > 0) {
      dedupKey = `fuzzy:${String(row.cmd || '')}:${String(row.device || '').toLowerCase()}:${String(row.action || '').toLowerCase()}:${Math.trunc(row.ts / 1000)}`
    }

    if (!dedupKey) {
      if (row.source === 'ack' && isUnknownDeviceText(row.device) && isUnknownActionText(row.action)) {
        return
      }
      out.push(row)
      return
    }

    const existedIndex = keyToIndex.get(dedupKey)
    if (existedIndex === undefined) {
      keyToIndex.set(dedupKey, out.length)
      out.push(row)
      return
    }

    out[existedIndex] = mergeActionRow(out[existedIndex], row)
  })

  out.sort((a, b) => Number(b?.ts || 0) - Number(a?.ts || 0))

  /* Suppress noisy repeated action rows (same device/mode/action in a very short window). */
  const compacted = []
  const lastTsBySig = new Map()
  out.forEach((row) => {
    const device = String(row?.device || '').toLowerCase()
    const action = String(row?.action || '').toLowerCase()
    const mode = String(row?.mode || '').toLowerCase()
    const cmd = String(row?.cmd || '').toLowerCase()
    const ts = Number(row?.ts || 0)
    const isDeviceAction = (cmd === 'device') || (row?.source === 'action')
    const hasKnown = !isUnknownDeviceText(device) && !isUnknownActionText(action) && (ts > 0)

    if (isDeviceAction && hasKnown) {
      const sig = `${cmd}|${device}|${mode}|${action}`
      const prevTs = Number(lastTsBySig.get(sig) || 0)
      if ((prevTs > 0) && (Math.abs(prevTs - ts) <= ACTION_DUP_SUPPRESS_MS)) {
        return
      }
      lastTsBySig.set(sig, ts)
    }

    compacted.push(row)
  })

  if (compacted.length > 240) {
    compacted.splice(240)
  }
  return compacted
}

/* 将实时 action/ack 事件塞入前端动作记录缓存。 */
function upsertRealtimeAction(payload, source) {
  if (!payload || typeof payload !== 'object') {
    return
  }

  const summary = summarizeRealtimeAction(payload, source)
  const seq = Number(payload?.seq)
  const reqId = typeof payload?.req_id === 'string' ? payload.req_id : ''
  const tsRaw = Number(payload?.ts)
  const ts = Number.isFinite(tsRaw) && tsRaw > 0
    ? (tsRaw < 1e12 ? Math.trunc(tsRaw * 1000) : Math.trunc(tsRaw))
    : Date.now()
  const ok = Number(payload?.ok)

  const row = {
    ts,
    seq: Number.isFinite(seq) ? Math.trunc(seq) : null,
    reqId,
    cmd: summary.cmd,
    device: summary.device,
    mode: summary.mode,
    action: summary.action,
    ok: Number.isFinite(ok) ? Math.trunc(ok) : null,
    reason: typeof payload?.reason === 'string' ? payload.reason : null,
    source,
    detail: payload
  }

  if (source === 'ack' && !reqId && isUnknownDeviceText(row.device) && isUnknownActionText(row.action)) {
    return
  }

  let index = -1
  if (reqId) {
    index = actionHistory.value.findIndex((item) => extractActionReqId(item) === reqId)
  }
  if ((index < 0) && Number.isFinite(row.seq)) {
    index = actionHistory.value.findIndex((item) =>
      Number(item?.seq) === row.seq &&
      String(item?.cmd || '') === String(row.cmd || '')
    )
  }
  if ((index < 0) && !isUnknownDeviceText(row.device) && !isUnknownActionText(row.action)) {
    index = actionHistory.value.findIndex((item) =>
      String(item?.cmd || '') === String(row.cmd || '') &&
      String(item?.device || '').toLowerCase() === String(row.device || '').toLowerCase() &&
      String(item?.action || '').toLowerCase() === String(row.action || '').toLowerCase() &&
      Math.abs(Number(item?.ts || 0) - row.ts) <= 10000
    )
  }

  if (index >= 0) {
    actionHistory.value.splice(index, 1, mergeActionRow(actionHistory.value[index], row))
  } else {
    actionHistory.value.unshift(row)
  }

  actionHistory.value = normalizeActionHistoryRows(actionHistory.value)
}

/* ACK reason 码转展示文案。 */
function mapAckReason(reason) {
  if (!reason) {
    return '未知错误'
  }
  return ACK_REASON_TEXT[reason] ? `${reason} (${ACK_REASON_TEXT[reason]})` : reason
}

/* 统一发送 query 命令，并对同类 query 做最小发送间隔限制。 */
function sendQuery(what = 'all', trackAck = true, qos = CMD_QOS, minGapMs = QUERY_MIN_GAP_MS) {
  if (!mqttClient.value || !isConnected.value) {
    return false
  }

  const key = String(what || 'all')
  const now = Date.now()
  const gap = Number.isFinite(Number(minGapMs)) ? Math.max(0, Math.trunc(Number(minGapMs))) : 0
  const last = Number(queryLastSendAt.get(key) || 0)

  if (gap > 0 && (now - last) < gap) {
    return false
  }

  queryLastSendAt.set(key, now)
  publishCommand('query', `${cmdBaseTopic.value}/query`, { what: key }, { trackAck, supersedeSameCmd: false, qos })
  return true
}

/* 将任意输入安全收敛到整数区间。 */
function clampInt(value, minValue, maxValue, fallbackValue) {
  const raw = Number(value)
  let n = Number.isFinite(raw) ? Math.trunc(raw) : fallbackValue
  if (!Number.isFinite(n)) {
    n = fallbackValue
  }
  if (n < minValue) {
    n = minValue
  }
  if (n > maxValue) {
    n = maxValue
  }
  return n
}

/* 对手动控制表单做设备级校验和归一化。 */
function normalizeManualPayload(dev, form) {
  const safeSec = clampInt(form?.sec, 0, 86400, 0)
  let safeValue

  if (dev === 'fan') {
    safeValue = clampInt(form?.value, 1, 5, 1)
  } else {
    const n = Number(form?.value)
    safeValue = (Number.isFinite(n) && n > 0) ? 1 : 0
  }

  return {
    sec: safeSec,
    value: safeValue
  }
}

/*
 * 手动命令的发送策略：
 * - 先发一次带 ACK 跟踪的主命令。
 * - 若未确认，再延迟重发若干次。
 * - 并在合适时机补 query status 作为 ACK fallback。
 */
function publishManualFast(topic, payload) {
  const devFromTopic = parseManualDeviceFromTopic(topic)
  const dev = String(devFromTopic || payload?.dev || '').toLowerCase()
  if (!dev) {
    lastError.value = '无效设备'
    return
  }
  const genericTopic = `${cmdBaseTopic.value}/manual`
  const reqId = nextReqId('manual')
  const payloadWithDev = { ...payload, dev, req_id: reqId }

  /* Primary send: keep ACK tracking for command log. */
  publishCommand('manual', genericTopic, payloadWithDev, {
    trackAck: true,
    supersedeSameCmd: false,
    qos: CMD_QOS,
    timeoutMs: MANUAL_ACK_TIMEOUT_MS
  })

  const matchKey = buildCommandMatchKey('manual', genericTopic, payloadWithDev)
  const retryDelays = [MANUAL_RETRY_DELAY_MS, MANUAL_RETRY_DELAY_MS + 7000]
  retryDelays.forEach((delayMs) => setTimeout(() => {
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    const pending = findPendingByReq(reqId)
    if (!pending || pending.status !== 'pending') {
      return
    }
    publishCommand('manual', genericTopic, payloadWithDev, {
      trackAck: false,
      supersedeSameCmd: false,
      qos: CMD_QOS
    })
    scheduleManualStatusQuery(matchKey, MANUAL_STATUS_SYNC_DELAY_MS, reqId)
  }, Math.max(0, Math.trunc(delayMs))))
}

/* 手动控制发送按钮入口。 */
function sendManual(dev) {
  if (!ensureAdminPermission()) {
    return
  }
  if (!checkRateLimit()) {
    return
  }

  const form = manualForms[dev]
  if (!form) {
    lastError.value = '无效设备'
    return
  }

  const normalized = normalizeManualPayload(dev, form)
  form.sec = normalized.sec
  form.value = normalized.value

  publishManualFast(`${cmdBaseTopic.value}/manual/${dev}`, {
    mode: 'manual',
    sec: normalized.sec,
    value: normalized.value
  })
}

/* 将某个设备从手动模式恢复到自动模式。 */
function restoreAuto(dev) {
  if (!ensureAdminPermission()) {
    return
  }
  if (!checkRateLimit()) {
    return
  }

  if (!manualForms[dev]) {
    return
  }
  publishManualFast(`${cmdBaseTopic.value}/manual/${dev}`, { mode: 'auto' })
}

/* 单个阈值保存入口。 */
function sendThreshold(item, skipRateLimit = false) {
  if (!ensureAdminPermission()) {
    return
  }
  if (!skipRateLimit && !checkRateLimit()) {
    return
  }

  const key = String(item.id)
  const raw = thresholdDrafts[key]
  const value = Number(raw)

  if (!Number.isFinite(value)) {
    lastError.value = `${item.label} 输入值无效`
    return
  }

  const rawValue = mapThresholdUiToRaw(item, value)
  const payloadValue = (item.type === 'int')
    ? Math.trunc(rawValue)
    : Number(rawValue.toFixed(1))

  const reqId = nextReqId('threshold')
  const payloadWithReq = { id: item.id, value: payloadValue, req_id: reqId }

  publishCommand(
    'threshold',
    `${cmdBaseTopic.value}/threshold`,
    payloadWithReq,
    { timeoutMs: MANUAL_ACK_TIMEOUT_MS, supersedeSameCmd: false }
  )

  const retryDelays = [THRESHOLD_RETRY_DELAY_MS, THRESHOLD_RETRY_DELAY_MS + 9000]
  retryDelays.forEach((delayMs) => setTimeout(() => {
    if (!mqttClient.value || !isConnected.value) {
      return
    }
    const pending = findPendingByReq(reqId)
    if (!pending || pending.status !== 'pending') {
      return
    }
    publishCommand(
      'threshold',
      `${cmdBaseTopic.value}/threshold`,
      payloadWithReq,
      { trackAck: false, supersedeSameCmd: false, qos: CMD_QOS }
    )
  }, Math.max(0, Math.trunc(delayMs))))
}

/* 顺序保存所有已修改阈值，避免一次性并发写命令过多。 */
async function saveAllThresholdDrafts() {
  if (!ensureAdminPermission()) {
    return
  }

  if (thresholdSavingAll.value) {
    return
  }

  const changed = thresholdRows.value.filter((row) => row.changed)
  if (!changed.length) {
    return
  }

  if (!checkRateLimit()) {
    return
  }

  thresholdSavingAll.value = true
  try {
    for (let i = 0; i < changed.length; i += 1) {
      sendThreshold(changed[i], true)
      await new Promise((resolve) => setTimeout(resolve, CMD_RATE_LIMIT_MS))
    }
    lastCmdSentAt = Date.now()
  } finally {
    thresholdSavingAll.value = false
  }
}

/* 手动状态码格式化。 */
function manualStateText(code) {
  if (!Number.isFinite(code)) {
    return '-'
  }
  return MANUAL_STATE_TEXT[code] ?? String(code)
}

/* 数值 + 单位格式化，空值时返回 '-'。 */
function formatValue(value, unit) {
  if ((value === undefined) || (value === null)) {
    return '-'
  }
  return unit ? `${value}${unit}` : String(value)
}

/* ISO 时间格式化为本地可读文本。 */
function formatTime(iso) {
  if (!iso) {
    return '-'
  }
  return new Date(iso).toLocaleTimeString('zh-CN', { hour12: false })
}

/* 供日志区域展示格式化 JSON。 */
function prettyJson(value) {
  if (!value) {
    return '-'
  }
  return JSON.stringify(value, null, 2)
}

/* 毫秒时间戳转“日期 时间”短文本，用于图表横轴。 */
function formatDateTimeMs(ms) {
  const n = Number(ms)
  if (!Number.isFinite(n) || n <= 0) {
    return '-'
  }
  return new Date(n).toLocaleString('zh-CN', { hour12: false })
}

/* 毫秒时间戳转 datetime-local 输入框所需格式。 */
function toDatetimeLocalValue(ms) {
  const n = Number(ms)
  if (!Number.isFinite(n) || n <= 0) {
    return ''
  }
  const d = new Date(n)
  const y = d.getFullYear()
  const m = String(d.getMonth() + 1).padStart(2, '0')
  const day = String(d.getDate()).padStart(2, '0')
  const hh = String(d.getHours()).padStart(2, '0')
  const mm = String(d.getMinutes()).padStart(2, '0')
  return `${y}-${m}-${day}T${hh}:${mm}`
}

/* 从 datetime-local 文本解析时间戳。 */
function parseDatetimeLocalValue(value) {
  if (!value || typeof value !== 'string') {
    return NaN
  }
  const t = new Date(value).getTime()
  return Number.isFinite(t) ? t : NaN
}

/* 当历史查询模式为“最近 N 小时”时，同步生成自定义时间范围。 */
function syncCustomRangeFromHours() {
  const now = Date.now()
  const from = now - Number(historyHours.value || 6) * 3600 * 1000
  historyFromLocal.value = toDatetimeLocalValue(from)
  historyToLocal.value = toDatetimeLocalValue(now)
}

/* 动作记录查询的“最近 N 小时”同步。 */
function syncActionRangeFromHours() {
  const now = Date.now()
  const from = now - Number(actionHours.value || 6) * 3600 * 1000
  actionFromLocal.value = toDatetimeLocalValue(from)
  actionToLocal.value = toDatetimeLocalValue(now)
}

/* 构造传感器历史 API 查询参数。 */
function buildHistoryRangeQuery() {
  if (historyRangeMode.value === 'custom') {
    const from = parseDatetimeLocalValue(historyFromLocal.value)
    const to = parseDatetimeLocalValue(historyToLocal.value)
    if (!Number.isFinite(from) || !Number.isFinite(to)) {
      throw new Error('请填写有效的开始和结束时间')
    }
    if (from >= to) {
      throw new Error('结束时间必须晚于开始时间')
    }
    return { from: Math.trunc(from), to: Math.trunc(to) }
  }
  return { hours: historyHours.value }
}

/* 构造动作历史 API 查询参数。 */
function buildActionRangeQuery() {
  if (actionRangeMode.value === 'custom') {
    const from = parseDatetimeLocalValue(actionFromLocal.value)
    const to = parseDatetimeLocalValue(actionToLocal.value)
    if (!Number.isFinite(from) || !Number.isFinite(to)) {
      throw new Error('请填写有效的开始和结束时间')
    }
    if (from >= to) {
      throw new Error('结束时间必须晚于开始时间')
    }
    return { from: Math.trunc(from), to: Math.trunc(to) }
  }
  return { hours: actionHours.value }
}

/* 将动作结果记录进一步格式化成页面展示文本。 */
function formatActionResult(row) {
  if (row?.ok === 1) {
    return '成功'
  }
  if (row?.ok === 0) {
    return row?.reason ? `失败(${row.reason})` : '失败'
  }
  return '-'
}

/* 通用 GET /api/history 请求封装。 */
async function fetchHistory(pathname, params = {}) {
  const qs = new URLSearchParams()
  Object.entries(params).forEach(([k, v]) => {
    if (v === undefined || v === null || v === '') {
      return
    }
    qs.set(k, String(v))
  })

  const url = `/api${pathname}?${qs.toString()}`
  const resp = await fetch(url)
  if (!resp.ok) {
    throw new Error(`HTTP ${resp.status}`)
  }

  const body = await resp.json()
  if (body?.ok !== 1) {
    throw new Error('API返回失败')
  }

  return body
}

/* 通用 DELETE /api/history 请求封装。 */
async function deleteHistory(pathname, params = {}) {
  const qs = new URLSearchParams()
  Object.entries(params).forEach(([k, v]) => {
    if (v === undefined || v === null || v === '') {
      return
    }
    qs.set(k, String(v))
  })

  const url = `/api${pathname}?${qs.toString()}`
  const resp = await fetch(url, { method: 'DELETE' })
  if (!resp.ok) {
    throw new Error(`HTTP ${resp.status}`)
  }

  const body = await resp.json()
  if (body?.ok !== 1) {
    throw new Error('API返回失败')
  }
  return body
}

/* 根据 panelKey 找到对应的 ECharts 实例。 */
function getHistoryChart(panelKey) {
  if (panelKey === 'th') return historyChartTh
  if (panelKey === 'pm_light') return historyChartPmLight
  if (panelKey === 'gas_pwm') return historyChartGasPwm
  if (panelKey === 'human_dist_door') return historyChartHumanDistDoor
  return null
}

/* 夹取任意数值到指定区间。 */
function clampValue(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

/* 用户拖动图表缩放条时，回写当前缩放区间到状态。 */
function onHistoryDataZoom(panelKey) {
  const chart = getHistoryChart(panelKey)
  if (!chart) {
    return
  }
  const option = chart.getOption()
  const dz = Array.isArray(option?.dataZoom) ? option.dataZoom[0] : null
  const start = Number(dz?.start)
  const end = Number(dz?.end)
  if (Number.isFinite(start) && Number.isFinite(end)) {
    historyZoom[panelKey].start = clampValue(start, 0, 100)
    historyZoom[panelKey].end = clampValue(end, 0, 100)
  }
}

/*
 * 对历史传感器数据做“拼接 + 前值延续”处理：
 * - 把时间接近且字段互补的记录拼为一条。
 * - 对缺失字段延续上一条已知值，避免图表断裂过于严重。
 */
function buildCarriedHistoryData() {
  const toSeriesValue = (v) => {
    if (v === null || v === undefined || v === '') {
      return null
    }
    const n = Number(v)
    return Number.isFinite(n) ? n : null
  }

  const keys = ['temp', 'humi', 'pm25', 'gas', 'pwm', 'human', 'dist', 'light', 'door']
  const mergeWindowMs = Math.max(3000, Number(historyIntervalSec.value || 0) * 1000 || 10000)

  const normalized = (Array.isArray(sensorHistory.value) ? sensorHistory.value : [])
    .map((item) => {
      const row = { ts: Number(item?.ts) || 0 }
      keys.forEach((key) => {
        row[key] = toSeriesValue(item?.[key])
      })
      return row
    })
    .filter((row) => row.ts > 0)
    .sort((a, b) => a.ts - b.ts)

  const stitched = []
  normalized.forEach((row) => {
    const prev = stitched.length ? stitched[stitched.length - 1] : null
    if (!prev) {
      stitched.push({ ...row })
      return
    }

    let overlapConflict = false
    let complementFound = false
    keys.forEach((key) => {
      if ((prev[key] !== null) && (row[key] !== null)) {
        overlapConflict = true
      } else if ((prev[key] === null) && (row[key] !== null)) {
        complementFound = true
      }
    })

    if (!overlapConflict && complementFound && Math.abs(row.ts - prev.ts) <= mergeWindowMs) {
      keys.forEach((key) => {
        if ((prev[key] === null) && (row[key] !== null)) {
          prev[key] = row[key]
        }
      })
      prev.ts = Math.max(prev.ts, row.ts)
      return
    }

    stitched.push({ ...row })
  })

  const carried = []
  const last = {}
  stitched.forEach((row) => {
    const next = { ts: row.ts }
    keys.forEach((key) => {
      if (row[key] === null || row[key] === undefined) {
        next[key] = Object.prototype.hasOwnProperty.call(last, key) ? last[key] : null
      } else {
        next[key] = row[key]
        last[key] = row[key]
      }
    })
    carried.push(next)
  })

  return carried
}

/* 渲染单个历史图表面板。 */
function renderHistoryPanel(panelKey, chart, xAxisData, series, yAxis) {
  if (!chart) {
    return
  }

  const rightAxisCount = yAxis.filter((axis) => axis.position === 'right').length
  const gridRight = 24 + rightAxisCount * 46

  chart.setOption({
    backgroundColor: 'transparent',
    animation: false,
    tooltip: { trigger: 'axis' },
    legend: {
      data: series.map((s) => s.name),
      bottom: 34,
      left: 'center',
      icon: 'circle',
      itemWidth: 14,
      itemHeight: 8,
      itemGap: 14,
      textStyle: { color: '#333333', fontSize: 12 }
    },
    grid: { top: 20, left: 52, right: gridRight, bottom: 104 },
    xAxis: {
      type: 'category',
      data: xAxisData,
      boundaryGap: false,
      axisLabel: { color: '#666666', hideOverlap: true, margin: 14 }
    },
    yAxis,
    dataZoom: [
      {
        type: 'inside',
        xAxisIndex: 0,
        filterMode: 'none',
        start: historyZoom[panelKey].start,
        end: historyZoom[panelKey].end,
        zoomOnMouseWheel: true,
        moveOnMouseMove: true,
        moveOnMouseWheel: true
      },
      {
        type: 'slider',
        xAxisIndex: 0,
        filterMode: 'none',
        start: historyZoom[panelKey].start,
        end: historyZoom[panelKey].end,
        height: 14,
        bottom: 8,
        borderColor: 'rgba(120,186,226,0.25)',
        backgroundColor: 'rgba(15,35,49,0.25)',
        fillerColor: 'rgba(53,210,255,0.20)',
        handleStyle: {
          color: '#35d2ff',
          borderColor: 'rgba(53,210,255,0.6)'
        },
        textStyle: {
          color: '#9dbdd5'
        }
      }
    ],
    series
  }, {
    notMerge: true
  })
}

/* 生成 ECharts 折线 series 配置。 */
function makeHistoryLineSeries(name, yAxisIndex, data, color, extra = {}) {
  return {
    name,
    type: 'line',
    yAxisIndex,
    data,
    smooth: true,
    showSymbol: false,
    color,
    lineStyle: { width: 2, color },
    itemStyle: { color },
    ...extra
  }
}

/* 将当前 sensorHistory 渲染成 4 张历史折线图。 */
function renderHistoryCharts() {
  const carried = buildCarriedHistoryData()
  const xAxis = carried.map((item) => formatDateTimeMs(item.ts))
  const temp = carried.map((item) => item.temp)
  const humi = carried.map((item) => item.humi)
  const pm25 = carried.map((item) => item.pm25)
  const gas = carried.map((item) => item.gas)
  const pwm = carried.map((item) => item.pwm)
  const dist = carried.map((item) => item.dist)
  const light = carried.map((item) => item.light)
  const human = carried.map((item) => ((item.human === null) ? null : (item.human >= 0.5 ? 1 : 0)))
  const door = carried.map((item) => ((item.door === null) ? null : (item.door >= 0.5 ? 1 : 0)))

  const isOn = (key) => historySeriesSelected[key] !== false

  const thSeries = []
  if (isOn('temp')) thSeries.push(makeHistoryLineSeries('温度', 0, temp, HISTORY_SERIES_COLORS.temp))
  if (isOn('humi')) thSeries.push(makeHistoryLineSeries('湿度', 0, humi, HISTORY_SERIES_COLORS.humi))

  const pmLightSeries = []
  if (isOn('pm25')) pmLightSeries.push(makeHistoryLineSeries('PM2.5', 0, pm25, HISTORY_SERIES_COLORS.pm25))
  if (isOn('light')) pmLightSeries.push(makeHistoryLineSeries('光照', 1, light, HISTORY_SERIES_COLORS.light))

  const gasPwmSeries = []
  if (isOn('gas')) gasPwmSeries.push(makeHistoryLineSeries('气体', 0, gas, HISTORY_SERIES_COLORS.gas))
  if (isOn('pwm')) gasPwmSeries.push(makeHistoryLineSeries('PWM', 0, pwm, HISTORY_SERIES_COLORS.pwm))

  const humanSeries = []
  if (isOn('dist')) humanSeries.push(makeHistoryLineSeries('距离', 0, dist, HISTORY_SERIES_COLORS.dist))
  if (isOn('human')) humanSeries.push(makeHistoryLineSeries('人员(0/1)', 1, human, HISTORY_SERIES_COLORS.human, { step: 'middle', smooth: false }))
  if (isOn('door')) humanSeries.push(makeHistoryLineSeries('门状态(0/1)', 1, door, HISTORY_SERIES_COLORS.door, { step: 'middle', smooth: false }))

  renderHistoryPanel(
    'th',
    historyChartTh,
    xAxis,
    thSeries,
    [{ type: 'value', name: '温湿度', scale: true, axisLabel: { color: '#666666' }, splitLine: { lineStyle: { color: 'rgba(0,0,0,0.08)' } } }]
  )
  renderHistoryPanel(
    'pm_light',
    historyChartPmLight,
    xAxis,
    pmLightSeries,
    [
      { type: 'value', name: 'PM2.5', scale: true, axisLabel: { color: '#666666' }, splitLine: { lineStyle: { color: 'rgba(0,0,0,0.08)' } } },
      { type: 'value', name: '光照', scale: true, position: 'right', axisLabel: { color: '#666666' }, splitLine: { show: false } }
    ]
  )
  renderHistoryPanel(
    'gas_pwm',
    historyChartGasPwm,
    xAxis,
    gasPwmSeries,
    [{ type: 'value', name: '气体/PWM', scale: true, axisLabel: { color: '#666666' }, splitLine: { lineStyle: { color: 'rgba(0,0,0,0.08)' } } }]
  )
  renderHistoryPanel(
    'human_dist_door',
    historyChartHumanDistDoor,
    xAxis,
    humanSeries,
    [
      { type: 'value', name: '距离', scale: true, axisLabel: { color: '#666666' }, splitLine: { lineStyle: { color: 'rgba(0,0,0,0.08)' } } },
      { type: 'value', name: '0/1', min: 0, max: 1, interval: 1, position: 'right', axisLabel: { color: '#666666' }, splitLine: { show: false } }
    ]
  )
}

/* 一键全选 / 全不选所有历史曲线。 */
function toggleAllHistorySeries(checked) {
  HISTORY_SERIES_META.forEach((item) => {
    historySeriesSelected[item.key] = checked
  })
  renderHistoryCharts()
}

/* 加载传感器历史数据并重绘图表。 */
async function loadSensorHistory(rangeQuery) {
  historyLoading.value = true
  historyError.value = ''

  try {
    const body = await fetchHistory('/history/sensors', {
      gateway: gatewayId.value,
      ...rangeQuery,
      interval: historyIntervalSec.value,
      limit: 2000
    })
    sensorHistory.value = Array.isArray(body.items) ? body.items : []
    renderHistoryCharts()
  } catch (err) {
    historyError.value = err?.message ?? '历史曲线加载失败'
  } finally {
    historyLoading.value = false
  }
}

/* 加载动作历史记录。 */
async function loadActionHistory(rangeQuery) {
  actionLoading.value = true
  actionError.value = ''

  try {
    const body = await fetchHistory('/history/actions', {
      gateway: gatewayId.value,
      ...rangeQuery,
      page: 1,
      pageSize: 120
    })
    actionHistory.value = normalizeActionHistoryRows(body.items)
  } catch (err) {
    actionError.value = err?.message ?? '动作记录加载失败'
  } finally {
    actionLoading.value = false
  }
}

/* 对外暴露的“刷新历史图表”入口。 */
async function refreshHistoryData() {
  if (!isLoggedIn.value) {
    return
  }

  historyError.value = ''
  historyNotice.value = ''
  let rangeQuery
  try {
    rangeQuery = buildHistoryRangeQuery()
  } catch (err) {
    historyError.value = err?.message ?? '时间范围参数错误'
    return
  }

  await loadSensorHistory(rangeQuery)
}

/* 对外暴露的“刷新动作记录”入口。 */
async function refreshActionData() {
  if (!isLoggedIn.value) {
    return
  }

  actionError.value = ''
  actionNotice.value = ''
  let rangeQuery
  try {
    rangeQuery = buildActionRangeQuery()
  } catch (err) {
    actionError.value = err?.message ?? '时间范围参数错误'
    return
  }

  await loadActionHistory(rangeQuery)
}

/* 删除指定时间范围内的传感器历史数据。 */
async function deleteSensorHistoryInRange() {
  let rangeQuery
  try {
    rangeQuery = buildHistoryRangeQuery()
  } catch (err) {
    historyError.value = err?.message ?? '时间范围参数错误'
    return
  }

  if (!window.confirm('确认删除当前时间范围内的传感器历史数据？')) {
    return
  }

  historyDeleteBusy.value = true
  historyError.value = ''
  historyNotice.value = ''
  try {
    const body = await deleteHistory('/history/sensors', {
      gateway: gatewayId.value,
      ...rangeQuery
    })
    await refreshHistoryData()
    historyNotice.value = `删除成功，影响 ${body.deleted ?? 0} 条`
  } catch (err) {
    historyError.value = err?.message ?? '删除失败'
  } finally {
    historyDeleteBusy.value = false
  }
}

/* 清空当前网关的全部传感器历史。 */
async function deleteAllSensorHistory() {
  if (!window.confirm('确认删除该网关全部传感器历史数据？此操作不可恢复。')) {
    return
  }

  historyDeleteBusy.value = true
  historyError.value = ''
  historyNotice.value = ''
  try {
    const body = await deleteHistory('/history/sensors', {
      gateway: gatewayId.value,
      all: 1
    })
    await refreshHistoryData()
    historyNotice.value = `已清空，删除 ${body.deleted ?? 0} 条`
  } catch (err) {
    historyError.value = err?.message ?? '清空失败'
  } finally {
    historyDeleteBusy.value = false
  }
}

/* 清空动作历史。 */
async function clearActionHistory() {
  if (!ensureAdminPermission()) {
    return
  }

  if (!window.confirm('确认清空设备动作记录？此操作不可恢复。')) {
    return
  }

  actionDeleteBusy.value = true
  actionError.value = ''
  actionNotice.value = ''
  try {
    const body = await deleteHistory('/history/actions', {
      gateway: gatewayId.value,
      all: 1
    })
    await refreshActionData()
    actionNotice.value = `已清空，删除 ${body.deleted ?? 0} 条`
  } catch (err) {
    actionError.value = err?.message ?? '清空失败'
  } finally {
    actionDeleteBusy.value = false
  }
}
/* 阈值回包变化时，只补齐草稿，不强制覆盖用户正在编辑的输入。 */
watch(
  thresholdsData,
  () => {
    syncThresholdDrafts()
  },
  { immediate: true }
)

/* 历史曲线查询条件变化后自动刷新。 */
watch([gatewayId, historyHours, historyIntervalSec, historyRangeMode], () => {
  if (historyRangeMode.value === 'hours') {
    syncCustomRangeFromHours()
  }
  refreshHistoryData()
})

/* 动作记录查询条件变化后自动刷新。 */
watch([gatewayId, actionHours, actionRangeMode], () => {
  if (actionRangeMode.value === 'hours') {
    syncActionRangeFromHours()
  }
  refreshActionData()
})

/* 勾选的历史曲线集合变化时立即重绘。 */
watch(
  () => JSON.stringify(historySeriesSelected),
  () => {
    renderHistoryCharts()
  }
)

/* 页面挂载：初始化图表、加载历史、启动自动刷新。 */
onMounted(() => {
  syncCustomRangeFromHours()
  syncActionRangeFromHours()
  if (historyChartElTh.value) {
    historyChartTh = echarts.init(historyChartElTh.value)
    historyZoomHandlers.th = () => onHistoryDataZoom('th')
    historyChartTh.on('datazoom', historyZoomHandlers.th)
  }
  if (historyChartElPmLight.value) {
    historyChartPmLight = echarts.init(historyChartElPmLight.value)
    historyZoomHandlers.pm_light = () => onHistoryDataZoom('pm_light')
    historyChartPmLight.on('datazoom', historyZoomHandlers.pm_light)
  }
  if (historyChartElGasPwm.value) {
    historyChartGasPwm = echarts.init(historyChartElGasPwm.value)
    historyZoomHandlers.gas_pwm = () => onHistoryDataZoom('gas_pwm')
    historyChartGasPwm.on('datazoom', historyZoomHandlers.gas_pwm)
  }
  if (historyChartElHumanDistDoor.value) {
    historyChartHumanDistDoor = echarts.init(historyChartElHumanDistDoor.value)
    historyZoomHandlers.human_dist_door = () => onHistoryDataZoom('human_dist_door')
    historyChartHumanDistDoor.on('datazoom', historyZoomHandlers.human_dist_door)
  }

  refreshHistoryData()
  refreshActionData()

  historyTimer = setInterval(() => {
    refreshHistoryData()
  }, HISTORY_REFRESH_MS)

  actionTimer = setInterval(() => {
    refreshActionData()
  }, HISTORY_REFRESH_MS)

  resizeHandler = () => {
    if (historyChartTh) historyChartTh.resize()
    if (historyChartPmLight) historyChartPmLight.resize()
    if (historyChartGasPwm) historyChartGasPwm.resize()
    if (historyChartHumanDistDoor) historyChartHumanDistDoor.resize()
  }
  window.addEventListener('resize', resizeHandler)
})

/* 页面销毁：清理图表、定时器、MQTT 和所有命令超时句柄。 */
onBeforeUnmount(() => {
  commandLogs.value.forEach((entry) => {
    clearLogTimer(entry)
  })

  stopWebAliveHeartbeat()

  if (historyTimer) {
    clearInterval(historyTimer)
    historyTimer = null
  }

  if (actionTimer) {
    clearInterval(actionTimer)
    actionTimer = null
  }

  if (resizeHandler) {
    window.removeEventListener('resize', resizeHandler)
    resizeHandler = null
  }

  if (historyChartTh) {
    if (historyZoomHandlers.th) historyChartTh.off('datazoom', historyZoomHandlers.th)
    historyChartTh.dispose()
    historyChartTh = null
  }
  if (historyChartPmLight) {
    if (historyZoomHandlers.pm_light) historyChartPmLight.off('datazoom', historyZoomHandlers.pm_light)
    historyChartPmLight.dispose()
    historyChartPmLight = null
  }
  if (historyChartGasPwm) {
    if (historyZoomHandlers.gas_pwm) historyChartGasPwm.off('datazoom', historyZoomHandlers.gas_pwm)
    historyChartGasPwm.dispose()
    historyChartGasPwm = null
  }
  if (historyChartHumanDistDoor) {
    if (historyZoomHandlers.human_dist_door) historyChartHumanDistDoor.off('datazoom', historyZoomHandlers.human_dist_door)
    historyChartHumanDistDoor.dispose()
    historyChartHumanDistDoor = null
  }

  stopManualQueue()
  disconnectMqtt()
})
</script>

<template>
  <div class="page-root">
    <!-- 登录遮罩：未登录时锁住整个控制台，只允许先选择角色并进入系统。 -->
    <div v-if="!isLoggedIn" class="login-overlay">
      <div class="login-card">
        <h2>系统登录</h2>
        <p class="login-subtitle">请选择身份后进入控制台</p>
        <div class="login-role-row">
          <button class="btn" :class="{ 'btn-primary': loginRole === 'normal' }" @click="loginAs('normal')">普通用户</button>
          <button class="btn" :class="{ 'btn-primary': loginRole === 'admin' }" @click="loginAs('admin')">管理员</button>
        </div>
        <label v-if="loginRole === 'admin'">
          管理员密码
          <input v-model="adminPasswordInput" type="password" placeholder="请输入管理员密码" @keyup.enter="submitLogin" />
        </label>
        <button class="btn btn-primary login-btn" @click="submitLogin">进入系统</button>
        <p v-if="loginError" class="text-bad">{{ loginError }}</p>
      </div>
    </div>

    <div class="app-shell" :class="{ locked: !isLoggedIn }">
    <!-- 顶栏：展示系统标题、角色信息和全局连接操作。 -->
    <header class="topbar card">
      <div class="title-group">
        <p class="subtitle">Gateway MQTT Console</p>
        <h1 class="title">环境保障系统控制台</h1>
      </div>

      <div class="top-actions">
        <span class="role-badge">当前角色: {{ roleText }}</span>
        <button class="btn btn-primary" @click="connectMqtt">连接</button>
        <button class="btn btn-ghost" @click="disconnectMqtt">断开</button>
        <button class="btn btn-ghost" @click="refreshClientId">随机 ClientId</button>
        <button class="btn btn-ghost" @click="logoutCurrentUser">退出登录</button>
      </div>
    </header>

    <!-- 连接配置：修改 Broker 地址、网关 ID、Client ID。 -->
    <section class="card section">
      <h2>连接配置</h2>
      <div class="config-grid">
        <label>
          Broker URL
          <input v-model="brokerUrl" type="text" />
        </label>
        <label>
          Gateway ID
          <input v-model="gatewayId" type="text" />
        </label>
        <label>
          Client ID
          <input v-model="clientId" type="text" />
        </label>
      </div>

      <div class="status-row">
        <span>
          连接状态:
          <strong :class="isConnected ? 'text-ok' : 'text-bad'">{{ connectionState }}</strong>
        </span>
        <span>最近 ACK: <strong>{{ formatTime(lastAckAt) }}</strong></span>
        <span>错误: <strong class="text-bad">{{ lastError || '-' }}</strong></span>
      </div>
    </section>

    <!-- 连接监测：展示 WiFi、继电器链路和各类传感器在线状态。 -->
    <section class="card section">
      <h2>连接状态监测</h2>
      <div class="conn-grid">
        <div v-for="item in connectionCards" :key="item.label" class="conn-card">
          <span>{{ item.label }}</span>
          <strong>{{ item.value }}</strong>
        </div>
      </div>
    </section>

    <!-- 实时传感器区：直接显示最近一次 tele/sensors / tele/status 汇总结果。 -->
    <section class="card section">
      <div class="section-head">
        <h2>传感器实时数据</h2>
        <span>更新时间: {{ formatTime(lastSensorsAt) }}</span>
      </div>
      <div class="sensor-grid">
        <div v-for="item in sensorCards" :key="item.label" class="sensor-card">
          <span>{{ item.label }}</span>
          <strong>{{ item.value }}</strong>
        </div>
      </div>
    </section>

    <!-- 历史曲线区：数据来源于本地 Node + SQLite，并不是直接来自实时 MQTT。 -->
    <section class="card section">
      <div class="section-head">
        <h2>传感器历史曲线</h2>
        <span>数据库曲线/自动刷新</span>
      </div>
      <div class="history-toolbar">
        <label>
          查询模式
          <select v-model="historyRangeMode">
            <option value="hours">按最近小时</option>
            <option value="custom">自定义时间</option>
          </select>
        </label>
        <label>
          时间范围(小时)
          <select v-model.number="historyHours" :disabled="historyRangeMode !== 'hours'">
            <option :value="1">1</option>
            <option :value="6">6</option>
            <option :value="12">12</option>
            <option :value="24">24</option>
            <option :value="72">72</option>
          </select>
        </label>
        <label>
          开始时间
          <input v-model="historyFromLocal" type="datetime-local" :disabled="historyRangeMode !== 'custom'" />
        </label>
        <label>
          结束时间
          <input v-model="historyToLocal" type="datetime-local" :disabled="historyRangeMode !== 'custom'" />
        </label>
        <label>
          采样间隔(秒)
          <select v-model.number="historyIntervalSec">
            <option :value="0">原始</option>
            <option :value="10">10</option>
            <option :value="30">30</option>
            <option :value="60">60</option>
            <option :value="300">300</option>
          </select>
        </label>
        <button class="btn btn-primary" @click="refreshHistoryData">刷新历史</button>
        <span>{{ historyLoading ? '加载中...' : `点数: ${sensorHistory.length}` }}</span>
      </div>
      <div class="history-filter-row">
        <span class="history-filter-title">传感器筛选</span>
        <label
          v-for="item in HISTORY_SERIES_META"
          :key="item.key"
          class="history-filter-item"
        >
          <input
            v-model="historySeriesSelected[item.key]"
            type="checkbox"
          />
          <span>{{ item.name }}</span>
        </label>
        <button class="btn btn-ghost" @click="toggleAllHistorySeries(true)">全选</button>
        <button class="btn btn-ghost" @click="toggleAllHistorySeries(false)">全不选</button>
      </div>
      <div class="history-chart-grid">
        <div class="history-chart-panel">
          <div class="history-chart-title">温度 / 湿度</div>
          <div ref="historyChartElTh" class="history-chart-sub"></div>
        </div>
        <div class="history-chart-panel">
          <div class="history-chart-title">PM2.5 / 光照</div>
          <div ref="historyChartElPmLight" class="history-chart-sub"></div>
        </div>
        <div class="history-chart-panel">
          <div class="history-chart-title">气体 / PWM</div>
          <div ref="historyChartElGasPwm" class="history-chart-sub"></div>
        </div>
        <div class="history-chart-panel">
          <div class="history-chart-title">人员 / 距离 / 门状态</div>
          <div ref="historyChartElHumanDistDoor" class="history-chart-sub"></div>
        </div>
      </div>
      <p v-if="historyNotice" class="text-ok">{{ historyNotice }}</p>
      <p v-if="historyError" class="text-bad">{{ historyError }}</p>
    </section>

    <!-- 设备状态区：与屏幕状态保持同源，主要读取 tele/status.manual。 -->
    <section class="card section">
      <div class="section-head">
        <h2>设备状态</h2>
        <span>更新时间: {{ formatTime(lastStatusAt || lastSensorsAt) }}</span>
      </div>
      <div class="device-grid">
        <div v-for="item in deviceStatusCards" :key="item.dev" class="device-card">
          <span>{{ item.label }}</span>
          <strong>{{ item.text }}</strong>
        </div>
      </div>
    </section>

    <!-- 管理员专属：手动控制各设备，发送 cmd/manual。 -->
    <section v-if="isAdmin" class="card section">
      <h2>手动模式控制</h2>
      <div class="manual-grid">
        <div v-for="item in DEVICE_ITEMS" :key="item.dev" class="manual-card">
          <h3>{{ item.label }}</h3>
          <label>
            持续时间 sec
            <input
              v-model.number="manualForms[item.dev].sec"
              type="number"
              min="0"
              max="86400"
            />
          </label>
          <label>
            {{ item.kind === 'gear' ? '值(1~5档位)' : '值(0=关,1=开)' }}
            <input
              v-model.number="manualForms[item.dev].value"
              type="number"
              :min="item.kind === 'gear' ? 1 : 0"
              :max="item.kind === 'gear' ? 5 : 1"
            />
          </label>
          <div class="action-row">
            <button class="btn btn-primary" @click="sendManual(item.dev)">发送</button>
            <button class="btn" @click="restoreAuto(item.dev)">恢复自动</button>
          </div>
        </div>
      </div>
    </section>

    <!-- 管理员专属：按统一列表编辑阈值，发送 cmd/threshold。 -->
    <section v-if="isAdmin" class="card section">
      <div class="section-head">
        <h2>阈值编辑</h2>
        <span>更新时间: {{ formatTime(lastThresholdsAt) }}</span>
      </div>
      <div class="threshold-toolbar">
        <label class="threshold-search">
          搜索(ID/名称)
          <input v-model.trim="thresholdKeyword" type="text" placeholder="例如: 风扇 / 11 / 粉尘" />
        </label>
        <label class="threshold-only-changed">
          <input v-model="thresholdOnlyChanged" type="checkbox" />
          <span>只看已改动</span>
        </label>
        <div class="threshold-toolbar-actions">
          <button class="btn" @click="resetAllThresholdDrafts">重置全部</button>
        </div>
      </div>
      <p class="threshold-summary">
        共 {{ thresholdRows.length }} 项，当前显示 {{ thresholdRowsVisible.length }} 项，已改动 {{ thresholdChangedCount }} 项
      </p>
      <div class="threshold-list">
        <div v-for="row in thresholdRowsVisible" :key="row.id" class="threshold-row" :class="{ changed: row.changed }">
          <div class="threshold-main">
            <div class="threshold-row-title">
              <strong>{{ row.label }}</strong>
              <small>ID {{ row.id }}</small>
            </div>
            <span class="threshold-current">当前值: {{ row.current ?? '-' }}{{ row.unit }}</span>
          </div>
          <div class="threshold-edit">
            <input
              v-model.number="thresholdDrafts[String(row.id)]"
              type="number"
              :step="row.type === 'float' ? '0.1' : '1'"
            />
            <span class="threshold-unit">{{ row.unit || '-' }}</span>
          </div>
          <div class="threshold-actions">
            <button class="btn btn-primary" @click="sendThreshold(row)">保存</button>
          </div>
        </div>
        <p v-if="!thresholdRowsVisible.length" class="empty">没有匹配项</p>
      </div>
    </section>

    <!-- 动作历史：来源于本地数据库，由 Node 服务聚合后返回。 -->
    <section class="card section">
      <div class="section-head">
        <h2>设备动作记录</h2>
        <div class="section-head-actions">
          <span>{{ actionLoading ? '加载中...' : `条数: ${actionDisplayRows.length}` }}</span>
          <button v-if="isAdmin" class="btn btn-danger" :disabled="actionDeleteBusy" @click="clearActionHistory">清空记录</button>
        </div>
      </div>
      <div class="history-toolbar">
        <label>
          查询模式
          <select v-model="actionRangeMode">
            <option value="hours">按最近小时</option>
            <option value="custom">自定义时间</option>
          </select>
        </label>
        <label>
          时间范围(小时)
          <select v-model.number="actionHours" :disabled="actionRangeMode !== 'hours'">
            <option :value="1">1</option>
            <option :value="6">6</option>
            <option :value="12">12</option>
            <option :value="24">24</option>
            <option :value="72">72</option>
          </select>
        </label>
        <label>
          开始时间
          <input v-model="actionFromLocal" type="datetime-local" :disabled="actionRangeMode !== 'custom'" />
        </label>
        <label>
          结束时间
          <input v-model="actionToLocal" type="datetime-local" :disabled="actionRangeMode !== 'custom'" />
        </label>
        <button class="btn btn-primary" @click="refreshActionData">刷新记录</button>
      </div>
      <div class="action-latest">
        <strong>最新上次动作:</strong>
        <span>{{ latestAction.dateText }}</span>
        <span>{{ latestAction.timeText }}</span>
        <span>{{ latestAction.deviceText }}</span>
        <span>{{ latestAction.actionText }}</span>
      </div>
      <div class="table-wrap action-table-wrap">
        <table>
          <thead>
            <tr>
              <th>日期</th>
              <th>时间</th>
              <th>设备</th>
              <th>动作</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="(row, idx) in actionDisplayRows" :key="`act-${idx}-${row.ts}`">
              <td>{{ row.dateText }}</td>
              <td>{{ row.timeText }}</td>
              <td>{{ row.deviceText }}</td>
              <td>{{ row.actionText }}</td>
            </tr>
          </tbody>
        </table>
      </div>
      <p v-if="actionNotice" class="text-ok">{{ actionNotice }}</p>
      <p v-if="actionError" class="text-bad">{{ actionError }}</p>
    </section>

    <!-- 命令日志：显示每次发出的命令、req_id、ACK 和超时状态。 -->
    <section class="card section">
      <h2>命令日志（最近 {{ MAX_LOGS }} 条）</h2>
      <div class="log-list">
        <article v-for="log in commandLogs" :key="`log-${log.localSeq}`" class="log-item">
          <div class="log-head">
            <strong>#{{ log.localSeq }} {{ log.cmd }}</strong>
            <span class="time">{{ formatTime(log.createdAt) }}</span>
            <span :class="['badge', log.status]">{{ log.status }}</span>
          </div>
          <p><span class="label">topic</span>{{ log.topic }}</p>
          <p><span class="label">payload</span>{{ log.payloadText }}</p>
          <p v-if="log.reqId"><span class="label">req_id</span>{{ log.reqId }}</p>
          <p v-if="log.reason"><span class="label">reason</span>{{ log.reason }}</p>
          <pre v-if="log.ack">{{ prettyJson(log.ack) }}</pre>
        </article>
        <p v-if="!commandLogs.length" class="empty">暂无命令日志</p>
      </div>
    </section>
  </div>
  <!-- Teleport 到 body：确保 toast 不受布局容器裁切和层级影响。 -->
  <Teleport to="body">
    <div v-if="toastVisible" style="position:fixed;top:20%;left:50%;transform:translateX(-50%);z-index:99999;background:rgba(0,0,0,0.78);color:#fff;padding:14px 32px;border-radius:10px;font-size:15px;pointer-events:none;white-space:nowrap;box-shadow:0 4px 20px rgba(0,0,0,0.3);animation:toast-in .25s ease">
      {{ toastMessage }}
    </div>
  </Teleport>
  </div>
</template>
