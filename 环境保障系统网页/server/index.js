import fs from 'node:fs'
import path from 'node:path'
import process from 'node:process'
import { fileURLToPath } from 'node:url'
import { DatabaseSync } from 'node:sqlite'
import express from 'express'
import cors from 'cors'
import mqtt from 'mqtt'

const __filename = fileURLToPath(import.meta.url)
const __dirname = path.dirname(__filename)

const SERVER_PORT = Number(process.env.PORT || 3001)
const MQTT_URL = process.env.MQTT_URL || 'mqtt://8.212.157.225:1883'
const MQTT_USERNAME = process.env.MQTT_USERNAME || ''
const MQTT_PASSWORD = process.env.MQTT_PASSWORD || ''
const DB_FILE = process.env.DB_FILE || path.join(__dirname, 'data', 'history.db')
const DEFAULT_GATEWAY_ID = process.env.DEFAULT_GATEWAY_ID || 'gw001'
const SENSOR_STATUS_MERGE_WINDOW_MS = 15000
const ACTION_DUP_SUPPRESS_MS = envNumber('ACTION_DUP_SUPPRESS_MS', 5000)

function envNumber(name, fallback) {
  const n = Number(process.env[name])
  return Number.isFinite(n) ? n : fallback
}

const DB_SIZE_LIMIT_BYTES = envNumber('DB_SIZE_LIMIT_BYTES', 20 * 1024 * 1024 * 1024)
const DB_SIZE_TARGET_BYTES = Math.min(
  DB_SIZE_LIMIT_BYTES,
  envNumber('DB_SIZE_TARGET_BYTES', Math.trunc(DB_SIZE_LIMIT_BYTES * 0.9))
)
const DB_SIZE_CHECK_INTERVAL_MS = envNumber('DB_SIZE_CHECK_INTERVAL_MS', 60_000)
const DB_PRUNE_BATCH_ROWS = envNumber('DB_PRUNE_BATCH_ROWS', 5000)
const DB_PRUNE_MAX_STEPS = envNumber('DB_PRUNE_MAX_STEPS', 500)

const app = express()
app.use(cors())
app.use(express.json({ limit: '1mb' }))

fs.mkdirSync(path.dirname(DB_FILE), { recursive: true })
const db = new DatabaseSync(DB_FILE)

function initDb() {
  db.exec(`
    PRAGMA journal_mode = WAL;
    PRAGMA synchronous = NORMAL;

    CREATE TABLE IF NOT EXISTS sensor_history (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      uniq_key TEXT NOT NULL UNIQUE,
      gateway_id TEXT NOT NULL,
      ts INTEGER NOT NULL,
      seq INTEGER,
      temp REAL,
      humi REAL,
      pm25 REAL,
      gas REAL,
      pwm REAL,
      human REAL,
      dist REAL,
      light REAL,
      door REAL,
      raw_json TEXT
    );

    CREATE TABLE IF NOT EXISTS action_history (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      uniq_key TEXT NOT NULL UNIQUE,
      gateway_id TEXT NOT NULL,
      ts INTEGER NOT NULL,
      seq INTEGER,
      cmd TEXT,
      device TEXT,
      mode TEXT,
      action TEXT,
      ok INTEGER,
      reason TEXT,
      source TEXT,
      detail_json TEXT
    );

    CREATE INDEX IF NOT EXISTS idx_sensor_gateway_ts ON sensor_history(gateway_id, ts);
    CREATE INDEX IF NOT EXISTS idx_action_gateway_ts ON action_history(gateway_id, ts);
  `)
}

initDb()

function ensureSensorHistoryColumns() {
  const cols = db.prepare(`PRAGMA table_info(sensor_history)`).all()
  const colSet = new Set(cols.map((c) => String(c.name)))
  const addColumn = (name) => {
    if (colSet.has(name)) {
      return
    }
    db.exec(`ALTER TABLE sensor_history ADD COLUMN ${name} REAL`)
  }

  addColumn('human')
  addColumn('dist')
  addColumn('light')
  addColumn('door')
}

ensureSensorHistoryColumns()

const insertSensorStmt = db.prepare(`
  INSERT OR IGNORE INTO sensor_history
  (uniq_key, gateway_id, ts, seq, temp, humi, pm25, gas, pwm, human, dist, light, door, raw_json)
  VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
`)

const upsertActionStmt = db.prepare(`
  INSERT INTO action_history
  (uniq_key, gateway_id, ts, seq, cmd, device, mode, action, ok, reason, source, detail_json)
  VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  ON CONFLICT(uniq_key) DO UPDATE SET
    ts = excluded.ts,
    seq = COALESCE(excluded.seq, action_history.seq),
    cmd = CASE
      WHEN excluded.cmd IS NULL OR excluded.cmd = '' THEN action_history.cmd
      ELSE excluded.cmd
    END,
    device = CASE
      WHEN excluded.device IS NULL OR excluded.device = '' OR lower(excluded.device) = 'unknown'
      THEN action_history.device
      ELSE excluded.device
    END,
    mode = CASE
      WHEN excluded.mode IS NULL OR excluded.mode = '' THEN action_history.mode
      ELSE excluded.mode
    END,
    action = CASE
      WHEN excluded.action IS NULL OR excluded.action = '' OR lower(excluded.action) IN ('unknown', 'pwm_unknown', '-')
      THEN action_history.action
      ELSE excluded.action
    END,
    ok = COALESCE(excluded.ok, action_history.ok),
    reason = COALESCE(excluded.reason, action_history.reason),
    source = excluded.source,
    detail_json = CASE
      WHEN excluded.detail_json IS NULL OR excluded.detail_json = '' THEN action_history.detail_json
      ELSE excluded.detail_json
    END
`)
const deleteSensorRangeStmt = db.prepare(`
  DELETE FROM sensor_history
  WHERE gateway_id = ? AND ts BETWEEN ? AND ?
`)
const deleteSensorAllStmt = db.prepare(`
  DELETE FROM sensor_history
  WHERE gateway_id = ?
`)
const deleteActionRangeStmt = db.prepare(`
  DELETE FROM action_history
  WHERE gateway_id = ? AND ts BETWEEN ? AND ?
`)
const deleteActionAllStmt = db.prepare(`
  DELETE FROM action_history
  WHERE gateway_id = ?
`)
const countSensorRowsStmt = db.prepare(`SELECT COUNT(*) AS n FROM sensor_history`)
const countActionRowsStmt = db.prepare(`SELECT COUNT(*) AS n FROM action_history`)
const findOldestSensorTsStmt = db.prepare(`
  SELECT ts
  FROM sensor_history
  ORDER BY ts ASC, id ASC
  LIMIT 1
`)
const findOldestActionTsStmt = db.prepare(`
  SELECT ts
  FROM action_history
  ORDER BY ts ASC, id ASC
  LIMIT 1
`)
const deleteOldestSensorBatchStmt = db.prepare(`
  DELETE FROM sensor_history
  WHERE id IN (
    SELECT id
    FROM sensor_history
    ORDER BY ts ASC, id ASC
    LIMIT ?
  )
`)
const deleteOldestActionBatchStmt = db.prepare(`
  DELETE FROM action_history
  WHERE id IN (
    SELECT id
    FROM action_history
    ORDER BY ts ASC, id ASC
    LIMIT ?
  )
`)
const findRecentSameActionStmt = db.prepare(`
  SELECT ts
  FROM action_history
  WHERE gateway_id = ?
    AND cmd = ?
    AND device = ?
    AND mode = ?
    AND action = ?
    AND source = 'action'
  ORDER BY ts DESC, id DESC
  LIMIT 1
`)

const findSensorCoreRowStmt = db.prepare(`
  SELECT id
  FROM sensor_history
  WHERE gateway_id = ?
    AND ts BETWEEN ? AND ?
    AND (temp IS NOT NULL OR humi IS NOT NULL OR pm25 IS NOT NULL OR gas IS NOT NULL OR pwm IS NOT NULL)
  ORDER BY ABS(ts - ?) ASC
  LIMIT 1
`)

const findStatusCoreRowStmt = db.prepare(`
  SELECT id
  FROM sensor_history
  WHERE gateway_id = ?
    AND ts BETWEEN ? AND ?
    AND (human IS NOT NULL OR dist IS NOT NULL OR light IS NOT NULL OR door IS NOT NULL)
  ORDER BY ABS(ts - ?) ASC
  LIMIT 1
`)

const updateSensorCoreStmt = db.prepare(`
  UPDATE sensor_history
  SET
    temp = COALESCE(?, temp),
    humi = COALESCE(?, humi),
    pm25 = COALESCE(?, pm25),
    gas = COALESCE(?, gas),
    pwm = COALESCE(?, pwm),
    raw_json = COALESCE(?, raw_json)
  WHERE id = ?
`)

const updateStatusCoreStmt = db.prepare(`
  UPDATE sensor_history
  SET
    human = COALESCE(?, human),
    dist = COALESCE(?, dist),
    light = COALESCE(?, light),
    door = COALESCE(?, door),
    raw_json = COALESCE(?, raw_json)
  WHERE id = ?
`)

let dbPruneRunning = false
let nextDbCheckAt = 0
let dbSizeGuardTimer = null

function safeFileSize(filePath) {
  try {
    return fs.statSync(filePath).size
  } catch {
    return 0
  }
}

function getDbTotalBytes() {
  return (
    safeFileSize(DB_FILE) +
    safeFileSize(`${DB_FILE}-wal`) +
    safeFileSize(`${DB_FILE}-shm`) +
    safeFileSize(`${DB_FILE}-journal`)
  )
}

function getDbHistoryRowCount() {
  const sensorCount = Number(countSensorRowsStmt.get()?.n || 0)
  const actionCount = Number(countActionRowsStmt.get()?.n || 0)
  return sensorCount + actionCount
}

function deleteOldestHistoryBatch(batchRows) {
  const sensorCount = Number(countSensorRowsStmt.get()?.n || 0)
  const actionCount = Number(countActionRowsStmt.get()?.n || 0)

  if ((sensorCount + actionCount) <= 0) {
    return 0
  }

  if (sensorCount <= 0) {
    return Number(deleteOldestActionBatchStmt.run(batchRows)?.changes || 0)
  }
  if (actionCount <= 0) {
    return Number(deleteOldestSensorBatchStmt.run(batchRows)?.changes || 0)
  }

  const sensorTs = Number(findOldestSensorTsStmt.get()?.ts || Number.MAX_SAFE_INTEGER)
  const actionTs = Number(findOldestActionTsStmt.get()?.ts || Number.MAX_SAFE_INTEGER)

  if (sensorTs <= actionTs) {
    return Number(deleteOldestSensorBatchStmt.run(batchRows)?.changes || 0)
  }

  return Number(deleteOldestActionBatchStmt.run(batchRows)?.changes || 0)
}

function enforceDbSizeLimit(reason = 'timer') {
  if (!Number.isFinite(DB_SIZE_LIMIT_BYTES) || DB_SIZE_LIMIT_BYTES <= 0) {
    return
  }
  if (dbPruneRunning) {
    return
  }

  const beforeBytes = getDbTotalBytes()
  if (beforeBytes <= DB_SIZE_LIMIT_BYTES) {
    return
  }

  dbPruneRunning = true
  try {
    console.warn(`[server] DB size limit exceeded (${beforeBytes} bytes), reason=${reason}, pruning...`)

    const batchRows = Math.max(100, Math.trunc(DB_PRUNE_BATCH_ROWS))
    const maxSteps = Math.max(1, Math.trunc(DB_PRUNE_MAX_STEPS))
    const targetBytes = Math.max(0, Math.trunc(DB_SIZE_TARGET_BYTES))

    let deletedRows = 0
    let step = 0

    while (step < maxSteps) {
      const changed = deleteOldestHistoryBatch(batchRows)
      if (changed <= 0) {
        break
      }
      deletedRows += changed
      step += 1
    }

    db.exec('PRAGMA wal_checkpoint(TRUNCATE);')
    db.exec('VACUUM;')
    db.exec('PRAGMA wal_checkpoint(TRUNCATE);')

    const afterBytes = getDbTotalBytes()
    const rowCount = getDbHistoryRowCount()
    const reachTarget = afterBytes <= targetBytes

    console.warn(
      `[server] DB prune done: deletedRows=${deletedRows}, sizeBefore=${beforeBytes}, sizeAfter=${afterBytes}, rowsLeft=${rowCount}, reachTarget=${reachTarget ? 1 : 0}`
    )
  } catch (err) {
    console.error(`[server] DB prune failed: ${err?.message || err}`)
  } finally {
    dbPruneRunning = false
  }
}

function maybeEnforceDbSizeLimit(reason = 'ingest') {
  const now = Date.now()
  if (now < nextDbCheckAt) {
    return
  }
  nextDbCheckAt = now + 5000
  enforceDbSizeLimit(reason)
}

function toFiniteNumber(v) {
  const n = Number(v)
  return Number.isFinite(n) ? n : null
}

function normalizeTimestampMs(v) {
  const n = Number(v)
  if (Number.isFinite(n) && n > 0) {
    return n < 1e12 ? Math.trunc(n * 1000) : Math.trunc(n)
  }
  return Date.now()
}

function parseTopic(topic) {
  const m = /^ws\/([^/]+)\/tele\/(.+)$/.exec(topic)
  if (!m) {
    return null
  }
  return {
    gatewayId: m[1],
    suffix: m[2]
  }
}

function makeSensorUniqKey(gatewayId, seq, ts) {
  if (Number.isFinite(seq)) {
    // Avoid collisions after device reboot when seq restarts from a low value.
    // Keep same-packet dedupe by including the source timestamp.
    return `${gatewayId}:sensor:${Math.trunc(seq)}:${Math.trunc(ts)}`
  }
  return `${gatewayId}:sensor:${ts}`
}

function parseManualDeviceFromTopic(topic) {
  if (typeof topic !== 'string') {
    return ''
  }
  const m = /\/manual\/([^/?#]+)$/.exec(topic.trim())
  return m ? String(m[1] || '').toLowerCase() : ''
}

function isUnknownActionText(v) {
  const s = String(v || '').trim().toLowerCase()
  return !s || s === 'unknown' || s === 'pwm_unknown' || s === '-'
}

function summarizeAction(payload, suffix) {
  const cmd = typeof payload.cmd === 'string' ? payload.cmd : suffix

  if (cmd === 'relay') {
    const ch = Number(payload.ch)
    const on = Number(payload.on)
    return {
      cmd,
      device: Number.isFinite(ch) ? `relay_ch${ch}` : 'relay',
      mode: 'manual',
      action: on === 1 ? 'on' : on === 0 ? 'off' : 'unknown'
    }
  }

  if (cmd === 'fan') {
    const pwm = Number(payload.pwm)
    return {
      cmd,
      device: 'fan',
      mode: 'manual',
      action: Number.isFinite(pwm) ? `pwm_${Math.trunc(pwm)}` : 'pwm_unknown'
    }
  }

  if (cmd === 'manual') {
    const devRaw = typeof payload.dev === 'string'
      ? payload.dev
      : (typeof payload.device === 'string' ? payload.device : '')
    const devFromTopic = parseManualDeviceFromTopic(payload.topic)
    const dev = String(devRaw || devFromTopic || 'unknown').toLowerCase()
    const mode = typeof payload.mode === 'string' ? payload.mode : 'manual'
    const value = Number(payload.value)
    let action = ''
    if (Number.isFinite(value)) {
      action = `value_${Math.trunc(value)}`
    } else if (mode === 'auto') {
      action = 'auto'
    } else if (typeof payload.action === 'string' && payload.action) {
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

  const fallbackDeviceRaw = typeof payload.dev === 'string'
    ? payload.dev
    : (typeof payload.device === 'string' ? payload.device : '')
  const fallbackDevice = String(fallbackDeviceRaw || parseManualDeviceFromTopic(payload.topic) || 'unknown').toLowerCase()
  const fallbackOn = Number(payload.on)
  const fallbackPwm = Number(payload.pwm)
  let fallbackAction = typeof payload.action === 'string' ? payload.action : ''
  if (!fallbackAction) {
    if (fallbackOn === 1 || fallbackOn === 0) {
      fallbackAction = fallbackOn === 1 ? 'on' : 'off'
    } else if (Number.isFinite(fallbackPwm)) {
      fallbackAction = `pwm_${Math.trunc(fallbackPwm)}`
    }
  }

  return {
    cmd,
    device: fallbackDevice,
    mode: typeof payload.mode === 'string' ? payload.mode : '',
    action: fallbackAction
  }
}

function makeActionUniqKey(gatewayId, payload, ts, suffix, actionMeta) {
  const reqId = typeof payload.req_id === 'string' ? payload.req_id : ''
  if (reqId) {
    return `${gatewayId}:action:req:${reqId}`
  }

  const cmd = typeof payload.cmd === 'string' ? payload.cmd : suffix
  const seq = Number(payload.seq)
  if (Number.isFinite(seq)) {
    return `${gatewayId}:action:seq:${cmd}:${Math.trunc(seq)}`
  }

  if (
    actionMeta &&
    typeof actionMeta.device === 'string' &&
    typeof actionMeta.action === 'string' &&
    actionMeta.device.toLowerCase() !== 'unknown' &&
    !isUnknownActionText(actionMeta.action)
  ) {
    return `${gatewayId}:action:sig:${cmd}:${actionMeta.device}:${actionMeta.action}:${Math.trunc(ts / 1000)}`
  }

  return `${gatewayId}:action:${cmd}:${ts}`
}

function ingestSensor(gatewayId, payload) {
  const gid = (payload.id && typeof payload.id === 'string') ? payload.id : gatewayId
  const ts = normalizeTimestampMs(payload.ts)
  const seq = toFiniteNumber(payload.seq)
  const uniqKey = makeSensorUniqKey(gid, seq, ts)
  const fromTs = ts - SENSOR_STATUS_MERGE_WINDOW_MS
  const toTs = ts + SENSOR_STATUS_MERGE_WINDOW_MS
  const temp = toFiniteNumber(payload.temp)
  const humi = toFiniteNumber(payload.humi)
  const pm25 = toFiniteNumber(payload.pm25)
  const gas = toFiniteNumber(payload.gas)
  const pwm = toFiniteNumber(payload.pwm)
  const rawJson = JSON.stringify(payload)
  const candidate = findStatusCoreRowStmt.get(gid, fromTs, toTs, ts)

  if (candidate?.id) {
    updateSensorCoreStmt.run(
      temp,
      humi,
      pm25,
      gas,
      pwm,
      rawJson,
      candidate.id
    )
    maybeEnforceDbSizeLimit('ingest_sensor_update')
    return
  }

  insertSensorStmt.run(
    uniqKey,
    gid,
    ts,
    seq,
    temp,
    humi,
    pm25,
    gas,
    pwm,
    toFiniteNumber(payload.human),
    toFiniteNumber(payload.dist),
    toFiniteNumber(payload.light),
    toFiniteNumber(payload.door),
    rawJson
  )
  maybeEnforceDbSizeLimit('ingest_sensor_insert')
}

function ingestStatus(gatewayId, payload) {
  const sensor = (payload && typeof payload === 'object' && payload.sensor && typeof payload.sensor === 'object')
    ? payload.sensor
    : {}
  const merged = {
    id: payload?.id,
    ts: payload?.ts,
    seq: payload?.seq,
    human: sensor.human,
    dist: sensor.dist,
    light: sensor.light,
    door: sensor.door
  }
  const gid = (merged.id && typeof merged.id === 'string') ? merged.id : gatewayId
  const ts = normalizeTimestampMs(merged.ts)
  const seq = toFiniteNumber(merged.seq)
  const uniqKey = makeSensorUniqKey(gid, seq, ts)
  const human = toFiniteNumber(merged.human)
  const dist = toFiniteNumber(merged.dist)
  const light = toFiniteNumber(merged.light)
  const door = toFiniteNumber(merged.door)
  const rawJson = JSON.stringify(payload)
  const fromTs = ts - SENSOR_STATUS_MERGE_WINDOW_MS
  const toTs = ts + SENSOR_STATUS_MERGE_WINDOW_MS
  const candidate = findSensorCoreRowStmt.get(gid, fromTs, toTs, ts)

  if (candidate?.id) {
    updateStatusCoreStmt.run(
      human,
      dist,
      light,
      door,
      rawJson,
      candidate.id
    )
    maybeEnforceDbSizeLimit('ingest_status_update')
    return
  }

  insertSensorStmt.run(
    uniqKey,
    gid,
    ts,
    seq,
    null,
    null,
    null,
    null,
    null,
    human,
    dist,
    light,
    door,
    rawJson
  )
  maybeEnforceDbSizeLimit('ingest_status_insert')
}

function ingestAction(gatewayId, payload, suffix) {
  const gid = (payload.id && typeof payload.id === 'string') ? payload.id : gatewayId
  const ts = normalizeTimestampMs(payload.ts)
  const seq = toFiniteNumber(payload.seq)
  const actionMeta = summarizeAction(payload, suffix)
  const uniqKey = makeActionUniqKey(gid, payload, ts, suffix, actionMeta)
  const ok = Number(payload.ok)
  const reqId = typeof payload.req_id === 'string' ? payload.req_id : ''

  if (
    suffix === 'ack' &&
    !reqId &&
    !Number.isFinite(seq) &&
    String(actionMeta.device || '').toLowerCase() === 'unknown' &&
    isUnknownActionText(actionMeta.action)
  ) {
    return
  }

  if (
    suffix === 'action' &&
    actionMeta &&
    typeof actionMeta.device === 'string' &&
    typeof actionMeta.action === 'string' &&
    actionMeta.device.toLowerCase() !== 'unknown' &&
    !isUnknownActionText(actionMeta.action)
  ) {
    const recent = findRecentSameActionStmt.get(
      gid,
      String(actionMeta.cmd || ''),
      String(actionMeta.device || ''),
      String(actionMeta.mode || ''),
      String(actionMeta.action || '')
    )
    if (recent && Math.abs(ts - Number(recent.ts || 0)) <= ACTION_DUP_SUPPRESS_MS) {
      return
    }
  }

  upsertActionStmt.run(
    uniqKey,
    gid,
    ts,
    seq,
    actionMeta.cmd,
    actionMeta.device,
    actionMeta.mode,
    actionMeta.action,
    Number.isFinite(ok) ? Math.trunc(ok) : null,
    typeof payload.reason === 'string' ? payload.reason : null,
    suffix,
    JSON.stringify(payload)
  )
  maybeEnforceDbSizeLimit('ingest_action_insert')
}

const mqttClient = mqtt.connect(MQTT_URL, {
  clientId: `history_srv_${Math.random().toString(16).slice(2, 10)}`,
  clean: true,
  reconnectPeriod: 2000,
  connectTimeout: 10000,
  username: MQTT_USERNAME || undefined,
  password: MQTT_PASSWORD || undefined
})

const SUBSCRIPTIONS = ['ws/+/tele/sensors', 'ws/+/tele/status', 'ws/+/tele/action']

mqttClient.on('connect', () => {
  console.log(`[server] MQTT connected: ${MQTT_URL}`)
  mqttClient.subscribe(SUBSCRIPTIONS, { qos: 0 }, (err) => {
    if (err) {
      console.error('[server] MQTT subscribe failed:', err.message)
      return
    }
    console.log(`[server] subscribed: ${SUBSCRIPTIONS.join(', ')}`)
  })
})

mqttClient.on('reconnect', () => {
  console.log('[server] MQTT reconnecting...')
})

mqttClient.on('error', (err) => {
  console.error('[server] MQTT error:', err.message)
})

mqttClient.on('message', (topic, message) => {
  const parsedTopic = parseTopic(topic)
  if (!parsedTopic) {
    return
  }

  let payload
  try {
    payload = JSON.parse(message.toString())
  } catch {
    return
  }

  if (parsedTopic.suffix === 'sensors') {
    ingestSensor(parsedTopic.gatewayId, payload)
    return
  }

  if (parsedTopic.suffix === 'status') {
    ingestStatus(parsedTopic.gatewayId, payload)
    return
  }

  if (parsedTopic.suffix === 'action') {
    ingestAction(parsedTopic.gatewayId, payload, parsedTopic.suffix)
  }
})

function parseNumberQuery(v, fallback) {
  const n = Number(v)
  return Number.isFinite(n) ? n : fallback
}

function parseTimeRange(query) {
  const now = Date.now()
  const hours = Math.max(1, Math.min(24 * 30, parseNumberQuery(query.hours, 6)))
  const from = parseNumberQuery(query.from, now - hours * 3600 * 1000)
  const to = parseNumberQuery(query.to, now)

  return {
    from: Math.trunc(Math.min(from, to)),
    to: Math.trunc(Math.max(from, to))
  }
}

function parseBoolQuery(v, fallback = false) {
  if (v === undefined || v === null) {
    return fallback
  }
  if (v === true || v === 1 || v === '1' || v === 'true') {
    return true
  }
  if (v === false || v === 0 || v === '0' || v === 'false') {
    return false
  }
  return fallback
}

app.get('/api/health', (req, res) => {
  const sensorCount = countSensorRowsStmt.get().n
  const actionCount = countActionRowsStmt.get().n
  const dbTotalBytes = getDbTotalBytes()

  res.json({
    ok: 1,
    db: path.resolve(DB_FILE),
    mqtt: mqttClient.connected ? 'connected' : 'disconnected',
    sensorCount,
    actionCount,
    dbTotalBytes,
    dbLimitBytes: DB_SIZE_LIMIT_BYTES
  })
})

app.get('/api/history/sensors', (req, res) => {
  const gatewayId = String(req.query.gateway || DEFAULT_GATEWAY_ID)
  const { from, to } = parseTimeRange(req.query)
  const intervalSec = Math.max(0, Math.min(3600, Math.trunc(parseNumberQuery(req.query.interval, 60))))
  const limit = Math.max(10, Math.min(5000, Math.trunc(parseNumberQuery(req.query.limit, 1200))))

  let items

  if (intervalSec > 0) {
    const bucketMs = intervalSec * 1000
    items = db.prepare(`
      SELECT
        ((ts / ?) * ?) AS ts,
        ROUND(AVG(temp), 2) AS temp,
        ROUND(AVG(humi), 2) AS humi,
        ROUND(AVG(pm25), 2) AS pm25,
        ROUND(AVG(gas), 2) AS gas,
        ROUND(AVG(pwm), 2) AS pwm,
        ROUND(AVG(human), 2) AS human,
        ROUND(AVG(dist), 2) AS dist,
        ROUND(AVG(light), 2) AS light,
        ROUND(AVG(door), 2) AS door,
        COUNT(*) AS samples
      FROM sensor_history
      WHERE gateway_id = ? AND ts BETWEEN ? AND ?
      GROUP BY ((ts / ?) * ?)
      ORDER BY ts ASC
      LIMIT ?
    `).all(bucketMs, bucketMs, gatewayId, from, to, bucketMs, bucketMs, limit)
  } else {
    items = db.prepare(`
      SELECT ts, temp, humi, pm25, gas, pwm, human, dist, light, door, seq
      FROM sensor_history
      WHERE gateway_id = ? AND ts BETWEEN ? AND ?
      ORDER BY ts ASC
      LIMIT ?
    `).all(gatewayId, from, to, limit)
  }

  res.json({
    ok: 1,
    gatewayId,
    from,
    to,
    intervalSec,
    items
  })
})

app.get('/api/history/actions', (req, res) => {
  const gatewayId = String(req.query.gateway || DEFAULT_GATEWAY_ID)
  const { from, to } = parseTimeRange(req.query)
  const page = Math.max(1, Math.trunc(parseNumberQuery(req.query.page, 1)))
  const pageSize = Math.max(10, Math.min(500, Math.trunc(parseNumberQuery(req.query.pageSize, 100))))
  const offset = (page - 1) * pageSize

  const rows = db.prepare(`
    SELECT ts, seq, cmd, device, mode, action, ok, reason, source, detail_json
    FROM action_history
    WHERE gateway_id = ? AND ts BETWEEN ? AND ? AND source = 'action'
    ORDER BY ts DESC
    LIMIT ? OFFSET ?
  `).all(gatewayId, from, to, pageSize, offset)

  const items = rows.map((row) => ({
    ...row,
    detail: (() => {
      try {
        return row.detail_json ? JSON.parse(row.detail_json) : null
      } catch {
        return null
      }
    })()
  }))

  res.json({
    ok: 1,
    gatewayId,
    from,
    to,
    page,
    pageSize,
    items
  })
})

app.delete('/api/history/sensors', (req, res) => {
  const gatewayId = String(req.query.gateway || DEFAULT_GATEWAY_ID)
  const deleteAll = parseBoolQuery(req.query.all, false)
  const { from, to } = parseTimeRange(req.query)

  const result = deleteAll
    ? deleteSensorAllStmt.run(gatewayId)
    : deleteSensorRangeStmt.run(gatewayId, from, to)

  res.json({
    ok: 1,
    gatewayId,
    from: deleteAll ? null : from,
    to: deleteAll ? null : to,
    deleted: Number(result?.changes || 0),
    all: deleteAll ? 1 : 0
  })
})

app.delete('/api/history/actions', (req, res) => {
  const gatewayId = String(req.query.gateway || DEFAULT_GATEWAY_ID)
  const deleteAll = parseBoolQuery(req.query.all, false)
  const { from, to } = parseTimeRange(req.query)

  const result = deleteAll
    ? deleteActionAllStmt.run(gatewayId)
    : deleteActionRangeStmt.run(gatewayId, from, to)

  res.json({
    ok: 1,
    gatewayId,
    from: deleteAll ? null : from,
    to: deleteAll ? null : to,
    deleted: Number(result?.changes || 0),
    all: deleteAll ? 1 : 0
  })
})

app.listen(SERVER_PORT, () => {
  enforceDbSizeLimit('startup')
  dbSizeGuardTimer = setInterval(() => {
    enforceDbSizeLimit('periodic')
  }, Math.max(5000, Math.trunc(DB_SIZE_CHECK_INTERVAL_MS)))

  console.log(`[server] API listening on http://127.0.0.1:${SERVER_PORT}`)
  console.log(`[server] SQLite: ${path.resolve(DB_FILE)}`)
  console.log(`[server] DB size guard: limit=${DB_SIZE_LIMIT_BYTES} target=${DB_SIZE_TARGET_BYTES} interval=${DB_SIZE_CHECK_INTERVAL_MS}ms`)
})
