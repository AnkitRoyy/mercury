import { useState, useEffect, useRef, useCallback } from 'react'

const fmt = (value, digits = 1, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

const PAN_MIN = -90, PAN_MAX = 90
const TILT_MIN = -45, TILT_MAX = 45
const STEP = 2        // deg per tick
const TICK_MS = 60     // repeat rate while key held

const KEY_DELTA = {
  ArrowUp:    [0,  STEP],
  ArrowDown:  [0, -STEP],
  ArrowLeft:  [-STEP, 0],
  ArrowRight: [ STEP, 0],
}

export default function ServoPanel({ servo, sendCommand }) {
  const [pan, setPan] = useState(servo?.pan ?? 0)
  const [tilt, setTilt] = useState(servo?.tilt ?? 0)
  const [active, setActive] = useState(false)
  const posRef = useRef({ pan, tilt })
  const heldKeys = useRef(new Set())

  useEffect(() => { setPan(servo?.pan ?? 0) }, [servo?.pan])
  useEffect(() => { setTilt(servo?.tilt ?? 0) }, [servo?.tilt])
  useEffect(() => { posRef.current = { pan, tilt } }, [pan, tilt])

  const move = useCallback((dPan, dTilt) => {
    const nextPan  = Math.min(PAN_MAX,  Math.max(PAN_MIN,  posRef.current.pan  + dPan))
    const nextTilt = Math.min(TILT_MAX, Math.max(TILT_MIN, posRef.current.tilt + dTilt))
    posRef.current = { pan: nextPan, tilt: nextTilt }
    setPan(nextPan)
    setTilt(nextTilt)
    sendCommand({ type: 'SERVO', pan: nextPan, tilt: nextTilt })
  }, [sendCommand])

  useEffect(() => {
    if (!active) return

    const onKeyDown = (e) => {
      if (!(e.key in KEY_DELTA)) return
      e.preventDefault()
      heldKeys.current.add(e.key)
    }
    const onKeyUp = (e) => heldKeys.current.delete(e.key)

    const tick = setInterval(() => {
      if (heldKeys.current.size === 0) return
      let dPan = 0, dTilt = 0
      for (const key of heldKeys.current) {
        const [dp, dt] = KEY_DELTA[key]
        dPan += dp; dTilt += dt
      }
      if (dPan || dTilt) move(dPan, dTilt)
    }, TICK_MS)

    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      clearInterval(tick)
      heldKeys.current.clear()
    }
  }, [active, move])

  const handlePan = (value) => {
    posRef.current.pan = value
    setPan(value)
    sendCommand({ type: 'SERVO', pan: value, tilt: posRef.current.tilt })
  }

  const handleTilt = (value) => {
    posRef.current.tilt = value
    setTilt(value)
    sendCommand({ type: 'SERVO', pan: posRef.current.pan, tilt: value })
  }

  const center = () => {
    posRef.current = { pan: 0, tilt: 0 }
    setPan(0)
    setTilt(0)
    sendCommand({ type: 'SERVO', pan: 0, tilt: 0 })
  }

  return (
    <div
      tabIndex={0}
      onFocus={() => setActive(true)}
      onBlur={() => setActive(false)}
      className={`bg-zinc-950 border rounded-lg p-3 flex flex-col gap-3 h-full outline-none transition-colors ${
        active ? 'border-emerald-400' : 'border-zinc-800'
      }`}
    >
      <div className="flex items-center justify-between shrink-0">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Turret Servo</span>
        <div className="flex items-center gap-2">
          <span className={`text-[10px] font-bold ${active ? 'text-emerald-400' : 'text-zinc-600'}`}>
            {active ? 'ARROW KEYS ACTIVE' : 'CLICK TO CONTROL'}
          </span>
          <button
            onClick={center}
            className="text-[10px] font-bold px-2 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-zinc-300 hover:bg-zinc-800"
          >
            CENTER
          </button>
        </div>
      </div>

      <div className="flex-1 min-h-0 grid grid-cols-2 gap-4">
        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-xs text-zinc-500">Pan</span>
            <span className="text-xs font-mono text-white">{fmt(pan, 1, '°')}</span>
          </div>
          <input type="range" min={PAN_MIN} max={PAN_MAX} step={1} value={pan}
            onChange={(e) => handlePan(Number(e.target.value))}
            className="w-full accent-emerald-400" />
        </div>

        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-xs text-zinc-500">Tilt</span>
            <span className="text-xs font-mono text-white">{fmt(tilt, 1, '°')}</span>
          </div>
          <input type="range" min={TILT_MIN} max={TILT_MAX} step={1} value={tilt}
            onChange={(e) => handleTilt(Number(e.target.value))}
            className="w-full accent-emerald-400" />
        </div>
      </div>
    </div>
  )
}