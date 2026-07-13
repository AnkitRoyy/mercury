import { useState, useEffect } from 'react'

const fmt = (value, digits = 1, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

export default function ServoPanel({ servo, sendCommand }) {
  const [pan, setPan] = useState(servo?.pan ?? 0)
  const [tilt, setTilt] = useState(servo?.tilt ?? 0)

  // Keep sliders in sync if the backend reports an authoritative value
  // (e.g. after a reconnect) without fighting the user mid-drag.
  useEffect(() => { setPan(servo?.pan ?? 0) }, [servo?.pan])
  useEffect(() => { setTilt(servo?.tilt ?? 0) }, [servo?.tilt])

  const handlePan = (value) => {
    setPan(value)
    sendCommand({ type: 'SERVO', pan: value, tilt })
  }

  const handleTilt = (value) => {
    setTilt(value)
    sendCommand({ type: 'SERVO', pan, tilt: value })
  }

  const center = () => {
    setPan(0)
    setTilt(0)
    sendCommand({ type: 'SERVO', pan: 0, tilt: 0 })
  }

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-3 h-full">
      <div className="flex items-center justify-between shrink-0">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Turret Servo</span>
        <button
          onClick={center}
          className="text-[10px] font-bold px-2 py-0.5 rounded border border-zinc-700 bg-zinc-900 text-zinc-300 hover:bg-zinc-800"
        >
          CENTER
        </button>
      </div>

      <div className="flex-1 min-h-0 grid grid-cols-2 gap-4">
        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-xs text-zinc-500">Pan</span>
            <span className="text-xs font-mono text-white">{fmt(pan, 1, '°')}</span>
          </div>
          <input
            type="range"
            min={-90}
            max={90}
            step={1}
            value={pan}
            onChange={(e) => handlePan(Number(e.target.value))}
            className="w-full accent-emerald-400"
          />
        </div>

        <div className="flex flex-col gap-1">
          <div className="flex items-center justify-between">
            <span className="text-xs text-zinc-500">Tilt</span>
            <span className="text-xs font-mono text-white">{fmt(tilt, 1, '°')}</span>
          </div>
          <input
            type="range"
            min={-45}
            max={45}
            step={1}
            value={tilt}
            onChange={(e) => handleTilt(Number(e.target.value))}
            className="w-full accent-emerald-400"
          />
        </div>
      </div>
    </div>
  )
}
