const fmt = (value, digits = 2, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

export default function EncoderPanel({ encoders }) {
  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-2 shrink-0">
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Encoders</span>
        <span className="text-[11px] text-zinc-500">{encoders.names?.length || 0} joints</span>
      </div>
      <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
        <div className="grid grid-cols-2 gap-x-3 gap-y-1 max-h-40 overflow-y-auto">
          {(encoders.names || []).map((name, index) => (
            <div key={`${name}-${index}`} className="flex justify-between gap-2 text-[11px]">
              <span className="text-zinc-500 truncate">{name}</span>
              <span className="text-white font-mono">{fmt(encoders.velocity?.[index], 2)}</span>
            </div>
          ))}
          {(!encoders.names || encoders.names.length === 0) && (
            <span className="text-xs text-zinc-600">Waiting for encoder packets</span>
          )}
        </div>
      </div>
    </div>
  )
}
