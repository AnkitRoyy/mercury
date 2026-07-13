import { useRobot } from './hooks/useRobot'
import CameraFeed from './components/CameraFeed'
import EncoderPanel from './components/EncoderPanel'

export default function App() {
  const { connected, state } = useRobot()

  return (
    <div className="h-screen w-screen bg-neutral-950 text-zinc-100 flex flex-col overflow-hidden font-sans">
      <div className="flex items-center justify-between px-5 py-3 bg-black border-b border-zinc-800 shrink-0">
        <div className="flex items-center gap-3">
          <span className="font-black text-base tracking-wide text-white">Mercury Base Station</span>
          <span className="text-xs text-zinc-500">192.168.88.2</span>
        </div>

        <div className="flex items-center gap-3 text-xs">
          <span className={`px-2.5 py-1 rounded border font-bold ${
            connected ? 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30' : 'bg-red-500/10 text-red-300 border-red-500/30'
          }`}>
            {connected ? 'BRIDGE LIVE' : 'NO BRIDGE'}
          </span>
          <span className="text-zinc-500 font-mono">{new Date().toLocaleTimeString()}</span>
        </div>
      </div>

      <div className="flex-1 min-h-0 overflow-hidden p-3 flex flex-col gap-3 max-w-3xl mx-auto w-full">
        <div className="grid grid-cols-2 gap-3 shrink-0">
          <CameraFeed title="Lane Camera" stream="lane" age={state.ages?.lane_video} />
          <CameraFeed title="Turret Camera" stream="turret" age={state.ages?.turret_video} />
        </div>

        <EncoderPanel encoders={state.encoders} />
      </div>
    </div>
  )
}
