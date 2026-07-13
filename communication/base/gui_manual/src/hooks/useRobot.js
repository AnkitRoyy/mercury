import { useState, useEffect, useRef } from 'react'

const WS_URL =
  import.meta.env.VITE_WS_URL ||
  `ws://${window.location.hostname}:9090`

const EMPTY_STATE = {
  encoders: {
    names: [],
    position: [],
    velocity: [],
  },
  ages: {
    lane_video: null,
    turret_video: null,
  },
}

export function useRobot() {
  const [connected, setConnected] = useState(false)
  const [state, setState] = useState(EMPTY_STATE)

  const ws = useRef(null)
  const reconnect = useRef(null)

  useEffect(() => {
    let stop = false

    function connect() {
      ws.current = new WebSocket(WS_URL)

      ws.current.onopen = () => setConnected(true)

      ws.current.onclose = () => {
        setConnected(false)
        if (!stop)
          reconnect.current = setTimeout(connect, 1000)
      }

      ws.current.onerror = () => ws.current.close()

      ws.current.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data)

          setState({
            encoders: msg.encoders ?? EMPTY_STATE.encoders,
            ages: msg.ages ?? EMPTY_STATE.ages,
          })
        } catch (e) {
          console.error(e)
        }
      }
    }

    connect()

    return () => {
      stop = true
      clearTimeout(reconnect.current)
      ws.current?.close()
    }
  }, [])

  return { connected, state }
}