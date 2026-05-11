import React, { useState, useEffect, useRef } from "react";
import { Battery, MapPin, Activity, Wifi, WifiOff, Map, Target, Settings, Terminal, Crosshair, AlertTriangle, StopCircle } from "lucide-react";
import ROSLIB from "roslib";

const ROS_URL = `ws://${window.location.hostname}:9090`;

export default function Dashboard() {
  const [tab, setTab] = useState("mission");
  const [activeRobot, setActiveRobot] = useState("VX-01");
  const [rosConnected, setRosConnected] = useState(false);
  const [robotActive, setRobotActive] = useState(false);
  const [speedMultiplier, setSpeedMultiplier] = useState(1.0);
  const [runningProcesses, setRunningProcesses] = useState([]);
  const [stopConfirm, setStopConfirm] = useState(false);
  const [isHardwareMode, setIsHardwareMode] = useState(false);

  const [telemetry, setTelemetry] = useState({
    battery: "--",
    latitude: "--",
    longitude: "--",
    altitude: "--",
    speed: "--",
    cpu: "--",
    mode: "--",
    mission_phase: "IDLE",
    victims: 0,
    terrain: "UNKNOWN",
    walkability: "--",
    robot_mode: "UNKNOWN",
    rel_alt: "--",
    heading: "--",
    satellites: "--",
    roll: 0,
    pitch: 0,
  });

  const [victimData, setVictimData] = useState([]);
  const [logs, setLogs] = useState([]);
  const [mapImage, setMapImage] = useState(null);
  const [mapMetadata, setMapMetadata] = useState(null);
  const [droneRCState, setDroneRCState] = useState({ roll: 1500, pitch: 1500, throttle: 1000, yaw: 1500 });
  const [hexapodState, setHexapodState] = useState({ vx: 0.0, vy: 0.0, omega: 0.0 });

  const rosRef = useRef(null);
  const droneIntervalRef = useRef(null);
  const canvasRef = useRef(null);
  const mapSubscriptionRef = useRef(null);
  const currentCmdRef = useRef({ lx: 0, ly: 0, lz: 0, az: 0 });

  const robots = [
    { name: "VX-01", status: robotActive ? "online" : "offline" },
  ];

  const addLog = (level, msg) => {
    const ts = new Date().toLocaleTimeString();
    setLogs((prev) => [...prev.slice(-49), { level, msg, ts }]);
  };

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROS_URL });
    rosRef.current = ros;

    ros.on("connection", () => {
      setRosConnected(true);
      addLog("INFO", `Uplink established at ${ROS_URL}`);

      // Force MAVROS to stream Data
      const svc = new ROSLIB.Service({ ros, name: "/mavros/set_stream_rate", serviceType: "mavros_msgs/srv/StreamRate" });
      svc.callService(new ROSLIB.ServiceRequest({ stream_id: 0, message_rate: 10, on_off: true }), () => {
        addLog("INFO", "Telemetry stream explicitly requested.");
      });
    });

    ros.on("error", () => {
      setRosConnected(false);
      setRobotActive(false);
    });

    ros.on("close", () => {
      setRosConnected(false);
      setRobotActive(false);
      addLog("WARN", "Uplink Disconnected — Standby...");
    });

    const battery = new ROSLIB.Topic({
      ros,
      name: "/mavros/battery",
      messageType: "sensor_msgs/BatteryState",
    });
    battery.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, battery: `${Math.round(msg.percentage * 100)}%` }));
    });

    const gps = new ROSLIB.Topic({
      ros,
      name: "/mavros/global_position/global",
      messageType: "sensor_msgs/NavSatFix",
    });
    gps.subscribe((msg) => {
      setTelemetry((t) => ({
        ...t,
        latitude: msg.latitude.toFixed(5),
        longitude: msg.longitude.toFixed(5),
        altitude: `${msg.altitude.toFixed(1)}m`,
      }));
    });

    const relAlt = new ROSLIB.Topic({
      ros,
      name: "/mavros/global_position/rel_alt",
      messageType: "std_msgs/Float64",
    });
    relAlt.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, rel_alt: `${msg.data.toFixed(1)}m` }));
    });

    const compass = new ROSLIB.Topic({
      ros,
      name: "/mavros/global_position/compass_hdg",
      messageType: "std_msgs/Float64",
    });
    compass.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, heading: `${msg.data.toFixed(0)}°` }));
    });

    const sats = new ROSLIB.Topic({
      ros,
      name: "/mavros/global_position/raw/satellites",
      messageType: "std_msgs/UInt32",
    });
    sats.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, satellites: msg.data }));
    });

    const imu = new ROSLIB.Topic({
      ros,
      name: "/mavros/imu/data",
      messageType: "sensor_msgs/Imu",
    });
    imu.subscribe((msg) => {
      const q = msg.orientation;
      const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      let roll = Math.atan2(sinr_cosp, cosr_cosp);

      const sinp = 2 * (q.w * q.y - q.z * q.x);
      let pitch;
      if (Math.abs(sinp) >= 1) pitch = Math.sign(sinp) * Math.PI / 2;
      else pitch = Math.asin(sinp);

      roll = (roll * 180) / Math.PI;
      pitch = (pitch * 180) / Math.PI;

      setTelemetry((t) => ({ ...t, roll, pitch }));
    });

    const velocity = new ROSLIB.Topic({
      ros,
      name: "/mavros/local_position/velocity_local",
      messageType: "geometry_msgs/TwistStamped",
    });
    velocity.subscribe((msg) => {
      const vx = msg.twist.linear.x;
      const vy = msg.twist.linear.y;
      setTelemetry((t) => ({ ...t, speed: `${Math.sqrt(vx * vx + vy * vy).toFixed(1)} m/s` }));
    });

    const cmdVelSub = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    cmdVelSub.subscribe((msg) => {
      // Only update speed if velocity_local isn't providing data (simple heuristic)
      const speed = Math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2);
      if (speed > 0) {
        setTelemetry((t) => ({ ...t, speed: `${speed.toFixed(1)} m/s` }));
      }
    });

    const state = new ROSLIB.Topic({
      ros,
      name: "/mavros/state",
      messageType: "mavros_msgs/State",
    });
    state.subscribe((msg) => {
      setRobotActive(msg.connected);
      setTelemetry((t) => ({ ...t, mode: msg.mode }));
      // Update ref for forceArm's polling loop (avoids stale closure)
      isArmedRef.current = msg.armed;
      setDroneArmed(msg.armed);
    });

    const missionState = new ROSLIB.Topic({
      ros,
      name: "/mission_state",
      messageType: "std_msgs/String",
    });
    missionState.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, mission_phase: msg.data }));
    });

    const victimDet = new ROSLIB.Topic({
      ros,
      name: "/victim_detections",
      messageType: "vx01_msgs/msg/VictimArray",
    });
    victimDet.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, victims: msg.victims.length }));
      setVictimData((prev) => {
        const currentMap = new Map(prev.map(v => [v.id, v]));
        msg.victims.forEach(v => currentMap.set(v.id, v));
        return Array.from(currentMap.values()).sort((a, b) => b.id - a.id);
      });
    });

    const terrainType = new ROSLIB.Topic({
      ros,
      name: "/terrain_type",
      messageType: "vx01_msgs/msg/Terrain",
    });
    terrainType.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, terrain: msg.terrain_type }));
    });

    const walkScore = new ROSLIB.Topic({
      ros,
      name: "/walkability_score",
      messageType: "vx01_msgs/msg/Walkability",
    });
    walkScore.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, walkability: `${(msg.score * 100).toFixed(0)}%` }));
    });

    const robotModeTopic = new ROSLIB.Topic({
      ros,
      name: "/robot_mode",
      messageType: "std_msgs/String",
    });
    robotModeTopic.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, robot_mode: msg.data }));
    });

    // Subscribe to RTAB-Map occupancy grid
    const mapTopic = new ROSLIB.Topic({
      ros,
      name: "/map",
      messageType: "nav_msgs/OccupancyGrid",
    });
    mapTopic.subscribe((msg) => {
      setMapMetadata({
        width: msg.info.width,
        height: msg.info.height,
        resolution: msg.info.resolution,
        origin: msg.info.origin,
      });
      setMapImage(msg.data);
      mapSubscriptionRef.current = mapTopic;
    });

    return () => {
      battery.unsubscribe();
      gps.unsubscribe();
      velocity.unsubscribe();
      state.unsubscribe();
      missionState.unsubscribe();
      victimDet.unsubscribe();
      terrainType.unsubscribe();
      walkScore.unsubscribe();
      robotModeTopic.unsubscribe();
      mapTopic.unsubscribe();
      relAlt.unsubscribe();
      compass.unsubscribe();
      sats.unsubscribe();
      imu.unsubscribe();
      ros.close();
      if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    };
  }, []);

  const updateHexapod = (dx, dy, da, reset = false) => {
    if (!rosRef.current || !rosConnected) return;
    const newState = reset ? { vx: 0, vy: 0, omega: 0 } : {
      vx: Math.max(-0.2, Math.min(0.2, hexapodState.vx + dx)),
      vy: Math.max(-0.2, Math.min(0.2, hexapodState.vy + dy)),
      omega: Math.max(-0.6, Math.min(0.6, hexapodState.omega + da))
    };
    setHexapodState(newState);
    const cmdVel = new ROSLIB.Topic({ ros: rosRef.current, name: "/cmd_vel", messageType: "geometry_msgs/msg/Twist" });
    cmdVel.publish(new ROSLIB.Message({
      linear: { x: newState.vx, y: newState.vy, z: 0 },
      angular: { x: 0, y: 0, z: newState.omega },
    }));
  };

  const stopHexapod = () => {
    updateHexapod(0, 0, 0, true);
  };

  const updateDroneRC = (dr, dp, dt, dy) => {
    if (!rosRef.current || !rosConnected) return;
    const newState = {
      roll: Math.max(1000, Math.min(2000, droneRCState.roll + dr)),
      pitch: Math.max(1000, Math.min(2000, droneRCState.pitch + dp)),
      throttle: Math.max(1000, Math.min(2000, droneRCState.throttle + dt)),
      yaw: Math.max(1000, Math.min(2000, droneRCState.yaw + dy))
    };
    setDroneRCState(newState);
    
    if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);

    const rcTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/mavros/rc/override",
      messageType: "mavros_msgs/OverrideRCIn"
    });

    const publishRC = () => {
      const channels = Array(18).fill(0);
      channels[0] = newState.roll;
      channels[1] = newState.pitch;
      channels[2] = newState.throttle;
      channels[3] = newState.yaw;
      rcTopic.publish(new ROSLIB.Message({ channels }));
    };

    droneIntervalRef.current = setInterval(publishRC, 50);
  };

  const stopDrone = () => {
    if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    const rcTopic = new ROSLIB.Topic({ ros: rosRef.current, name: "/mavros/rc/override", messageType: "mavros_msgs/OverrideRCIn" });
    const channels = Array(18).fill(0);
    channels[0] = 1500; channels[1] = 1500; channels[2] = 1000; channels[3] = 1500;
    rcTopic.publish(new ROSLIB.Message({ channels }));
    setDroneRCState({ roll: 1500, pitch: 1500, throttle: 1000, yaw: 1500 });
  };

  const isArmedRef = useRef(false);
  const [droneArmed, setDroneArmed] = useState(false);

  const armDrone = () => {
    if (!rosRef.current || !rosConnected) return;

    // Start publishing RC overrides so Pixhawk sees valid RC signals before arming!
    updateDroneRC(0, 0, 0, 0);

    const modeSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/set_mode", serviceType: "mavros_msgs/srv/SetMode" });
    const armSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/cmd/arming", serviceType: "mavros_msgs/srv/CommandBool" });

    modeSvc.callService(new ROSLIB.ServiceRequest({ custom_mode: "STABILIZE" }), () => {
      setTimeout(() => {
        armSvc.callService(new ROSLIB.ServiceRequest({ value: true }), (res) => {
          if (res.success) {
            addLog("INFO", "Armed — drone ready for manual control");
          } else {
            addLog("ERROR", "Arming rejected by Pixhawk (Check RC or pre-arm checks)");
          }
        });
      }, 1000);
    });
  };

  const disarmDrone = () => {
    if (!rosRef.current || !rosConnected) return;

    const armSvc = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/mavros/cmd/arming",
      serviceType: "mavros_msgs/srv/CommandBool"
    });

    const modeSvc = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/mavros/set_mode",
      serviceType: "mavros_msgs/srv/SetMode"
    });

    stopDrone();

    modeSvc.callService(new ROSLIB.ServiceRequest({ custom_mode: "LAND" }), () => {
      setTimeout(() => {
        armSvc.callService(new ROSLIB.ServiceRequest({ value: false }), () => { });
      }, 3000);
    });
  };


  // Poll process status from backend every 2s
  useEffect(() => {
    const poll = setInterval(async () => {
      try {
        const res = await fetch(`http://${window.location.hostname}:3001/api/status`);
        const data = await res.json();
        setRunningProcesses(data.running || []);
      } catch (_) { /* backend not up yet */ }
    }, 2000);
    return () => clearInterval(poll);
  }, []);

  const stopAll = async () => {
    if (!stopConfirm) {
      // First click: arm the button (show confirmation state for 4s then reset)
      setStopConfirm(true);
      setTimeout(() => setStopConfirm(false), 4000);
      addLog("WARN", "STOP ALL armed — click again within 4s to confirm shutdown");
      return;
    }
    // Second click: fire the shutdown
    setStopConfirm(false);
    addLog("WARN", " EMERGENCY SHUTDOWN — terminating all simulation processes...");
    try {
      const res = await fetch(`http://${window.location.hostname}:3001/api/stop-all`, {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ container_name: isHardwareMode ? 'vx01-robot' : 'vx01-dev' })
      });
      const data = await res.json();
      addLog("INFO", data.message || "Shutdown initiated");
      setRunningProcesses([]);
      setRobotActive(false);
      // Status display will update via polling
    } catch (err) {
      addLog("ERROR", "Could not reach backend for stop-all");
    }
  };

  const backendLaunch = async (command_id, command_string) => {
    try {
      const res = await fetch(`http://${window.location.hostname}:3001/api/launch`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ command_id, command_string, container_name: isHardwareMode ? 'vx01-robot' : 'vx01-dev' })
      });
      const data = await res.json();
      if (!res.ok) addLog("ERROR", data.error || "Failed to launch process");
      else addLog("INFO", data.message);
    } catch (err) {
      addLog("ERROR", "Backend unreachable");
    }
  };

  const logColor = { INFO: "text-emerald-400", WARN: "text-yellow-400", ERROR: "text-red-500", FATAL: "text-rose-600", DEBUG: "text-gray-500" };

  return (
    <div className="h-screen flex bg-gray-950 text-gray-200 font-sans">
      {/* Sidebar */}
      <div className="w-64 bg-gray-900 border-r border-gray-800 p-5 flex flex-col justify-between shadow-2xl z-10">
        <div>
          <h1 className="text-2xl font-bold text-gray-100 mb-8 tracking-tight">
            VX-01 MISSION CONTROL
          </h1>

          <div className="mb-8">
            <p className="text-gray-500 text-xs font-bold tracking-widest uppercase mb-3 text-center">Active Robots</p>
            {robots.map((robot, i) => (
              <div key={i} className="flex justify-between items-center bg-gray-800/50 backdrop-blur-md border border-gray-700 p-3 rounded-lg cursor-default shadow-inner">
                <div className="flex items-center gap-3">
                  <span className={`h-2.5 w-2.5 rounded-full shadow-lg ${robot.status === "online" ? "bg-emerald-400 shadow-emerald-400/50 animate-pulse" : "bg-rose-500 shadow-rose-500/50"}`} />
                  <span className="font-bold tracking-wide uppercase text-sm">{robot.name}</span>
                </div>
                <span className={`text-[10px] font-bold px-2 py-0.5 rounded border ${robot.status === "online" ? "text-emerald-400 border-emerald-500/30 bg-emerald-500/10" : "text-rose-400 border-rose-500/30 bg-rose-500/10"}`}>
                  {robot.status === "online" ? "ONLINE" : "OFFLINE"}
                </span>
              </div>
            ))}
          </div>

          <div className="mb-8">
            <p className="text-gray-500 text-xs font-bold tracking-widest uppercase mb-3 text-center">Operation Mode</p>
            <div className="flex gap-1 bg-gray-950 p-1 rounded-xl border border-gray-800">
              <button 
                onClick={() => setIsHardwareMode(false)}
                className={`flex-1 py-2 text-[10px] font-bold rounded-lg transition-all ${!isHardwareMode ? 'bg-blue-600 text-white shadow-lg' : 'text-gray-500 hover:text-gray-300'}`}
              >
                SIMULATION
              </button>
              <button 
                onClick={() => setIsHardwareMode(true)}
                className={`flex-1 py-2 text-[10px] font-bold rounded-lg transition-all ${isHardwareMode ? 'bg-amber-600 text-white shadow-lg' : 'text-gray-500 hover:text-gray-300'}`}
              >
                HARDWARE
              </button>
            </div>
          </div>

          <div className="space-y-2">
            <p className="text-gray-500 text-xs font-bold tracking-widest uppercase mb-3 text-center">Modules</p>
            <button onClick={() => setTab("mission")} className={`w-full flex items-center gap-3 p-3 rounded-lg transition-all ${tab === "mission" ? "bg-blue-600 text-white shadow-lg shadow-blue-900/50" : "hover:bg-gray-800"}`}>
              <Map size={18} /> Mission Control
            </button>
            <button onClick={() => setTab("victims")} className={`w-full flex items-center gap-3 p-3 rounded-lg transition-all ${tab === "victims" ? "bg-blue-600 text-white shadow-lg shadow-blue-900/50" : "hover:bg-gray-800"}`}>
              <Crosshair size={18} /> Victim Database
            </button>
            <button onClick={() => setTab("teleop")} className={`w-full flex items-center gap-3 p-3 rounded-lg transition-all ${tab === "teleop" ? "bg-blue-600 text-white shadow-lg shadow-blue-900/50" : "hover:bg-gray-800"}`}>
              <Settings size={18} /> Manual Deck
            </button>
            <button onClick={() => setTab("mapping")} className={`w-full flex items-center gap-3 p-3 rounded-lg transition-all ${tab === "mapping" ? "bg-blue-600 text-white shadow-lg shadow-blue-900/50" : "hover:bg-gray-800"}`}>
              <Map size={18} /> Live Mapping
            </button>
            <button onClick={() => setTab("logs")} className={`w-full flex items-center gap-3 p-3 rounded-lg transition-all ${tab === "logs" ? "bg-blue-600 text-white shadow-lg shadow-blue-900/50" : "hover:bg-gray-800"}`}>
              <Terminal size={18} /> System Logs
            </button>
          </div>
        </div>

        <div className="bg-gray-950 border border-gray-800 p-4 rounded-xl text-xs space-y-2">
          <div className={`flex items-center gap-2 ${rosConnected ? "text-emerald-400" : "text-rose-500"}`}>
            {rosConnected ? <Wifi size={14} className="animate-pulse" /> : <WifiOff size={14} />}
            <span className="font-mono tracking-wider">{rosConnected ? "UPLINK SECURE" : "UPLINK SEVERED"}</span>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex flex-col h-full overflow-hidden">

        {/* Top Telemetry Dashboard */}
        <div className="bg-gray-900 border-b border-gray-800 p-4 shadow-md z-0 flex justify-between items-center">
          <div className="flex gap-6">
            <div className="flex flex-col">
              <span className="text-gray-500 text-[10px] uppercase font-bold tracking-wider">Asset</span>
              <span className="font-mono font-bold text-lg text-blue-400">VX-01</span>
            </div>
            <div className="w-px h-10 bg-gray-700"></div>
            <div className="flex flex-col">
              <span className="text-gray-500 text-[10px] uppercase font-bold tracking-wider">Mission Phase</span>
              <span className={`font-mono font-bold text-lg ${telemetry.mission_phase === 'VICTIM_FOUND' ? 'text-rose-400 animate-pulse' : 'text-gray-200'}`}>{telemetry.mission_phase}</span>
            </div>
            <div className="w-px h-10 bg-gray-700"></div>
            <div className="flex flex-col">
              <span className="text-gray-500 text-[10px] uppercase font-bold tracking-wider">Chassis Mode</span>
              <span className="font-mono font-bold text-lg text-amber-400">{telemetry.robot_mode}</span>
            </div>
            <div className="w-px h-10 bg-gray-700"></div>
            <div className="flex flex-col">
              <span className="text-gray-500 text-[10px] uppercase font-bold tracking-wider">Current Speed</span>
              <span className="font-mono font-bold text-lg text-gray-200">{telemetry.speed}</span>
            </div>
          </div>

          <div className="flex gap-3">
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-3 py-1.5 rounded-lg flex items-center gap-2 shadow-lg">
              <Battery size={14} className={telemetry.battery === "--" ? "text-gray-500" : parseInt(telemetry.battery) < 20 ? "text-rose-500" : "text-emerald-400"} />
              <span className="font-mono text-sm font-bold text-gray-100">{telemetry.battery}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-3 py-1.5 rounded-lg flex items-center gap-2 shadow-lg">
              <Activity size={14} className={telemetry.rel_alt === "--" ? "text-gray-500" : "text-blue-400"} />
              <span className="font-mono text-sm font-bold text-gray-100" title="Relative Altitude">Alt: {telemetry.rel_alt}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-3 py-1.5 rounded-lg flex items-center gap-2 shadow-lg">
              <Target size={14} className={telemetry.heading === "--" ? "text-gray-500" : "text-amber-400"} />
              <span className="font-mono text-sm font-bold text-gray-100" title="Compass Heading">Hdg: {telemetry.heading}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-3 py-1.5 rounded-lg flex items-center gap-2 shadow-lg">
              <Wifi size={14} className={telemetry.satellites === "--" ? "text-gray-500" : telemetry.satellites < 6 ? "text-rose-500" : "text-emerald-400"} />
              <span className="font-mono text-sm font-bold text-gray-100" title="GPS Satellites">Sats: {telemetry.satellites}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-3 py-1.5 rounded-lg flex items-center gap-2 shadow-lg">
              <MapPin size={14} className={telemetry.latitude === "--" ? "text-gray-500" : "text-indigo-400"} />
              <span className="font-mono text-xs text-gray-100">{telemetry.latitude}, {telemetry.longitude}</span>
            </div>
          </div>
        </div>

        <div className="flex-1 overflow-y-auto p-6 bg-gray-950">

          {tab === "mission" && (
            <div className="grid grid-cols-12 gap-6 h-full">
              <div className="col-span-8 bg-gray-900 border border-gray-800 rounded-xl shadow-2xl flex flex-col overflow-hidden">
                <div className="bg-gray-800 px-4 py-2 border-b border-gray-700 flex justify-between items-center text-sm font-mono text-gray-400">
                  <div className="flex items-center gap-2"><Target size={14} /> RGB OPTICAL ARRAY</div>
                  <span className="text-emerald-400 animate-pulse">● LIVE</span>
                </div>
                <div className="flex-1 bg-black relative flex items-center justify-center">
                  {rosConnected ? (
                    <img
                      src={`http://${window.location.hostname}:8080/stream?topic=/depth_camera/color/image_raw&width=1280&height=720`}
                      className="w-full h-full object-cover"
                      alt="Primary Optical"
                      onError={(e) => { e.target.style.display = 'none'; e.target.nextSibling.style.display = 'flex'; }}
                    />
                  ) : null}
                  <div className={`absolute inset-0 items-center justify-center flex-col text-gray-600 font-mono ${rosConnected ? 'hidden' : 'flex'}`}>
                    <AlertTriangle size={48} className="mb-4 opacity-50" />
                    OPTICAL ARRAY OFFLINE
                  </div>
                </div>
              </div>

              <div className="col-span-4 flex flex-col gap-6">
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-5 shadow-2xl">
                  <h3 className="font-mono text-gray-400 text-sm mb-4 border-b border-gray-800 pb-2">ENVIRONMENTAL ANALYSIS</h3>
                  <div className="space-y-4 font-mono text-sm">
                    <div className="flex justify-between items-center">
                      <span className="text-gray-500">Surface Type:</span>
                      <span className="text-emerald-400 font-bold">{telemetry.terrain}</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-gray-500">Walkability Score:</span>
                      <span className="text-amber-400 font-bold">{telemetry.walkability}</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-gray-500">Victim Count:</span>
                      <span className="text-rose-500 font-bold text-xl">{telemetry.victims}</span>
                    </div>
                  </div>
                </div>

                <div className="bg-gray-900 border border-gray-800 rounded-xl p-5 shadow-2xl flex-1 flex flex-col">
                  <h3 className="font-mono text-gray-400 text-sm mb-4 border-b border-gray-800 pb-2 flex items-center gap-2">
                    <Settings size={14} /> IGNITION SEQUENCE
                  </h3>
                  <div className="space-y-3 font-mono flex-1">
                    {(isHardwareMode ? [
                      { id: "hw_base", label: "1. INITIALIZE HARDWARE", icon: "", cmd: "ros2 launch vx01_bringup vx01.launch.py", color: "amber" },
                      { id: "map", label: "2. INITIALIZE SLAM (RDK)", icon: "", cmd: "ros2 launch vx01_simulation vx01_mapping.launch.py", color: "indigo" },
                      { id: "walk", label: "3. START HEXAPOD GAIT", icon: "", cmd: "ros2 launch vx01_hexapod_locomotion hexapod.launch.py", color: "emerald" },
                      { id: "teleop_hx", label: "4. LAUNCH HEXAPOD TELEOP", icon: "", cmd: "ros2 run vx01_hexapod_locomotion teleop_node", color: "blue" },
                    ] : [
                      { id: "sim", label: "1. BOOT HYBRID SIMULATOR", icon: "", cmd: "ros2 launch vx01_bringup vx01_hybrid_sim.launch.py", color: "blue" },
                      { id: "map", label: "2. INITIALIZE VISUAL SLAM", icon: "", cmd: "ros2 launch vx01_simulation vx01_mapping.launch.py use_sim_time:=true", color: "purple" },
                      { id: "walk", label: "3. START HEXAPOD GAIT", icon: "", cmd: "ros2 launch vx01_hexapod_locomotion hexapod.launch.py use_sim_time:=true", color: "teal" },
                      { id: "teleop_dr", label: "4. LAUNCH DRONE TELEOP", icon: "", cmd: "ros2 run vx01_aerial_control drone_teleop", color: "orange" },
                    ]).map(({ id, label, icon, cmd, color }) => {
                      const isRunning = runningProcesses.includes(id);
                      return (
                        <button
                          key={id}
                          onClick={() => backendLaunch(id, cmd)}
                          className={`w-full p-4 rounded-xl border transition-all flex items-center justify-between group relative overflow-hidden
                            ${isRunning
                              ? `bg-${color}-500/10 text-${color}-400 border-${color}-500/50 shadow-lg shadow-${color}-500/10`
                              : `bg-gray-800/40 text-gray-400 border-gray-800 hover:border-gray-600 hover:bg-gray-800/60`}`}
                        >
                          <div className="flex items-center gap-3 relative z-10">
                            <span className="text-xl group-hover:scale-110 transition-transform">{icon}</span>
                            <span className="text-[10px] font-bold tracking-widest">{label}</span>
                          </div>
                          {isRunning && (
                            <div className="flex items-center gap-2 relative z-10">
                              <span className="h-1.5 w-1.5 rounded-full bg-current animate-ping" />
                              <span className="text-[9px] font-bold tracking-tight">ACTIVE</span>
                            </div>
                          )}
                          {!isRunning && <div className={`absolute inset-y-0 left-0 w-0 bg-${color}-500/10 group-hover:w-full transition-all duration-500`} />}
                        </button>
                      );
                    })}

                    <div className="pt-4 border-t border-gray-800 mt-4 space-y-3">
                      <button
                        onClick={() => backendLaunch("auto", "python3 /vx01_ws/src/vx01_bringup/scripts/mission_coordinator.py --ros-args -p use_sim_time:=true")}
                        className={`w-full font-bold tracking-widest p-4 rounded transition-all shadow-lg flex items-center justify-center gap-3
                          ${runningProcesses.includes("auto")
                            ? "bg-rose-600 text-white shadow-rose-900/50 ring-2 ring-rose-400 ring-offset-2 ring-offset-gray-900"
                            : "bg-gray-800 text-gray-500 hover:bg-gray-700 border border-gray-700 font-bold"}`}
                      >
                        <span className="text-lg">🤖</span> {runningProcesses.includes("auto") ? "AUTONOMY ACTIVE" : "ENGAGE AUTONOMY"}
                      </button>

                      <button
                        onClick={stopAll}
                        className={`w-full font-bold tracking-widest p-4 rounded border-2 transition-all duration-300
                          ${stopConfirm
                            ? "bg-red-600 border-red-400 text-white shadow-lg shadow-red-900/80 scale-[1.02] animate-pulse"
                            : "bg-gray-900 border-gray-700 text-gray-500 hover:border-red-800 hover:text-red-500"
                          }`}
                      >
                        <div className="flex items-center justify-center gap-3">
                          <span className="text-lg">{stopConfirm ? "⚠" : "⏹"}</span>
                          {stopConfirm ? "CONFIRM SHUTDOWN" : "KILL ALL PROCESSES"}
                        </div>
                      </button>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          )}

          {tab === "victims" && (
            <div className="h-full flex flex-col">
              <div className="flex justify-between items-end border-b border-gray-800 pb-4 mb-6">
                <div>
                  <h2 className="text-xl font-bold tracking-tight text-gray-200">VICTIM IDENTIFICATION REGISTRY</h2>
                  <p className="text-gray-500 font-mono text-xs mt-1">Total Targets Detected: {victimData.length}</p>
                </div>
                {victimData.length > 0 && <div className="px-4 py-1 bg-red-600 text-white font-mono text-xs font-bold rounded">ACTIVE DETECTIONS</div>}
              </div>

              {victimData.length === 0 ? (
                <div className="flex-1 flex flex-col items-center justify-center text-gray-600 font-mono">
                  <Crosshair size={64} className="mb-4 opacity-20" />
                  <p>NO IDENTIFICATIONS LOGGED</p>
                </div>
              ) : (
                <div className="grid grid-cols-3 gap-6 auto-rows-max">
                  {victimData.map((v) => (
                    <div key={v.id} className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden shadow-2xl flex flex-col">
                      <div className="bg-gray-800 px-3 py-2 border-b border-gray-700 flex justify-between font-mono text-xs">
                        <span className="text-gray-400">ID-TARGET-{v.id.toString().padStart(4, '0')}</span>
                        <span className="text-emerald-400 font-bold">{(v.confidence * 100).toFixed(1)}% MATCH</span>
                      </div>
                      <div className="h-48 bg-black flex items-center justify-center relative group">
                        {v.image_base64 && v.image_base64 !== "" ? (
                          <img src={`data:image/jpeg;base64,${v.image_base64}`} alt={`Detection ${v.id}`} className="max-h-full max-w-full object-contain" />
                        ) : (
                          <span className="text-red-500 font-mono text-xs border border-red-500 p-1">IMAGE FRAGMENT CORRUPT</span>
                        )}
                        <div className="absolute inset-0 border-2 border-rose-500 opacity-50 pointer-events-none"></div>
                      </div>
                      <div className="p-4 flex flex-col font-mono text-sm space-y-2">
                        <div className="flex justify-between border-b border-gray-800 pb-2">
                          <span className="text-gray-500">Classification:</span>
                          <span className="font-bold text-amber-500 uppercase">{v.label}</span>
                        </div>
                        <div className="flex justify-between border-b border-gray-800 pb-2">
                          <span className="text-gray-500">Relative X:</span>
                          <span className="text-gray-300">{v.position.point.x.toFixed(2)} m</span>
                        </div>
                        <div className="flex justify-between border-b border-gray-800 pb-2">
                          <span className="text-gray-500">Relative Y:</span>
                          <span className="text-gray-300">{v.position.point.y.toFixed(2)} m</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-500">Depth (Z):</span>
                          <span className="text-gray-300">{v.position.point.z.toFixed(2)} m</span>
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}

          {tab === "teleop" && (
            <div className="flex flex-col gap-8 max-w-5xl mx-auto h-full">
              
              <div className="grid grid-cols-2 gap-8 items-stretch">

                {/* Hexapod Control Card */}
                <div className="bg-blue-900/10 border-2 border-blue-800/40 rounded-xl p-8 shadow-xl flex flex-col">
                  <div className="flex justify-between items-center mb-8 pb-4 border-b border-gray-800">
                    <div>
                      <h3 className="font-bold text-xl text-blue-400 tracking-tight">GROUND PROTOCOL</h3>
                      <p className="text-[10px] text-gray-500 font-mono uppercase tracking-widest mt-1">Tripod Locomotion</p>
                    </div>
                    <div className="flex flex-col items-end font-mono text-[10px] text-gray-400 gap-1">
                      <span className="bg-blue-500/10 px-2 py-0.5 rounded border border-blue-500/20">VX: {hexapodState.vx.toFixed(2)}</span>
                      <span className="bg-blue-500/10 px-2 py-0.5 rounded border border-blue-500/20">VY: {hexapodState.vy.toFixed(2)}</span>
                    </div>
                  </div>

                  <div className="flex-1 flex flex-col justify-center">
                    <div className="grid grid-cols-3 gap-4 text-center max-w-[280px] mx-auto font-mono">
                      <button onClick={() => updateHexapod(0, 0.05, 0)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">←</button>
                      <button onClick={() => updateHexapod(0.05, 0, 0)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">↑</button>
                      <button onClick={() => updateHexapod(0, -0.05, 0)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">→</button>

                      <button onClick={() => updateHexapod(0, 0, 0.3)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">⟲</button>
                      <button onClick={stopHexapod} className="bg-red-950 text-white font-bold p-5 rounded-xl border border-red-800 hover:bg-red-800 transition-all flex items-center justify-center shadow-lg"><StopCircle size={24} /></button>
                      <button onClick={() => updateHexapod(0, 0, -0.3)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">⟳</button>

                      <div></div>
                      <button onClick={() => updateHexapod(-0.05, 0, 0)} className="bg-gray-800 text-gray-300 p-5 rounded-xl border border-gray-700 hover:border-blue-500 hover:text-white transition-all text-2xl shadow-inner">↓</button>
                      <div></div>
                    </div>
                  </div>

                  <div className="mt-10 pt-6 border-t border-gray-800">
                    <button
                      onClick={() => {
                        if (!rosRef.current || !rosConnected) return;
                        stopHexapod();
                        const goHome = new ROSLIB.Topic({ ros: rosRef.current, name: "/hexapod/go_home", messageType: "std_msgs/msg/Empty" });
                        goHome.publish(new ROSLIB.Message({}));
                        addLog("INFO", "Executing stance normalization sequence.");
                      }}
                      className="w-full bg-blue-900/20 border border-blue-800/50 text-blue-400 p-4 rounded-lg hover:bg-blue-800 hover:text-white transition-all text-xs font-bold uppercase tracking-widest"
                    >
                      HOME STANCE
                    </button>
                  </div>
                </div>

                {/* Drone Control Card */}
                <div className="bg-indigo-900/10 border-2 border-indigo-800/40 rounded-xl p-8 shadow-xl flex flex-col">
                  <div className="flex justify-between items-center mb-8 pb-4 border-b border-gray-800">
                    <div>
                      <h3 className="font-bold text-xl text-indigo-400 tracking-tight">AERIAL PROTOCOL</h3>
                      <p className="text-[10px] text-gray-500 font-mono uppercase tracking-widest mt-1">Autonomous Guided Flight</p>
                    </div>
                    <div className="flex flex-col items-end font-mono text-[10px] text-gray-400 gap-1">
                      <span className="bg-indigo-500/10 px-2 py-0.5 rounded border border-indigo-500/20">THROTTLE: {droneRCState.throttle}</span>
                      <span className="bg-indigo-500/10 px-2 py-0.5 rounded border border-indigo-500/20">PITCH: {droneRCState.pitch}</span>
                    </div>
                  </div>

                  <div className="flex-1 grid grid-cols-3 gap-4 items-center">
                    <div className="flex flex-col gap-3">
                      <p className="text-[9px] text-center text-gray-600 font-bold uppercase tracking-[0.2em]">Elevation / Yaw</p>
                      <div className="grid grid-cols-3 gap-3 font-mono text-xl">
                        <div />
                        <button onClick={() => updateDroneRC(0, 0, 25, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">↑</button>
                        <div />
                        <button onClick={() => updateDroneRC(0, 0, 0, -25)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">⟲</button>
                        <button onClick={stopDrone} className="bg-red-950 text-white font-bold p-4 rounded-xl border border-red-800 hover:bg-red-800 transition-all flex items-center justify-center shadow-lg"><StopCircle size={20} /></button>
                        <button onClick={() => updateDroneRC(0, 0, 0, 25)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">⟳</button>
                        <div />
                        <button onClick={() => updateDroneRC(0, 0, -25, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">↓</button>
                        <div />
                      </div>
                    </div>

                    <div className="flex flex-col items-center justify-center gap-3">
                      <p className="text-[9px] text-center text-gray-600 font-bold uppercase tracking-[0.2em]">Attitude</p>
                      <div className="relative w-28 h-28 rounded-full overflow-hidden border-4 border-gray-800 bg-sky-400 shadow-inner">
                        <div 
                          className="absolute w-[300%] h-[300%] -left-full -top-full flex flex-col transition-transform duration-75"
                          style={{
                            transform: `rotate(${-telemetry.roll}deg) translateY(${telemetry.pitch * 1.5}px)`,
                            transformOrigin: 'center center'
                          }}
                        >
                          <div className="flex-1 bg-sky-500"></div>
                          <div className="h-0.5 bg-white w-full shadow-lg"></div>
                          <div className="flex-1 bg-amber-700"></div>
                        </div>
                        <div className="absolute inset-0 flex items-center justify-center pointer-events-none drop-shadow-md">
                          <div className="w-12 h-0.5 bg-yellow-400"></div>
                          <div className="absolute w-0.5 h-3 bg-yellow-400 -mt-2"></div>
                        </div>
                      </div>
                      <div className="flex gap-3 text-[10px] font-mono text-gray-400 font-bold tracking-widest">
                        <span>P: {telemetry.pitch.toFixed(1)}°</span>
                        <span>R: {telemetry.roll.toFixed(1)}°</span>
                      </div>
                    </div>

                    <div className="flex flex-col gap-3">
                      <p className="text-[9px] text-center text-gray-600 font-bold uppercase tracking-[0.2em]">Position / Roll</p>
                      <div className="grid grid-cols-3 gap-3 font-mono text-xl">
                        <div />
                        <button onClick={() => updateDroneRC(0, -25, 0, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">↑</button>
                        <div />
                        <button onClick={() => updateDroneRC(-25, 0, 0, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">←</button>
                        <button onClick={stopDrone} className="bg-red-950 text-white font-bold p-4 rounded-xl border border-red-800 hover:bg-red-800 transition-all flex items-center justify-center shadow-lg"><StopCircle size={20} /></button>
                        <button onClick={() => updateDroneRC(25, 0, 0, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">→</button>
                        <div />
                        <button onClick={() => updateDroneRC(0, 25, 0, 0)} className="bg-gray-800 text-gray-300 p-4 rounded-xl border border-gray-700 hover:border-indigo-500 hover:text-white transition-all font-bold shadow-inner flex items-center justify-center">↓</button>
                        <div />
                      </div>
                    </div>
                  </div>

                  <div className="mt-10 grid grid-cols-2 gap-4">
                    <button
                      onClick={armDrone}
                      className={`p-4 rounded-lg border-2 transition-all text-[10px] font-bold uppercase tracking-widest ${droneArmed ? 'bg-red-600 border-red-400 text-white shadow-lg' : 'bg-gray-800 border-gray-700 text-gray-500 hover:text-white'}`}>
                      {droneArmed ? 'SYSTEM ARMED' : 'ARM / TAKEOFF'}
                    </button>
                    <button
                      onClick={disarmDrone}
                      className="bg-gray-800 border-2 border-gray-700 text-gray-400 p-4 rounded-lg hover:bg-gray-700 hover:text-white transition-all text-[10px] font-bold uppercase tracking-widest">
                      Land / Disarm
                    </button>
                  </div>
                </div>

              </div>
            </div>
          )}

          {tab === "mapping" && (
            <div className="flex flex-col gap-4 h-full">
              <div className="bg-gray-900 border border-gray-800 rounded-xl shadow-2xl p-6 flex-1 flex flex-col overflow-hidden">
                <div className="flex justify-between items-center mb-4 pb-3 border-b border-gray-800">
                  <h2 className="text-xl font-black text-teal-400">RTAB-MAP ENVIRONMENT MAPPING</h2>
                  <div className="text-xs font-mono text-gray-500">
                    {mapMetadata ? `${mapMetadata.width}×${mapMetadata.height} | ${(mapMetadata.resolution * 100).toFixed(1)}cm/px` : 'No map data'}
                  </div>
                </div>
                
                <div className="flex-1 bg-black rounded-lg overflow-hidden flex items-center justify-center relative border border-gray-700">
                  {mapMetadata && mapImage ? (
                    <div className="relative w-full h-full">
                      <MapCanvas
                        width={mapMetadata.width}
                        height={mapMetadata.height}
                        resolution={mapMetadata.resolution}
                        data={mapImage}
                        ref={canvasRef}
                      />
                    </div>
                  ) : (
                    <div className="flex flex-col items-center justify-center text-gray-600 font-mono">
                      <Map size={48} className="mb-4 opacity-30" />
                      <p className="text-sm">Waiting for map data from RTAB-Map...</p>
                      <p className="text-xs text-gray-700 mt-2">Launch mapping with "2. INITIALIZE VISUAL SLAM"</p>
                    </div>
                  )}
                </div>
                
                <div className="mt-4 bg-gray-800/50 p-3 rounded text-xs text-gray-400 font-mono">
                  <div className="flex justify-between mb-2">
                    <span>Legend:</span>
                    <span></span>
                  </div>
                  <div className="grid grid-cols-3 gap-2 text-[10px]">
                    <div><span className="inline-block w-3 h-3 bg-black border border-gray-600 mr-1"></span>Free Space</div>
                    <div><span className="inline-block w-3 h-3 bg-gray-500 mr-1"></span>Unknown</div>
                    <div><span className="inline-block w-3 h-3 bg-white mr-1"></span>Obstacle</div>
                  </div>
                </div>
              </div>
            </div>
          )}

          {tab === "logs" && (
            <div className="bg-black text-gray-300 p-6 rounded-xl shadow-lg font-mono text-xs h-full overflow-y-auto border border-gray-800">
              <div className="sticky top-0 bg-black/90 pb-2 mb-4 border-b border-gray-800 flex justify-between items-center backdrop-blur-sm">
                <span className="text-gray-500">RAW TELEMETRY STREAM</span>
                <span className="text-emerald-400 animate-pulse font-bold tracking-widest">{ROS_URL}</span>
              </div>
              {logs.length === 0 && <p className="text-gray-600 animate-pulse">Awaiting data sequence...</p>}
              {logs.map((l, i) => (
                <div key={i} className="mb-1 hover:bg-gray-900 transition px-2 py-0.5 rounded flex gap-4">
                  <span className="text-gray-600 w-24 shrink-0">[{l.ts}]</span>
                  <span className={`${logColor[l.level] || "text-gray-500"} w-16 shrink-0`}>[{l.level}]</span>
                  <span className="break-all">{l.msg}</span>
                </div>
              ))}
            </div>
          )}

        </div>
      </div>
    </div>
  );
}

// MapCanvas component to render occupancy grid
const MapCanvas = React.forwardRef(({ width, height, resolution, data }, ref) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!canvasRef.current || !data) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    
    // Scale to fit container while maintaining aspect ratio
    const maxWidth = 800;
    const maxHeight = 600;
    const scale = Math.min(maxWidth / width, maxHeight / height, 2);
    
    canvas.width = width * scale;
    canvas.height = height * scale;

    // Render the occupancy grid
    const imageData = ctx.createImageData(canvas.width, canvas.height);
    const pixelData = imageData.data;

    for (let i = 0; i < width * height; i++) {
      const value = data[i];
      const scaledIndex = i * Math.pow(scale, 2);
      
      // Map occupancy value to color
      // -1 or 255 = unknown (gray), 0-50 = free (white), 51-100 = occupied (black)
      let color;
      if (value < 0 || value === 255) {
        color = { r: 128, g: 128, b: 128 }; // Unknown - gray
      } else if (value <= 50) {
        color = { r: 255, g: 255, b: 255 }; // Free - white
      } else {
        color = { r: 0, g: 0, b: 0 }; // Occupied - black
      }

      // Fill the scaled pixels
      for (let sy = 0; sy < scale; sy++) {
        for (let sx = 0; sx < scale; sx++) {
          const yi = Math.floor(i / width);
          const xi = i % width;
          const idx = ((yi * scale + sy) * canvas.width + (xi * scale + sx)) * 4;
          
          if (idx < pixelData.length) {
            pixelData[idx] = color.r;
            pixelData[idx + 1] = color.g;
            pixelData[idx + 2] = color.b;
            pixelData[idx + 3] = 255;
          }
        }
      }
    }

    ctx.putImageData(imageData, 0, 0);
  }, [data, width, height, resolution]);

  return <canvas ref={canvasRef} className="max-w-full max-h-full" />;
});

MapCanvas.displayName = "MapCanvas";