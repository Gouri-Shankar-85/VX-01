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
  const [stopConfirm, setStopConfirm] = useState(false);  // 2-step confirm for stop-all

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
  });

  const [victimData, setVictimData] = useState([]);
  const [logs, setLogs] = useState([]);
  const [mapImage, setMapImage] = useState(null);
  const [mapMetadata, setMapMetadata] = useState(null);

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
      messageType: "vx01_msgs/msg/MissionState",
    });
    missionState.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, mission_phase: msg.mission_phase }));
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
      messageType: "vx01_msgs/msg/RobotMode",
    });
    robotModeTopic.subscribe((msg) => {
      setTelemetry((t) => ({ ...t, robot_mode: msg.mode }));
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
      ros.close();
      if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    };
  }, []);

  const sendHexapodCmd = (lx, ly, az) => {
    if (!rosRef.current || !rosConnected) return;
    const cmdVel = new ROSLIB.Topic({ ros: rosRef.current, name: "/cmd_vel", messageType: "geometry_msgs/msg/Twist" });
    cmdVel.publish(new ROSLIB.Message({
      linear: { x: lx * speedMultiplier, y: ly * speedMultiplier, z: 0 },
      angular: { x: 0, y: 0, z: az * speedMultiplier },
    }));
  };

  const stopHexapod = () => {
    if (!rosRef.current || !rosConnected) return;
    const cmdVel = new ROSLIB.Topic({ ros: rosRef.current, name: "/cmd_vel", messageType: "geometry_msgs/msg/Twist" });
    cmdVel.publish(new ROSLIB.Message({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    }));
  };

  const startDroneCmd = (lx, ly, lz, az) => {
    if (!rosRef.current || !rosConnected) return;

    if (droneIntervalRef.current) {
      clearInterval(droneIntervalRef.current);
    }

    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/drone/cmd_vel",
      messageType: "geometry_msgs/msg/TwistStamped"
    });

    const publishCmd = () => {
      const now = Date.now();

      const msg = new ROSLIB.Message({
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: "base_link"
        },
        twist: {
          linear: {
            x: lx * speedMultiplier,
            y: ly * speedMultiplier,
            z: lz * speedMultiplier
          },
          angular: {
            x: 0,
            y: 0,
            z: az * speedMultiplier
          }
        }
      });

      topic.publish(msg);
    };

    droneIntervalRef.current = setInterval(publishCmd, 50);
  };

  const stopDrone = () => {
    if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    startDroneCmd(0, 0, 0, 0);
  };

  const isArmedRef = useRef(false);
  const [droneArmed, setDroneArmed] = useState(false);

  const armDrone = () => {
    if (!rosRef.current || !rosConnected) return;

    const modeSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/set_mode", serviceType: "mavros_msgs/srv/SetMode" });
    const armSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/cmd/arming", serviceType: "mavros_msgs/srv/CommandBool" });

    modeSvc.callService(new ROSLIB.ServiceRequest({ custom_mode: "GUIDED" }), () => {
      setTimeout(() => {
        armSvc.callService(new ROSLIB.ServiceRequest({ value: true }), (res) => {
          if (res.success) {
            addLog("INFO", "Armed — drone ready for manual control");
            startDroneCmd(0, 0, 0, 0);
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
        method: "POST", headers: { "Content-Type": "application/json" }, body: "{}"
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
        body: JSON.stringify({ command_id, command_string })
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
          <h1 className="text-3xl font-black text-transparent bg-clip-text bg-gradient-to-r from-blue-400 to-indigo-600 mb-8 tracking-wider">
            VELATRIX
          </h1>

          <div className="mb-8">
            <p className="text-gray-500 text-xs font-bold tracking-widest uppercase mb-3 text-center">Active Robots</p>
            {robots.map((robot, i) => (
              <div key={i} className="flex justify-between items-center bg-gray-800/50 backdrop-blur-md border border-gray-700 p-3 rounded-lg cursor-default shadow-inner">
                <div className="flex items-center gap-3">
                  <span className={`h-2.5 w-2.5 rounded-full shadow-lg ${robot.status === "online" ? "bg-emerald-400 shadow-emerald-400/50 animate-pulse" : "bg-rose-500 shadow-rose-500/50"}`} />
                  <span className="font-bold tracking-wide uppercase text-sm">{robot.name}</span>
                </div>
                <span className={`text-[10px] font-black px-2 py-0.5 rounded border ${robot.status === "online" ? "text-emerald-400 border-emerald-500/30 bg-emerald-500/10" : "text-rose-400 border-rose-500/30 bg-rose-500/10"}`}>
                  {robot.status === "online" ? "ONLINE" : "OFFLINE"}
                </span>
              </div>
            ))}
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

          <div className="flex gap-4">
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3 shadow-lg">
              <Battery size={16} className={telemetry.battery === "--" ? "text-gray-500" : parseInt(telemetry.battery) < 20 ? "text-rose-500" : "text-emerald-400"} />
              <span className="font-mono font-bold text-gray-100">{telemetry.battery}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3 shadow-lg">
              <Activity size={16} className={telemetry.altitude === "--" ? "text-gray-500" : "text-blue-400"} />
              <span className="font-mono font-bold text-gray-100">{telemetry.altitude}</span>
            </div>
            <div className="bg-gray-800/80 backdrop-blur-sm border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3 shadow-lg">
              <MapPin size={16} className={telemetry.latitude === "--" ? "text-gray-500" : "text-indigo-400"} />
              <span className="font-mono text-gray-100 text-sm">{telemetry.latitude}, {telemetry.longitude}</span>
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
                    {[
                      { id: "sim", label: "1. BOOT HYBRID SIMULATOR", icon: "", cmd: "ros2 launch vx01_bringup vx01_hybrid_sim.launch.py", color: "blue" },
                      { id: "map", label: "2. INITIALIZE VISUAL SLAM", icon: "", cmd: "ros2 launch vx01_simulation vx01_mapping.launch.py use_sim_time:=true", color: "purple" },
                      { id: "walk", label: "3. START HEXAPOD", icon: "", cmd: "ros2 launch vx01_hexapod_locomotion hexapod.launch.py use_sim_time:=true", color: "teal" },
                    ].map(({ id, label, icon, cmd, color }) => {
                      const isRunning = runningProcesses.includes(id);
                      return (
                        <button
                          key={id}
                          onClick={() => backendLaunch(id, cmd)}
                          className={`w-full p-4 rounded border-l-4 transition-all flex items-center justify-between gap-3
                            ${isRunning
                              ? `bg-${color}-600/20 text-${color}-400 border-${color}-500 ring-1 ring-${color}-500/30`
                              : `bg-gray-800/40 text-gray-400 border-gray-700 hover:bg-gray-800 hover:text-gray-200`}`}
                        >
                          <div className="flex items-center gap-3">
                            <span className="text-xl opacity-80">{icon}</span>
                            <span className="text-xs font-black tracking-tight">{label}</span>
                          </div>
                          {isRunning && <span className="text-[10px] animate-pulse font-black px-2 py-0.5 rounded bg-black/40">READY</span>}
                        </button>
                      );
                    })}

                    <div className="pt-4 border-t border-gray-800 mt-4 space-y-3">
                      <button
                        onClick={() => backendLaunch("auto", "python3 /vx01_ws/src/vx01_bringup/scripts/mission_coordinator.py --ros-args -p use_sim_time:=true")}
                        className={`w-full font-black tracking-widest p-4 rounded transition-all shadow-lg flex items-center justify-center gap-3
                          ${runningProcesses.includes("auto")
                            ? "bg-rose-600 text-white shadow-rose-900/50 ring-2 ring-rose-400 ring-offset-2 ring-offset-gray-900"
                            : "bg-gray-800 text-gray-500 hover:bg-gray-700 border border-gray-700 font-bold"}`}
                      >
                        <span className="text-lg">🤖</span> {runningProcesses.includes("auto") ? "AUTONOMY ACTIVE" : "ENGAGE AUTONOMY"}
                      </button>

                      <button
                        onClick={stopAll}
                        className={`w-full font-black tracking-widest p-4 rounded border-2 transition-all duration-300
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
                  <h2 className="text-2xl font-black tracking-wide">VICTIM IDENTIFICATION REGISTRY</h2>
                  <p className="text-gray-500 font-mono text-sm mt-1">Total Targets Detected: {victimData.length}</p>
                </div>
                {victimData.length > 0 && <div className="px-4 py-1 bg-rose-500 text-white font-mono text-xs font-bold rounded animate-pulse">ACTIVE DETECTIONS</div>}
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

              <div className="bg-gray-900 border border-gray-800 rounded-xl shadow-2xl p-6 mb-4">
                <h3 className="font-bold text-gray-300 mb-3 font-mono">THROTTLE MULTIPLIER (SPEED OVERRIDE)</h3>
                <div className="flex items-center gap-6">
                  <input
                    type="range" min="0.1" max="3.0" step="0.1"
                    value={speedMultiplier}
                    onChange={(e) => setSpeedMultiplier(parseFloat(e.target.value))}
                    className="flex-1 accent-blue-500 bg-gray-800 h-2 rounded-lg cursor-pointer"
                  />
                  <div className="text-2xl font-black text-white w-20 text-right">{speedMultiplier.toFixed(1)}x</div>
                </div>
              </div>

              <div className="grid grid-cols-2 gap-8 items-start">

                {/* Hexapod Control */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl shadow-2xl p-6">
                  <h3 className="font-black text-xl mb-2 text-blue-500 border-b border-gray-800 pb-4">GROUND MODE</h3>
                  <p className="text-gray-500 font-mono text-xs mb-4">First launch hexapod, then enable manual control to move</p>

                  <div className="mb-6">
                    <button
                      onClick={() => setHexapodManualEnabled(!hexapodManualEnabled)}
                      className={`w-full font-black tracking-widest p-3 rounded transition-all ${
                        hexapodManualEnabled
                          ? "bg-emerald-600 text-white border border-emerald-500"
                          : "bg-gray-700 text-gray-400 border border-gray-600 hover:bg-gray-600"
                      }`}
                    >
                      {hexapodManualEnabled ? "✓ MANUAL CONTROL ENABLED" : "ENABLE MANUAL CONTROL"}
                    </button>
                  </div>

                  <p className="text-gray-500 font-mono text-xs mb-8">Click Direction To Cruise, Click Stop To Halt</p>

                  <div className="grid grid-cols-3 gap-4 text-center max-w-xs mx-auto font-mono text-xl">
                    <button onClick={() => sendHexapodCmd(0, 0.2, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">↰</button>
                    <button onClick={() => sendHexapodCmd(0.2, 0, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl border border-gray-700 hover:bg-white hover:text-black transition shadow-inner">↑</button>
                    <button onClick={() => sendHexapodCmd(0, -0.2, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">↱</button>

                    <button onClick={() => sendHexapodCmd(0, 0, 0.6)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">⟲</button>
                    <button onClick={stopHexapod} className="bg-rose-900 text-white font-bold p-6 rounded-xl border border-rose-700 hover:bg-rose-600 transition flex items-center justify-center shadow-lg"><StopCircle size={28} /></button>
                    <button onClick={() => sendHexapodCmd(0, 0, -0.6)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">⟳</button>

                    <div></div>
                    <button onClick={() => sendHexapodCmd(-0.2, 0, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl border border-gray-700 hover:bg-white hover:text-black transition shadow-inner">↓</button>
                    <div></div>
                  </div>

                  <div className="mt-6 border-t border-gray-800 pt-4">
                    <button
                      onClick={() => {
                        if (!rosRef.current || !rosConnected) return;
                        stopHexapod();
                        const goHome = new ROSLIB.Topic({ ros: rosRef.current, name: "/hexapod/go_home", messageType: "std_msgs/msg/Empty" });
                        goHome.publish(new ROSLIB.Message({}));
                        addLog("INFO", "Go-home command sent to hexapod.");
                      }}
                      className="w-full bg-amber-900/40 border border-amber-700 text-amber-400 p-3 rounded-lg hover:bg-amber-700 hover:text-white transition font-mono text-sm font-bold tracking-widest"
                    >
                      ⌂ SEND TO HOME POSITION
                    </button>
                  </div>
                </div>

                {/* Drone Control */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl shadow-2xl p-6">
                  <h3 className="font-black text-xl mb-2 text-indigo-400 border-b border-gray-800 pb-4">AERIAL MODE</h3>
                  <p className="text-gray-500 font-mono text-xs mb-8">Click Direction To Cruise, Click Stop To Hover</p>

                  <div className="grid grid-cols-2 gap-8 font-mono text-sm max-w-sm mx-auto">
                    <div className="flex flex-col items-center">
                      <p className="text-gray-500 mb-4 whitespace-nowrap">THROTTLE/YAW</p>
                      <div className="grid grid-cols-3 gap-2">
                        <div></div>
                        <button onClick={() => startDroneCmd(0, 0, 0.5, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">UP</button>
                        <div></div>
                        <button onClick={() => startDroneCmd(0, 0, 0, 0.5)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">⟲</button>
                        <button onClick={stopDrone} className="bg-rose-900 text-white p-4 rounded hover:bg-rose-600 transition flex justify-center"><StopCircle size={20} /></button>
                        <button onClick={() => startDroneCmd(0, 0, 0, -0.5)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">⟳</button>
                        <div></div>
                        <button onClick={() => startDroneCmd(0, 0, -0.5, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">DN</button>
                        <div></div>
                      </div>
                    </div>

                    <div className="flex flex-col items-center">
                      <p className="text-gray-500 mb-4 whitespace-nowrap">PITCH/ROLL</p>
                      <div className="grid grid-cols-3 gap-2">
                        <div></div>
                        <button onClick={() => startDroneCmd(1.0, 0, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">FWD</button>
                        <div></div>
                        <button onClick={() => startDroneCmd(0, 0.5, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">LF</button>
                        <button onClick={stopDrone} className="bg-rose-900 text-white p-4 rounded hover:bg-rose-600 transition flex justify-center"><StopCircle size={20} /></button>
                        <button onClick={() => startDroneCmd(0, -0.5, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">RT</button>
                        <div></div>
                        <button onClick={() => startDroneCmd(-1.0, 0, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">BCK</button>
                        <div></div>
                      </div>
                    </div>
                  </div>

                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-8 font-mono text-sm max-w-sm mx-auto">

                    <button
                      onClick={armDrone}
                      className="bg-red-900/40 border border-red-800 text-red-500 p-3 rounded hover:bg-red-800 hover:text-white transition font-black tracking-widest">
                      {droneArmed ? "ARMED" : "ARM"}
                    </button>

                    <button
                      onClick={disarmDrone}
                      className="bg-orange-900/40 border border-orange-800 text-orange-400 p-3 rounded hover:bg-orange-800 hover:text-white transition font-black tracking-widest">
                      LAND + DISARM
                    </button>

                    <button onClick={() => {
                      if (!rosRef.current || !rosConnected) return;
                      addLog("INFO", "Takeoff sequence initiated...");
                      let t = 0;
                      const topic = new ROSLIB.Topic({
                        ros: rosRef.current,
                        name: "/drone/cmd_vel",
                        messageType: "geometry_msgs/msg/TwistStamped"
                      });
                      const climbInterval = setInterval(() => {
                        const now = Date.now();
                        topic.publish(new ROSLIB.Message({
                          header: {
                            stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 },
                            frame_id: "base_link"
                          },
                          twist: {
                            linear: { x: 0, y: 0, z: 0.5 },
                            angular: { x: 0, y: 0, z: 0 }
                          }
                        }));
                        if (++t >= 10) {
                          clearInterval(climbInterval);
                          addLog("INFO", "Takeoff complete — altitude locked");
                        }
                      }, 1000);
                    }} className="bg-indigo-900/40 border border-indigo-800 text-indigo-400 p-3 rounded hover:bg-indigo-800 hover:text-white transition font-black tracking-widest">
                      CLIMB TO 5M
                    </button>

                    <button onClick={() => {
                      stopDrone();
                      const modeSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/set_mode", serviceType: "mavros_msgs/srv/SetMode" });
                      modeSvc.callService(new ROSLIB.ServiceRequest({ custom_mode: "LAND" }), () => addLog("INFO", "Executing LAND sequence"));
                    }} className="bg-gray-800 border border-gray-700 text-gray-400 p-3 rounded hover:bg-gray-600 hover:text-white transition font-black tracking-widest">
                      PERFORM LAND
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