import { useState, useEffect, useRef } from "react";
import { Battery, MapPin, Activity, Wifi, WifiOff, Map, Target, Settings, Terminal, Crosshair, AlertTriangle, StopCircle } from "lucide-react";
import ROSLIB from "roslib";

const ROS_URL = `ws://${window.location.hostname}:9090`;

export default function Dashboard() {
  const [tab, setTab] = useState("mission");
  const [activeRobot, setActiveRobot] = useState("VX-01");
  const [rosConnected, setRosConnected] = useState(false);
  const [robotActive, setRobotActive] = useState(false);
  const [speedMultiplier, setSpeedMultiplier] = useState(1.0);

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
  
  const rosRef = useRef(null);
  const droneIntervalRef = useRef(null);
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
        return Array.from(currentMap.values()).sort((a,b) => b.id - a.id);
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
    sendHexapodCmd(0, 0, 0);
  };

  const startDroneCmd = (lx, ly, lz, az) => {
    if (!rosRef.current || !rosConnected) return;
    if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    
    currentCmdRef.current = { lx: lx * speedMultiplier, ly: ly * speedMultiplier, lz: lz * speedMultiplier, az: az * speedMultiplier };
    
    const droneVel = new ROSLIB.Topic({ ros: rosRef.current, name: "/mavros/setpoint_velocity/cmd_vel_unstamped", messageType: "geometry_msgs/msg/Twist" });
    
    droneVel.publish(new ROSLIB.Message({
      linear: { x: currentCmdRef.current.lx, y: currentCmdRef.current.ly, z: currentCmdRef.current.lz },
      angular: { x: 0, y: 0, z: currentCmdRef.current.az },
    }));

    droneIntervalRef.current = setInterval(() => {
      droneVel.publish(new ROSLIB.Message({
        linear: { x: currentCmdRef.current.lx, y: currentCmdRef.current.ly, z: currentCmdRef.current.lz },
        angular: { x: 0, y: 0, z: currentCmdRef.current.az },
      }));
    }, 100);
  };

  const stopDrone = () => {
    if (droneIntervalRef.current) clearInterval(droneIntervalRef.current);
    if (!rosRef.current || !rosConnected) return;
    const droneVel = new ROSLIB.Topic({ ros: rosRef.current, name: "/mavros/setpoint_velocity/cmd_vel_unstamped", messageType: "geometry_msgs/msg/Twist" });
    droneVel.publish(new ROSLIB.Message({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }));
  };

  const forceArm = () => {
    if (!rosRef.current || !rosConnected) return;
    const paramSet = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/param/set", serviceType: "mavros_msgs/srv/ParamSet" });
    paramSet.callService(new ROSLIB.ServiceRequest({ param_id: "ARMING_CHECK", value: { integer: 0, real: 0.0 } }), () => {
      addLog("WARN", "Bypassed arming checks");
      setTimeout(() => {
        const armSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/cmd/arming", serviceType: "mavros_msgs/srv/CommandBool" });
        armSvc.callService(new ROSLIB.ServiceRequest({ value: true }), () => addLog("INFO", "Force Arm successful"));
      }, 500);
    });
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
            <p className="text-gray-500 text-xs font-bold tracking-widest uppercase mb-3 text-center">Active Assets</p>
            {robots.map((robot, i) => (
              <div key={i} className="flex justify-between items-center bg-gray-800 border border-gray-700 p-3 rounded-lg cursor-default">
                <div className="flex items-center gap-3">
                  <span className={`h-2.5 w-2.5 rounded-full shadow-lg ${robot.status === "online" ? "bg-emerald-400 shadow-emerald-400/50" : "bg-rose-500 shadow-rose-500/50"}`} />
                  <span className="font-bold tracking-wide">{robot.name}</span>
                </div>
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
            <div className="bg-gray-800 border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3">
              <Battery size={16} className={telemetry.battery === "--" ? "text-gray-500" : parseInt(telemetry.battery) < 20 ? "text-rose-500" : "text-emerald-400"} /> 
              <span className="font-mono font-bold">{telemetry.battery}</span>
            </div>
            <div className="bg-gray-800 border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3">
              <Activity size={16} className={telemetry.altitude === "--" ? "text-gray-500" : "text-blue-400"} />
              <span className="font-mono font-bold">{telemetry.altitude}</span>
            </div>
            <div className="bg-gray-800 border border-gray-700 px-4 py-2 rounded-lg flex items-center gap-3">
              <MapPin size={16} className={telemetry.latitude === "--" ? "text-gray-500" : "text-indigo-400"} />
              <span className="font-mono">{telemetry.latitude}, {telemetry.longitude}</span>
            </div>
          </div>
        </div>

        <div className="flex-1 overflow-y-auto p-6 bg-gray-950">
          
          {tab === "mission" && (
            <div className="grid grid-cols-12 gap-6 h-full">
              <div className="col-span-8 bg-gray-900 border border-gray-800 rounded-xl shadow-2xl flex flex-col overflow-hidden">
                <div className="bg-gray-800 px-4 py-2 border-b border-gray-700 flex justify-between items-center text-sm font-mono text-gray-400">
                  <div className="flex items-center gap-2"><Target size={14}/> RGB OPTICAL ARRAY</div>
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
                    <AlertTriangle size={48} className="mb-4 opacity-50"/>
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
                  <h3 className="font-mono text-gray-400 text-sm mb-4 border-b border-gray-800 pb-2">IGNITION SEQUENCE</h3>
                  <div className="space-y-3 font-mono flex-1">
                    <button onClick={() => backendLaunch("sim", "ros2 launch vx01_bringup vx01_hybrid_sim.launch.py")} className="w-full bg-blue-600/20 text-blue-400 border border-blue-600/50 p-3 rounded hover:bg-blue-600/40 transition">
                      1. BOOT HYBRID SIMULATOR
                    </button>
                    <button onClick={() => backendLaunch("map", "ros2 launch vx01_simulation vx01_mapping.launch.py use_sim_time:=true")} className="w-full bg-purple-600/20 text-purple-400 border border-purple-600/50 p-3 rounded hover:bg-purple-600/40 transition">
                      2. INITIALIZE VISUAL SLAM
                    </button>
                    <button onClick={() => backendLaunch("walk", "ros2 launch vx01_locomotion_control walk.launch.py use_sim_time:=true")} className="w-full bg-teal-600/20 text-teal-400 border border-teal-600/50 p-3 rounded hover:bg-teal-600/40 transition">
                      3. START HEXAPOD KINEMATICS
                    </button>
                    <div className="pt-4 border-t border-gray-800 mt-4">
                      <button onClick={() => backendLaunch("auto", "python3 /vx01_ws/src/vx01_bringup/scripts/mission_coordinator.py --ros-args -p use_sim_time:=true")} className="w-full bg-rose-600 text-white font-bold tracking-widest p-4 rounded hover:bg-rose-500 transition shadow-lg shadow-rose-900/50">
                        ENGAGE AUTONOMY
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
                  <p className="text-gray-500 font-mono text-xs mb-8">Click Direction To Cruise, Click Stop To Halt</p>
                  
                  <div className="grid grid-cols-3 gap-4 text-center max-w-xs mx-auto font-mono text-xl">
                    <button onClick={() => sendHexapodCmd(0, 0.5, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">↰</button>
                    <button onClick={() => sendHexapodCmd(0.5, 0, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl border border-gray-700 hover:bg-white hover:text-black transition shadow-inner">↑</button>
                    <button onClick={() => sendHexapodCmd(0, -0.5, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">↱</button>

                    <button onClick={() => sendHexapodCmd(0, 0, 0.5)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">⟲</button>
                    <button onClick={stopHexapod} className="bg-rose-900 text-white font-bold p-6 rounded-xl border border-rose-700 hover:bg-rose-600 transition flex items-center justify-center shadow-lg"><StopCircle size={28}/></button>
                    <button onClick={() => sendHexapodCmd(0, 0, -0.5)} className="bg-gray-800 text-gray-400 p-6 rounded-xl hover:bg-white hover:text-black transition shadow-inner">⟳</button>

                    <div></div>
                    <button onClick={() => sendHexapodCmd(-0.5, 0, 0)} className="bg-gray-800 text-gray-400 p-6 rounded-xl border border-gray-700 hover:bg-white hover:text-black transition shadow-inner">↓</button>
                    <div></div>
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
                        <button onClick={stopDrone} className="bg-rose-900 text-white p-4 rounded hover:bg-rose-600 transition flex justify-center"><StopCircle size={20}/></button>
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
                        <button onClick={stopDrone} className="bg-rose-900 text-white p-4 rounded hover:bg-rose-600 transition flex justify-center"><StopCircle size={20}/></button>
                        <button onClick={() => startDroneCmd(0, -0.5, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">RT</button>
                        <div></div>
                        <button onClick={() => startDroneCmd(-1.0, 0, 0, 0)} className="bg-gray-800 p-4 rounded hover:bg-white hover:text-black transition">BCK</button>
                        <div></div>
                      </div>
                    </div>
                  </div>

                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-8 font-mono text-sm max-w-sm mx-auto">
                    <button onClick={forceArm} className="bg-red-900/40 border border-red-800 text-red-500 p-3 rounded hover:bg-red-800 hover:text-white transition">FORCE ARM / BYPASS</button>
                    <button onClick={() => {
                        const armSvc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/cmd/arming", serviceType: "mavros_msgs/srv/CommandBool" });
                        armSvc.callService(new ROSLIB.ServiceRequest({ value: true }), () => addLog("INFO", "Normal Arming"));
                      }} className="bg-emerald-900/40 border border-emerald-800 text-emerald-500 p-3 rounded hover:bg-emerald-800 hover:text-white transition">ARM (NORMAL)</button>
                    <button onClick={() => {
                        const svc = new ROSLIB.Service({ ros: rosRef.current, name: "/mavros/cmd/takeoff", serviceType: "mavros_msgs/srv/CommandTOL" });
                        svc.callService(new ROSLIB.ServiceRequest({ altitude: 5.0 }), () => addLog("INFO", "Dispatched Takeoff 5m"));
                      }} className="bg-indigo-900/40 border border-indigo-800 text-indigo-500 p-3 rounded hover:bg-indigo-800 hover:text-white transition col-span-2">TAKEOFF (5.0m)</button>
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