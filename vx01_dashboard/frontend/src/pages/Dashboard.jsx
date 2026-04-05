import { useState, useEffect, useRef } from "react";
import { Battery, MapPin, Activity, Wifi, WifiOff } from "lucide-react";
import ROSLIB from "roslib";

const ROS_URL = `ws://${window.location.hostname}:9090`;

export default function Dashboard() {
  const [tab, setTab] = useState("dashboard");
  const [activeRobot, setActiveRobot] = useState("VX-01");
  const [rosConnected, setRosConnected] = useState(false);
  const [robotActive, setRobotActive] = useState(false);

  const [telemetry, setTelemetry] = useState({
    battery: "--",
    latitude: "--",
    longitude: "--",
    altitude: "--",
    speed: "--",
    cpu: "--",
    mode: "--",
    mission_phase: "--",
    victims: 0,
    terrain: "--",
    walkability: "--",
    robot_mode: "--",
  });

  const [logs, setLogs] = useState([]);
  const rosRef = useRef(null);
  const robotTimeoutRef = useRef(null);

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
      addLog("INFO", `Connected to rosbridge at ${ROS_URL}`);
    });
    ros.on("error", () => {
      setRosConnected(false);
      setRobotActive(false);
      addLog("ERROR", "Rosbridge connection error");
    });
    ros.on("close", () => {
      setRosConnected(false);
      setRobotActive(false);
      if (robotTimeoutRef.current) clearTimeout(robotTimeoutRef.current);
      addLog("WARN", "Rosbridge disconnected — retrying...");
    });

    const battery = new ROSLIB.Topic({
      ros,
      name: "/mavros/battery",
      messageType: "sensor_msgs/BatteryState",
    });
    battery.subscribe((msg) => {
      setTelemetry((t) => ({
        ...t,
        battery: `${Math.round(msg.percentage * 100)}%`,
      }));
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
        altitude: `${msg.altitude.toFixed(1)} m`,
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
      setTelemetry((t) => ({
        ...t,
        speed: `${Math.sqrt(vx * vx + vy * vy).toFixed(1)} m/s`,
      }));
    });

    const state = new ROSLIB.Topic({
      ros,
      name: "/mavros/state",
      messageType: "mavros_msgs/State",
    });
    state.subscribe((msg) => {
      setRobotActive(true);
      if (robotTimeoutRef.current) clearTimeout(robotTimeoutRef.current);
      robotTimeoutRef.current = setTimeout(() => setRobotActive(false), 2000);
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

    const rosout = new ROSLIB.Topic({
      ros,
      name: "/rosout",
      messageType: "rcl_interfaces/msg/Log",
    });
    rosout.subscribe((msg) => {
      const levelMap = { 10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL" };
      addLog(levelMap[msg.level] || "INFO", msg.msg);
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
      rosout.unsubscribe();
      ros.close();
      if (robotTimeoutRef.current) clearTimeout(robotTimeoutRef.current);
    };
  }, []);

  const sendCmd = (lx, ly, az) => {
    if (!rosRef.current || !rosConnected) return;
    const cmdVel = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/cmd_vel",
      messageType: "geometry_msgs/msg/Twist",
    });
    cmdVel.publish(
      new ROSLIB.Message({
        linear: { x: lx, y: ly, z: 0 },
        angular: { x: 0, y: 0, z: az },
      })
    );
  };

  const sendOperatorCmd = (cmd) => {
    if (!rosRef.current || !rosConnected) return;
    const cmdTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/operator_command",
      messageType: "std_msgs/msg/String",
    });
    cmdTopic.publish(new ROSLIB.Message({ data: cmd }));
  };

  const callService = (name, type, request = {}) => {
    if (!rosRef.current || !rosConnected) return;
    const svc = new ROSLIB.Service({ ros: rosRef.current, name, serviceType: type });
    svc.callService(new ROSLIB.ServiceRequest(request), () => {});
  };

  const logColor = { INFO: "text-green-400", WARN: "text-yellow-400", ERROR: "text-red-500", FATAL: "text-red-700", DEBUG: "text-gray-400" };

  return (
    <div className="h-screen flex bg-gradient-to-br from-gray-100 to-blue-50">

      <div className="w-64 bg-white shadow-xl p-5 flex flex-col justify-between">
        <div>
          <h1 className="text-2xl font-bold text-blue-600 mb-6">VELATRIX</h1>

          <div className="mb-6">
            <p className="text-gray-500 text-sm mb-2">Robots</p>
            {robots.map((robot, i) => (
              <div
                key={i}
                onClick={() => setActiveRobot(robot.name)}
                className={`flex justify-between items-center p-3 rounded-lg mb-2 cursor-pointer transition ${
                  activeRobot === robot.name ? "bg-blue-600 text-white" : "hover:bg-gray-100"
                }`}
              >
                <span>{robot.name}</span>
                <span className={`h-2 w-2 rounded-full ${robot.status === "online" ? "bg-green-400" : "bg-red-500"}`} />
              </div>
            ))}
          </div>

          <div className="space-y-2">
            {["dashboard", "control", "logs"].map((item) => (
              <button
                key={item}
                onClick={() => setTab(item)}
                className={`w-full text-left p-3 rounded-lg capitalize ${tab === item ? "bg-blue-600 text-white" : "hover:bg-gray-200"}`}
              >
                {item}
              </button>
            ))}
          </div>
        </div>

        <div className="bg-gray-100 p-3 rounded-lg text-sm space-y-1">
          <p className="font-semibold">{activeRobot}</p>
          <p className={robotActive ? "text-green-600" : "text-red-500"}>
            {robotActive ? "● Active" : "● Offline"}
          </p>
          <div className={`flex items-center gap-1 text-xs ${rosConnected ? "text-green-600" : "text-red-500"}`}>
            {rosConnected ? <Wifi size={12} /> : <WifiOff size={12} />}
            {rosConnected ? "ROS Connected" : "ROS Disconnected"}
          </div>
        </div>
      </div>

      <div className="flex-1 p-6 overflow-y-auto">

        <div className="flex justify-between items-center mb-6">
          <div>
            <h2 className="text-2xl font-bold text-blue-600">{activeRobot} Control Panel</h2>
            <p className="text-gray-500">Search & Rescue System</p>
          </div>
          <div className="flex gap-3 text-sm">
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <Battery size={16} /> {telemetry.battery}
            </div>
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <Activity size={16} /> {telemetry.mode}
            </div>
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <MapPin size={16} /> {telemetry.latitude}, {telemetry.longitude}
            </div>
          </div>
        </div>

        {tab === "dashboard" && (
          <div className="grid grid-cols-4 gap-6">
            <div className="col-span-3 bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">Camera Feed</h3>
              <div className="h-80 bg-gray-900 rounded-lg flex items-center justify-center text-gray-400">
                Live Stream
              </div>
            </div>

            <div className="bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">Map</h3>
              <div className="h-80 bg-gray-100 rounded-lg flex items-center justify-center">
                Map View
              </div>
            </div>

            <div className="col-span-4 grid grid-cols-5 gap-4">
              {[
                ["Speed", telemetry.speed],
                ["Altitude", telemetry.altitude],
                ["Mission Phase", telemetry.mission_phase],
                ["Robot Mode", telemetry.robot_mode],
                ["GPS", `${telemetry.latitude}, ${telemetry.longitude}`],
                ["Battery", telemetry.battery],
                ["Mode", telemetry.mode],
                ["Terrain", telemetry.terrain],
                ["Walkability", telemetry.walkability],
                ["Victims", telemetry.victims],
              ].map((item, i) => (
                <div key={i} className="bg-white p-4 rounded-lg shadow text-center">
                  <p className="text-gray-500 text-sm">{item[0]}</p>
                  <p className="text-xl font-bold text-blue-600 truncate">{item[1]}</p>
                </div>
              ))}
            </div>
          </div>
        )}

        {tab === "control" && (
          <div className="grid grid-cols-3 gap-6">
            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">Manual Control</h3>
              <div className="grid grid-cols-3 gap-3 text-center">
                <button onMouseDown={() => sendCmd(0, 0.5, 0)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 text-sm">↰</button>
                <button onMouseDown={() => sendCmd(0.5, 0, 0)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 font-bold">↑</button>
                <button onMouseDown={() => sendCmd(0, -0.5, 0)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 text-sm">↱</button>

                <button onMouseDown={() => sendCmd(0, 0, 0.5)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 font-bold">⟲</button>
                <button onClick={() => sendCmd(0, 0, 0)}
                  className="bg-red-600 text-white p-4 rounded-lg font-bold">STOP</button>
                <button onMouseDown={() => sendCmd(0, 0, -0.5)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 font-bold">⟳</button>

                <div />
                <button onMouseDown={() => sendCmd(-0.5, 0, 0)} onMouseUp={() => sendCmd(0, 0, 0)}
                  className="bg-blue-600 text-white p-4 rounded-lg active:bg-blue-800 font-bold">↓</button>
                <div />
              </div>
              {!rosConnected && (
                <p className="text-red-500 text-xs text-center mt-3">ROS not connected — controls disabled</p>
              )}
            </div>

            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">Mission Control</h3>
              <div className="space-y-3">
                <button
                  onClick={() => sendOperatorCmd("START")}
                  className="w-full bg-green-600 text-white p-3 rounded-lg disabled:opacity-50 hover:bg-green-700"
                  disabled={!rosConnected}
                >
                  Start Mission
                </button>
                <button
                  onClick={() => callService("/mavros/cmd/return_to_launch", "mavros_msgs/CommandBool", { value: true })}
                  className="w-full bg-yellow-500 text-white p-3 rounded-lg disabled:opacity-50 hover:bg-yellow-600"
                  disabled={!rosConnected}
                >
                  Return Home
                </button>
                <button
                  onClick={() => { sendCmd(0, 0, 0); sendOperatorCmd("ABORT"); }}
                  className="w-full bg-red-600 text-white p-3 rounded-lg hover:bg-red-700"
                >
                  Emergency Stop
                </button>
                <button
                  onClick={() => sendOperatorCmd("RESET")}
                  className="w-full bg-gray-600 text-white p-3 rounded-lg hover:bg-gray-700 mt-2"
                >
                  Reset Mission
                </button>
              </div>
            </div>

            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">System Status</h3>
              <div className="space-y-2 text-sm">
                <div className="flex justify-between">
                  <span>Battery</span>
                  <span className="text-green-600 font-bold">{telemetry.battery}</span>
                </div>
                <div className="flex justify-between">
                  <span>Mission Phase</span>
                  <span className="font-bold">{telemetry.mission_phase}</span>
                </div>
                <div className="flex justify-between">
                  <span>Terrain</span>
                  <span className="font-bold">{telemetry.terrain}</span>
                </div>
                <div className="flex justify-between">
                  <span>Walk Score</span>
                  <span className="font-bold">{telemetry.walkability}</span>
                </div>
                <div className="flex justify-between">
                  <span>Victims</span>
                  <span className="font-bold">{telemetry.victims}</span>
                </div>
                <div className="flex justify-between">
                  <span>ROS Bridge</span>
                  <span className={rosConnected ? "text-green-600 font-bold" : "text-red-500 font-bold"}>
                    {rosConnected ? "Connected" : "Disconnected"}
                  </span>
                </div>
              </div>
            </div>
          </div>
        )}

        {tab === "logs" && (
          <div className="bg-black text-green-400 p-6 rounded-xl shadow-lg font-mono text-sm h-96 overflow-y-auto">
            <p className="text-gray-400 mb-2">SYSTEM LOGS — {ROS_URL}</p>
            {logs.length === 0 && <p className="text-gray-500">Waiting for logs...</p>}
            {logs.map((l, i) => (
              <p key={i}>
                <span className="text-gray-500">[{l.ts}]</span>{" "}
                <span className={logColor[l.level] || "text-green-400"}>[{l.level}]</span>{" "}
                {l.msg}
              </p>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}