import { useState } from "react";
import { Battery, MapPin, Activity } from "lucide-react";

export default function Dashboard() {
  const [tab, setTab] = useState("dashboard");
  const [activeRobot, setActiveRobot] = useState("VX-01");

  const robots = [
    { name: "VX-01", status: "online" },
    { name: "VX-02", status: "offline" },
    { name: "VX-03", status: "online" },
  ];

  return (
    <div className="h-screen flex bg-gradient-to-br from-gray-100 to-blue-50">

      {/* SIDEBAR */}
      <div className="w-64 bg-white shadow-xl p-5 flex flex-col justify-between">

        <div>
          <h1 className="text-2xl font-bold text-blue-600 mb-6">VELATRIX</h1>

          {/* ROBOT LIST */}
          <div className="mb-6">
            <p className="text-gray-500 text-sm mb-2">Robots</p>

            {robots.map((robot, i) => (
              <div
                key={i}
                onClick={() => setActiveRobot(robot.name)}
                className={`flex justify-between items-center p-3 rounded-lg mb-2 cursor-pointer transition ${
                  activeRobot === robot.name
                    ? "bg-blue-600 text-white"
                    : "hover:bg-gray-100"
                }`}
              >
                <span>{robot.name}</span>
                <span
                  className={`h-2 w-2 rounded-full ${
                    robot.status === "online"
                      ? "bg-green-400"
                      : "bg-red-500"
                  }`}
                ></span>
              </div>
            ))}
          </div>

          {/* NAV */}
          <div className="space-y-2">
            {["dashboard", "control", "logs"].map((item) => (
              <button
                key={item}
                onClick={() => setTab(item)}
                className={`w-full text-left p-3 rounded-lg capitalize ${
                  tab === item
                    ? "bg-blue-600 text-white"
                    : "hover:bg-gray-200"
                }`}
              >
                {item}
              </button>
            ))}
          </div>
        </div>

        {/* STATUS */}
        <div className="bg-gray-100 p-3 rounded-lg text-sm">
          <p className="font-semibold">{activeRobot}</p>
          <p className="text-green-600">● Active</p>
        </div>
      </div>

      {/* MAIN */}
      <div className="flex-1 p-6 overflow-y-auto">

        {/* HEADER */}
        <div className="flex justify-between items-center mb-6">
          <div>
            <h2 className="text-2xl font-bold text-blue-600">
              {activeRobot} Control Panel
            </h2>
            <p className="text-gray-500">Search & Rescue System</p>
          </div>

          <div className="flex gap-3 text-sm">
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <Battery size={16}/> 87%
            </div>
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <Activity size={16}/> HEXAPOD
            </div>
            <div className="bg-white px-4 py-2 rounded shadow flex items-center gap-2">
              <MapPin size={16}/> 8.55, 76.88
            </div>
          </div>
        </div>

        {/* ================= DASHBOARD ================= */}
        {tab === "dashboard" && (
          <div className="grid grid-cols-4 gap-6">

            {/* CAMERA */}
            <div className="col-span-3 bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">Camera Feed</h3>
              <div className="h-80 bg-gray-900 rounded-lg flex items-center justify-center text-gray-400">
                Live Stream
              </div>
            </div>

            {/* MAP */}
            <div className="bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">Map</h3>
              <div className="h-80 bg-gray-100 rounded-lg flex items-center justify-center">
                Map View
              </div>
            </div>

            {/* TELEMETRY */}
            <div className="col-span-4 grid grid-cols-5 gap-4">
              {[
                ["Speed", "1.2 m/s"],
                ["Altitude", "2.5 m"],
                ["Temp", "32°C"],
                ["CPU", "45%"],
                ["Signal", "Strong"],
              ].map((item, i) => (
                <div key={i} className="bg-white p-4 rounded-lg shadow text-center">
                  <p className="text-gray-500 text-sm">{item[0]}</p>
                  <p className="text-xl font-bold text-blue-600">{item[1]}</p>
                </div>
              ))}
            </div>

            {/* GRAPH */}
            <div className="col-span-2 bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">Mission Progress</h3>
              <div className="h-48 bg-gray-100 rounded flex items-center justify-center">
                Graph Area
              </div>
            </div>

            {/* AI */}
            <div className="col-span-2 bg-white p-4 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-2">AI Detection</h3>

              <div className="space-y-2 text-sm">
                <div className="flex justify-between bg-gray-100 p-2 rounded">
                  <span>Victim</span>
                  <span className="text-green-600">Detected</span>
                </div>
                <div className="flex justify-between bg-gray-100 p-2 rounded">
                  <span>Confidence</span>
                  <span className="text-blue-600">92%</span>
                </div>
                <div className="flex justify-between bg-gray-100 p-2 rounded">
                  <span>Status</span>
                  <span className="text-red-500">Critical</span>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* ================= CONTROL ================= */}
        {tab === "control" && (
          <div className="grid grid-cols-3 gap-6">

            {/* MANUAL */}
            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">Manual Control</h3>

              <div className="grid grid-cols-3 gap-3 text-center">
                <div></div>
                <button className="bg-blue-600 text-white p-4 rounded-lg">↑</button>
                <div></div>

                <button className="bg-blue-600 text-white p-4 rounded-lg">←</button>
                <button className="bg-red-600 text-white p-4 rounded-lg">STOP</button>
                <button className="bg-blue-600 text-white p-4 rounded-lg">→</button>

                <div></div>
                <button className="bg-blue-600 text-white p-4 rounded-lg">↓</button>
                <div></div>
              </div>
            </div>

            {/* MISSION */}
            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">Mission Control</h3>

              <div className="space-y-3">
                <button className="w-full bg-green-600 text-white p-3 rounded-lg">
                  Start Mission
                </button>
                <button className="w-full bg-yellow-500 text-white p-3 rounded-lg">
                  Return Home
                </button>
                <button className="w-full bg-red-600 text-white p-3 rounded-lg">
                  Emergency Stop
                </button>
              </div>
            </div>

            {/* STATUS PANEL */}
            <div className="bg-white p-6 rounded-xl shadow-lg">
              <h3 className="font-semibold mb-4">System Status</h3>

              <div className="space-y-2 text-sm">
                <div className="flex justify-between">
                  <span>Battery</span>
                  <span className="text-green-600">87%</span>
                </div>
                <div className="flex justify-between">
                  <span>Motors</span>
                  <span>OK</span>
                </div>
                <div className="flex justify-between">
                  <span>Navigation</span>
                  <span>Active</span>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* ================= LOGS ================= */}
        {tab === "logs" && (
          <div className="bg-black text-green-400 p-6 rounded-xl shadow-lg font-mono text-sm">
            <p className="text-gray-400 mb-2">SYSTEM LOGS</p>
            <p>[INFO] System initialized</p>
            <p className="text-yellow-400">[WARN] Weak GPS</p>
            <p className="text-red-500">[ERROR] Motor delay</p>
          </div>
        )}

      </div>
    </div>
  );
}