const express = require('express');
const cors = require('cors');
const { exec, execSync } = require('child_process');

const app = express();
app.use(cors());
app.use(express.json());

const PORT = 3001;
const processes = {};

const ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /vx01_ws/install/setup.bash';

function killInsideDocker(pattern, signal = 'SIGINT') {
    exec(`docker exec vx01-dev bash -c "pkill -${signal} -f '${pattern}' 2>/dev/null; true"`);
}

function shutdownAllInsideDocker(onDone) {
    const steps = [
        `pkill -SIGINT  -f 'ros2 launch'       2>/dev/null; true`,
        `pkill -SIGINT  -f 'ros2 run'          2>/dev/null; true`,
        `pkill -SIGINT  -f 'arducopter'        2>/dev/null; true`,
        `pkill -SIGINT  -f 'rosbridge'         2>/dev/null; true`,
        `sleep 3`,
        `pkill -SIGKILL -f 'ros2'              2>/dev/null; true`,
        `pkill -SIGKILL -f 'mavros_node'       2>/dev/null; true`,
        `pkill -SIGKILL -f 'arducopter'        2>/dev/null; true`,
        `pkill -SIGKILL -f 'rosbridge'         2>/dev/null; true`,
        `pkill -SIGKILL -f 'web_video_server'  2>/dev/null; true`,
        `pkill -SIGKILL -f 'ruby'              2>/dev/null; true`,
        `pkill -SIGKILL -f 'ign'               2>/dev/null; true`,
        `pkill -SIGKILL -f 'gz'                2>/dev/null; true`,
        `pkill -SIGKILL -f 'gzserver'          2>/dev/null; true`,
        `pkill -SIGKILL -f 'gzclient'          2>/dev/null; true`,
        `rm -f /tmp/.ros_*                     2>/dev/null; true`,
        `rm -f /root/.ros/log/latest           2>/dev/null; true`,
    ].join(' && ');

    exec(`docker exec vx01-dev bash -c "${steps}"`, (err, stdout, stderr) => {

        if (err && err.code !== 1) {
            console.error('[stop-all] Shutdown error:', err.message);
        } else {
            console.log('[stop-all] All processes terminated inside container');
        }
        if (onDone) onDone(null);
    });
}

app.post('/api/launch', (req, res) => {
    const { command_id, command_string } = req.body;

    if (processes[command_id]) {
        return res.status(400).json({ error: `Process '${command_id}' already running` });
    }

    console.log(`Starting process: ${command_id} -> ${command_string}`);

    const child = exec(
        `docker exec vx01-dev bash -c "${ROS_SETUP} && ${command_string}"`
    );

    processes[command_id] = child;

    child.stdout.on('data', data => process.stdout.write(`[${command_id}] ${data}`));
    child.stderr.on('data', data => process.stderr.write(`[${command_id}] ${data}`));

    child.on('close', code => {
        console.log(`Process '${command_id}' exited with code ${code}`);
        delete processes[command_id];
    });

    res.json({ success: true, message: `Started ${command_id}` });
});


app.post('/api/stop', (req, res) => {
    const { command_id } = req.body;

    if (!processes[command_id]) {
        return res.status(404).json({ error: `Process '${command_id}' not found` });
    }

    console.log(`Stopping process: ${command_id}`);

    processes[command_id].kill('SIGINT');

    const patternMap = {
        sim: 'vx01_hybrid_sim',
        map: 'vx01_mapping',
        walk: 'walk.launch',
        auto: 'mission_coordinator',
    };
    const pattern = patternMap[command_id] || command_id;
    killInsideDocker(pattern, 'SIGINT');

    res.json({ success: true, message: `Stopped ${command_id}` });
});


app.post('/api/stop-all', (req, res) => {
    const runningIds = Object.keys(processes);
    console.log(`[stop-all] Graceful shutdown requested. Running: [${runningIds.join(', ')}]`);

    for (const [id, child] of Object.entries(processes)) {
        try {
            child.kill('SIGINT');
            console.log(`[stop-all] Sent SIGINT to '${id}'`);
        } catch (e) {
            console.warn(`[stop-all] Could not SIGINT '${id}':`, e.message);
        }
    }

    setTimeout(() => {
        shutdownAllInsideDocker(_err => {
            for (const id of Object.keys(processes)) delete processes[id];
        });
    }, 1000);

    res.json({
        success: true,
        message: `Shutdown initiated for: [${runningIds.join(', ')}]. Full cleanup takes ~5s.`,
        stopped: runningIds,
    });
});

app.get('/api/status', (req, res) => {
    res.json({
        running: Object.keys(processes),
        count: Object.keys(processes).length,
    });
});

app.listen(PORT, () => {
    console.log(`VX-01 Launch Backend running on http://localhost:${PORT}`);
    console.log(`  POST /api/launch   { command_id, command_string }`);
    console.log(`  POST /api/stop     { command_id }`);
    console.log(`  POST /api/stop-all — kills everything in the container`);
    console.log(`  GET  /api/status   — list running processes`);
});
