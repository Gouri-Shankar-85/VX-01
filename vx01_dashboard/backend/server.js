const express = require('express');
const cors = require('cors');
const { exec, execSync } = require('child_process');

const app = express();
app.use(cors());
app.use(express.json());

const PORT = 3001;
const processes = {};

const ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /vx01_ws/install/setup.bash';

function killInsideDocker(pattern, container = 'vx01-dev', signal = 'SIGINT') {
    exec(`docker exec ${container} bash -c "pkill -${signal} -f '${pattern}' 2>/dev/null; true"`);
}

function shutdownAllInsideDocker(container = 'vx01-dev', onDone) {
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

    exec(`docker exec ${container} bash -c "${steps}"`, (err, stdout, stderr) => {
        if (err && err.code !== 1) {
            console.error('[stop-all] Shutdown error:', err.message);
        }

        const hostSteps = [
            `pkill -SIGKILL -f 'gzclient' 2>/dev/null; true`,
            `pkill -SIGKILL -f 'gzserver' 2>/dev/null; true`,
            `pkill -SIGKILL -f 'gz' 2>/dev/null; true`,
            `pkill -SIGKILL -f 'ign' 2>/dev/null; true`,
            `pkill -SIGKILL -f 'ruby' 2>/dev/null; true`
        ].join(' && ');

        exec(hostSteps, () => {
            console.log(`[stop-all] All processes terminated inside container ${container} AND cleared from host GUI`);
            if (onDone) onDone(null);
        });
    });
}

app.post('/api/launch', (req, res) => {
    const { command_id, command_string, container_name } = req.body;
    const container = container_name || 'vx01-dev';

    if (processes[command_id]) {
        return res.status(400).json({ error: `Process '${command_id}' already running` });
    }

    console.log(`Starting process in ${container}: ${command_id} -> ${command_string}`);

    const child = exec(
        `docker exec ${container} bash -c "${ROS_SETUP} && ${command_string}"`
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
    const { command_id, container_name } = req.body;
    const container = container_name || 'vx01-dev';

    if (!processes[command_id]) {
        return res.status(404).json({ error: `Process '${command_id}' not found` });
    }

    console.log(`Stopping process in ${container}: ${command_id}`);

    processes[command_id].kill('SIGINT');

    const patternMap = {
        sim: 'vx01_hybrid_sim',
        map: 'vx01_mapping',
        walk: 'hexapod.launch',
        auto: 'mission_coordinator',
    };
    const pattern = patternMap[command_id] || command_id;
    killInsideDocker(pattern, container, 'SIGINT');

    res.json({ success: true, message: `Stopped ${command_id}` });
});


app.post('/api/stop-all', (req, res) => {
    const { container_name } = req.body;
    const container = container_name || 'vx01-dev';
    const runningIds = Object.keys(processes);
    console.log(`[stop-all] Graceful shutdown requested for ${container}. Running: [${runningIds.join(', ')}]`);

    for (const [id, child] of Object.entries(processes)) {
        try {
            child.kill('SIGINT');
            console.log(`[stop-all] Sent SIGINT to '${id}'`);
        } catch (e) {
            console.warn(`[stop-all] Could not SIGINT '${id}':`, e.message);
        }
    }

    setTimeout(() => {
        shutdownAllInsideDocker(container, _err => {
            for (const id of Object.keys(processes)) delete processes[id];
        });
    }, 1000);

    res.json({
        success: true,
        message: `Shutdown initiated for ${container}. Full cleanup takes ~5s.`,
        stopped: runningIds,
    });
});

app.get('/api/status', (req, res) => {
    res.json({
        running: Object.keys(processes),
        count: Object.keys(processes).length,
    });
});

const server = app.listen(PORT, '0.0.0.0', () => {
    console.log(`VX-01 Launch Backend running on http://0.0.0.0:${PORT}`);
    console.log(`  POST /api/launch   { command_id, command_string }`);
    console.log(`  POST /api/stop     { command_id }`);
    console.log(`  POST /api/stop-all — kills everything in the container`);
    console.log(`  GET  /api/status   — list running processes`);
}).on('error', (err) => {
    if (err.code === 'EADDRINUSE') {
        console.error(`[ERROR] Port ${PORT} is already in use! Please kill the process using it: 'fuser -k 3001/tcp'`);
    } else {
        console.error('[ERROR] Server failed to start:', err);
    }
    process.exit(1);
});

// Handle unexpected crashes
process.on('uncaughtException', (err) => {
    console.error('[CRITICAL] Uncaught Exception:', err);
    process.exit(1);
});

process.on('unhandledRejection', (reason, promise) => {
    console.error('[CRITICAL] Unhandled Rejection at:', promise, 'reason:', reason);
    process.exit(1);
});
