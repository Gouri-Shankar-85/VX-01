const express = require('express');
const cors    = require('cors');
const { exec, execSync } = require('child_process');

const app = express();
app.use(cors());
app.use(express.json());

const PORT      = 3001;
const processes = {};   // command_id → child process

const ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /vx01_ws/install/setup.bash';

// ─── Helpers ─────────────────────────────────────────────────────────────────

/**
 * Kill a specific process inside Docker by name pattern.
 * Uses pkill with -SIGINT first (graceful), then -SIGKILL after 3s.
 */
function killInsideDocker(pattern, signal = 'SIGINT') {
    exec(`docker exec vx01-dev bash -c "pkill -${signal} -f '${pattern}' 2>/dev/null; true"`);
}

/**
 * Full graceful shutdown of everything running inside the vx01-dev container.
 * Order matters: stop ROS nodes first, then sim bridges, then Gazebo, then SITL.
 */
function shutdownAllInsideDocker(onDone) {
    const steps = [
        // 1. Graceful SIGINT to all ros2 launch processes
        `pkill -SIGINT -f 'ros2 launch' 2>/dev/null; true`,
        `pkill -SIGINT -f 'ros2 run'    2>/dev/null; true`,
        // 2. Give ROS nodes 3s to clean up, then force-kill remaining ros2 processes
        `sleep 3`,
        `pkill -SIGKILL -f 'ros2'                  2>/dev/null; true`,
        // 3. Kill MAVROS
        `pkill -SIGKILL -f 'mavros_node'            2>/dev/null; true`,
        // 4. Kill ArduPilot SITL
        `pkill -SIGKILL -f 'arducopter'             2>/dev/null; true`,
        // 5. Kill Gazebo (server + client + ign gazebo)
        `pkill -SIGKILL -f 'ign gazebo'             2>/dev/null; true`,
        `pkill -SIGKILL -f 'gz sim'                 2>/dev/null; true`,
        `pkill -SIGKILL -f 'gzserver'               2>/dev/null; true`,
        `pkill -SIGKILL -f 'gzclient'               2>/dev/null; true`,
        // 6. Kill ROS bridge (rosbridge_server)
        `pkill -SIGKILL -f 'rosbridge'              2>/dev/null; true`,
        // 7. Kill web_video_server
        `pkill -SIGKILL -f 'web_video_server'       2>/dev/null; true`,
        // 8. Remove any stale ROS lock files to allow clean restart
        `rm -f /tmp/.ros_*                          2>/dev/null; true`,
        `rm -f /root/.ros/log/latest                2>/dev/null; true`,
    ];

    const cmd = steps.join(' && ');
    exec(`docker exec vx01-dev bash -c "${cmd}"`, (err, stdout, stderr) => {
        if (err) console.error('[stop-all] Inner shutdown error:', err.message);
        else      console.log('[stop-all] All processes terminated inside container');
        if (onDone) onDone(err);
    });
}

// ─── Routes ──────────────────────────────────────────────────────────────────

/** Start a named ROS launch/run command */
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


/** Stop a single named process gracefully */
app.post('/api/stop', (req, res) => {
    const { command_id } = req.body;

    if (!processes[command_id]) {
        return res.status(404).json({ error: `Process '${command_id}' not found` });
    }

    console.log(`Stopping process: ${command_id}`);

    // Send SIGINT to the exec() child (closes the docker exec stdin → propagates SIGINT inside)
    processes[command_id].kill('SIGINT');

    // Also kill the corresponding process pattern inside Docker by command_id
    const patternMap = {
        sim:  'vx01_hybrid_sim',
        map:  'vx01_mapping',
        walk: 'walk.launch',
        auto: 'mission_coordinator',
    };
    const pattern = patternMap[command_id] || command_id;
    killInsideDocker(pattern, 'SIGINT');

    res.json({ success: true, message: `Stopped ${command_id}` });
});


/** STOP ALL — gracefully shuts down the entire simulation stack */
app.post('/api/stop-all', (req, res) => {
    const runningIds = Object.keys(processes);
    console.log(`[stop-all] Graceful shutdown requested. Running: [${runningIds.join(', ')}]`);

    // 1. SIGINT all tracked child processes from the Node side
    for (const [id, child] of Object.entries(processes)) {
        try {
            child.kill('SIGINT');
            console.log(`[stop-all] Sent SIGINT to '${id}'`);
        } catch (e) {
            console.warn(`[stop-all] Could not SIGINT '${id}':`, e.message);
        }
    }

    // 2. Wait 1s then do the full internal container shutdown
    setTimeout(() => {
        shutdownAllInsideDocker(_err => {
            // Clear the tracked process table (they're all dead now)
            for (const id of Object.keys(processes)) delete processes[id];
        });
    }, 1000);

    // Respond immediately so the UI doesn't time out (shutdown takes ~5s)
    res.json({
        success: true,
        message: `Shutdown initiated for: [${runningIds.join(', ')}]. Full cleanup takes ~5s.`,
        stopped: runningIds,
    });
});


/** Query which processes are currently tracked as running */
app.get('/api/status', (req, res) => {
    res.json({
        running: Object.keys(processes),
        count:   Object.keys(processes).length,
    });
});


// ─── Start ────────────────────────────────────────────────────────────────────
app.listen(PORT, () => {
    console.log(`VX-01 Launch Backend running on http://localhost:${PORT}`);
    console.log(`  POST /api/launch   { command_id, command_string }`);
    console.log(`  POST /api/stop     { command_id }`);
    console.log(`  POST /api/stop-all — kills everything in the container`);
    console.log(`  GET  /api/status   — list running processes`);
});
