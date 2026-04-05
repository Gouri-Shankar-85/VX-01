const express = require('express');
const cors = require('cors');
const { exec } = require('child_process');

const app = express();
app.use(cors());
app.use(express.json());

const PORT = 3001;

// Keep track of running processes
const processes = {};

const ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /vx01_ws/install/setup.bash';

app.post('/api/launch', (req, res) => {
    const { command_id, command_string } = req.body;
    
    if (processes[command_id]) {
        return res.status(400).json({ error: 'Process already running' });
    }

    console.log(`Starting process: ${command_id} -> ${command_string}`);
    
    // Launch inside bash shell with ROS sourced
    const child = exec(`bash -c "${ROS_SETUP} && ${command_string}"`);
    
    processes[command_id] = child;

    child.stdout.on('data', data => console.log(`[${command_id}] ${data.trim()}`));
    child.stderr.on('data', data => console.error(`[${command_id}] ERR: ${data.trim()}`));
    
    child.on('close', code => {
        console.log(`Process ${command_id} exited with code ${code}`);
        delete processes[command_id];
    });

    res.json({ success: true, message: `Started ${command_id}` });
});

app.post('/api/stop', (req, res) => {
    const { command_id } = req.body;
    
    if (!processes[command_id]) {
        return res.status(404).json({ error: 'Process not found' });
    }

    console.log(`Stopping process: ${command_id}`);
    
    // SIGINT to allow ROS nodes to shutdown cleanly
    processes[command_id].kill('SIGINT');
    
    res.json({ success: true, message: `Stopped ${command_id}` });
});

app.listen(PORT, () => {
    console.log(`VX-01 Launch Backend running on http://localhost:${PORT}`);
});
