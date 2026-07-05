/* Core Mesh Networking Simulation Engine */

// Coordinates reference center: Yosemite Valley
const MAP_LAT_CENTER = 37.7456;
const MAP_LNG_CENTER = -119.5332;
const GPS_SCALE = 0.00003; // Conversion factor from pixels to degrees

// UI State
let nodes = [];
let obstacles = [];
let packets = [];
let selectedNode = null;
let isPlaying = true;
let simSpeed = 1.0;
let autoWander = true;
let isDrawingObstacles = false;
let obstacleStart = null;
let globalStats = {
    totalUniqueAlerts: new Set(), // Set of unique message IDs received by hubs
    totalDuplicatesFiltered: 0,
    totalPacketsTransmitted: 0
};

// Console Log Helper
const consoleLogStream = document.getElementById('console-log-stream');
function logConsole(message, type = 'system') {
    const time = new Date();
    const pad = (num) => String(num).padStart(2, '0');
    const timeStr = `[${pad(time.getHours())}:${pad(time.getMinutes())}:${pad(time.getSeconds())}]`;
    
    const line = document.createElement('div');
    line.className = `console-line type-${type}`;
    line.innerHTML = `<span class="timestamp">${timeStr}</span> <span>${message}</span>`;
    
    consoleLogStream.appendChild(line);
    consoleLogStream.scrollTop = consoleLogStream.scrollHeight;
}

// Convert canvas X/Y coordinates to simulated GPS values
function getSimulatedGPS(x, y, canvas) {
    const dx = x - canvas.width / 2;
    const dy = canvas.height / 2 - y;
    const lat = MAP_LAT_CENTER + dy * GPS_SCALE;
    const lng = MAP_LNG_CENTER + dx * GPS_SCALE;
    return {
        lat: lat.toFixed(5),
        lng: lng.toFixed(5)
    };
}

/* ==========================================
   1. DATA STRUCTURES
   ========================================== */

class Message {
    constructor(senderId, type, lat, lng, maxHops = 15, id = null, path = []) {
        this.id = id || `pkt-${Math.random().toString(36).substr(2, 9)}`;
        this.senderId = senderId;
        this.type = type;
        this.lat = lat;
        this.lng = lng;
        this.timestamp = Date.now();
        this.maxHops = maxHops;
        this.path = [...path, senderId]; // Node traversal path log
    }
}

class Node {
    constructor(id, type, x, y) {
        this.id = id;
        this.type = type; // 'civilian', 'vehicle', 'tower'
        this.x = x;
        this.y = y;
        
        // Size & display parameters
        this.radius = 12;
        this.color = this.getDefaultColor();
        this.range = this.getDefaultRange();
        
        // Navigation & Wander
        this.vx = 0;
        this.vy = 0;
        this.targetX = x;
        this.targetY = y;
        this.wanderTimer = 0;
        this.speed = this.getDefaultSpeed();
        
        // Battery (Fix C)
        this.battery = 100;
        this.isDead = false;
        
        // SQLite representation: messages database
        this.messages = new Map(); // messageId -> Message object
        this.seenMessages = new Set(); // Duplication Filter lookup (Fix A & B)
        
        // Session Tracker (Fix B)
        this.knownPeers = new Map(); // peerNodeId -> { synced: boolean, lastSeen: timestamp }
    }

    getDefaultColor() {
        switch(this.type) {
            case 'civilian': return '#00f0ff'; // Cyan
            case 'vehicle': return '#f59e0b'; // Amber/Orange
            case 'tower': return '#a100ff'; // Violet
            default: return '#ffffff';
        }
    }

    getDefaultRange() {
        switch(this.type) {
            case 'civilian': return 95;  // Bluetooth / Wi-Fi Direct
            case 'vehicle': return 150; // Medium-range emergency hub
            case 'tower': return 230;   // High-Power Stationary LoRa
            default: return 100;
        }
    }

    getDefaultSpeed() {
        switch(this.type) {
            case 'civilian': return 0.25; // Walking speed
            case 'vehicle': return 0.75; // Vehicle cruising speed
            case 'tower': return 0;      // Fixed/Stationary
            default: return 0.1;
        }
    }

    updateBattery(amount) {
        if (this.isDead || this.type === 'tower') return; // LoRa towers have stationary grid power
        
        // Vehicle hubs have automotive batteries, drain at 10x slower rate
        const actualDrain = this.type === 'vehicle' ? amount * 0.1 : amount;
        
        this.battery = Math.max(0, this.battery - actualDrain);
        if (this.battery <= 0 && !this.isDead) {
            this.isDead = true;
            this.color = '#4b5563'; // Neutral gray
            this.messages.clear(); // SQLite goes offline
            logConsole(`⚠️ Node [${this.id}] battery depleted. Node is OFFLINE.`, 'battery');
            if (selectedNode === this) {
                updateInspectorUI();
            }
        }
    }
}

class Obstacle {
    constructor(x1, y1, x2, y2) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }
}

class PacketParticle {
    constructor(startX, startY, endX, endY, message, senderId, receiverId) {
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.x = startX;
        this.y = startY;
        this.message = message;
        this.senderId = senderId;
        this.receiverId = receiverId;
        this.progress = 0;
        this.speed = 0.04; // Animation movement speed
    }

    update() {
        this.progress += this.speed * simSpeed;
        if (this.progress > 1) this.progress = 1;
        this.x = this.startX + (this.endX - this.startX) * this.progress;
        this.y = this.startY + (this.endY - this.startY) * this.progress;
        return this.progress >= 1; // Completed travel
    }
}

/* ==========================================
   2. COLLISION / OBSTACLE DETECTION (Extensible)
   ========================================== */

function lineSegmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
    const det = (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1);
    if (det === 0) return false; // Parallel lines
    
    const lambda = ((y4 - y3) * (x4 - x1) + (x3 - x4) * (y4 - y1)) / det;
    const gamma = ((y1 - y2) * (x4 - x1) + (x1 - x2) * (y4 - y1)) / det;
    
    return (0 < lambda && lambda < 1) && (0 < gamma && gamma < 1);
}

function isSignalBlocked(nodeA, nodeB) {
    if (obstacles.length === 0) return false;
    for (let obs of obstacles) {
        if (lineSegmentsIntersect(nodeA.x, nodeA.y, nodeB.x, nodeB.y, obs.x1, obs.y1, obs.x2, obs.y2)) {
            return true; // Intersection: Signal blocked!
        }
    }
    return false;
}

/* ==========================================
   3. SIMULATION SETUP & CONTROLLER
   ========================================== */

const canvas = document.getElementById('simulationCanvas');
const ctx = canvas.getContext('2d');
const container = document.getElementById('canvasContainer');

function resizeCanvas() {
    canvas.width = container.clientWidth;
    canvas.height = container.clientHeight;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Initial Setup
function initSimulation(nodeCount = 12) {
    nodes = [];
    obstacles = [];
    packets = [];
    selectedNode = null;
    globalStats = {
        totalUniqueAlerts: new Set(),
        totalDuplicatesFiltered: 0,
        totalPacketsTransmitted: 0
    };
    
    updateDashboardUI();
    document.getElementById('inspector-no-selection').style.display = 'block';
    document.getElementById('inspector-phone').style.display = 'none';

    // Generate random nodes across the map grid
    for (let i = 0; i < nodeCount; i++) {
        const type = i === 0 ? 'tower' : (i === 1 ? 'vehicle' : 'civilian');
        const margin = 80;
        const rx = margin + Math.random() * (canvas.width - margin * 2);
        const ry = margin + Math.random() * (canvas.height - margin * 2);
        
        let idStr = '';
        if (type === 'tower') idStr = `Lora-Tower-T1`;
        else if (type === 'vehicle') idStr = `Rescue-Hub-V1`;
        else idStr = `Civ-Phone-${i}`;

        const n = new Node(idStr, type, rx, ry);
        if (type === 'civilian') {
            n.battery = 45 + Math.floor(Math.random() * 55); // Random starting battery for reality
        }
        nodes.push(n);
    }
    
    // Add default obstacle representing a mountain ridge/building blocking signals
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    obstacles.push(new Obstacle(cx - 100, cy - 80, cx + 50, cy + 120));

    logConsole(`✅ Simulation initialized with ${nodeCount} nodes and 1 obstruction ridge.`, 'system');
}

/* ==========================================
   4. MESH NETWORKING PROTOCOL STATE MACHINE
   ========================================== */

function runMeshProtocol() {
    const now = Date.now();
    
    // 1. Proximity scanning and session updates
    for (let i = 0; i < nodes.length; i++) {
        const nodeA = nodes[i];
        if (nodeA.isDead) continue;

        let activePeers = [];

        for (let j = 0; j < nodes.length; j++) {
            if (i === j) continue;
            const nodeB = nodes[j];
            if (nodeB.isDead) continue;

            const dist = Math.hypot(nodeA.x - nodeB.x, nodeA.y - nodeB.y);
            
            // Handshake range validation & obstacle blocking check
            if (dist <= nodeA.range && !isSignalBlocked(nodeA, nodeB)) {
                activePeers.push(nodeB.id);

                // Handshake Logic (Fix B): Check session table
                if (!nodeA.knownPeers.has(nodeB.id)) {
                    // Start brand-new handshake session
                    nodeA.knownPeers.set(nodeB.id, { synced: false, lastSeen: now });
                    nodeB.knownPeers.set(nodeA.id, { synced: false, lastSeen: now });
                    
                    logConsole(`🤝 Handshake: [${nodeA.id}] ⬌ [${nodeB.id}] established.`, 'handshake');
                } else {
                    // Update timestamp on active session
                    nodeA.knownPeers.get(nodeB.id).lastSeen = now;
                }

                // Synchronization execution: Sync only once per session
                const sessionA = nodeA.knownPeers.get(nodeB.id);
                if (!sessionA.synced) {
                    // Sync Node A -> Node B (Send missing messages)
                    let transferredAny = false;
                    
                    nodeA.messages.forEach((msg, msgId) => {
                        if (!nodeB.seenMessages.has(msgId)) {
                            // Spawn visual flying packet particle
                            packets.push(new PacketParticle(nodeA.x, nodeA.y, nodeB.x, nodeB.y, msg, nodeA.id, nodeB.id));
                            transferredAny = true;
                            globalStats.totalPacketsTransmitted++;
                        }
                    });

                    // Sync Node B -> Node A (Gather missing messages)
                    nodeB.messages.forEach((msg, msgId) => {
                        if (!nodeA.seenMessages.has(msgId)) {
                            packets.push(new PacketParticle(nodeB.x, nodeB.y, nodeA.x, nodeA.y, msg, nodeB.id, nodeA.id));
                            transferredAny = true;
                            globalStats.totalPacketsTransmitted++;
                        }
                    });

                    // Mark session as synced on both ends to avoid infinite loop (Fix B)
                    sessionA.synced = true;
                    if (nodeB.knownPeers.has(nodeA.id)) {
                        nodeB.knownPeers.get(nodeA.id).synced = true;
                    }

                    if (transferredAny) {
                        // Slightly drain battery on handshake packet transfer activity (Fix C)
                        nodeA.updateBattery(1.0); // Sender energy drain
                        nodeB.updateBattery(0.5); // Receiver energy drain
                        
                        if (selectedNode === nodeA || selectedNode === nodeB) {
                            updateInspectorUI();
                        }
                    }
                }
            }
        }

        // Clean up out-of-range sessions (Fix B)
        nodeA.knownPeers.forEach((session, peerId) => {
            if (!activePeers.includes(peerId) && (now - session.lastSeen > 2000)) {
                nodeA.knownPeers.delete(peerId);
                logConsole(`🔌 Disconnected: [${nodeA.id}] lost contact with [${peerId}].`, 'handshake');
                
                // If peer is alive, delete opposite side session too
                const peerNode = nodes.find(n => n.id === peerId);
                if (peerNode && peerNode.knownPeers.has(nodeA.id)) {
                    peerNode.knownPeers.delete(nodeA.id);
                }
            }
        });
    }
}

// Receive a packet logic
function deliverPacket(packet) {
    const receiver = nodes.find(n => n.id === packet.receiverId);
    if (!receiver || receiver.isDead) return;

    const msg = packet.message;
    
    // 1. Message Duplication Filtering
    if (receiver.seenMessages.has(msg.id)) {
        globalStats.totalDuplicatesFiltered++;
        logConsole(`🛡️ Duplication Filter: [${receiver.id}] dropped duplicate alert [${msg.id.substring(0, 7)}].`, 'drop');
        updateDashboardUI();
        return;
    }

    // 2. Accept and Register packet into SQLite Database
    receiver.seenMessages.add(msg.id);
    
    // Decrement Hop / TTL Counter (Fix A)
    const newHopsRemaining = msg.maxHops - 1;
    
    if (newHopsRemaining <= 0) {
        logConsole(`❌ TTL Expired: Packet [${msg.id.substring(0, 7)}] hit 0 hops remaining. Dropped by [${receiver.id}].`, 'drop');
        return; // Message is deleted / ceases to propagate
    }

    // Create a new updated message instance to forward
    const forwardedMsg = new Message(
        msg.senderId,
        msg.type,
        msg.lat,
        msg.lng,
        newHopsRemaining,
        msg.id,
        msg.path
    );
    forwardedMsg.path.push(receiver.id); // Add current receiver to the hop path registry

    receiver.messages.set(forwardedMsg.id, forwardedMsg);
    logConsole(`📥 SQLite Log: [${receiver.id}] saved emergency alert [${msg.type}] (Hops remaining: ${newHopsRemaining}).`, 'packet');

    // 3. Trigger immediate re-sync with neighbors to propagate the new message
    receiver.knownPeers.forEach((session, peerId) => {
        session.synced = false; // Invalidate session to trigger transmission
        const peerNode = nodes.find(n => n.id === peerId);
        if (peerNode && peerNode.knownPeers.has(receiver.id)) {
            peerNode.knownPeers.get(receiver.id).synced = false;
        }
    });

    // 4. Check if packet arrived at High-Power Responder Hub (Vehicle or LoRa Tower)
    if (receiver.type === 'vehicle' || receiver.type === 'tower') {
        processResponderData(forwardedMsg, receiver);
    }

    if (selectedNode === receiver) {
        updateInspectorUI();
    }
}

// First Responder Database Entry Process
function processResponderData(message, hubNode) {
    if (!globalStats.totalUniqueAlerts.has(message.id)) {
        globalStats.totalUniqueAlerts.add(message.id);
        
        // Post message visual feedback in Dashboard tab
        appendAlertToDashboard(message);
        logConsole(`🚨 EMERGENCY ALERT DELIVERED TO RESPONDER HUB [${hubNode.id}]! Path: ${message.path.join(' ➔ ')}`, 'packet');
        
        updateDashboardUI();
    }
}

// Generate New Alert (Civilian SOS Trigger)
function triggerSOS(templateType) {
    if (!selectedNode || selectedNode.isDead) return;

    const gps = getSimulatedGPS(selectedNode.x, selectedNode.y, canvas);
    const msg = new Message(selectedNode.id, templateType, gps.lat, gps.lng, 15);
    
    // Store in sender's local database
    selectedNode.seenMessages.add(msg.id);
    selectedNode.messages.set(msg.id, msg);
    
    logConsole(`🚨 SOS Triggered on [${selectedNode.id}]: "${templateType}" @ (${gps.lat}, ${gps.lng})`, 'packet');
    
    // Drain battery for device hardware broadcast burst (Fix C)
    selectedNode.updateBattery(5.0);

    // Invalidate peer syncing session so it immediately pushes to nearby neighbors
    selectedNode.knownPeers.forEach((session, peerId) => {
        session.synced = false;
        const peerNode = nodes.find(n => n.id === peerId);
        if (peerNode && peerNode.knownPeers.has(selectedNode.id)) {
            peerNode.knownPeers.get(selectedNode.id).synced = false;
        }
    });

    updateInspectorUI();
}

/* ==========================================
   5. RENDER LOOP (CANVAS PAINTING)
   ========================================== */

let pulseOffset = 0;

function drawSimulation() {
    // Clear screen
    ctx.fillStyle = '#060913';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw grid overlay lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
    ctx.lineWidth = 1;
    const gridSize = 40;
    for (let x = 0; x < canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }

    // Draw Obstacles (Simulated Terrains/Walls)
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 4;
    ctx.setLineDash([8, 6]);
    for (let obs of obstacles) {
        ctx.beginPath();
        ctx.moveTo(obs.x1, obs.y1);
        ctx.lineTo(obs.x2, obs.y2);
        ctx.stroke();
    }
    ctx.setLineDash([]); // Reset line dash

    // Pulse animation range parameter
    pulseOffset = (pulseOffset + 0.5 * simSpeed) % 40;

    // Draw connection channels (Dashed lines between handshaking peers)
    ctx.lineWidth = 1.5;
    for (let i = 0; i < nodes.length; i++) {
        const nA = nodes[i];
        if (nA.isDead) continue;
        
        nA.knownPeers.forEach((session, peerId) => {
            const nB = nodes.find(n => n.id === peerId);
            if (nB && !nB.isDead && nA.id < nB.id) { // Draw once
                ctx.strokeStyle = session.synced ? 'rgba(16, 185, 129, 0.35)' : 'rgba(0, 240, 255, 0.25)';
                ctx.setLineDash([4, 4]);
                ctx.beginPath();
                ctx.moveTo(nA.x, nA.y);
                ctx.lineTo(nB.x, nB.y);
                ctx.stroke();
            }
        });
    }
    ctx.setLineDash([]); // Reset

    // Draw signal range rings
    for (let n of nodes) {
        if (n.isDead) continue;

        // Visual Coverage Radiuses
        ctx.strokeStyle = n.type === 'civilian' ? 'rgba(0, 240, 255, 0.08)' : 
                          (n.type === 'vehicle' ? 'rgba(245, 158, 11, 0.08)' : 'rgba(161, 0, 255, 0.08)');
        ctx.fillStyle = n.type === 'civilian' ? 'rgba(0, 240, 255, 0.015)' : 
                        (n.type === 'vehicle' ? 'rgba(245, 158, 11, 0.015)' : 'rgba(161, 0, 255, 0.015)');
        
        ctx.beginPath();
        ctx.arc(n.x, n.y, n.range, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Pulsing radio wave
        ctx.strokeStyle = n.type === 'civilian' ? `rgba(0, 240, 255, ${0.12 * (1 - pulseOffset / 40)})` : 
                          (n.type === 'vehicle' ? `rgba(245, 158, 11, ${0.12 * (1 - pulseOffset / 40)})` : `rgba(161, 0, 255, ${0.12 * (1 - pulseOffset / 40)})`);
        ctx.beginPath();
        ctx.arc(n.x, n.y, (n.range * (pulseOffset / 40)), 0, Math.PI * 2);
        ctx.stroke();
    }

    // Draw Nodes
    for (let n of nodes) {
        // Highlight Selected Node ring
        if (n === selectedNode) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(n.x, n.y, n.radius + 6, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Draw node core body
        ctx.fillStyle = n.color;
        ctx.beginPath();
        ctx.arc(n.x, n.y, n.radius, 0, Math.PI * 2);
        ctx.fill();
        
        // Inner shadow dot
        ctx.fillStyle = '#060913';
        ctx.beginPath();
        ctx.arc(n.x, n.y, n.radius - 4, 0, Math.PI * 2);
        ctx.fill();

        // Center status core
        ctx.fillStyle = n.isDead ? '#4b5563' : (n.messages.size > 0 ? '#ff5722' : n.color);
        ctx.beginPath();
        ctx.arc(n.x, n.y, n.radius - 7, 0, Math.PI * 2);
        ctx.fill();

        // Battery Ring Indicator
        if (!n.isDead && n.type !== 'tower') {
            ctx.lineWidth = 2.5;
            ctx.strokeStyle = n.battery < 25 ? '#ef4444' : (n.battery < 50 ? '#f59e0b' : '#10b981');
            ctx.beginPath();
            ctx.arc(n.x, n.y, n.radius + 2, -Math.PI / 2, (-Math.PI / 2) + (Math.PI * 2) * (n.battery / 100));
            ctx.stroke();
        }

        // Text IDs labels
        ctx.fillStyle = 'rgba(255,255,255,0.7)';
        ctx.font = '500 11px Outfit';
        ctx.textAlign = 'center';
        ctx.fillText(n.id, n.x, n.y - n.radius - 12);
        
        // Message counts overlay indicator
        if (n.messages.size > 0 && !n.isDead) {
            ctx.fillStyle = '#ff5722';
            ctx.beginPath();
            ctx.arc(n.x + 9, n.y - 9, 7, 0, Math.PI * 2);
            ctx.fill();
            
            ctx.fillStyle = '#fff';
            ctx.font = '700 9px Outfit';
            ctx.fillText(n.messages.size, n.x + 9, n.y - 6);
        }
    }

    // Draw Flying Packet Particles (Hops visualization)
    for (let p of packets) {
        ctx.fillStyle = '#10b981'; // Green energy dots
        ctx.shadowColor = '#10b981';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0; // Reset
    }

    // Draw obstacle preview line if drawing active
    if (isDrawingObstacles && obstacleStart) {
        ctx.strokeStyle = 'rgba(245, 158, 11, 0.6)';
        ctx.lineWidth = 3;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(obstacleStart.x, obstacleStart.y);
        ctx.lineTo(mouseCoords.x, mouseCoords.y);
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

/* ==========================================
   6. PHYSICAL LOCOMOTION & TICK TIMERS
   ========================================== */

function updateSimulation() {
    if (!isPlaying) return;

    // 1. Move Wandering Nodes (Walking Path Simulator)
    if (autoWander) {
        for (let n of nodes) {
            if (n.isDead || n.type === 'tower') continue; // Static towers do not move
            
            n.wanderTimer -= 1 * simSpeed;
            if (n.wanderTimer <= 0) {
                // Set new target coordinates
                const radiusLimit = 220;
                n.targetX = Math.max(80, Math.min(canvas.width - 80, n.x + (Math.random() - 0.5) * radiusLimit));
                n.targetY = Math.max(80, Math.min(canvas.height - 80, n.y + (Math.random() - 0.5) * radiusLimit));
                n.wanderTimer = 180 + Math.random() * 240; // Clock ticks
            }
            
            // Increment node coordinates toward target destination
            const dx = n.targetX - n.x;
            const dy = n.targetY - n.y;
            const dist = Math.hypot(dx, dy);
            
            if (dist > 5) {
                n.x += (dx / dist) * n.speed * simSpeed;
                n.y += (dy / dist) * n.speed * simSpeed;
            }
        }
    }

    // 2. Update particle packet animations
    for (let i = packets.length - 1; i >= 0; i--) {
        const completed = packets[i].update();
        if (completed) {
            deliverPacket(packets[i]);
            packets.splice(i, 1);
        }
    }

    // 3. Execute Mesh Sync Routines
    runMeshProtocol();
}

// Main Frame loop
function simulationLoop() {
    updateSimulation();
    drawSimulation();
    requestAnimationFrame(simulationLoop);
}

/* ==========================================
   7. UI COMPONENT BINDINGS & SIDEBAR INTERACTION
   ========================================== */

// Tabs Switcher
document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
        
        btn.classList.add('active');
        document.getElementById(`tab-${btn.dataset.tab}`).classList.add('active');
    });
});

// Update selected Node Phone Inspector Panel
function updateInspectorUI() {
    const noSelectPanel = document.getElementById('inspector-no-selection');
    const phonePanel = document.getElementById('inspector-phone');

    if (!selectedNode) {
        noSelectPanel.style.display = 'block';
        phonePanel.style.display = 'none';
        return;
    }

    noSelectPanel.style.display = 'none';
    phonePanel.style.display = 'block';

    // Populate Fields
    document.getElementById('phone-node-id').innerText = selectedNode.id;
    document.getElementById('phone-stat-type').innerText = selectedNode.type;
    
    // Battery Meter Display
    document.getElementById('phone-battery-pct').innerText = `${Math.floor(selectedNode.battery)}%`;
    const fill = document.getElementById('phone-battery-fill');
    fill.style.width = `${selectedNode.battery}%`;

    const meter = document.getElementById('phone-battery-meter');
    meter.className = 'phone-battery';
    if (selectedNode.isDead) {
        meter.classList.add('critical');
    } else if (selectedNode.battery < 25) {
        meter.classList.add('critical');
    } else if (selectedNode.battery < 50) {
        meter.classList.add('warning');
    } else {
        meter.classList.add('good');
    }

    const statusEl = document.getElementById('phone-stat-status');
    if (selectedNode.isDead) {
        statusEl.innerText = 'OFFLINE (Dead)';
        statusEl.style.color = 'var(--color-danger)';
    } else {
        statusEl.innerText = 'ONLINE';
        statusEl.style.color = 'var(--color-success)';
    }

    // Coords
    const gps = getSimulatedGPS(selectedNode.x, selectedNode.y, canvas);
    document.getElementById('phone-stat-coords').innerText = `${gps.lat}N, ${gps.lng}W`;

    // Active connection peer counts
    document.getElementById('phone-stat-peers').innerText = selectedNode.isDead ? '0' : selectedNode.knownPeers.size;

    // Local messages SQLite database rows
    const dbQueue = document.getElementById('phone-db-queue');
    dbQueue.innerHTML = '';
    
    document.getElementById('phone-queue-count').innerText = `(${selectedNode.messages.size})`;

    if (selectedNode.messages.size === 0) {
        dbQueue.innerHTML = `<div class="empty-state">SQLite queue is empty. Device has sent or forwarded 0 alerts.</div>`;
    } else {
        selectedNode.messages.forEach(msg => {
            const card = document.createElement('div');
            card.className = 'db-packet-card';
            // Highlight card if message has hit target dashboard
            const reachesResponder = globalStats.totalUniqueAlerts.has(msg.id);
            if (reachesResponder) {
                card.classList.add('success-delivery');
            }

            card.innerHTML = `
                <div class="packet-card-info">
                    <span class="packet-card-title">${msg.type}</span>
                    <span class="packet-card-meta">${msg.id.substring(0, 8)} • Sender: ${msg.senderId}</span>
                </div>
                <span class="packet-card-ttl" title="Time to Live (Hops Left)">Hops: ${msg.maxHops}</span>
            `;
            dbQueue.appendChild(card);
        });
    }
}

// Update Top Navigation Bar Stats
function updateDashboardUI() {
    // Unique Emergencies
    const count = globalStats.totalUniqueAlerts.size;
    document.getElementById('header-emergency-count').innerText = count;
    document.getElementById('dash-total-received').innerText = count;

    // Deduplication Ratio calculation
    const totalSeen = globalStats.totalUniqueAlerts.size + globalStats.totalDuplicatesFiltered;
    const ratio = totalSeen > 0 ? Math.round((globalStats.totalDuplicatesFiltered / totalSeen) * 100) : 0;
    
    document.getElementById('header-dedup-ratio').innerText = `${ratio}%`;
    document.getElementById('dash-duplicates-filtered').innerText = globalStats.totalDuplicatesFiltered;
    
    // Label counters in Controls sidebar
    document.getElementById('label-node-count').innerText = nodes.length;
}

// Append unique received emergencies in Responder dashboard
function appendAlertToDashboard(message) {
    const feed = document.getElementById('dash-alerts-feed');
    
    // Remove default empty status text
    const emptyState = feed.querySelector('.empty-state');
    if (emptyState) emptyState.remove();

    const time = new Date();
    const timeStr = `${String(time.getHours()).padStart(2, '0')}:${String(time.getMinutes()).padStart(2, '0')}`;

    const card = document.createElement('div');
    card.className = 'alert-feed-card';
    card.id = `dash-alert-${message.id}`;
    
    card.innerHTML = `
        <div class="alert-card-header">
            <span class="alert-badge">${message.type}</span>
            <span class="alert-time">${timeStr}</span>
        </div>
        <div class="alert-card-body">
            <div>📍 Coordinate: <strong>${message.lat}N, ${message.lng}W</strong></div>
            <div>🔗 Route trace hops:</div>
            <div class="alert-card-path">${message.path.join(' ➔ ')}</div>
        </div>
        <div class="alert-card-actions">
            <button class="btn btn-secondary btn-small" onclick="dispatchRescue('${message.id}')" style="background: rgba(16, 185, 129, 0.15); color: var(--color-success);">🚒 Dispatch Help</button>
        </div>
    `;
    feed.prepend(card);
}

// Dispatch / Resolve Alert Actions
window.dispatchRescue = function(messageId) {
    const card = document.getElementById(`dash-alert-${messageId}`);
    if (card) {
        card.classList.add('resolved');
        const header = card.querySelector('.alert-card-header');
        header.classList.add('resolved-header');
        const badge = card.querySelector('.alert-badge');
        badge.innerText = "Rescued";
        
        const actions = card.querySelector('.alert-card-actions');
        actions.innerHTML = `<span style="font-size: 0.75rem; color: var(--color-success); font-weight: 600; display:flex; align-items:center; gap:0.25rem;">✔ Rescue Dispatched</span>`;
        
        logConsole(`🚒 Dispatch Command: Rescue units deployed to coordinates for alert [${messageId.substring(0, 7)}].`, 'system');
    }
};

/* ==========================================
   8. INPUT DRAG & SPANNING LOGICS
   ========================================== */

let draggedNode = null;
let dragOffset = { x: 0, y: 0 };
let mouseCoords = { x: 0, y: 0 };

// Detect clicks on node coordinates
function getNodeAtPosition(x, y) {
    for (let n of nodes) {
        const dist = Math.hypot(n.x - x, n.y - y);
        if (dist <= n.radius + 8) return n;
    }
    return null;
}

// Mouse Event listeners on canvas
canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (isDrawingObstacles) {
        obstacleStart = { x: mx, y: my };
        return;
    }

    const clicked = getNodeAtPosition(mx, my);
    if (clicked) {
        draggedNode = clicked;
        dragOffset.x = mx - clicked.x;
        dragOffset.y = my - clicked.y;
        
        // Select & Inspect node
        selectedNode = clicked;
        updateInspectorUI();
        
        // Force Switch to Inspector tab so user instantly views phone interface
        document.querySelectorAll('.tab-btn').forEach(b => {
            if (b.dataset.tab === 'inspector') {
                b.click();
            }
        });
    } else {
        selectedNode = null;
        updateInspectorUI();
    }
});

canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    mouseCoords.x = e.clientX - rect.left;
    mouseCoords.y = e.clientY - rect.top;

    if (draggedNode) {
        draggedNode.x = Math.max(15, Math.min(canvas.width - 15, mouseCoords.x - dragOffset.x));
        draggedNode.y = Math.max(15, Math.min(canvas.height - 15, mouseCoords.y - dragOffset.y));
        
        // Re-evaluate coordinates
        if (selectedNode === draggedNode) {
            updateInspectorUI();
        }
    }
});

canvas.addEventListener('mouseup', () => {
    if (isDrawingObstacles && obstacleStart) {
        // Complete drawing obstacle segment
        const dist = Math.hypot(obstacleStart.x - mouseCoords.x, obstacleStart.y - mouseCoords.y);
        if (dist > 15) {
            obstacles.push(new Obstacle(obstacleStart.x, obstacleStart.y, mouseCoords.x, mouseCoords.y));
            logConsole(`🚧 Barrier drawn. Coordinates: (${Math.round(obstacleStart.x)}, ${Math.round(obstacleStart.y)}) to (${Math.round(mouseCoords.x)}, ${Math.round(mouseCoords.y)})`, 'system');
        }
        obstacleStart = null;
    }
    draggedNode = null;
});

canvas.addEventListener('dblclick', (e) => {
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (getNodeAtPosition(mx, my) === null) {
        // Spawn a civilian node on double-click
        const idNum = nodes.length + 1;
        const n = new Node(`Civ-Phone-${idNum}`, 'civilian', mx, my);
        nodes.push(n);
        selectedNode = n;
        updateInspectorUI();
        updateDashboardUI();
        logConsole(`➕ Spawned civilian device [${n.id}] at coordinate location.`, 'system');
    }
});

/* ==========================================
   9. SIDEBAR BUTTON TRIGGERS
   ========================================== */

// One-tap SOS triggers
document.querySelectorAll('.sos-template-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        triggerSOS(btn.dataset.sos);
    });
});

// Control triggers
document.getElementById('btn-play-pause').addEventListener('click', (e) => {
    isPlaying = !isPlaying;
    e.target.innerText = isPlaying ? "Pause Simulation" : "Resume Simulation";
    e.target.className = isPlaying ? "btn btn-primary" : "btn btn-secondary";
    logConsole(isPlaying ? "▶ Simulation resumed." : "⏸ Simulation paused.", 'system');
});

document.getElementById('btn-reset').addEventListener('click', () => {
    const val = parseInt(document.getElementById('slider-node-count').value);
    initSimulation(val);
    document.getElementById('dash-alerts-feed').innerHTML = `
        <div class="empty-state" style="padding: 2.5rem 0;">No mesh network alerts received by responder hubs yet. Wait for a message to hop to a Rescue Vehicle or LoRa Tower.</div>`;
});

document.getElementById('btn-add-civilian').addEventListener('click', () => {
    const rx = 50 + Math.random() * (canvas.width - 100);
    const ry = 50 + Math.random() * (canvas.height - 100);
    const id = `Civ-Phone-${nodes.length + 1}`;
    const n = new Node(id, 'civilian', rx, ry);
    nodes.push(n);
    selectedNode = n;
    updateInspectorUI();
    updateDashboardUI();
    logConsole(`➕ Spawned Civilian node: [${id}]`, 'system');
});

document.getElementById('btn-add-vehicle').addEventListener('click', () => {
    const rx = 50 + Math.random() * (canvas.width - 100);
    const ry = 50 + Math.random() * (canvas.height - 100);
    const id = `Rescue-Hub-V${nodes.filter(n => n.type==='vehicle').length + 2}`;
    const n = new Node(id, 'vehicle', rx, ry);
    nodes.push(n);
    selectedNode = n;
    updateInspectorUI();
    updateDashboardUI();
    logConsole(`➕ Spawned High-Power Vehicle Hub: [${id}]`, 'system');
});

document.getElementById('btn-add-tower').addEventListener('click', () => {
    const rx = 50 + Math.random() * (canvas.width - 100);
    const ry = 50 + Math.random() * (canvas.height - 100);
    const id = `Lora-Tower-T${nodes.filter(n => n.type==='tower').length + 2}`;
    const n = new Node(id, 'tower', rx, ry);
    nodes.push(n);
    selectedNode = n;
    updateInspectorUI();
    updateDashboardUI();
    logConsole(`➕ Spawned Stationary LoRa Tower: [${id}]`, 'system');
});

document.getElementById('btn-clear-console').addEventListener('click', () => {
    consoleLogStream.innerHTML = '';
});

// Settings Sliders
document.getElementById('slider-sim-speed').addEventListener('input', (e) => {
    simSpeed = parseFloat(e.target.value);
    document.getElementById('label-sim-speed').innerText = `${simSpeed.toFixed(1)}x`;
});

document.getElementById('slider-node-count').addEventListener('input', (e) => {
    const val = parseInt(e.target.value);
    document.getElementById('label-node-count').innerText = val;
});

// Auto wander toggle
document.getElementById('chk-auto-wander').addEventListener('change', (e) => {
    autoWander = e.target.checked;
    logConsole(`Auto-Wander path movement: ${autoWander ? 'ENABLED' : 'DISABLED'}`, 'system');
});

// Draw Obstacles toggle
const toggleObstaclesBtn = document.getElementById('btn-toggle-obstacles');
toggleObstaclesBtn.addEventListener('click', () => {
    isDrawingObstacles = !isDrawingObstacles;
    toggleObstaclesBtn.classList.toggle('active');
    
    if (isDrawingObstacles) {
        toggleObstaclesBtn.innerText = "🚧 Drawing Obstacles (ON)";
        canvas.style.cursor = 'crosshair';
        logConsole("🚧 Click and drag on the map grid to draw signal blocking obstacle walls.", 'system');
    } else {
        toggleObstaclesBtn.innerText = "🚧 Draw Obstacles (OFF)";
        canvas.style.cursor = 'grab';
    }
});

// Run Self Tests Diagnostics
function runSelfTests() {
    const btn = document.getElementById('btn-run-diagnostics');
    const resultsContainer = document.getElementById('diagnostics-results');
    
    if (!btn || !resultsContainer) return;

    btn.disabled = true;
    btn.innerText = "⏳ Diagnostics Running...";
    resultsContainer.style.display = 'flex';
    
    const updateBadge = (id, status, type) => {
        const badge = document.querySelector(`#${id} .test-status`);
        if (badge) {
            badge.innerText = status.toUpperCase();
            badge.className = `test-status badge-${type}`;
        }
    };
    
    // Set all to running
    updateBadge('test-ttl', 'running', 'running');
    updateBadge('test-dedup', 'running', 'running');
    updateBadge('test-battery', 'running', 'running');
    
    logConsole("🧪 Starting system self-test diagnostics...", "system");
    
    // Save original simulation state
    const originalNodes = nodes;
    const originalPackets = packets;
    const originalIsPlaying = isPlaying;
    const originalSelectedNode = selectedNode;
    const originalGlobalStats = {
        totalUniqueAlerts: new Set(globalStats.totalUniqueAlerts),
        totalDuplicatesFiltered: globalStats.totalDuplicatesFiltered,
        totalPacketsTransmitted: globalStats.totalPacketsTransmitted
    };
    
    // Pause main simulation
    isPlaying = false;
    selectedNode = null;
    
    // Execute tests with slight delay for realistic visual feedback
    setTimeout(() => {
        // --- Test 1: TTL Limit ---
        let ttlPassed = false;
        try {
            nodes = [];
            packets = [];
            
            const nodeA = new Node("Test-A", "civilian", 100, 100);
            const nodeB = new Node("Test-B", "civilian", 150, 100);
            const nodeC = new Node("Test-C", "civilian", 200, 100);
            
            nodes = [nodeA, nodeB, nodeC];
            
            // A gets message with maxHops = 2
            const msg = new Message(nodeA.id, "Test SOS", 37.74, -119.53, 2, "test-ttl-msg");
            nodeA.seenMessages.add(msg.id);
            nodeA.messages.set(msg.id, msg);
            
            // Step 1: Sync A <-> B
            runMeshProtocol();
            while (packets.length > 0) {
                deliverPacket(packets.shift());
            }
            
            // Assert B got the message and hops is 1
            const msgInB = nodeB.messages.get("test-ttl-msg");
            const hasB = !!msgInB && msgInB.maxHops === 1;
            
            // Step 2: Sync B <-> C
            runMeshProtocol();
            while (packets.length > 0) {
                deliverPacket(packets.shift());
            }
            
            // Assert C does NOT have the message because hops hit 0
            const hasC = nodeC.messages.has("test-ttl-msg");
            
            if (hasB && !hasC) {
                ttlPassed = true;
                logConsole("✔ Test 1/3 (TTL Limit): PASSED. Packets expire correctly at hop threshold.", "packet");
                updateBadge('test-ttl', 'passed', 'pass');
            } else {
                logConsole(`❌ Test 1/3 (TTL Limit): FAILED. B got msg: ${hasB}, C got msg: ${hasC} (expected B: true, C: false)`, "drop");
                updateBadge('test-ttl', 'failed', 'fail');
            }
        } catch (err) {
            logConsole(`❌ Test 1/3 (TTL Limit): ERROR: ${err.message}`, "drop");
            updateBadge('test-ttl', 'error', 'fail');
        }
        
        setTimeout(() => {
            // --- Test 2: Deduplication ---
            let dedupPassed = false;
            try {
                nodes = [];
                packets = [];
                
                const nodeA = new Node("Test-A", "civilian", 100, 100);
                nodes = [nodeA];
                
                const msg = new Message("Sender", "Test SOS", 37.74, -119.53, 10, "test-dedup-msg");
                
                const pkt1 = new PacketParticle(100, 100, 100, 100, msg, "Sender", nodeA.id);
                deliverPacket(pkt1);
                
                const countAfterPkt1 = nodeA.messages.size;
                
                const pkt2 = new PacketParticle(100, 100, 100, 100, msg, "Sender", nodeA.id);
                deliverPacket(pkt2);
                
                const countAfterPkt2 = nodeA.messages.size;
                
                if (countAfterPkt1 === 1 && countAfterPkt2 === 1) {
                    dedupPassed = true;
                    logConsole("✔ Test 2/3 (Deduplication): PASSED. Duplicate message IDs are successfully dropped.", "packet");
                    updateBadge('test-dedup', 'passed', 'pass');
                } else {
                    logConsole(`❌ Test 2/3 (Deduplication): FAILED. Count after pkt 1: ${countAfterPkt1}, count after pkt 2: ${countAfterPkt2} (expected 1 & 1)`, "drop");
                    updateBadge('test-dedup', 'failed', 'fail');
                }
            } catch (err) {
                logConsole(`❌ Test 2/3 (Deduplication): ERROR: ${err.message}`, "drop");
                updateBadge('test-dedup', 'error', 'fail');
            }
            
            setTimeout(() => {
                // --- Test 3: Battery Depletion ---
                let batteryPassed = false;
                try {
                    nodes = [];
                    packets = [];
                    
                    const nodeA = new Node("Test-A", "civilian", 100, 100);
                    nodeA.battery = 1.0;
                    nodes = [nodeA];
                    
                    // Consume battery beyond limit
                    nodeA.updateBattery(1.5);
                    
                    const isDead = nodeA.isDead;
                    const colorCorrect = nodeA.color === '#4b5563';
                    const dbCleared = nodeA.messages.size === 0;
                    
                    // Test connection blocking when dead
                    const nodeB = new Node("Test-B", "civilian", 130, 100); // within range of A
                    nodes = [nodeA, nodeB];
                    
                    runMeshProtocol();
                    const sessionEstablished = nodeB.knownPeers.has(nodeA.id);
                    
                    if (isDead && colorCorrect && dbCleared && !sessionEstablished) {
                        batteryPassed = true;
                        logConsole("✔ Test 3/3 (Battery Depletion): PASSED. Out-of-battery nodes successfully shutdown.", "packet");
                        updateBadge('test-battery', 'passed', 'pass');
                    } else {
                        logConsole(`❌ Test 3/3 (Battery Depletion): FAILED. Dead: ${isDead}, Color: ${nodeA.color}, Connected: ${sessionEstablished}`, "drop");
                        updateBadge('test-battery', 'failed', 'fail');
                    }
                } catch (err) {
                    logConsole(`❌ Test 3/3 (Battery Depletion): ERROR: ${err.message}`, "drop");
                    updateBadge('test-battery', 'error', 'fail');
                }
                
                // --- Cleanup & Restore state ---
                nodes = originalNodes;
                packets = originalPackets;
                isPlaying = originalIsPlaying;
                selectedNode = originalSelectedNode;
                globalStats.totalUniqueAlerts = originalGlobalStats.totalUniqueAlerts;
                globalStats.totalDuplicatesFiltered = originalGlobalStats.totalDuplicatesFiltered;
                globalStats.totalPacketsTransmitted = originalGlobalStats.totalPacketsTransmitted;
                
                // Re-enable run diagnostics button
                btn.disabled = false;
                btn.innerText = "🔍 Run System Self-Test";
                
                if (ttlPassed && dedupPassed && batteryPassed) {
                    logConsole("🎉 ALL SYSTEM DIAGNOSTIC TESTS PASSED! Mesh protocol is fully verified.", "system");
                } else {
                    logConsole("⚠️ DIAGNOSTIC TESTS COMPLETED WITH FAILURES. Check logs.", "battery");
                }
                
                // Force update UI
                updateInspectorUI();
                updateDashboardUI();
                
            }, 500);
        }, 500);
    }, 500);
}

const btnRunDiagnostics = document.getElementById('btn-run-diagnostics');
if (btnRunDiagnostics) {
    btnRunDiagnostics.addEventListener('click', runSelfTests);
}

/* ==========================================
   10. INITIALIZATION RUN
   ========================================== */

const initialNodeSliderVal = document.getElementById('slider-node-count').value;
document.getElementById('label-node-count').innerText = initialNodeSliderVal;
initSimulation(parseInt(initialNodeSliderVal));
simulationLoop();
