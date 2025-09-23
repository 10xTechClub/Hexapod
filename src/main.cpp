#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
Preferences prefs;


//function declarations
void setServoAngle(int servo, int angle);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendServoUpdate(uint8_t clientNum, int servo, int angle, int pwm);
void broadcastServoUpdate(int servo, int angle, int pwm);
void handleRoot();
void moveForward();
void stopMovement();
void executeWalkStep(int step);
void moveBackward();

// WiFi Credentials
const char *ssid = "10xTC-AP2";
const char *password = "10xTechClub#";

// Create PCA9685 objects
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Right side (servos 0-17)
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);  // Left side (servos 18-35)

// Servers
WebSocketsServer webSocket = WebSocketsServer(81);
WebServer server(80);

// Servo configuration
#define SERVOMIN  195  // Minimum pulse length count (out of 4096)
#define SERVOMAX  431  // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// Current servo positions and PWM values (only using 18 servos total)
int servoPositions[18];
int servoPWMValues[18];


int heightOffset = 0; // default, can be negative (lower) or positive (higher)
int stride = 40;  // default stride length
bool walkingForward = true;  // true = forward, false = reverse


// Movement control variables
bool isWalking = false;
unsigned long lastStepTime = 0;
int currentStep = 0;
int stepDelay = 500; // Default 500ms between steps (adjustable via UI)

// Hexapod leg mapping based on your actual setup:
// PCA2 (0x41) - Left side: Legs 1,2,3 (channels 0-8)
// PCA1 (0x40) - Right side: Legs 4,5,6 (channels 0-8)
// Each leg: Coxa, Femur, Tibia

// Servo index mapping:
// Left side (PCA2): 0-8 (Leg1: 0,1,2  Leg2: 3,4,5  Leg3: 6,7,8)
// Right side (PCA1): 9-17 (Leg4: 9,10,11  Leg5: 12,13,14  Leg6: 15,16,17)

// Updated walking gait patterns from your spreadsheet (18 servos total)
// Global tuning variable
// Global tuning variables

// Walk pattern (4 steps, 6 legs × 3 joints each = 18 values per step)
// Format: [L1_C,L1_F,L1_T, L2_C,L2_F,L2_T, L3_C,L3_F,L3_T, L4_C,L4_F,L4_T, L5_C,L5_F,L5_T, L6_C,L6_F,L6_T]
int FemurHeight = 40;

void generateWalkPattern(int walkPattern[4][18]) {
  // Base positions
  int base = 90;
  
  // Direct mapping from your table - no calculations, just the exact values
  // Format: [L1_C,L1_F,L1_T, L2_C,L2_F,L2_T, L3_C,L3_F,L3_T, L4_C,L4_F,L4_T, L5_C,L5_F,L5_T, L6_C,L6_F,L6_T]
  
  // ---- Step 1 ---- (from your table's Step1 column)
  // Left Side (PCA2): Legs 1,2,3
  walkPattern[0][0] = base + stride;   // L1 Coxa
  walkPattern[0][1] = base - FemurHeight; // L1 Femur (lifting)
  walkPattern[0][2] = base;            // L1 Tibia
  
  walkPattern[0][3] = base;            // L2 Coxa
  walkPattern[0][4] = base;            // L2 Femur
  walkPattern[0][5] = base;            // L2 Tibia
  
  walkPattern[0][6] = base + stride;   // L3 Coxa  
  walkPattern[0][7] = base - FemurHeight; // L3 Femur (lifting)
  walkPattern[0][8] = base;            // L3 Tibia
  
  // Right Side (PCA1): Legs 4,5,6 - ALREADY INVERTED VALUES
  walkPattern[0][9]  = base;           // R4 Coxa (90-0=90)
  walkPattern[0][10] = base;           // R4 Femur (90-0=90)
  walkPattern[0][11] = base;           // R4 Tibia (90-0=90)
  
  walkPattern[0][12] = base - stride;  // R5 Coxa (90-(-40)=130, but we want 50, so 90-40=50)
  walkPattern[0][13] = base + FemurHeight; // R5 Femur (90-(-40)=130, but we want 50, so inverted lifting)
  walkPattern[0][14] = base;           // R5 Tibia
  
  walkPattern[0][15] = base;           // R6 Coxa
  walkPattern[0][16] = base;           // R6 Femur
  walkPattern[0][17] = base;           // R6 Tibia

  // ---- Step 2 ---- (from your table's Step2 column)
  // Left Side
  walkPattern[1][0] = base + stride;   // L1 Coxa (down)
  walkPattern[1][1] = base;            // L1 Femur (down)
  walkPattern[1][2] = base;            // L1 Tibia
  
  walkPattern[1][3] = base;            // L2 Coxa
  walkPattern[1][4] = base;            // L2 Femur
  walkPattern[1][5] = base;            // L2 Tibia
  
  walkPattern[1][6] = base + stride;   // L3 Coxa (down)
  walkPattern[1][7] = base;            // L3 Femur (down)
  walkPattern[1][8] = base;            // L3 Tibia
  
  // Right Side
  walkPattern[1][9]  = base;           // R4 Coxa
  walkPattern[1][10] = base;           // R4 Femur
  walkPattern[1][11] = base;           // R4 Tibia
  
  walkPattern[1][12] = base - stride;  // R5 Coxa (down)
  walkPattern[1][13] = base;           // R5 Femur (down)
  walkPattern[1][14] = base;           // R5 Tibia
  
  walkPattern[1][15] = base;           // R6 Coxa
  walkPattern[1][16] = base;           // R6 Femur
  walkPattern[1][17] = base;           // R6 Tibia

  // ---- Step 3 ---- (from your table's Step3 column)
  // Left Side
  walkPattern[2][0] = base;            // L1 Coxa
  walkPattern[2][1] = base;            // L1 Femur
  walkPattern[2][2] = base;            // L1 Tibia
  
  walkPattern[2][3] = base + stride;   // L2 Coxa (lift)
  walkPattern[2][4] = base - FemurHeight; // L2 Femur (lift)
  walkPattern[2][5] = base;            // L2 Tibia
  
  walkPattern[2][6] = base;            // L3 Coxa
  walkPattern[2][7] = base;            // L3 Femur
  walkPattern[2][8] = base;            // L3 Tibia
  
  // Right Side
  walkPattern[2][9]  = base - stride;  // R4 Coxa (lift, inverted)
  walkPattern[2][10] = base + FemurHeight; // R4 Femur (lift, inverted)
  walkPattern[2][11] = base;           // R4 Tibia
  
  walkPattern[2][12] = base;           // R5 Coxa
  walkPattern[2][13] = base;           // R5 Femur
  walkPattern[2][14] = base;           // R5 Tibia
  
  walkPattern[2][15] = base - stride;  // R6 Coxa (lift, inverted)
  walkPattern[2][16] = base + FemurHeight; // R6 Femur (lift, inverted)
  walkPattern[2][17] = base;           // R6 Tibia

  // ---- Step 4 ---- (from your table's Step4 column)
  // Left Side
  walkPattern[3][0] = base;            // L1 Coxa
  walkPattern[3][1] = base;            // L1 Femur
  walkPattern[3][2] = base;            // L1 Tibia
  
  walkPattern[3][3] = base + stride;   // L2 Coxa (down)
  walkPattern[3][4] = base;            // L2 Femur (down)
  walkPattern[3][5] = base;            // L2 Tibia
  
  walkPattern[3][6] = base;            // L3 Coxa
  walkPattern[3][7] = base;            // L3 Femur
  walkPattern[3][8] = base;            // L3 Tibia
  
  // Right Side  
  walkPattern[3][9]  = base - stride;  // R4 Coxa (down, inverted)
  walkPattern[3][10] = base;           // R4 Femur (down)
  walkPattern[3][11] = base;           // R4 Tibia
  
  walkPattern[3][12] = base;           // R5 Coxa
  walkPattern[3][13] = base;           // R5 Femur
  walkPattern[3][14] = base;           // R5 Tibia
  
  walkPattern[3][15] = base - stride;  // R6 Coxa (down, inverted)
  walkPattern[3][16] = base;           // R6 Femur (down)
  walkPattern[3][17] = base;           // R6 Tibia
}

// Complete HTML page with embedded CSS and JavaScript
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>36 Servo Hexapod Controller</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta charset="UTF-8">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 0; 
            padding: 20px; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
        }
        .container { 
            max-width: 1400px; 
            margin: 0 auto; 
            background: white;
            border-radius: 15px;
            padding: 30px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
        }
        .header { 
            text-align: center; 
            margin-bottom: 30px; 
            color: #333;
        }
        .header h1 {
            margin: 0 0 15px 0;
            font-size: 2.5em;
            color: #4a5568;
        }
        .servo-grid { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); 
            gap: 20px;
            margin-top: 30px;
        }
        .servo-card { 
            background: linear-gradient(145deg, #f8f9fa, #e9ecef);
            padding: 20px; 
            border-radius: 12px; 
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            border: 1px solid #e0e6ed;
            transition: transform 0.2s ease;
        }
        .servo-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(0,0,0,0.15);
        }
        .servo-title { 
            font-weight: bold; 
            margin-bottom: 15px; 
            color: #2d3748;
            font-size: 1.1em;
            text-align: center;
        }
        .controller-info {
            font-size: 0.85em;
            color: #718096;
            margin-bottom: 10px;
            text-align: center;
        }
        .servo-controls { 
            display: flex; 
            align-items: center; 
            gap: 15px; 
            margin-bottom: 15px; 
        }
        .servo-slider { 
            flex: 1; 
            height: 8px;
            -webkit-appearance: none;
            appearance: none;
            background: #e2e8f0;
            border-radius: 5px;
            outline: none;
        }
        .servo-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            background: #4299e1;
            border-radius: 50%;
            cursor: pointer;
        }
        .servo-slider::-moz-range-thumb {
            width: 20px;
            height: 20px;
            background: #4299e1;
            border-radius: 50%;
            cursor: pointer;
            border: none;
        }
        .servo-value { 
            min-width: 70px; 
            text-align: center; 
            font-weight: bold; 
            font-size: 1.1em;
            color: #2d3748;
            background: #edf2f7;
            padding: 8px 12px;
            border-radius: 6px;
        }
        .pwm-info {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: #f7fafc;
            padding: 10px;
            border-radius: 6px;
            font-size: 0.9em;
        }
        .pwm-value { 
            color: #4a5568;
            font-weight: 500;
        }
        .control-buttons { 
            text-align: center; 
            margin: 30px 0;
            display: flex;
            gap: 15px;
            justify-content: center;
            flex-wrap: wrap;
        }
        .movement-controls {
            background: linear-gradient(145deg, #f0f8ff, #e6f3ff);
            padding: 25px;
            border-radius: 15px;
            margin: 30px 0;
            border: 2px solid #4299e1;
        }
        .movement-title {
            text-align: center;
            font-size: 1.5em;
            font-weight: bold;
            color: #2d3748;
            margin-bottom: 20px;
        }
        .movement-buttons {
            display: flex;
            gap: 20px;
            justify-content: center;
            flex-wrap: wrap;
        }
        .btn { 
            padding: 12px 24px; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer; 
            font-size: 16px;
            font-weight: 600;
            transition: all 0.2s ease;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .btn:hover {
            transform: translateY(-1px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        }
        .btn-primary { 
            background: linear-gradient(45deg, #4299e1, #3182ce); 
            color: white; 
        }
        .btn-success { 
            background: linear-gradient(45deg, #48bb78, #38a169); 
            color: white; 
        }
        .btn-warning { 
            background: linear-gradient(45deg, #ed8936, #dd6b20); 
            color: white; 
        }
        .btn-danger { 
            background: linear-gradient(45deg, #f56565, #e53e3e); 
            color: white; 
        }
        .btn-move {
            background: linear-gradient(45deg, #9f7aea, #805ad5);
            color: white;
            font-size: 18px;
            padding: 15px 30px;
        }
        .btn-stop {
            background: linear-gradient(45deg, #f56565, #e53e3e);
            color: white;
            font-size: 18px;
            padding: 15px 30px;
        }
        .status { 
            text-align: center; 
            margin: 15px 0; 
            padding: 12px; 
            border-radius: 8px;
            font-weight: 600;
            font-size: 1.1em;
        }
        .status.connected { 
            background: linear-gradient(45deg, #c6f6d5, #9ae6b4); 
            color: #22543d; 
        }
        .status.disconnected { 
            background: linear-gradient(45deg, #fed7d7, #fbb6ce); 
            color: #742a2a; 
        }
        .status.walking {
            background: linear-gradient(45deg, #bee3f8, #90cdf4);
            color: #2c5282;
        }
        .pca-section {
            margin: 30px 0;
        }
        .pca-header {
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white;
            padding: 15px;
            border-radius: 10px;
            text-align: center;
            font-size: 1.3em;
            font-weight: bold;
            margin-bottom: 20px;
        }
        @media (max-width: 768px) {
            .container { padding: 15px; }
            .control-buttons { flex-direction: column; }
            .movement-buttons { flex-direction: column; }
            .btn { width: 100%; margin: 5px 0; }
            .servo-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🦿 Hexapod Robot Controller</h1>
            <p>Dual PCA9685 18-Channel Controllers - Updated Walking Pattern</p>
            <div id="status" class="status disconnected">🔴 Disconnected</div>
            <div id="walkStatus" class="status" style="display: none;"></div>
        </div>
        
        <div class="movement-controls">
            <div class="movement-title">🚶‍♂️ Hexapod Movement Control</div>
            
            <!-- Speed Control Section -->
            <div style="background: rgba(255,255,255,0.7); padding: 20px; border-radius: 10px; margin-bottom: 25px;">
                <div style="text-align: center; font-weight: bold; color: #2d3748; margin-bottom: 15px; font-size: 1.1em;">
                    ⚡ Walking Speed Control
                </div>
                <div style="display: flex; align-items: center; gap: 15px; justify-content: center; flex-wrap: wrap;">
                    <label style="font-weight: 600; color: #4a5568;">Speed:</label>
                    <input type="range" min="100" max="1000" value="500" 
                           style="flex: 1; min-width: 200px; height: 8px; -webkit-appearance: none; appearance: none; background: #e2e8f0; border-radius: 5px; outline: none;"
                           id="speedSlider" onchange="updateSpeed(this.value)" oninput="updateSpeedDisplay(this.value)">
                    <div id="speedValue" style="min-width: 100px; text-align: center; font-weight: bold; color: #2d3748; background: #edf2f7; padding: 8px 12px; border-radius: 6px;">
                        500ms
                    </div>
                </div>
                <div style="display: flex; justify-content: space-between; margin-top: 10px; font-size: 0.9em; color: #718096;">
                    <span>🐰 Fast (100ms)</span>
                    <span>🚶 Normal (500ms)</span>
                    <span>🐢 Slow (1000ms)</span>
                </div>
            </div>

            <!-- Stride Length Control Section -->
            <div style="background: rgba(255,255,255,0.7); padding: 20px; border-radius: 10px; margin-bottom: 25px;">
                <div style="text-align: center; font-weight: bold; color: #2d3748; margin-bottom: 15px; font-size: 1.1em;">
                    👣 Stride Length Control
                </div>
                <div style="display: flex; align-items: center; gap: 15px; justify-content: center; flex-wrap: wrap;">
                    <label style="font-weight: 600; color: #4a5568;">Stride:</label>
                    <input type="range" min="10" max="90" value="40" 
                            style="flex: 1; min-width: 200px; height: 8px; background: #e2e8f0; border-radius: 5px; outline: none;"
                            id="strideSlider" onchange="updateStride(this.value)" oninput="updateStrideDisplay(this.value)">
                    <div id="strideValue" style="min-width: 100px; text-align: center; font-weight: bold; color: #2d3748; background: #edf2f7; padding: 8px 12px; border-radius: 6px;">
                        60°
                    </div>
                </div>
                <div style="display: flex; justify-content: space-between; margin-top: 10px; font-size: 0.9em; color: #718096;">
                    <span>🐢 Short (10°)</span>
                    <span>👣 Normal (60°)</span>
                    <span>🦘 Long (90°)</span>
                </div>
            </div>

            <div class="movement-buttons">
            <button class="btn btn-move" onclick="startWalking()" id="walkBtn">🚀 Walk Forward</button>
            <button class="btn btn-move" onclick="startWalkingBackward()" id="walkBackBtn" 
                    style="background: linear-gradient(45deg, #805ad5, #6b46c1);">⬅️ Walk Backward</button>
            <button class="btn btn-stop" onclick="stopWalking()">🛑 Stop</button>
        </div>
        
        <div class="control-buttons">
            <button class="btn btn-primary" onclick="setAllServos(90)">🎯 Center All (90°)</button>
            <button class="btn btn-success" onclick="setAllServos(0)">⬅️ Min All (0°)</button>
            <button class="btn btn-warning" onclick="setAllServos(180)">➡️ Max All (180°)</button>
            <button class="btn btn-danger" onclick="sweepTest()">🔥 Sweep Test</button>
        </div>

        <div class="pca-section">
            <div class="pca-header">PCA9685 Controller 2 (Address 0x41) - Left Side: Legs 1,2,3</div>
            <div class="servo-grid" id="pca2Grid"></div>
        </div>

        <div class="pca-section">
            <div class="pca-header">PCA9685 Controller 1 (Address 0x40) - Right Side: Legs 4,5,6</div>
            <div class="servo-grid" id="pca1Grid"></div>
        </div>
    </div>

    <script>
        let ws;
        let servoData = {};
        let sweeping = false;
        let walking = false;
        let currentSpeed = 500; // Default speed in milliseconds

        function connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            ws = new WebSocket(protocol + '//' + window.location.hostname + ':81');
            
            ws.onopen = function() {
                document.getElementById('status').className = 'status connected';
                document.getElementById('status').innerHTML = '🟢 Connected';
                console.log('WebSocket connected');
            };
            
            ws.onclose = function() {
                document.getElementById('status').className = 'status disconnected';
                document.getElementById('status').innerHTML = '🔴 Disconnected - Reconnecting...';
                console.log('WebSocket disconnected');
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = function(error) {
                console.log('WebSocket error:', error);
            };
            
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'servoUpdate') {
                        updateServoDisplay(data.servo, data.angle, data.pwm);
                    } else if (data.type === 'walkStatus') {
                        updateWalkStatus(data.walking, data.step);
                    } else if (data.type === 'speedUpdate') {
                        updateSpeedFromServer(data.speed);
                    } else if (data.type === 'strideUpdate') {
                        updateStrideFromServer(data.stride);
                    }
                } catch (e) {
                    console.error('Error parsing WebSocket message:', e);
                }
            };
        }

        function updateStride(stride) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({
                    type: 'setStride',
                    stride: parseInt(stride)
                });
                ws.send(message);
                console.log('Updated stride to:', stride);
            }
        }

        function updateStrideDisplay(stride) {
            document.getElementById('strideValue').textContent = stride + '°';
        }

        function updateStrideFromServer(stride) {
            document.getElementById('strideSlider').value = stride;
            document.getElementById('strideValue').textContent = stride + '°';
            console.log('Stride updated from server:', stride, 'degrees');
        }
            

        function createServoControls() {
            const pca1Grid = document.getElementById('pca1Grid');
            const pca2Grid = document.getElementById('pca2Grid');
            
            // Left side (PCA2): servos 0-8 (Legs 1,2,3)
            for (let i = 0; i < 9; i++) {
                const legNum = Math.floor(i / 3) + 1;
                const jointNum = i % 3;
                const jointNames = ['Coxa', 'Femur', 'Tibia'];
                const card = createServoCard(i, 2, i, `L${legNum} ${jointNames[jointNum]}`);
                pca2Grid.appendChild(card);
            }
            
            // Right side (PCA1): servos 9-17 (Legs 4,5,6)
            for (let i = 9; i < 18; i++) {
                const legNum = Math.floor((i - 9) / 3) + 4;
                const jointNum = (i - 9) % 3;
                const jointNames = ['Coxa', 'Femur', 'Tibia'];
                const card = createServoCard(i, 1, i - 9, `L${legNum} ${jointNames[jointNum]}`);
                pca1Grid.appendChild(card);
            }
        }
            function updateHeight(value) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        const message = JSON.stringify({
            type: 'setHeight',
            height: parseInt(value)
        });
        ws.send(message);
        console.log('Updated height to:', value);
    }
}

function updateHeightDisplay(value) {
    document.getElementById('heightValue').textContent = value;
}



        function createServoCard(servoIndex, pcaNum, channelNum, jointName) {
            const card = document.createElement('div');
            card.className = 'servo-card';
            
            card.innerHTML = `
                <div class="servo-title">${jointName}</div>
                <div class="controller-info">PCA${pcaNum} Channel ${channelNum} (Servo ${servoIndex})</div>
                <div class="servo-controls">
                    <input type="range" min="0" max="180" value="90" class="servo-slider" 
                           id="slider${servoIndex}" onchange="setServo(${servoIndex}, this.value)"
                           oninput="updateValueDisplay(${servoIndex}, this.value)">
                    <div class="servo-value" id="value${servoIndex}">90°</div>
                </div>
                <div class="pwm-info">
                    <span class="pwm-value" id="pwm${servoIndex}">PWM: 400</span>
                    <span id="address${servoIndex}">0x${pcaNum === 1 ? '40' : '41'}</span>
                </div>
            `;
            
            servoData[servoIndex] = { angle: 90, pwm: 400 };
            return card;
        }

        function updateValueDisplay(servo, angle) {
            document.getElementById('value' + servo).textContent = angle + '°';
        }

        function setServo(servo, angle) {
            angle = parseInt(angle);
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({
                    type: 'setServo',
                    servo: servo,
                    angle: angle
                });
                ws.send(message);
                console.log('Sent:', message);
            } else {
                console.log('WebSocket not connected');
            }
        }

        function startWalking() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({
                    type: 'startWalk'
                });
                ws.send(message);
                console.log('Started walking');
            }
        }

        function stopWalking() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({
                    type: 'stopWalk'
                });
                ws.send(message);
                console.log('Stopped walking');
            }
        }

        function updateSpeed(speed) {
            currentSpeed = parseInt(speed);
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({
                    type: 'setSpeed',
                    speed: currentSpeed
                });
                ws.send(message);
                console.log('Updated speed to:', currentSpeed, 'ms');
            }
        }

        function updateSpeedDisplay(speed) {
            document.getElementById('speedValue').textContent = speed + 'ms';
        }

        function updateSpeedFromServer(speed) {
            currentSpeed = speed;
            document.getElementById('speedSlider').value = speed;
            document.getElementById('speedValue').textContent = speed + 'ms';
            console.log('Speed updated from server:', speed, 'ms');
        }

        function updateWalkStatus(isWalking, step) {
            const walkStatus = document.getElementById('walkStatus');
            const walkBtn = document.getElementById('walkBtn');
            
            if (isWalking) {
                walkStatus.style.display = 'block';
                walkStatus.className = 'status walking';
                walkStatus.innerHTML = `🚶‍♂️ Walking - Step ${step + 1}/4`;
                walkBtn.innerHTML = '⏸️ Pause Walk';
                walking = true;
            } else {
                walkStatus.style.display = 'none';
                walkBtn.innerHTML = '🚀 Walk Forward';
                walking = false;
            }
        }

        function setAllServos(angle) {
            if (sweeping || walking) return;
            
            for (let i = 0; i < 18; i++) {
                document.getElementById('slider' + i).value = angle;
                updateValueDisplay(i, angle);
                setServo(i, angle);
            }
        }

        function startWalkingBackward() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        const message = JSON.stringify({
            type: 'startWalkBackward'
        });
        ws.send(message);
        console.log('Started walking backward');
    }
}


        function sweepTest() {
            if (sweeping || walking) return;
            
            sweeping = true;
            const btn = event.target;
            btn.textContent = '⏹⏹⏹ Stop Sweep';
            
            let angle = 0;
            let direction = 1;
            
            const sweepInterval = setInterval(() => {
                if (!sweeping) {
                    clearInterval(sweepInterval);
                    btn.textContent = '🔥 Sweep Test';
                    return;
                }
                
                for (let i = 0; i < 18; i++) {
                    document.getElementById('slider' + i).value = angle;
                    updateValueDisplay(i, angle);
                    setServo(i, angle);
                }
                
                angle += direction * 10;
                if (angle >= 180) {
                    angle = 180;
                    direction = -1;
                } else if (angle <= 0) {
                    angle = 0;
                    direction = 1;
                }
            }, 200);
            
            setTimeout(() => {
                sweeping = false;
            }, 30000);
        }

        function updateServoDisplay(servo, angle, pwm) {
            servoData[servo] = { angle: angle, pwm: pwm };
            document.getElementById('value' + servo).textContent = angle + '°';
            document.getElementById('pwm' + servo).textContent = 'PWM: ' + pwm;
            document.getElementById('slider' + servo).value = angle;
        }

        createServoControls();
        connectWebSocket();
        
        window.addEventListener('focus', () => {
            if (ws.readyState !== WebSocket.OPEN) {
                connectWebSocket();
            }
        });
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
    prefs.begin("hexapod", false);  // namespace "hexapod"
    heightOffset = prefs.getInt("heightOffset", 0); // default = 0
    Serial.println("Restored height offset: " + String(heightOffset));


  Serial.println("Starting 36 Servo Hexapod Controller...");
  
  // Initialize I2C with specified pins (SDA=21, SCL=22)
  Wire.begin(21, 22);
  Serial.println("I2C initialized on pins SDA=21, SCL=22");
  
  // Initialize PCA9685 controllers
  pca1.begin();
  pca2.begin();
  
  pca1.setOscillatorFrequency(27000000);
  pca2.setOscillatorFrequency(27000000);
  
  pca1.setPWMFreq(SERVO_FREQ);
  pca2.setPWMFreq(SERVO_FREQ);
  
  Serial.println("PCA9685 controllers initialized");
  Serial.println("PCA1: Address 0x40 - Right Side Legs 4,5,6 (Channels 0-8)");
  Serial.println("PCA2: Address 0x41 - Left Side Legs 1,2,3 (Channels 0-8)");
  
  // Initialize all servos to 90 degrees (center position)
  for (int i = 0; i < 18; i++) {
    setServoAngle(i, 90);
    servoPositions[i] = 90;
    delay(10); // Small delay between servo initializations
  }
  Serial.println("All 18 servos initialized to 90 degrees");
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  ArduinoOTA.setHostname("Hexapod");
  ArduinoOTA.setPassword("10xTechClub");  // Set OTA password
  ArduinoOTA.begin();
  
  Serial.println();
  Serial.println("WiFi connected successfully!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Web Dashboard: http://");
  Serial.println(WiFi.localIP());
  Serial.print("WebSocket Port: 81");
  
  // Start Web Server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started on port 80");
  
  // Start WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
  
  Serial.println("=================================");
  Serial.println("36 Servo Hexapod Controller Ready!");
  Serial.println("Updated with corrected walking pattern");
  Serial.println("=================================");
}

void loop() {
  webSocket.loop();
  server.handleClient();
  ArduinoOTA.handle();

  // Handle walking movement
  if (isWalking) {
    if (millis() - lastStepTime > stepDelay) {
      executeWalkStep(currentStep);
      currentStep = (currentStep + 1) % 4; // Cycle through 4 steps
      lastStepTime = millis();
      
      // Broadcast walk status to all clients
      DynamicJsonDocument doc(200);
      doc["type"] = "walkStatus";
      doc["walking"] = isWalking;
      doc["step"] = currentStep;
      
      String message;
      serializeJson(doc, message);
      webSocket.broadcastTXT(message);
    }
  }

  // Optional: Add a heartbeat or status LED blink here
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 10000) { // Every 10 seconds
    Serial.println("System running... Connected clients: " + String(webSocket.connectedClients()));
    if (isWalking) {
      Serial.println("Status: Walking - Step " + String(currentStep + 1) + "/4");
    }
    lastHeartbeat = millis();
  }
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void setServoAngle(int servo, int angle) {
  // Clamp to safe range
  angle = constrain(angle, 0, 180);

  int jointNum = servo % 3;

  // --- Apply height offset (Femur & Tibia only) ---
  int adjustedAngle = angle;
  if (jointNum == 1 || jointNum == 2) {  // Femur or Tibia
    adjustedAngle += heightOffset;
    adjustedAngle = constrain(adjustedAngle, 0, 180);
  }

  // --- NO INVERSION HERE - the walk pattern already has correct values ---
  int finalAngle = adjustedAngle;

  // --- Convert to PWM ---
  int pwmValue = map(finalAngle, 0, 180, SERVOMIN, SERVOMAX);

  // --- Send to correct PCA9685 ---
  if (servo < 9) {
    pca2.setPWM(servo, 0, pwmValue);       // Left side (servos 0-8)
  } else {
    pca1.setPWM(servo - 9, 0, pwmValue);   // Right side (servos 9-17 -> channels 0-8)
  }

  // --- Save logical position (with height offset applied) ---
  servoPositions[servo] = adjustedAngle;
  servoPWMValues[servo] = pwmValue;
}
void executeWalkStep(int step) {
  Serial.println("Executing walk step: " + String(step + 1));

  int targetPattern[4][18];
  generateWalkPattern(targetPattern);

  // For reverse, use steps in reverse order: 3,2,1,0 instead of 0,1,2,3
  int actualStep = walkingForward ? step : (3 - step);

  const int interpSteps = 10; // higher = smoother, slower

  for (int s = 1; s <= interpSteps; s++) {
    for (int i = 0; i < 18; i++) {
      int startAngle = servoPositions[i];        
      int endAngle   = targetPattern[actualStep][i];   // Use actualStep instead of step
      int interpAngle = startAngle + (endAngle - startAngle) * s / interpSteps;

      setServoAngle(i, interpAngle);
    }
    delay(stepDelay / interpSteps);
  }

  // Send final update to UI
  for (int i = 0; i < 18; i++) {
    broadcastServoUpdate(i, servoPositions[i], servoPWMValues[i]);
  }
}

// 3. Add a moveBackward function (add this after your moveForward function)
void moveBackward() {
  if (!isWalking) {
    isWalking = true;
    walkingForward = false;  // Set direction to reverse
    currentStep = 0;
    lastStepTime = millis();
    Serial.println("Started walking backward");
  }
}

// 4. Update moveForward function to set direction (replace your existing moveForward)
void moveForward() {
  if (!isWalking) {
    isWalking = true;
    walkingForward = true;   // Set direction to forward
    currentStep = 0;
    lastStepTime = millis();
    Serial.println("Started walking forward");
  }
}

void stopMovement() {
  if (isWalking) {
    isWalking = false;
    Serial.println("Stopped walking");
    
    // Return to center position
    for (int i = 0; i < 18; i++) {
      setServoAngle(i, 90);
      delay(10);
    }
    
    // Broadcast stop status to all clients
    DynamicJsonDocument doc(200);
    doc["type"] = "walkStatus";
    doc["walking"] = false;
    doc["step"] = 0;
    
    String message;
    serializeJson(doc, message);
    webSocket.broadcastTXT(message);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client disconnected\n", num);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Client connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      
      // Send current servo positions to newly connected client
      for (int i = 0; i < 18; i++) {
        sendServoUpdate(num, i, servoPositions[i], servoPWMValues[i]);
        delay(10); // Small delay to prevent overwhelming the client
      }
      
      // Send current walk status
      DynamicJsonDocument doc(200);
      doc["type"] = "walkStatus";
      doc["walking"] = isWalking;
      doc["step"] = currentStep;
      
      String message;
      serializeJson(doc, message);
      webSocket.sendTXT(num, message);
      
      // Send current speed setting
      DynamicJsonDocument speedDoc(150);
      speedDoc["type"] = "speedUpdate";
      speedDoc["speed"] = stepDelay;
      
      String speedMessage;
      serializeJson(speedDoc, speedMessage);
      webSocket.sendTXT(num, speedMessage);

        DynamicJsonDocument strideDoc(150);
        strideDoc["type"] = "strideUpdate";
        strideDoc["stride"] = stride;

        String strideMessage;
        serializeJson(strideDoc, strideMessage);
        webSocket.sendTXT(num, strideMessage);

      break;
    }
    
    case WStype_TEXT: {
      String message = String((char*)payload);
      Serial.printf("[%u] Received: %s\n", num, message.c_str());
      
      // Parse JSON message
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, message);
      
      if (error) {
        Serial.println("JSON parsing failed: " + String(error.c_str()));
        return;
      }
      
      if (doc["type"] == "setServo") {
        int servo = doc["servo"];
        int angle = doc["angle"];
        
        if (servo >= 0 && servo < 18 && angle >= 0 && angle <= 180) {
          setServoAngle(servo, angle);
          
          // Broadcast update to all connected clients
          broadcastServoUpdate(servo, angle, servoPWMValues[servo]);
        } else {
          Serial.println("Invalid servo or angle values");
        }
      }
      else if (doc["type"] == "startWalk") {
        moveForward();
      }
      else if (doc["type"] == "startWalkBackward") {
        moveBackward();
      }
      else if (doc["type"] == "stopWalk") {
        stopMovement();
      }
      else if (doc["type"] == "setSpeed") {
        int newSpeed = doc["speed"];
        if (newSpeed >= 100 && newSpeed <= 1000) {
          stepDelay = newSpeed;
          Serial.println("Walking speed updated to: " + String(stepDelay) + "ms");
          
          // Broadcast speed update to all clients
          DynamicJsonDocument speedDoc(150);
          speedDoc["type"] = "speedUpdate";
          speedDoc["speed"] = stepDelay;
          
          String speedMessage;
          serializeJson(speedDoc, speedMessage);
          webSocket.broadcastTXT(speedMessage);
        } else {
          Serial.println("Invalid speed value: " + String(newSpeed));
        }
      }
    else if (doc["type"] == "setHeight") 
    {
        int newHeight = doc["height"];
        if (newHeight >= -30 && newHeight <= 30) 
        {  
                heightOffset = newHeight;
                prefs.putInt("heightOffset", heightOffset);   // ✅ Save persistently
                Serial.println("Height offset updated: " + String(heightOffset));

            // Immediately update all femur + tibia joints
            for (int i = 0; i < 18; i++) 
            {
                int jointNum = i % 3;
                if (jointNum == 1 || jointNum == 2) 
                {
                    setServoAngle(i, servoPositions[i]); // reapply with new offset
                    broadcastServoUpdate(i, servoPositions[i], servoPWMValues[i]);
                }
            }
        }
    }

    else if (doc["type"] == "setStride") {
    int newStride = doc["stride"];
    if (newStride >= 10 && newStride <= 90) {  
        stride = newStride;
        Serial.println("Stride updated to: " + String(stride) + " degrees");
        
        // Broadcast stride update to all clients
        DynamicJsonDocument strideDoc(150);
        strideDoc["type"] = "strideUpdate";
        strideDoc["stride"] = stride;
        
        String strideMessage;
        serializeJson(strideDoc, strideMessage);
        webSocket.broadcastTXT(strideMessage);
    } else {
        Serial.println("Invalid stride value: " + String(newStride));
    }
    }

      break;
    }
    
    case WStype_ERROR:
      Serial.printf("[%u] WebSocket Error\n", num);
      break;
      
    default:
      break;
  }
}

void sendServoUpdate(uint8_t clientNum, int servo, int angle, int pwm) {
  DynamicJsonDocument doc(200);
  doc["type"] = "servoUpdate";
  doc["servo"] = servo;
  doc["angle"] = angle;
  doc["pwm"] = pwm;
  
  String message;
  serializeJson(doc, message);
  webSocket.sendTXT(clientNum, message);
}

void broadcastServoUpdate(int servo, int angle, int pwm) {
  DynamicJsonDocument doc(200);
  doc["type"] = "servoUpdate";
  doc["servo"] = servo;
  doc["angle"] = angle;
  doc["pwm"] = pwm;
  
  String message;
  serializeJson(doc, message);
  webSocket.broadcastTXT(message);
}