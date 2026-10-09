#ifndef WEB_PAGE_H
#define WEB_PAGE_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-S3 Master Test Harness</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; margin: 0; padding: 20px; }
        h2 { text-align: center; color: #4CAF50; border-bottom: 2px solid #333; padding-bottom: 10px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1000px; margin: auto; }
        .card { background-color: #1e1e1e; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
        .btn { display: block; width: 100%; padding: 12px; margin: 8px 0; font-size: 16px; font-weight: bold; color: white; background-color: #3b82f6; border: none; border-radius: 5px; cursor: pointer; transition: 0.3s; }
        .btn:hover { background-color: #2563eb; }
        .btn.danger { background-color: #ef4444; }
        .btn.danger:hover { background-color: #dc2626; }
        .btn.warning { background-color: #f59e0b; }
        .btn.warning:hover { background-color: #d97706; }
        .btn.success { background-color: #10b981; }
        .btn.success:hover { background-color: #059669; }
        .data-row { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #333; }
        .data-label { color: #9ca3af; font-weight: bold; }
        .data-value { font-family: monospace; font-size: 18px; color: #6ee7b7; }
        .status-badge { padding: 5px 10px; border-radius: 15px; font-size: 14px; font-weight: bold; }
        .status-ok { background: #064e3b; color: #34d399; }
        .status-error { background: #7f1d1d; color: #fca5a5; }
    </style>
</head>
<body>
    <h2>ESP32-S3 TELEMETRY & FAULT INJECTION</h2>
    <div style="text-align:center; margin-bottom: 20px;">
        Cập nhật Dữ liệu (AJAX): <span id="ws-status" class="status-badge status-ok">Đang hoạt động...</span>
    </div>
    
    <div class="grid-container">
        <!-- Bảng điều khiển kịch bản -->
        <div class="card">
            <h3 style="color: #60a5fa;">🎯 Kịch bản quỹ đạo (Profiles)</h3>
            <button class="btn success" onclick="sendCommand('CMD_RUN_P1')">Profile 1 (Chạy thẳng 2m)</button>
            <button class="btn success" onclick="sendCommand('CMD_RUN_P2')">Profile 2 (Quay tại chỗ 360°)</button>
            <button class="btn success" onclick="sendCommand('CMD_RUN_P3')">Profile 3 (Step Response)</button>
            <button class="btn success" onclick="sendCommand('CMD_RUN_P4')">Profile 4 (S-Curve Sinusoial)</button>
            <button class="btn warning" onclick="sendCommand('CMD_STOP')" style="margin-top: 20px;">⏸ Tạm dừng lệnh (v=0, w=0)</button>
        </div>

        <!-- Bảng tạo lỗi chủ động -->
        <div class="card">
            <h3 style="color: #f87171;">⚠️ Kích hoạt lỗi (Fault Injection)</h3>
            <button class="btn danger" onclick="sendCommand('FAULT_TIMEOUT')">Ngắt kết nối (Mô phỏng đứt cáp)</button>
            <button class="btn danger" onclick="sendCommand('FAULT_CRC')">Gửi sai mã CRC16</button>
            <button class="btn warning" onclick="sendCommand('FAULT_JITTER')">Tạo độ trễ Jitter (Chờ 400ms)</button>
            <button class="btn danger" onclick="sendCommand('FAULT_ESTOP')" style="margin-top: 20px; font-size: 20px;">🛑 KHẨN CẤP (E-STOP)</button>
            <button class="btn" onclick="sendCommand('CMD_RESET')" style="margin-top: 10px; background:#4b5563;">Khôi phục (Reset FSM)</button>
        </div>

        <!-- Telemetry Data -->
        <div class="card" style="grid-column: span 2;">
            <h3 style="color: #34d399;">📊 Dữ liệu Phản hồi (STM32 -> ESP32)</h3>
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px;">
                <div>
                    <div class="data-row"><span class="data-label">Trạng thái STM32:</span> <span class="data-value" id="val_state">---</span></div>
                    <div class="data-row"><span class="data-label">Seq ID / Thời gian:</span> <span class="data-value" id="val_seq">0 / 0 ms</span></div>
                    <div class="data-row"><span class="data-label">Tọa độ X (m):</span> <span class="data-value" id="val_x">0.00</span></div>
                    <div class="data-row"><span class="data-label">Tọa độ Y (m):</span> <span class="data-value" id="val_y">0.00</span></div>
                    <div class="data-row"><span class="data-label">Góc xoay Theta (rad):</span> <span class="data-value" id="val_theta">0.00</span></div>
                </div>
                <div>
                    <div class="data-row"><span class="data-label">Vận tốc v (m/s):</span> <span class="data-value" id="val_v">0.00</span></div>
                    <div class="data-row"><span class="data-label">Vận tốc w (rad/s):</span> <span class="data-value" id="val_w">0.00</span></div>
                    <div class="data-row"><span class="data-label">Enc FL / FR:</span> <span class="data-value" id="val_enc1">0 / 0</span></div>
                    <div class="data-row"><span class="data-label">Enc RL / RR:</span> <span class="data-value" id="val_enc2">0 / 0</span></div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Gọi dữ liệu từ server liên tục mỗi 200ms bằng AJAX
        setInterval(function() {
            fetch('/telemetry')
            .then(response => response.json())
            .then(data => {
                document.getElementById('ws-status').innerText = "Đang kết nối tốt";
                document.getElementById('ws-status').className = "status-badge status-ok";
                
                document.getElementById('val_x').innerText = data.x.toFixed(3);
                document.getElementById('val_y').innerText = data.y.toFixed(3);
                document.getElementById('val_theta').innerText = data.theta.toFixed(3);
                document.getElementById('val_v').innerText = data.v.toFixed(3);
                document.getElementById('val_w').innerText = data.w.toFixed(3);
                document.getElementById('val_seq').innerText = `${data.seq} / ${data.time} ms`;
                document.getElementById('val_enc1').innerText = `${data.efl} / ${data.efr}`;
                document.getElementById('val_enc2').innerText = `${data.erl} / ${data.err}`;
                
                let stateStr = "UNKNOWN";
                if (data.state == 0) stateStr = "INIT";
                else if (data.state == 1) stateStr = "READY";
                else if (data.state == 2) stateStr = "RUNNING";
                else if (data.state == 3) stateStr = "FAULT_STOP";
                document.getElementById('val_state').innerText = stateStr;
            })
            .catch(error => {
                document.getElementById('ws-status').innerText = "Mất kết nối!";
                document.getElementById('ws-status').className = "status-badge status-error";
            });
        }, 200);

        // Gửi lệnh xuống Server
        function sendCommand(cmd) {
            fetch('/command?cmd=' + cmd)
                .then(response => console.log('Đã gửi lệnh:', cmd))
                .catch(error => console.error('Lỗi khi gửi lệnh:', error));
        }
    </script>
</body>
</html>
)rawliteral";

#endif
