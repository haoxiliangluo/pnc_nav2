#!/usr/bin/env python3
"""Web teleop for pnc_diff_drive (mapping control).

Serves a control page on http://localhost:8090 and publishes /cmd_vel.
Includes a watchdog: if no command arrives within 0.6s the robot stops
(safe against page refresh / network loss).

Usage (inside pnc_nav container, isolated domain):
  export ROS_DOMAIN_ID=43
  python3 /home/hao/pnc_nav2/scripts/web_teleop.py
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PORT = 8091
WATCHDOG_S = 0.6

PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PNC 机器人控制台</title>
<style>
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         display:flex; flex-direction:column; align-items:center; margin:0; padding:20px; }
  h1 { font-size:20px; margin:0 0 16px; }
  .pad { display:grid; grid-template-columns:repeat(3,90px); gap:10px; }
  .btn { height:90px; font-size:18px; border:none; border-radius:12px;
         background:#2a2a2a; color:#eee; cursor:pointer; user-select:none; touch-action:none; }
  .btn:active { background:#4a7; }
  .btn.stop { background:#a33; }
  .btn.stop:active { background:#f55; }
  #status { margin-top:14px; font-size:13px; color:#8a8; }
  #speed { margin-top:14px; width:240px; }
  label { font-size:13px; }
</style>
</head>
<body>
<h1>🕹️ PNC 机器人控制台</h1>
<div class="pad">
  <div></div>
  <button class="btn" id="fwd">⬆ 前进</button>
  <div></div>
  <button class="btn" id="left">⬅ 左转</button>
  <button class="btn stop" id="stop">⏹ 停止</button>
  <button class="btn" id="right">➡ 右转</button>
  <div></div>
  <button class="btn" id="back">⬇ 后退</button>
  <div></div>
</div>
<label for="speed">速度: <span id="spdval">0.15</span> m/s</label>
<input id="speed" type="range" min="0.05" max="1.0" step="0.05" value="0.15">
<div id="status">未连接</div>
<script>
const spd = document.getElementById('speed');
const spdval = document.getElementById('spdval');
const status = document.getElementById('status');
spd.oninput = () => spdval.textContent = spd.value;

async function send(vx, wz, hold) {
  try {
    const r = await fetch('/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({vx, wz, hold: !!hold})
    });
    status.textContent = hold ? ('HOLD vx=' + vx + ' wz=' + wz) : 'stop';
    status.style.color = '#8a8';
  } catch (e) {
    status.textContent = '连接失败!';
    status.style.color = '#f66';
  }
}
function bind(el, dir) {
  const on = (e) => { e.preventDefault(); const s = parseFloat(spd.value);
    send(dir === 'f' ? s : dir === 'b' ? -s : 0, dir === 'l' ? 0.4 : dir === 'r' ? -0.4 : 0, true); };
  const off = (e) => { e.preventDefault(); send(0, 0, false); };
  el.addEventListener('mousedown', on);
  el.addEventListener('mouseup', off);
  el.addEventListener('mouseleave', off);
  el.addEventListener('touchstart', on, {passive:false});
  el.addEventListener('touchend', off);
  el.addEventListener('touchcancel', off);
}
bind(document.getElementById('fwd'), 'f');
bind(document.getElementById('back'), 'b');
bind(document.getElementById('left'), 'l');
bind(document.getElementById('right'), 'r');
bind(document.getElementById('stop'), 's');
setInterval(() => { if (status.textContent === '未连接') send(0,0); }, 3000);
</script>
</body>
</html>"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_cmd = 0.0
        self.hold_vx = 0.0
        self.hold_wz = 0.0
        self.holding = False
        self.lock = threading.Lock()

    def set_cmd(self, vx, wz, hold=False):
        with self.lock:
            self.last_cmd = time.monotonic()
            self.holding = hold
            if hold:
                self.hold_vx = float(vx)
                self.hold_wz = float(wz)
            self.publish_(float(vx), float(wz))

    def publish_(self, vx, wz):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub.publish(t)

    def run_loop(self):
        """Continuous publisher: while holding, keep sending; watchdog stops on silence."""
        while rclpy.ok():
            time.sleep(0.1)
            with self.lock:
                if self.holding:
                    self.publish_(self.hold_vx, self.hold_wz)
                elif time.monotonic() - self.last_cmd > WATCHDOG_S:
                    self.publish_(0.0, 0.0)


node = None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/cmd':
            length = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(length) or b'{}')
            node.set_cmd(data.get('vx', 0.0), data.get('wz', 0.0),
                         hold=data.get('hold', False))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        pass


def main():
    global node
    rclpy.init()
    node = TeleopNode()
    threading.Thread(target=node.run_loop, daemon=True).start()
    server = HTTPServer(('0.0.0.0', PORT), Handler)
    print(f'Web teleop on http://localhost:{PORT} (ROS_DOMAIN_ID={__import__("os").environ.get("ROS_DOMAIN_ID","0")})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
