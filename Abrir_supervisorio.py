"""Serve the supervisory panel and relay its commands to the ESP32 over USB."""
import argparse
from base64 import b64encode
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import subprocess
from threading import Lock, Timer
from urllib.parse import parse_qs, urlsplit
import webbrowser

HOST = "127.0.0.1"
PORT = 8765
HEADER = Path(__file__).resolve().parent / "SonarRobotica" / "Supervisory.h"
ACTIONS = {"start", "stop", "clear", "center", "motor", "advance"}
WORKER = r"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$serial = [IO.Ports.SerialPort]::new('__PORT__', 115200)
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 150
$serial.WriteTimeout = 350
try {
  while ($null -ne ($request = [Console]::ReadLine())) {
    try {
      if (-not $serial.IsOpen) { $serial.Open(); $serial.DiscardInBuffer() }
      if ($request -notmatch '^@(\d+) (\w+)$') { throw 'Comando inválido.' }
      $expected = '@'+$Matches[1]+' '
      $serial.WriteLine($request)
      $timeout = [Diagnostics.Stopwatch]::StartNew()
      $response = $null
      while ($timeout.ElapsedMilliseconds -lt 1500) {
        try {
          $line = $serial.ReadLine().Trim()
          if ($line.StartsWith($expected)) { $response = $line; break }
        } catch [TimeoutException] {}
      }
      if ($null -eq $response) { throw 'A ESP32 não respondeu. Verifique se o firmware do sonar foi gravado.' }
      [Console]::WriteLine($response)
    } catch {
      if ($serial.IsOpen) { $serial.Close() }
      [Console]::WriteLine('! '+$_.Exception.Message.Replace([char]10,' ').Replace([char]13,' '))
    }
  }
} finally { $serial.Dispose() }
"""


class SerialBridge:
    def __init__(self, port):
        self.port = port
        self.worker = None
        self.lock = Lock()
        self.sequence = 0

    def request(self, action):
        with self.lock:
            if self.worker is None or self.worker.poll() is not None:
                source = WORKER.replace("__PORT__", self.port)
                encoded = b64encode(source.encode("utf-16-le")).decode("ascii")
                self.worker = subprocess.Popen(
                    ["powershell.exe", "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    encoding="utf-8", errors="replace", creationflags=subprocess.CREATE_NO_WINDOW,
                )
            self.sequence += 1
            request = f"@{self.sequence} {action}\n"
            self.worker.stdin.write(request)
            self.worker.stdin.flush()
            line = self.worker.stdout.readline().strip()
            if line.startswith("! "):
                raise OSError(line[2:])
            match = re.fullmatch(rf"@{self.sequence} (\d+) (.+)", line)
            if not match:
                raise OSError("A ponte USB perdeu a conexão.")
            status = int(match[1])
            if status == 200:
                state = json.loads(match[2])
                state["connection"] = f"USB {self.port}"
                return status, state
            return status, match[2]

    def close(self):
        with self.lock:
            if self.worker is not None:
                self.worker.stdin.close()
                try:
                    self.worker.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.worker.terminate()
                    self.worker.wait()


def page():
    header = HEADER.read_text(encoding="utf-8")
    html = header.split('R"SONAR(', 1)[1].split(')SONAR";', 1)[0]
    marker = "const usbMode ="
    if marker not in html:
        raise ValueError("A interface do firmware não contém a configuração USB.")
    html = re.sub(r"const usbMode = [^;]+;", "const usbMode = false;", html, count=1)
    html = html.replace(": 'ESP32 conectada')", ": 'ESP32 conectada · '+(state?.connection ?? 'USB'))")
    return html.encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def send_body(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def relay(self, action):
        try:
            status, result = self.server.bridge.request(action)
        except (OSError, ValueError) as error:
            status, result = 503, f"Não foi possível ler {self.server.bridge.port}. {error}"
        if isinstance(result, dict):
            self.send_body(status, json.dumps(result).encode("utf-8"), "application/json")
        else:
            self.send_body(status, result.encode("utf-8"), "text/plain")

    def do_GET(self):
        if self.path == "/":
            self.send_body(200, page(), "text/html")
        elif self.path == "/state":
            self.relay("state")
        else:
            self.send_error(404)

    def do_POST(self):
        url = urlsplit(self.path)
        action = parse_qs(url.query).get("action", [""])[0]
        expected = f"{HOST}:{self.server.server_port}"
        if self.headers.get("Host") != expected or self.headers.get("X-Robot-Command") != "local":
            self.send_error(403)
        elif url.path != "/command" or action not in ACTIONS:
            self.send_error(400)
        else:
            self.relay(action)

    def log_message(self, fmt, *args):
        if self.path != "/state":
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-open", action="store_true", help="não abrir o navegador")
    parser.add_argument("--probe", action="store_true", help="apenas consultar a ESP32")
    parser.add_argument("--com", default="COM3", help="porta da ESP32")
    parser.add_argument("--port", type=int, default=PORT, help="porta HTTP local")
    args = parser.parse_args()
    com = args.com.upper()
    if not re.fullmatch(r"COM[1-9]\d*", com):
        parser.error("Informe uma porta como COM3.")
    bridge = SerialBridge(com)
    try:
        if args.probe:
            print(json.dumps(bridge.request("state"), ensure_ascii=False), flush=True)
            return
        with ThreadingHTTPServer((HOST, args.port), Handler) as server:
            server.bridge = bridge
            url = f"http://{HOST}:{server.server_port}/"
            print(f"Supervisório em {url} | ESP32 em {com}", flush=True)
            if not args.no_open:
                Timer(0.25, webbrowser.open, args=(url,)).start()
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
