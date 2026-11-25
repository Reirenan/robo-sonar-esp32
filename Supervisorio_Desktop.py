"""Painel de bancada para a ESP32, com comunicação USB direta."""
from concurrent.futures import ThreadPoolExecutor
from math import cos, radians, sin
import tkinter as tk
from tkinter import messagebox, ttk

from Abrir_supervisorio import SerialBridge

LABELS = {
    "idle": "Pronto", "checking": "Checando a frente", "advancing": "Avançando",
    "settling": "Parado, preparando varredura", "running": "Medindo", "moving": "Movendo para o próximo ponto",
    "homing": "Indo para o extremo esquerdo", "returning": "Voltando ao centro",
    "stopped": "Interrompida", "done": "Concluída", "error": "Falha no servo",
}
ACTIVE = {"checking", "advancing", "settling", "homing", "moving", "running", "returning"}


class Supervisory:
    def __init__(self, window):
        self.window = window
        self.bridge = SerialBridge("COM3")
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.pending_action = None
        self.queued_action = None
        self.state = None
        window.title("Supervisório de sonar · USB COM3")
        window.geometry("1040x640")
        window.minsize(650, 480)
        window.protocol("WM_DELETE_WINDOW", self.close)

        content = ttk.Frame(window, padding=16)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="Supervisório de sonar", font=("Segoe UI", 20, "bold")).pack(anchor="w")
        self.status = ttk.Label(content, text="Abrindo COM3…", wraplength=900)
        self.status.pack(anchor="w", pady=(6, 10))
        self.metrics = ttk.Label(content, text="Aguardando medições")
        self.metrics.pack(anchor="w")

        toolbar = ttk.Frame(content)
        toolbar.pack(fill="x", pady=12)
        self.buttons = {}
        for row, items in enumerate([
            [("start", "Iniciar varredura"), ("advance", "Avançar e mapear"), ("stop", "Parar")],
            [("clear", "Limpar mapa"), ("center", "Centralizar servo"), ("motor", "Testar motores")],
        ]):
            for column, (action, label) in enumerate(items):
                button = ttk.Button(toolbar, text=label, command=lambda a=action: self.command(a))
                button.grid(row=row, column=column, padx=(0, 8), pady=3, sticky="ew")
                self.buttons[action] = button

        views = ttk.Frame(content)
        views.pack(fill="both", expand=True)
        self.canvases = []
        for index, title in enumerate(("Sonar", "Mapa 2D local")):
            box = ttk.LabelFrame(views, text=title, padding=7)
            box.grid(row=0, column=index, sticky="nsew", padx=(0, 8))
            views.columnconfigure(index, weight=1, uniform="plots")
            canvas = tk.Canvas(box, background="#10202a", highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            canvas.bind("<Configure>", lambda event: self.draw())
            self.canvases.append(canvas)
        views.rowconfigure(0, weight=1)
        self.refresh_buttons()
        self.submit("state")

    def submit(self, action):
        self.pending_action = action
        self.pending = self.pool.submit(self.bridge.request, action)
        self.refresh_buttons()
        self.window.after(60, self.read_result)

    def read_result(self):
        if not self.pending.done():
            self.window.after(60, self.read_result)
            return
        try:
            status, payload = self.pending.result()
            if status != 200:
                raise ValueError(payload)
            if self.state is None:
                print("ESP32 conectada · USB COM3", flush=True)
            self.state = payload
            self.show_state()
        except (OSError, ValueError) as error:
            self.state = None
            self.status.configure(text=str(error))
            self.metrics.configure(text="Aguardando comunicação pela COM3")
        self.pending = None
        self.pending_action = None
        action, self.queued_action = self.queued_action, None
        if action is not None and (self.state is not None or action == "stop"):
            self.submit(action)
        else:
            self.refresh_buttons()
            self.window.after(400, self.poll)

    def poll(self):
        if self.pending is None:
            self.submit("state")

    def command(self, action):
        if action in {"advance", "motor"}:
            if not messagebox.askyesno("Movimentação", "O robô vai avançar. O caminho e o cabo estão livres?"):
                return
        if self.pending is None:
            self.submit(action)
        elif action == "stop" or (self.pending_action == "state" and self.queued_action is None):
            self.queued_action = action
            self.refresh_buttons()

    def refresh_buttons(self):
        state = self.state or {}
        connected = bool(state)
        active = state.get("state") in ACTIVE
        motors = state.get("motorBusy", False)
        command_pending = self.pending_action not in {None, "state"} or self.queued_action is not None
        for action, button in self.buttons.items():
            enabled = connected and (
                action == "stop" or (not active and not command_pending and (
                    action == "clear" or (not motors and (
                        (action in {"start", "center"} and state.get("servoReady", True))
                        or (action == "advance" and state.get("motorReady") and state.get("servoReady", True))
                        or (action == "motor" and state.get("motorReady"))
                    ))
                ))
            )
            button.configure(state="normal" if enabled else "disabled")

    def show_state(self):
        state = self.state
        points = state["points"]
        phase = LABELS.get(state["state"], state["state"])
        if state.get("advanceEnd") == "obstacle" and state.get("frontCm") is not None:
            phase += f' · parada por obstáculo a {state["frontCm"]:.1f} cm'
        if state.get("advanceEnd") == "noEcho":
            phase += " · sem eco frontal válido"
        if state["state"] == "done" and state.get("scanEchoes") is not None:
            if state["scanEchoes"] == 0:
                phase += " · nenhum pulso retornou pelo ECHO (D19)"
            elif points and all(distance is None for _, distance in points):
                phase += f' · {state["scanValidEchoes"]} ecos na faixa, sem concordância suficiente'
        self.status.configure(text=f'ESP32 conectada · USB COM3 · {phase}')
        last = "—"
        if points:
            angle, distance = points[-1]
            last = f"{distance:.1f} cm em {angle}°" if distance is not None else f"sem eco em {angle}°"
        self.metrics.configure(text=f'Ângulo comandado {state["angle"]}°   |   Última leitura {last}   |   {len(points)}/{state["total"]} medições')
        self.draw()

    def draw(self):
        state = self.state or {}
        maximum = state.get("maxCm", 400)
        for polar, canvas in zip((True, False), self.canvases):
            canvas.delete("all")
            width, height = canvas.winfo_width(), canvas.winfo_height()
            x0, y0 = width / 2, height - 27
            radius = min(x0 - 28, y0 - 25)
            if radius < 10:
                continue

            def position(angle, distance):
                scale = distance / maximum * radius
                return x0 - cos(radians(angle)) * scale, y0 - sin(radians(angle)) * scale

            grid = "#375360"
            canvas.create_line(x0-radius, y0, x0+radius, y0, fill=grid)
            for part in (0.25, 0.5, 0.75, 1):
                span = radius * part
                if polar:
                    canvas.create_arc(x0-span, y0-span, x0+span, y0+span, start=0, extent=180, style="arc", outline=grid)
                else:
                    canvas.create_line(x0-radius, y0-span, x0+radius, y0-span, fill=grid)
                    for x in (x0-span, x0+span):
                        canvas.create_line(x, y0-radius, x, y0, fill=grid)
                canvas.create_text(x0+19 if polar else x0-radius+16, y0-span+9,
                                   text=f"{maximum*part:g}", fill="#a9c0cc")
            if polar:
                for angle in range(0, 181, 30):
                    x, y = position(angle, maximum)
                    canvas.create_line(x0, y0, x, y, fill=grid)
                if state:
                    x, y = position(state["angle"], maximum)
                    canvas.create_line(x0, y0, x, y, fill="#53c8f8", width=2)
            for angle, distance in state.get("points", []):
                if distance is not None:
                    x, y = position(angle, distance)
                    canvas.create_oval(x-4, y-4, x+4, y+4, fill="#69edbf", outline="")
            canvas.create_polygon(x0, y0-8, x0-6, y0+5, x0+6, y0+5, fill="#eff6f8")
            canvas.create_text(x0, y0+16, text="Robô", fill="#eff6f8")

    def close(self):
        self.pool.shutdown(wait=True)
        self.bridge.close()
        self.window.destroy()


if __name__ == "__main__":
    app = tk.Tk()
    Supervisory(app)
    app.mainloop()
