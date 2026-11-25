"""Supervisório autônomo para Windows. A comunicação serial roda fora do Tk."""
import math
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Thread
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from Abrir_supervisorio import SerialBridge
from ControleAutonomo import Navigator
from Navegacao import CELL, FREE, Map, MapaAutomatico, OCCUPIED, relative

SESSION = Path(__file__).resolve().with_name("mapa-sessao.json")


class Panel:
    def __init__(self, root):
        self.root = root
        self.map = None
        self.events = Queue()
        self.stop = Event()
        self.thread = None
        self.target = SESSION
        root.title("Robô sonar · mapa da arena")
        root.geometry("1120x760")
        root.minsize(810, 550)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda _: self.stop.set())
        body = ttk.Panedwindow(root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=12, pady=12)
        form = ttk.Frame(body, padding=8)
        view = ttk.Frame(body)
        body.add(form, weight=0)
        body.add(view, weight=1)
        ttk.Label(form, text="Exploração", font=("Segoe UI",15,"bold")).pack(anchor="w")
        ttk.Label(form, text="Posicione o robô em área livre.\nA partida será a origem (0,0).",
                  justify="left").pack(anchor="w", pady=(6,14))
        self.start_button = ttk.Button(form, text="Iniciar", command=self.start)
        self.start_button.pack(fill="x", pady=4)
        self.stop_button = ttk.Button(form, text="Parar  (Esc)", command=self.stop.set, state="disabled")
        self.stop_button.pack(fill="x", pady=4)
        self.open_button = ttk.Button(form, text="Abrir mapa JSON", command=self.open_map)
        self.open_button.pack(fill="x", pady=4)
        self.status = ttk.Label(view, text="Pronto para explorar. O chassi começa em (0,0), apontado para +Y.",
                                wraplength=620, padding=(8,6))
        self.status.pack(fill="x")
        self.legend = ttk.Label(view, padding=8,
                                text="Cinza escuro: desconhecido   ·   Azul: livre   ·   Vermelho: ocupado\n"
                                     "Verde: trajetória   ·   Amarelo: passagem provável   ·   Branco: posição estimada\nLaranja: eco com posição incerta")
        self.legend.pack(anchor="w")
        self.canvas = tk.Canvas(view, bg="#0b151c", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.draw())
        if SESSION.exists():
            self.read_map(SESSION)
        self.root.after(80, self.poll)

    def busy(self):
        return self.thread is not None

    def start(self):
        if self.busy():
            return
        self.map = MapaAutomatico()
        self.target = SESSION
        self.start_worker(self.run, self.map)

    def start_worker(self, function, *args):
        self.stop.clear()
        self.start_button.configure(state="disabled")
        self.open_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.thread = Thread(target=function, args=args, daemon=True)
        self.thread.start()

    def run(self, model):
        bridge = SerialBridge("COM3")

        def notify(message):
            model.save(self.target)
            # O Tk recebe uma imagem lógica independente da grade, sem ler
            # estruturas ainda em mutação na thread da comunicação serial.
            self.events.put((message, self.snapshot(model), False))

        try:
            result = Navigator(bridge, model, self.stop, notify).run()
            model.save(self.target)
        except (OSError, ValueError, RuntimeError) as error:
            result = f"Exploração interrompida: {error}"
        finally:
            bridge.close()
        self.events.put((result, self.snapshot(model), True))

    @staticmethod
    def snapshot(model):
        result = MapaAutomatico(model.profile)
        result.pose, result.trail = model.pose, list(model.trail)
        result.cells = model.cells.copy()
        result.tentative = set(model.tentative)
        result.localized = model.localized
        result.passages = {key: value.copy() for key,value in model.passages.items()}
        return result

    def poll(self):
        try:
            while True:
                message, self.map, done = self.events.get_nowait()
                self.status.configure(text=message)
                self.draw()
                if done:
                    self.thread = None
                    self.start_button.configure(state="normal")
                    self.open_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
        except Empty:
            pass
        self.root.after(80, self.poll)

    def read_map(self, path):
        try:
            self.map = Map.load(path)
            self.status.configure(text="Mapa salvo carregado. Para iniciar outra exploração, reposicione o robô na origem.")
            self.draw()
        except (OSError, ValueError, TypeError, KeyError) as error:
            messagebox.showerror("Mapa indisponível", str(error))

    def open_map(self):
        path = filedialog.askopenfilename(filetypes=[("Mapa JSON", "*.json")])
        if path:
            self.read_map(path)

    def draw(self):
        c, m = self.canvas, self.map
        c.delete("all")
        if m is None:
            return
        x0,x1,y0,y1 = m.bounds if isinstance(m, MapaAutomatico) else m.arena.walls
        pad = max(25, m.arena.radius + CELL)
        width, height = c.winfo_width(), c.winfo_height()
        scale = min((width-54)/(x1-x0+2*pad), (height-50)/(y1-y0+2*pad))
        if scale <= 0:
            return

        def xy(x,y):
            return 30+(x-x0+pad)*scale, height-30-(y-y0+pad)*scale

        for (i,j), value in m.cells.items():
            if value not in (FREE,OCCUPIED):
                continue
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,fill="#234c62" if value == FREE else "#ed766e",outline="")
        for i, j in m.tentative:
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,outline="#ffad5a",width=2)
        if not isinstance(m, MapaAutomatico):
            c.create_rectangle(*xy(x0,y1),*xy(x1,y0),outline="#637d89",dash=(4,4))
        marks = 100 if scale < .8 else 50
        for x in range(math.ceil(x0/marks)*marks, math.floor(x1/marks)*marks+1, marks):
            sx,sy = xy(x,y0)
            c.create_text(sx,sy+11,text=f"{x} cm",fill="#bacece")
        for y in range(math.ceil(y0/marks)*marks, math.floor(y1/marks)*marks+1, marks):
            sx,sy = xy(x0,y)
            c.create_text(sx-6,sy,text=f"{y} cm",anchor="e",fill="#bacece")
        if len(m.trail) > 1:
            c.create_line(*[v for p in m.trail for v in xy(p.x,p.y)],fill="#71e0a5",width=2)
        for opening in getattr(m,"passages",{}).values():
            sx,sy = xy(*opening["at"])
            crossed = opening["crossed"]
            c.create_oval(sx-5,sy-5,sx+5,sy+5,fill="#71e0a5" if crossed else "#f4c65f",outline="")
            c.create_text(sx+8,sy-10,anchor="w",text="Passagem atravessada" if crossed else "Passagem provável",fill="#f4c65f")
        if m.exit:
            sx,sy = xy(*m.exit["at"])
            c.create_oval(sx-6,sy-6,sx+6,sy+6,fill="#71e0a5" if m.exited else "#f4c65f",outline="")
            c.create_text(sx+9,sy-11,anchor="w",text="Saída confirmada" if m.exited else "Saída candidata",fill="#f4c65f")
        ox,oy = xy(m.pose.x,m.pose.y)
        dx,dy = relative(90, m.arena.radius, m.pose.heading)
        c.create_oval(ox-4,oy-4,ox+4,oy+4,fill="white",outline="")
        c.create_line(ox,oy,*xy(m.pose.x+dx,m.pose.y+dy),arrow="last",fill="white",width=2)
        c.create_line(24,height-15,24+50*scale,height-15,fill="white",width=3)
        c.create_text(29+50*scale,height-15,anchor="w",text="50 cm" if m.localized else "≈ 50 cm",fill="white")

    def close(self):
        if self.busy():
            self.stop.set()
            self.root.after(100, self.close)
        else:
            self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Panel(root)
    root.mainloop()
