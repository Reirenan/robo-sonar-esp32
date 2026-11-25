"""Supervisório autônomo para Windows. A comunicação serial roda fora do Tk."""
import math
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Thread
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from Abrir_supervisorio import SerialBridge
from ControleAutonomo import Navigator
from Navegacao import CELL, FREE, Map, MapaAutomatico, OCCUPIED, midpoint, relative

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
        body = ttk.Frame(root,padding=(16,12))
        body.pack(fill="both",expand=True)
        toolbar = ttk.Frame(body)
        toolbar.pack(fill="x")
        ttk.Label(toolbar,text="Mapa da arena",font=("Segoe UI",17,"bold")).pack(side="left",padx=(0,20))
        self.start_button = ttk.Button(toolbar,text="Iniciar",command=self.start)
        self.start_button.pack(side="left",padx=(0,6))
        self.stop_button = ttk.Button(toolbar,text="Parar (Esc)",command=self.stop.set,state="disabled")
        self.stop_button.pack(side="left",padx=(0,6))
        self.open_button = ttk.Button(toolbar,text="Abrir mapa",command=self.open_map)
        self.open_button.pack(side="right")
        self.cards = {}
        strip = tk.Frame(body,bg="#e8edf2")
        strip.pack(fill="x",pady=(12,0))
        for column,(direction,label) in enumerate((("l","ESQUERDA"),("f","FRENTE"),("r","DIREITA"))):
            strip.columnconfigure(column,weight=1,uniform="direcoes")
            card = tk.Frame(strip,bg="#f5f8fb",padx=12,pady=8)
            card.grid(row=0,column=column,sticky="nsew",padx=(0 if column==0 else 4,0))
            title = tk.Label(card,text=label,bg="#f5f8fb",fg="#415568",font=("Segoe UI",9,"bold"),anchor="w")
            value = tk.Label(card,text="—",bg="#f5f8fb",fg="#112838",font=("Segoe UI",19,"bold"),anchor="w")
            reason = tk.Label(card,text="Aguardando leitura",bg="#f5f8fb",fg="#415568",anchor="w",justify="left")
            for widget in (title,value,reason):
                widget.pack(fill="x")
            self.cards[direction] = card,title,value,reason

        self.status = ttk.Label(body,text="Pronto. Posicione o robô em área livre antes de iniciar.",
                                wraplength=760,padding=(0,9))
        self.status.pack(fill="x")
        view = ttk.Frame(body)
        view.pack(fill="both",expand=True)
        self.legend = ttk.Label(body,padding=(0,8),
                                text="Azul: livre   ·   Vermelho: obstáculo   ·   Âmbar: em revisão   ·   Violeta: bloqueio da manobra   ·   Verde: trajetória")
        self.legend.pack(side="bottom",anchor="w")
        self.viewport = None
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
        self.viewport = None
        self.map = MapaAutomatico()
        self.render_directions()
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
        result.tentative = set(model.quick_hits if model.quick_active else model.tentative)
        result.clearing = dict(model.clearing)
        result.awaiting_echo = set(model.awaiting_echo)
        result.directions = dict(model.directions)
        result.selected_direction = model.selected_direction
        result.selected_gap = model.selected_gap
        result.frontier_target = model.frontier_target
        result.frontier_path = list(model.frontier_path)
        result.maneuver_blockers = {d:set(keys) for d,keys in model.maneuver_blockers.items()}
        result.localized = model.localized
        result.passages = {key: value.copy() for key,value in model.passages.items()}
        return result

    def poll(self):
        try:
            while True:
                message, self.map, done = self.events.get_nowait()
                self.status.configure(text=message)
                self.render_directions()
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
            self.viewport = None
            self.render_directions()
            self.status.configure(text="Mapa salvo carregado. Para iniciar outra exploração, reposicione o robô na origem.")
            self.draw()
        except (OSError, ValueError, TypeError, KeyError) as error:
            messagebox.showerror("Mapa indisponível", str(error))

    def open_map(self):
        path = filedialog.askopenfilename(filetypes=[("Mapa JSON", "*.json")])
        if path:
            self.read_map(path)

    def render_directions(self):
        readings = getattr(self.map,"directions",{})
        selected = getattr(self.map,"selected_direction",None)
        for direction,(frame,title,value,reason) in self.cards.items():
            distance,label = readings.get(direction,(None,"Aguardando leitura"))
            active = selected == direction
            background = "#d2efe4" if active else "#f5f8fb"
            for widget in (frame,title,value,reason):
                widget.configure(bg=background)
            value.configure(text="—" if distance is None else f"{distance:.0f} cm")
            reason.configure(text=("Escolhida · " if active else "")+label)

    def draw(self):
        c, m = self.canvas, self.map
        c.delete("all")
        if m is None:
            return
        x0,x1,y0,y1 = m.bounds if isinstance(m, MapaAutomatico) else m.arena.walls
        pad = max(25,m.arena.radius+CELL)
        bounds = (x0-pad,x1+pad,y0-pad,y1+pad)
        if self.viewport:
            old = self.viewport
            bounds = (min(bounds[0],old[0]),max(bounds[1],old[1]),
                      min(bounds[2],old[2]),max(bounds[3],old[3]))
        self.viewport = bounds
        x0,x1,y0,y1 = bounds
        width, height = c.winfo_width(), c.winfo_height()
        scale = min((width-80)/(x1-x0), (height-64)/(y1-y0))
        if scale <= 0:
            return

        def xy(x,y):
            return (width-(x1-x0)*scale)/2+(x-x0)*scale, (height+(y1-y0)*scale)/2-(y-y0)*scale

        for (i,j), value in m.cells.items():
            if value not in (FREE,OCCUPIED):
                continue
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,fill="#183748" if value == FREE else "#df6675",outline="")
        for i, j in m.tentative:
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,outline="#ffad5a",width=2)
        blockers = getattr(m,"maneuver_blockers",{})
        for i,j in set().union(*blockers.values()) if blockers else ():
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,outline="#b8a2ff",width=2)
        for i,j in getattr(m,"awaiting_echo",set()) - m.clearing.keys():
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,outline="#f5c667",width=1,dash=(3,3))
        for (i,j), votes in m.clearing.items():
            ax,ay = xy(i*CELL,(j+1)*CELL)
            bx,by = xy((i+1)*CELL,j*CELL)
            c.create_rectangle(ax,ay,bx,by,outline="#f5c667",width=2)
            if scale*CELL >= 20:
                c.create_text((ax+bx)/2,(ay+by)/2,text=f"{votes}/2",fill="#fff1cc",font=("Segoe UI",8,"bold"))
        if not isinstance(m, MapaAutomatico):
            left,right,back,front = m.arena.walls
            c.create_rectangle(*xy(left,front),*xy(right,back),outline="#637d89",dash=(4,4))
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
        route = getattr(m,"frontier_path",[])
        if route:
            c.create_line(*xy(m.pose.x,m.pose.y), *[v for point in route for v in xy(*point)],
                          fill="#80b9ff",dash=(6,4),width=2)
        target = getattr(m,"frontier_target",None)
        if target is not None:
            tx,ty = xy(*midpoint(target))
            c.create_oval(tx-6,ty-6,tx+6,ty+6,outline="#80b9ff",width=2)
            c.create_text(tx+9,ty-9,anchor="w",text="Fronteira",fill="#b6d6ff")
        selection = getattr(m, "selected_gap", None)
        if selection:
            pose, gap = selection
            start = xy(pose.x, pose.y)
            for angle in (gap.first, gap.center, gap.last):
                dx,dy = relative(angle, min(gap.depth, 100), pose.heading)
                end = xy(pose.x + dx, pose.y + dy)
                if angle == gap.center:
                    c.create_line(*start, *end, fill="#5ce4ea", width=3, arrow="last")
                else:
                    c.create_line(*start, *end, fill="#5ce4ea", dash=(5,3))
        ox,oy = xy(m.pose.x,m.pose.y)
        dx,dy = relative(90, m.arena.radius, m.pose.heading)
        c.create_oval(ox-4,oy-4,ox+4,oy+4,fill="white",outline="")
        c.create_line(ox,oy,*xy(m.pose.x+dx,m.pose.y+dy),arrow="last",fill="white",width=2)
        c.create_text(width-18,height-17,anchor="e",
                      text="Pose confirmada" if m.localized else "Pose estimada",
                      fill="#dce7f2" if m.localized else "#f5c667")
        c.create_line(18,height-17,18+50*scale,height-17,fill="#dce7f2",width=2)
        c.create_text(25+50*scale,height-17,anchor="w",text="50 cm" if m.localized else "≈ 50 cm",fill="white")

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
