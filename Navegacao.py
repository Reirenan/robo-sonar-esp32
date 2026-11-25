"""Geometria, localização e planejamento da arena. Todas as distâncias usam cm."""
from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from statistics import median

CELL = 10
MARGIN = 5
UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@dataclass(frozen=True)
class Arena:
    width: float
    length: float
    from_left: float
    from_back: float
    robot_width: float
    robot_length: float
    speed: float
    left_rate: float
    right_rate: float

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError("Preencha as medidas e velocidades com números positivos.")
        if not (self.width <= 1000 and self.length <= 1000 and
                self.radius < self.from_left < self.width - self.radius and
                self.radius < self.from_back < self.length - self.radius):
            raise ValueError("Arena limitada a 1000 cm por eixo. A origem precisa ter espaço para o chassi girar.")
        if not (10 <= self.speed <= 150 and 30 <= self.left_rate <= 400 and 30 <= self.right_rate <= 400):
            raise ValueError("Use velocidade de 10–150 cm/s e giro de 30–400 graus/s, medidos no piso.")

    @property
    def radius(self):
        return math.hypot(self.robot_width, self.robot_length) / 2 + MARGIN

    @property
    def walls(self):
        # Coordenadas negativas são normais, pois a origem é a posição inicial.
        return (-self.from_left, self.width - self.from_left,
                -self.from_back, self.length - self.from_back)


@dataclass(frozen=True)
class Pose:
    x: float = 0
    y: float = 0
    heading: float = 0


def relative(angle, distance, heading=0):
    """0° do sonar é esquerda. Na arena, 0° de rumo é +Y e 90° é +X."""
    theta = math.radians(heading + angle - 90)
    return distance * math.sin(theta), distance * math.cos(theta)


def cell(x, y):
    return math.floor(x / CELL), math.floor(y / CELL)


def midpoint(key):
    return tuple((part + .5) * CELL for part in key)


def delta(target, current):
    return (target - current + 180) % 360 - 180


def returns(points):
    for angle, cm in points:
        if cm is not None and 2 <= cm <= 400:
            yield angle, cm


def localize(arena, pose, points):
    """Ajusta pose só se duas paredes ortogonais explicam retornos coerentes."""
    xmin, xmax, ymin, ymax = arena.walls
    best = None
    for shift in range(-12, 13, 3):
        heading = (pose.heading + shift) % 360
        xs, ys = [], []
        for angle, cm in returns(points):
            dx, dy = relative(angle, cm, heading)
            x, y = pose.x + dx, pose.y + dy
            candidates = []
            if ymin - 2 <= y <= ymax + 2 and abs(dx) >= cm * .45:
                candidates += [(wall - x, xs) for wall in (xmin, xmax)]
            if xmin - 2 <= x <= xmax + 2 and abs(dy) >= cm * .45:
                candidates += [(wall - y, ys) for wall in (ymin, ymax)]
            if candidates:
                correction, axis = min(candidates, key=lambda item: abs(item[0]))
                if abs(correction) <= 18:
                    axis.append(correction)
        if min(len(xs), len(ys)) < 2:
            continue
        cx, cy = median(xs), median(ys)
        ix = [x for x in xs if abs(x - cx) <= 6]
        iy = [y for y in ys if abs(y - cy) <= 6]
        if min(len(ix), len(iy)) < 2:
            continue
        score = (len(ix) + len(iy), -sum(abs(x-cx) for x in ix)-sum(abs(y-cy) for y in iy)-abs(shift)*.2)
        if best is None or score > best[0]:
            best = (score, Pose(pose.x + median(ix), pose.y + median(iy), heading))
    return best[1] if best else None


def ray_cells(x, y, dx, dy, limit):
    # Amostra mais fina que a grade. Um conjunto garante um voto por varredura,
    # mesmo que vários feixes do mesmo lote passem pela célula.
    length = math.hypot(dx, dy)
    steps = max(1, math.ceil(length / min(2, CELL / 4)))
    return {
        cell(x + dx * i / steps, y + dy * i / steps)
        for i in range(steps)
        if length - length * i / steps >= limit
    }


class Map:
    def __init__(self, arena):
        self.arena = arena
        self.pose = Pose()
        self.trail = [self.pose]
        self.cells = {}
        self.clearing = {}
        self.tentative = set()
        self.visited = {}
        self.localized = False
        self.exit = None
        self.exited = False
        # Ao configurar a origem, o operador confirma que há espaço para o giro.
        self.cells.update(dict.fromkeys(self.footprint(0, 0), FREE))

    def footprint(self, x, y):
        r = self.arena.radius
        for i in range(math.floor((x-r)/CELL), math.floor((x+r)/CELL)+1):
            for j in range(math.floor((y-r)/CELL), math.floor((y+r)/CELL)+1):
                # Distância até o quadrado da célula, inclusive quando o disco
                # cruza apenas uma quina. Usar o centro subestimaria a folga.
                dx = max(i*CELL-x, 0, x-(i+1)*CELL)
                dy = max(j*CELL-y, 0, y-(j+1)*CELL)
                if math.hypot(dx, dy) <= r:
                    yield i, j

    def scan(self, points):
        pose = self.pose
        corrected = localize(self.arena, pose, points)
        self.localized = corrected is not None
        if corrected:
            self.pose = pose = corrected
            # Corrige a última posição da trajetória com a referência do sonar.
            if len(self.trail) > 1:
                self.trail[-1] = pose
        hits, cleared = set(), set()
        for angle, cm in returns(points):
            dx, dy = relative(angle, cm, pose.heading)
            ex, ey = pose.x + dx, pose.y + dy
            if self.localized:
                xmin, xmax, ymin, ymax = self.arena.walls
                # O ruído residual junto à parede não deve espalhar seus
                # retornos por lados diferentes da mesma linha do perímetro.
                wx = min((xmin, xmax), key=lambda wall: abs(ex-wall))
                wy = min((ymin, ymax), key=lambda wall: abs(ey-wall))
                if ymin <= ey <= ymax and abs(ex-wx) <= 3:
                    ex = wx
                if xmin <= ex <= xmax and abs(ey-wy) <= 3:
                    ey = wy
            hits.add(cell(ex, ey))
            cleared.update(ray_cells(pose.x, pose.y, dx, dy, CELL))
        self.tentative = set() if self.localized else hits
        if self.localized:
            for key in hits:
                self.cells[key] = OCCUPIED
                self.clearing.pop(key, None)
            for key in cleared - hits:
                if self.cells.get(key) == OCCUPIED:
                    votes = self.clearing.get(key, 0) + 1
                    if votes < 2:
                        self.clearing[key] = votes
                        continue
                    self.clearing.pop(key, None)
                self.cells[key] = FREE
            self.exit = self.find_exit()
        visit = (*cell(pose.x, pose.y), round(pose.heading / 30) % 12)
        self.visited[visit] = self.visited.get(visit, 0) + 1

    def clear_disk(self, x, y):
        return all(self.cells.get(key) == FREE for key in self.footprint(x, y))

    def inside(self, x, y):
        a, b, c, d = self.arena.walls
        return a <= x <= b and c <= y <= d

    def find_exit(self):
        """Uma abertura precisa de duas bordas observadas e corredor livre."""
        xmin, xmax, ymin, ymax = self.arena.walls
        radius = self.arena.radius
        for axis, wall, lo, hi, sign in (
            (0, xmin, ymin, ymax, -1), (0, xmax, ymin, ymax, 1),
            (1, ymin, xmin, xmax, -1), (1, ymax, xmin, xmax, 1),
        ):
            def point(across, offset=0):
                return (wall + sign * offset, across) if axis == 0 else (across, wall + sign * offset)

            across_values = range(math.ceil(lo / CELL), math.floor(hi / CELL))
            row = [self.cells.get(cell(*point((i + .5) * CELL)), UNKNOWN) for i in across_values]
            left = None
            for i, value in enumerate(row):
                if value == UNKNOWN:
                    left = None
                elif value == OCCUPIED:
                    if left is not None and (i-left-1)*CELL >= 2*radius:
                        across = (across_values.start + (i+left)/2 + .5) * CELL
                        corridor = [point(across, off) for off in range(-math.ceil(radius), math.ceil(radius)+CELL+1, 4)]
                        if all(self.clear_disk(*p) for p in corridor):
                            return {"at": point(across), "outside": point(across, radius + CELL),
                                    "axis": axis, "wall": wall, "sign": sign}
                    left = i
        return None

    def past_exit(self):
        if not self.exit or not self.localized:
            return False
        e = self.exit
        coord = (self.pose.x, self.pose.y)[e["axis"]]
        across = (self.pose.y, self.pose.x)[e["axis"]]
        center = e["at"][1-e["axis"]]
        return e["sign"] * (coord-e["wall"]) >= self.arena.radius and abs(across-center) <= CELL

    def in_exit_corridor(self, x, y):
        if not self.exit:
            return False
        e = self.exit
        coordinate = (x,y)[e["axis"]]
        across = (y,x)[e["axis"]]
        return (0 <= e["sign"] * (coordinate - e["wall"]) <= self.arena.radius + 2 * CELL
                and abs(across - e["at"][1-e["axis"]]) <= CELL)

    def route(self):
        """Busca pontos de fronteira acessíveis usando somente espaço livre."""
        start = cell(self.pose.x, self.pose.y)
        target = cell(*self.exit["outside"]) if self.exit else None
        pending = deque([start])
        parent = {start: None}
        frontier = None
        closest_exit = None
        while pending:
            key = pending.popleft()
            x, y = midpoint(key)
            if key == target:
                frontier = key
                break
            if target is not None and key != start:
                score = math.hypot(key[0]-target[0], key[1]-target[1])
                if closest_exit is None or score < closest_exit[0]:
                    closest_exit = score, key
            # Uma fronteira está perto de área desconhecida. A grade ao redor
            # do centro do chassi ainda precisa comportar seu disco de giro.
            if frontier is None and key != start and target is None:
                ring = math.ceil(self.arena.radius/CELL) + 1
                unknown = any(self.inside(*midpoint(k)) and k not in self.cells
                              for k in ((key[0]+ring,key[1]), (key[0]-ring,key[1]),
                                        (key[0],key[1]+ring), (key[0],key[1]-ring)))
                if unknown and not any(visit[:2] == key for visit in self.visited):
                    frontier = key
                    break
            for di, dj in ((0,1),(1,0),(0,-1),(-1,0)):
                child = key[0]+di,key[1]+dj
                nx, ny = midpoint(child)
                if child not in parent and (self.inside(nx, ny) or self.in_exit_corridor(nx, ny)):
                    if all(self.clear_disk(x+(nx-x)*t, y+(ny-y)*t) for t in (0,.5,1)):
                        parent[child] = key
                        pending.append(child)

        if target is not None and frontier is None and closest_exit:
            frontier = closest_exit[1]
        if frontier is None:
            return []
        route = []
        while frontier != start:
            route.append(midpoint(frontier))
            frontier = parent[frontier]
        return route[::-1]

    def integrate(self, direction, elapsed_ms):
        """Integra tempo de tração informado pela ESP32, em vez do tempo USB."""
        pose = self.pose
        seconds = elapsed_ms / 1000
        if direction in {"f","b"}:
            sign = -1 if direction == "b" else 1
            dx, dy = relative(90, sign * self.arena.speed * seconds, pose.heading)
            self.pose = Pose(pose.x + dx, pose.y + dy, pose.heading)
            self.trail.append(self.pose)
        else:
            sign, rate = (1, self.arena.right_rate) if direction == "r" else (-1, self.arena.left_rate)
            self.pose = Pose(pose.x, pose.y, (pose.heading + sign*rate*seconds) % 360)
        self.localized = False

    def save(self, path):
        path = Path(path)
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps({
            "version": 1, "arena": asdict(self.arena), "pose": asdict(self.pose),
            "trail": [asdict(p) for p in self.trail], "cells": [[*k,v] for k,v in self.cells.items()],
            "clearing": [[*k,v] for k,v in self.clearing.items()], "tentative": sorted(self.tentative),
            "exit": self.exit, "exited": self.exited,
        }, ensure_ascii=False), encoding="utf-8")
        temp.replace(path)

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data["version"] == 2:
            return MapaAutomatico.from_data(data)
        if data["version"] != 1:
            raise ValueError("Versão de mapa incompatível.")
        result = cls(Arena(**data["arena"]))
        result.pose = Pose(**data["pose"])
        result.trail = [Pose(**p) for p in data["trail"]]
        result.cells = {(x,y): value for x,y,value in data["cells"]}
        result.clearing = {(x,y): value for x,y,value in data["clearing"]}
        result.tentative = {tuple(key) for key in data.get("tentative", [])}
        result.exit, result.exited = data["exit"], data["exited"]
        return result

@dataclass(frozen=True)
class PerfilAutomatico:
    """Referência conservadora do carrinho, em cm e segundos.

    As velocidades aproximadas servem para estimar a pose, enquanto o sonar
    e o limite de tempo na ESP32 controlam a segurança de cada movimento.
    """
    robot_width: float = 25
    robot_length: float = 30
    speed: float = 25
    left_rate: float = 100
    right_rate: float = 100

    @property
    def radius(self):
        return math.hypot(self.robot_width, self.robot_length) / 2 + MARGIN


def reconhecer_pose(cells, predicted, points):
    """Procura ecos repetidos em dois alinhamentos diferentes do mapa."""
    references = {k for k, value in cells.items() if value == OCCUPIED}
    surfaces = {
        (x,y): (any((x+d,y) in references for d in (-2,-1,1,2)),
                any((x,y+d) in references for d in (-2,-1,1,2)))
        for x,y in references
    }
    samples = list(returns(points))
    if len(samples) < 4 or not references:
        return None

    best = None
    for da in (0,-5,5,-10,10):
        heading = (predicted.heading + da) % 360
        projected = [relative(a,d,heading) for a,d in samples]
        for dx in (0,-5,5,-10,10):
            for dy in (0,-5,5,-10,10):
                x, y = predicted.x+dx, predicted.y+dy
                horizontal, vertical = set(), set()
                residual = 0
                for px,py in projected:
                    key = cell(x+px,y+py)
                    along_x, along_y = surfaces.get(key,(False,False))
                    if along_x == along_y:
                        continue
                    cx, cy = midpoint(key)
                    if along_x and abs(py) >= abs(px)/3:
                        horizontal.add(key)
                        residual += abs(y+py-cy)
                    elif along_y and abs(px) >= abs(py)/3:
                        vertical.add(key)
                        residual += abs(x+px-cx)
                if min(len(horizontal),len(vertical)) < 2:
                    continue
                rank = (len(horizontal)+len(vertical), -residual-abs(da)*.3-math.hypot(dx,dy)*.25)
                if best is None or rank > best[0]:
                    best = rank, Pose(x,y,heading)
    return best[1] if best else None


class MapaAutomatico(Map):
    """A origem é o local de partida. A área do mapa nasce dos ecos, sem paredes informadas."""

    def __init__(self, profile=None):
        self.profile = profile or PerfilAutomatico()
        super().__init__(self.profile)
        self.passages = {}
        self.blocked = set()
        self.uncertainty = 0
        self.has_scan = False
        # A origem é conhecida até o primeiro pulso. O repouso, por si só,
        # nunca valida uma pose que já estava incerta antes da parada.
        self.stationary_reference = True
        self.awaiting_echo = set()

    @property
    def bounds(self):
        keys = set(self.cells) | set(self.tentative) | {cell(self.pose.x,self.pose.y), (0,0)}
        return (min(i for i,_ in keys)*CELL, (max(i for i,_ in keys)+1)*CELL,
                min(j for _,j in keys)*CELL, (max(j for _,j in keys)+1)*CELL)

    def chave_direcao(self, heading):
        return (*cell(self.pose.x,self.pose.y), round(heading/30)%12)

    def direction_blocked(self, heading):
        x,y,sector = self.chave_direcao(heading)
        return any((x,y,(sector+offset)%12) in self.blocked for offset in (-1,0,1))

    def scan(self, points):
        corrected = reconhecer_pose(self.cells,self.pose,points) if self.has_scan and not self.stationary_reference else None
        self.localized = self.stationary_reference or corrected is not None
        if corrected:
            self.stationary_reference = True
            self.pose = corrected
            self.uncertainty = 0
            if len(self.trail) > 1:
                self.trail[-1] = corrected
        pose = self.pose
        hits, free = set(), set()
        for angle, cm in returns(points):
            dx,dy = relative(angle,cm,pose.heading)
            hits.add(cell(pose.x+dx,pose.y+dy))
            free.update(ray_cells(pose.x,pose.y,dx,dy,CELL))
        if self.has_scan and not self.localized:
            self.tentative.update(hits - {k for k in hits if self.cells.get(k) == OCCUPIED})
        else:
            for key in hits:
                self.cells[key] = OCCUPIED
                self.tentative.discard(key)
        # Um retorno novo confirma que a célula ainda não deve ser removida.
        for key in hits:
            self.clearing.pop(key,None)
        for key in free-hits:
            if self.cells.get(key) == OCCUPIED or key in self.tentative:
                if not self.localized:
                    continue
                self.clearing[key] = self.clearing.get(key,0)+1
                if self.clearing[key] < 2:
                    continue
                del self.clearing[key]
                self.tentative.discard(key)
            self.cells[key] = FREE
        # Sem retorno, a leitura só muda a etiqueta da marca. Ela nunca
        # fornece a evidência geométrica necessária para declarar espaço livre.
        missing_angles = [a for a,d in points if d is None]
        self.awaiting_echo = set()
        if missing_angles:
            for key in self.tentative | {k for k,v in self.cells.items() if v == OCCUPIED}:
                x,y = midpoint(key)
                dx,dy = x-pose.x,y-pose.y
                if math.hypot(dx,dy) <= 400:
                    heading = math.degrees(math.atan2(dx,dy))
                    angle = 90 + delta(heading,pose.heading)
                    if any(abs(angle-a) <= 5 for a in missing_angles):
                        self.awaiting_echo.add(key)
        if hits:
            self.has_scan = True
        key = self.chave_direcao(pose.heading)
        self.visited[key] = self.visited.get(key,0)+1
        for angle, cm in returns(points):
            if cm > 55 and self.localized:
                self.blocked.discard(self.chave_direcao(pose.heading+angle-90))
        self.observar_passagens(hits)

    def clear_disk(self, x, y):
        # Ecos de posição incerta ainda impedem que o planejador passe sobre
        # o obstáculo observado. Também se preserva a folga ao integrar pulsos.
        radius_margin = min(self.uncertainty,CELL)
        offsets = ((0,0),(radius_margin,0),(-radius_margin,0),(0,radius_margin),(0,-radius_margin))
        return all(self.cells.get(k) == FREE and k not in self.tentative
                   for dx,dy in offsets for k in self.footprint(x+dx,y+dy))

    def clear_reverse_body(self, x, y):
        # A ré reta não varre o disco de um giro. A folga usa o retângulo
        # orientado, expandido pela meia célula para cobrir quinas da grade.
        margin = MARGIN + math.sqrt(2) * CELL / 2 + min(self.uncertainty, CELL)
        half_width = self.profile.robot_width / 2 + margin
        half_length = self.profile.robot_length / 2 + margin
        radius = math.hypot(half_width, half_length)
        rad = math.radians(self.pose.heading)
        for i in range(math.floor((x-radius)/CELL), math.floor((x+radius)/CELL)+1):
            for j in range(math.floor((y-radius)/CELL), math.floor((y+radius)/CELL)+1):
                cx,cy = midpoint((i,j))
                dx,dy = cx-x,cy-y
                lateral = dx*math.cos(rad)-dy*math.sin(rad)
                longitudinal = dx*math.sin(rad)+dy*math.cos(rad)
                if abs(lateral) <= half_width and abs(longitudinal) <= half_length:
                    if self.cells.get((i,j)) != FREE or (i,j) in self.tentative:
                        return False
        return True

    def observar_passagens(self, hits):
        # Duas bordas alinhadas e um corredor medido nos dois lados formam uma
        # passagem possível. Sem perímetro declarado ela permanece provável.
        required = math.ceil(2*self.profile.radius/CELL)
        known = set(hits) | {k for k,v in self.cells.items() if v == OCCUPIED}
        for i,j in hits:
            for axis in (0,1):
                for gap in range(required+1,required+6):
                    mate = (i+gap,j) if axis == 0 else (i,j+gap)
                    if mate not in known:
                        continue
                    across = 1-axis
                    edge = (i,j)
                    mid = list(midpoint(edge))
                    mid[axis] += gap*CELL/2
                    samples = []
                    for offset in (-self.profile.radius,0,self.profile.radius):
                        point = mid.copy()
                        point[across] += offset
                        samples.append(point)
                    if not all(self.clear_disk(*point) for point in samples):
                        continue
                    key = (*cell(*mid),across)
                    self.passages.setdefault(key, {"at": mid,"axis": across,"crossed": False})

    def route(self):
        """Prioriza fronteiras e repassa por células conhecidas para mudar de ramo."""
        start = cell(self.pose.x,self.pose.y)
        pending, parents = deque([start]), {start: None}
        target, fallback, frontier = None, None, None
        passage_targets = set()
        for passage in self.passages.values():
            if passage["crossed"]:
                continue
            axis = passage["axis"]
            point = list(passage["at"])
            origin = (self.pose.x,self.pose.y)[axis]
            point[axis] += (1 if origin < point[axis] else -1) * (self.profile.radius + CELL)
            passage_targets.add(cell(*point))
        while pending:
            key = pending.popleft()
            x,y = (self.pose.x,self.pose.y) if key == start else midpoint(key)
            if key != start:
                if key in passage_targets:
                    target = key
                    break
                distance = math.ceil(self.profile.radius/CELL)+1
                seen = any(v[:2] == key for v in self.visited)
                beyond = ((key[0]+distance,key[1]),(key[0]-distance,key[1]),
                          (key[0],key[1]+distance),(key[0],key[1]-distance))
                if not seen and any(k not in self.cells for k in beyond):
                    if not passage_targets:
                        target = key
                        break
                    if frontier is None:
                        frontier = key
                if fallback is None and sum(v[:2] == key for v in self.visited) < 12:
                    fallback = key
            for di,dj in ((0,1),(1,0),(0,-1),(-1,0)):
                child = key[0]+di,key[1]+dj
                if child in parents:
                    continue
                nx,ny = midpoint(child)
                if not all(self.clear_disk(x+t*(nx-x),y+t*(ny-y)) for t in (0,.5,1)):
                    continue
                if key == start:
                    heading = math.degrees(math.atan2(nx-x,ny-y))
                    if self.direction_blocked(heading):
                        continue
                parents[child] = key
                pending.append(child)

        headings_here = sum(visit[:2] == start for visit in self.visited)
        target = target or frontier or (fallback if headings_here >= 12 else None)
        path = []
        while target is not None and target != start:
            path.append(midpoint(target))
            target = parents[target]
        return path[::-1]

    def integrate(self,direction,elapsed_ms):
        before = self.pose
        super().integrate(direction,elapsed_ms)
        after = self.pose
        self.stationary_reference = False
        self.uncertainty += (math.hypot(after.x-before.x,after.y-before.y)*.06
                             if direction in {"f","b"} else abs(delta(after.heading,before.heading))*.04)
        for opening in self.passages.values():
            axis = opening["axis"]
            x0,x1 = (before.x,before.y)[axis], (after.x,after.y)[axis]
            y0,y1 = (before.y,before.x)[axis], (after.y,after.x)[axis]
            p,q = opening["at"][axis], opening["at"][1-axis]
            if x0 != x1 and min(x0,x1) <= p <= max(x0,x1):
                if abs(y0+(y1-y0)*(p-x0)/(x1-x0)-q) <= CELL:
                    opening["crossed"] = True

    def save(self,path):
        path = Path(path)
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps({
            "version": 2,"pose": asdict(self.pose),"trail": [asdict(p) for p in self.trail],
            "cells": [[*key,value] for key,value in sorted(self.cells.items())],
            "tentative": sorted(self.tentative),"passages": list(self.passages.values()),
            "clearing": [[*key,value] for key,value in self.clearing.items()],
            "awaiting_echo": sorted(self.awaiting_echo),
        }, ensure_ascii=False),encoding="utf-8")
        temp.replace(path)

    @classmethod
    def from_data(cls,data):
        result = cls()
        result.stationary_reference = False
        result.pose = Pose(**data["pose"])
        result.trail = [Pose(**p) for p in data["trail"]]
        result.cells = {(x,y): value for x,y,value in data["cells"]}
        result.tentative = {tuple(key) for key in data.get("tentative",[])}
        result.clearing = {(x,y): value for x,y,value in data.get("clearing",[])}
        result.awaiting_echo = {tuple(k) for k in data.get("awaiting_echo",[])}
        result.passages = {(*cell(*entry["at"]),entry["axis"]): entry for entry in data["passages"]}
        return result
