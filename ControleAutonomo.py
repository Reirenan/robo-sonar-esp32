"""Ciclo da exploração. O despacho serial fica em uma única thread do painel."""
from dataclasses import dataclass
from collections import Counter
import math
import time
from statistics import median

from Navegacao import MapaAutomatico, cell, delta, relative, returns


@dataclass(frozen=True)
class Abertura:
    first: int
    last: int
    depth: float

    @property
    def center(self):
        return (self.first + self.last) / 2

    @property
    def width(self):
        # Corda entre os raios extremos na menor distância observada.
        return 2 * min(self.depth, 100) * math.sin(math.radians(self.last - self.first) / 2)


def encontrar_aberturas(points, angular_step):
    """Agrupa ecos úteis sem atravessar setores ausentes ou sem retorno."""
    runs = []
    current = []
    for angle, cm in sorted(points):
        if current and (cm is None or not 45 < cm <= 400 or angle - current[-1][0] != angular_step):
            runs.append(current)
            current = []
        if cm is not None and 45 < cm <= 400:
            current.append((angle, cm))
    runs.append(current)
    return [Abertura(run[0][0], run[-1][0], min(cm for _, cm in run))
            for run in runs if len(run) >= 2]


class StopRequested(Exception):
    pass


class Navigator:
    def __init__(self, bridge, arena_map, interrupt, notify):
        self.bridge = bridge
        self.map = arena_map
        self.interrupt = interrupt
        self.notify = notify
        self.owns_session = False
        self.automatic = isinstance(arena_map, MapaAutomatico)
        self.valid_scan = False
        self.scan_points = []
        self.scan_kind = "full"
        self.quick_supported = False
        self.side_readings_current = False
        self.turn_attempts = set()
        self.survey_turns = Counter()
        self.survey_turn_pending = False
        self.since_scan = 0.0
        self.turns_since_scan = 0
        self.last_direction = None
        self.reverse_limit = 0
        self.rear_origin = None
        self.rear_heading = None
        self.forward_at = 0.0
        self.wait_reason = "Aguardando passagem segura"
        self.deferred_frontiers = {}
        self.blocked_edges = {}
        self.frontier_baseline = set()
        self.frontier_observations = 0

    def check_stop(self):
        if self.interrupt.is_set():
            raise StopRequested

    def command(self, action):
        self.check_stop()
        status, response = self.bridge.request(action)
        if status != 200:
            raise RuntimeError(response)
        if self.owns_session and not response.get("autoActive"):
            self.owns_session = False
            raise RuntimeError("Sessão autônoma encerrada na ESP32.")
        return response

    def wait_for(self, check, limit):
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            state = self.command("state")
            if state["state"] == "error":
                raise RuntimeError("A ESP32 reportou falha no servo.")
            if check(state):
                return state
            self.interrupt.wait(.08)
        raise RuntimeError("A ESP32 não concluiu a operação a tempo.")

    def observe(self):
        self.notify("Atualizando mapa · rodas paradas")
        self.command("start")
        state = self.wait_for(lambda s: s["state"] == "done", 45)
        self.scan_points = state["points"]
        self.scan_kind = "full"
        self.side_readings_current = True
        self.since_scan = 0.0
        self.turns_since_scan = 0
        self.valid_scan = any(returns(self.scan_points))
        self.map.scan(state["points"])
        summary = "Posição de referência confirmada" if self.map.localized else "Posição estimada"
        if self.map.clearing:
            summary += f" · {len(self.map.clearing)} marca(s) com 1 de 2 confirmações de remoção"
        if getattr(self.map,"awaiting_echo",None):
            summary += f" · {len(self.map.awaiting_echo)} aguardando confirmação de eco"
        self.notify(summary)

    def quick_observe(self):
        self.notify("Leitura rápida · rodas paradas")
        self.command("probe")
        state = self.wait_for(lambda data: data["state"] == "done",12)
        points = state["points"]
        if (state.get("scanKind") != "quick" or state["angle"] != 90
                or [angle for angle,_ in points] != [30,60,90,120,150]):
            raise RuntimeError("Sondagem curta incompleta.")
        self.scan_points = points
        self.scan_kind = "quick"
        self.side_readings_current = True
        self.map.quick_observation(points)
        return dict(points).get(90)

    def move(self, direction, amount, rate):
        # O pulso é indivisível na placa e limitado a 350 ms. Após cada pulso,
        # a próxima decisão só ocorre com as rodas paradas e um mapa atualizado.
        limit = self.reverse_limit if direction == "b" else 350
        duration = min(limit, int(amount / rate * 1000))
        if duration < 40:
            return False
        self.notify({"f": "Avançando", "l": "Girando à esquerda", "r": "Girando à direita",
                     "b": "Recuando pelo trecho conhecido"}[direction])
        before = self.map.pose
        self.command(f"auto_{direction}_{duration}")
        state = self.wait_for(lambda s: not s["motorBusy"], 4)
        elapsed = state["autoElapsedMs"]
        self.last_direction = direction if elapsed else None
        if elapsed:
            self.map.integrate(direction, elapsed)
            self.side_readings_current = False
            if direction in {"f","b"} and cell(before.x,before.y) != cell(self.map.pose.x,self.map.pose.y):
                self.turn_attempts.clear()
            if direction in {"f","b"}:
                self.since_scan += elapsed * rate / 1000
        outcome = state["autoEnd"]
        if self.automatic and elapsed:
            if direction == "f":
                if (self.rear_origin is None or abs(delta(before.heading,self.rear_heading)) > 5
                        or time.monotonic()-self.forward_at > 90):
                    self.rear_origin, self.rear_heading = before, before.heading
                self.forward_at = time.monotonic()
            elif direction in {"l","r"}:
                self.turns_since_scan += 1
                self.rear_origin = None
        if self.automatic and direction == "f" and outcome in {"e","o"}:
            self.map.blocked.add(self.map.chave_direcao(self.map.pose.heading))
            self.wait_reason = ("Sem eco frontal" if outcome == "e" else "Obstáculo frontal")
            self.notify(self.wait_reason + " · procurando outro caminho")
            return False
        if outcome == "e":
            raise RuntimeError("Sem eco frontal válido. Avanço interrompido.")
        if outcome == "s":
            raise StopRequested
        if outcome not in {"o", "t"}:
            raise RuntimeError("Resposta de movimento inválida.")
        if outcome == "o":
            self.notify("Obstáculo frontal. Rodas paradas")
        return bool(elapsed) or outcome == "o"

    def front_probe(self):
        # O comando echo é diagnóstico e não cria sessão. Primeiro confirma
        # a concessão USB e a posição física comandada, com rodas já paradas.
        state = self.command("state")
        if state["angle"] != 90 or state["motorBusy"]:
            return None
        samples = []
        for _ in range(3):
            self.check_stop()
            code, data = self.bridge.request("echo")
            if code != 200:
                raise RuntimeError(data)
            cm = data["pulseUs"] * .0343 / 2
            if data["angle"] == 90 and 2 <= cm <= 400:
                samples.append(cm)
            if self.interrupt.wait(.07):
                raise StopRequested
        # Uma leitura próxima tem precedência sobre as demais.
        if any(cm <= 45 for cm in samples):
            return min(samples)
        # A leitura rápida serve de veto e liberação local, sem atualizar
        # a ocupação nem contar como uma confirmação de remoção do mapa.
        return median(samples) if len(samples) == 3 else None

    def turn_key(self, degrees):
        pose = self.map.pose
        return (*cell(pose.x,pose.y),round(pose.heading/15)%24,degrees)

    def gap_turns(self):
        m = self.map
        points = self.scan_points if self.side_readings_current else []
        openings = encontrar_aberturas(points, 30 if self.scan_kind == "quick" else 10)
        cards = {}
        for side, sign in (("l", -1), ("r", 1)):
            echoes = [cm for angle, cm in points if (angle - 90) * sign > 0 and cm is not None]
            reason = "Aguardando leitura no novo rumo" if not self.side_readings_current else "Sem eco lateral"
            if echoes:
                reason = "Abertura estreita ou sem feixes consecutivos"
            cards[side] = (min(echoes) if echoes else None, reason)

        ranked = sorted(openings, key=lambda gap: (-gap.width, abs(gap.center - 90), gap.center))
        options = []
        for gap in ranked:
            offset = gap.center - 90
            if gap.width < m.arena.robot_width + 10 or not offset:
                continue
            side = "l" if offset < 0 else "r"
            if m.direction_blocked(m.pose.heading + offset):
                cards[side] = (gap.depth, "Rumo bloqueado na última tentativa")
                continue
            rate = m.arena.left_rate if offset < 0 else m.arena.right_rate
            primary = min(abs(offset), 25)
            sizes = [primary] + [size for size in (15, 10, 5) if size < primary]
            blockers = set()
            turn = 0
            for size in sizes:
                candidate = math.copysign(size, offset)
                if self.turn_key(candidate) in self.turn_attempts or size / rate < .040:
                    continue
                cells = m.swept_blockers(turn=candidate)
                if not cells:
                    turn = candidate
                    break
                if not blockers or len(cells) < len(blockers):
                    blockers = cells
            if turn:
                options.append((gap, side, turn))
                cards[side] = (gap.depth, f"Abertura estimada: {gap.width:.0f} cm")
                m.maneuver_blockers.pop(side, None)
            elif not any(option[1] == side for option in options):
                if blockers:
                    m.maneuver_blockers[side] = blockers
                cards[side] = (gap.depth, "Chassi sem folga no giro" if blockers else "Giros já avaliados")
        return options, cards

    def scan_budget(self):
        # Em trecho saudável a frente é conferida a cada pulso. O orçamento
        # maior reduz pausas sem dispensar a verificação de folga no mapa.
        if (self.quick_supported and self.valid_scan and self.map.uncertainty <= 8
                and not self.map.clearing):
            return 50, 4
        return 25, 2

    def survey_turn(self):
        """Gira só para observar, sem presumir que há passagem para avançar."""
        if not self.valid_scan:
            return False
        m = self.map
        pose = m.pose
        position = cell(pose.x,pose.y)
        choices = sorted((-1,1),key=lambda sign: (self.survey_turns[position,sign],sign))
        for sign in choices:
            turn = sign * 10
            key = self.turn_key(turn)
            rate = m.arena.left_rate if sign < 0 else m.arena.right_rate
            if self.survey_turns[position,sign] >= 2 or key in self.turn_attempts or 10 / rate < .040:
                continue
            blockers = m.swept_blockers(turn=turn)
            side = "l" if sign < 0 else "r"
            if blockers:
                m.maneuver_blockers[side] = blockers
                continue
            self.turn_attempts.add(key)
            self.survey_turns[position,sign] += 1
            m.selected_direction = side
            self.notify("Giro de observação de 10° à " + ("esquerda" if sign < 0 else "direita"))
            if self.turn(turn):
                self.survey_turn_pending = self.quick_supported
                return True
        m.selected_direction = None
        return False

    def decision(self, front, allow_turn):
        m = self.map
        m.selected_direction = None
        m.selected_gap = None
        m.maneuver_blockers = {}
        distance_budget, _ = self.scan_budget()
        step = min(m.arena.speed*.350,distance_budget-self.since_scan)
        forward = front is not None and front > 45 and step >= m.arena.speed*.040
        if forward:
            blockers = m.swept_blockers(distance=step)
            if blockers:
                m.maneuver_blockers["f"] = blockers
            forward = not blockers and m.uncertainty <= 30
        options, cards = self.gap_turns()
        freason = ("Sem eco confiável" if front is None else "Obstáculo até 45 cm"
                   if front <= 45 else "Passagem livre" if forward else "Faixa sem folga confirmada")
        m.directions = {**cards, "f": (front, freason)}
        if forward:
            m.selected_direction = "f"
            return self.move("f",step,m.arena.speed)
        if not allow_turn:
            return False
        if m.uncertainty <= 30:
            for gap, direction, turn in options:
                m.selected_direction = direction
                m.selected_gap = (m.pose, gap)
                m.directions[direction] = (gap.depth, f"Rumo {gap.center - 90:+.0f}° · abertura {gap.width:.0f} cm")
                self.turn_attempts.add(self.turn_key(turn))
                if self.turn(turn):
                    return True
            m.selected_direction = None
            m.selected_gap = None
            if self.survey_turn():
                return True
            # Uma sondagem esparsa nunca é suficiente para optar pela ré.
            if self.scan_kind == "full" and self.backtrack():
                return True
        self.wait_reason = "Posição incerta" if m.uncertainty > 30 else freason + " · sem manobra com folga"
        return False

    def next_move(self):
        m, a = self.map, self.map.arena
        pose = m.pose
        if self.automatic and not self.valid_scan:
            self.wait_reason = "Sem eco válido · mapa anterior preservado"
            return False
        if self.automatic and m.uncertainty > 30:
            self.wait_reason = "Posição incerta · aguardando nova referência"
            return False
        path = m.route()
        can_turn = m.clear_disk(pose.x, pose.y)
        if path:
            target = path[0]
            distance = math.hypot(target[0]-pose.x,target[1]-pose.y)
            heading = math.degrees(math.atan2(target[0]-pose.x,target[1]-pose.y))
            turn = delta(heading,pose.heading)
            if abs(turn) > 12:
                if can_turn and self.turn(turn):
                    return True
            else:
                step = min(distance,a.speed*.350)
                # Valida a faixa no rumo estimado, pois a rota passa pelo
                # centro da célula e a pose nem sempre está nesse centro.
                for cm in range(1,math.ceil(step)+1):
                    dx,dy = relative(90,min(cm,step),pose.heading)
                    if not m.clear_disk(pose.x+dx,pose.y+dy):
                        break
                else:
                    return self.move("f",step,a.speed)
        elif can_turn:
            current = cell(pose.x,pose.y)
            heading = round(pose.heading/30)%12
            for step in (1,-1,2,-2,3,-3,4,-4,5,-5,6):
                if (*current,(heading+step)%12) not in m.visited:
                    return self.turn(delta(((heading+step)%12)*30,pose.heading))
        if self.automatic and self.backtrack():
            return True
        if self.automatic:
            self.wait_reason = ("Folga insuficiente para girar · sem caminho seguro para recuar"
                                if not can_turn else "Sem rota verificada · sem caminho seguro para recuar")
            if not m.localized and self.rear_origin is not None:
                self.wait_reason = "Posição incerta · ré bloqueada até confirmar a pose"
        return False

    def backtrack(self):
        m = self.map
        pose = m.pose
        if (not self.reverse_limit or self.rear_origin is None or not m.localized
                or time.monotonic()-self.forward_at > 90
                or abs(delta(pose.heading,self.rear_heading)) > 5):
            return False
        fx,fy = relative(90,1,self.rear_heading)
        dx,dy = pose.x-self.rear_origin.x,pose.y-self.rear_origin.y
        if abs(dx*fy-dy*fx) > 2:
            return False
        # Reserva 0,5 cm na origem e não extrapola o trecho já percorrido.
        # A folga é conferida ao longo do pulso, com o retângulo do chassi.
        remaining = dx*fx+dy*fy-.5
        step = min(remaining,m.arena.speed*self.reverse_limit/1000)
        if step < m.arena.speed*.040:
            return False
        for cm in range(math.ceil(step)+1):
            bx,by = relative(90,-min(cm,step),pose.heading)
            if not m.clear_reverse_body(pose.x+bx,pose.y+by):
                return False
        m.selected_direction = "b"
        return self.move("b",step,m.arena.speed)

    def turn(self, degrees):
        m = self.map
        if self.automatic:
            if not m.swept_clear(turn=max(-30,min(degrees,30))):
                return False
        elif not m.clear_disk(m.pose.x,m.pose.y):
            return False
        direction = "l" if degrees < 0 else "r"
        rate = m.arena.left_rate if direction == "l" else m.arena.right_rate
        return self.move(direction, min(abs(degrees), 30), rate)

    def plan_frontier(self):
        m = self.map
        current = cell(m.pose.x,m.pose.y)
        target = m.frontier_target
        if target == current:
            revealed = any(key in m.cells for key in self.frontier_baseline)
            self.frontier_observations += 1
            if revealed or self.frontier_observations >= 2:
                self.deferred_frontiers[target] = m.frontier_signature(target)
                target = None
            else:
                m.frontier_path = []
                m.maneuver_blockers = {}
                self.notify("Observando a fronteira novamente")
                return

        excluded = {edge for edge, signature in self.blocked_edges.items()
                    if signature == (m.frontier_signature(edge[0]),m.frontier_signature(edge[1]))}
        chosen,path = m.plan_frontier(target,self.deferred_frontiers,excluded)
        if chosen != target and target is not None:
            self.deferred_frontiers[target] = m.frontier_signature(target)
        if chosen != m.frontier_target:
            self.frontier_baseline = m.frontier_evidence(chosen) if chosen is not None else set()
            self.frontier_observations = 0
        m.frontier_target, m.frontier_path = chosen,path

    def pursue_frontier(self, front, allow_turn, replan=True):
        """Executa um trecho da rota e pede desvio apenas se ela bloquear."""
        m = self.map
        if m.frontier_target is None:
            return self.decision(front,allow_turn)
        pose = m.pose
        if not m.frontier_path:
            return False
        if m.uncertainty > 30:
            self.wait_reason = "Posição incerta · aguardando varredura"
            return False
        waypoint = m.frontier_path[0]
        dx,dy = waypoint[0]-pose.x,waypoint[1]-pose.y
        turn = delta(math.degrees(math.atan2(dx,dy)),pose.heading)
        m.selected_gap = None
        m.selected_direction = None
        m.maneuver_blockers = {}
        m.directions["f"] = (front,"Via planejada" if front is not None and front > 45 else "Aguardando eco livre")
        if abs(turn) > 12:
            rate = m.arena.left_rate if turn < 0 else m.arena.right_rate
            for size in dict.fromkeys((min(abs(turn),25),15,10,5)):
                if size > abs(turn) or size / rate < .040:
                    continue
                rotation = math.copysign(size,turn)
                key = self.turn_key(rotation)
                if key not in self.turn_attempts and m.swept_clear(turn=rotation):
                    self.turn_attempts.add(key)
                    m.selected_direction = "l" if turn < 0 else "r"
                    self.notify("Alinhando para seguir a rota")
                    if self.turn(rotation):
                        return True
        elif front is not None and front > 45:
            step = min(math.hypot(dx,dy),m.arena.speed*.350,self.scan_budget()[0]-self.since_scan)
            if step >= m.arena.speed*.040 and m.swept_clear(distance=step):
                m.selected_direction = "f"
                known = any(visit[:2] == cell(*waypoint) for visit in m.visited)
                self.notify("Voltando por caminho conhecido" if known else "Indo para uma nova área")
                if self.move("f",step,m.arena.speed):
                    return True

        if not allow_turn:
            return False
        if not replan:
            return self.decision(None,allow_turn=True)
        # Uma aresta rejeitada volta a ser elegível se a região mudar.
        origin,destination = cell(pose.x,pose.y),cell(*waypoint)
        self.blocked_edges[origin,destination] = (m.frontier_signature(origin),m.frontier_signature(destination))
        self.plan_frontier()
        if m.frontier_path and m.frontier_path[0] != waypoint:
            self.notify("Rota recalculada · seguindo pela área conhecida")
            return self.pursue_frontier(front,allow_turn=True,replan=False)
        return self.decision(None,allow_turn=True)

    def explore(self):
        while True:
            self.observe()
            self.plan_frontier()
            front = dict(self.scan_points).get(90)
            if self.map.frontier_target is not None and not self.map.frontier_path:
                continue
            moved = self.pursue_frontier(front,allow_turn=True)
            if not moved and self.quick_supported and any(self.map.maneuver_blockers.values()):
                self.notify("Reavaliando a margem da manobra")
                moved = self.pursue_frontier(self.quick_observe(),allow_turn=True)
            if moved:
                # A cada pulso, a nova frente pode evitar até mesmo a
                # leitura curta. Se fechar, mede os lados antes de remapear.
                while True:
                    if self.survey_turn_pending:
                        self.survey_turn_pending = False
                        front = self.quick_observe()
                        if self.pursue_frontier(front,allow_turn=True):
                            continue
                        break
                    distance_budget, turn_budget = self.scan_budget()
                    if (self.since_scan >= distance_budget-self.map.arena.speed*.040
                            or self.turns_since_scan >= turn_budget):
                        self.notify("Atualizando mapa · limite de distância ou giros atingido")
                        break
                    if self.map.frontier_target is not None:
                        if cell(self.map.pose.x,self.map.pose.y) == self.map.frontier_target:
                            self.map.frontier_path = []
                            break
                        self.plan_frontier()
                    if self.pursue_frontier(self.front_probe(), allow_turn=False):
                        continue
                    if self.quick_supported and self.pursue_frontier(self.quick_observe(), allow_turn=True):
                        continue
                    break
                continue
            self.notify("Aguardando passagem segura" if self.wait_reason == "Aguardando passagem segura"
                        else "Aguardando passagem segura · "+self.wait_reason)
            for _ in range(4):
                if self.interrupt.wait(.5):
                    raise StopRequested
                self.command("state")

    def run(self):
        try:
            state = self.command("state")
            if state.get("autoVersion") != 1:
                return "Grave o firmware que oferece o protocolo autônomo."
            if not state["servoReady"] or not state["motorReady"] or state["motorBusy"]:
                return "Confira o servo e a ponte H e aguarde o robô parar."
            self.reverse_limit = min(120,state.get("reverseMaxMs",0))
            self.quick_supported = bool(state.get("quickProbe"))
            self.command("auto_begin")
            self.owns_session = True
            if self.automatic:
                self.explore()
            for _ in range(180):
                self.observe()
                if not self.map.localized:
                    return "Navegação parada. Duas paredes não confirmaram a posição."
                if self.map.past_exit():
                    self.map.exited = True
                    return "Saída confirmada. Robô parado fora da arena."
                if not self.next_move():
                    return "Sem trajeto verificável. Saída não confirmada."
            return "Limite de exploração atingido. Robô parado."
        except StopRequested:
            return "Parado a pedido do operador."
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            return f"Navegação interrompida: {error}"
        finally:
            # A placa corta o PWM sozinha se o cabo cair durante um pulso.
            try:
                self.bridge.request("stop")
            except (OSError, ValueError):
                pass
