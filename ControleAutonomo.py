"""Ciclo da exploração. O despacho serial fica em uma única thread do painel."""
import math
import time

from Navegacao import MapaAutomatico, cell, delta, relative, returns


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
        self.reverse_limit = 0
        self.rear_origin = None
        self.rear_heading = None
        self.forward_at = 0.0
        self.wait_reason = "Aguardando passagem segura"

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
        self.notify("Varrendo a arena com as rodas paradas")
        self.command("start")
        state = self.wait_for(lambda s: s["state"] == "done", 45)
        self.valid_scan = any(returns(state["points"]))
        self.map.scan(state["points"])
        summary = "Posição de referência confirmada" if self.map.localized else "Posição estimada"
        if self.map.clearing:
            summary += f" · {len(self.map.clearing)} marca(s) com 1 de 2 confirmações de remoção"
        if getattr(self.map,"awaiting_echo",None):
            summary += f" · {len(self.map.awaiting_echo)} aguardando confirmação de eco"
        self.notify(summary)

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
        if elapsed:
            self.map.integrate(direction, elapsed)
        outcome = state["autoEnd"]
        if self.automatic and elapsed:
            if direction == "f":
                if (self.rear_origin is None or abs(delta(before.heading,self.rear_heading)) > 5
                        or time.monotonic()-self.forward_at > 90):
                    self.rear_origin, self.rear_heading = before, before.heading
                self.forward_at = time.monotonic()
            elif direction in {"l","r"}:
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
        return self.move("b",step,m.arena.speed)

    def turn(self, degrees):
        m = self.map
        if not m.clear_disk(m.pose.x, m.pose.y):
            return False
        direction = "l" if degrees < 0 else "r"
        rate = m.arena.left_rate if direction == "l" else m.arena.right_rate
        return self.move(direction, min(abs(degrees), 30), rate)

    def explore(self):
        # Uma passagem atravessada fica no mapa. Só o operador encerra a
        # exploração normal; uma falha serial continua encerrando a sessão.
        while True:
            self.observe()
            if not self.next_move():
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
