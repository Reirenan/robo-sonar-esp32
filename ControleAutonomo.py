"""Ciclo da exploração. O despacho serial fica em uma única thread do painel."""
import math
import time

from Navegacao import cell, delta, relative


class StopRequested(Exception):
    pass


class Navigator:
    def __init__(self, bridge, arena_map, interrupt, notify):
        self.bridge = bridge
        self.map = arena_map
        self.interrupt = interrupt
        self.notify = notify
        self.owns_session = False

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
        self.map.scan(state["points"])
        self.notify("Pose confirmada pelo sonar" if self.map.localized else "Pose estimada, sem referência em duas paredes")

    def move(self, direction, amount, rate):
        # O pulso é indivisível na placa e limitado a 350 ms. Após cada pulso,
        # a próxima decisão só ocorre com as rodas paradas e um mapa atualizado.
        duration = min(350, int(amount / rate * 1000))
        if duration < 40:
            return False
        self.notify({"f": "Avançando", "l": "Girando à esquerda", "r": "Girando à direita"}[direction])
        self.command(f"auto_{direction}_{duration}")
        state = self.wait_for(lambda s: not s["motorBusy"], 4)
        elapsed = state["autoElapsedMs"]
        if elapsed:
            self.map.integrate(direction, elapsed)
        outcome = state["autoEnd"]
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
        path = m.route()
        if not path:
            current = cell(pose.x, pose.y)
            heading = round(pose.heading / 30) % 12
            unseen = [step for step in range(1, 12)
                      if (*current, (heading + step) % 12) not in m.visited]
            if not unseen or not m.clear_disk(pose.x, pose.y):
                return False
            # Inspecionar uma direção ainda não observada no mesmo lugar.
            turn = delta(((heading + unseen[0]) % 12) * 30, pose.heading)
            return self.turn(turn)
        target = path[0]
        distance = math.hypot(target[0]-pose.x, target[1]-pose.y)
        heading = math.degrees(math.atan2(target[0]-pose.x, target[1]-pose.y))
        turn = delta(heading, pose.heading)
        if abs(turn) > 12:
            return self.turn(turn)
        step = min(distance, a.speed*.350)
        # Valida o trajeto na direção real estimada, inclusive a margem
        # lateral. A busca na grade pode ter usado o centro de outra célula.
        for cm in range(1, math.ceil(step)+1):
            dx, dy = relative(90, min(cm, step), pose.heading)
            if not m.clear_disk(pose.x+dx, pose.y+dy):
                return False
        return self.move("f", step, a.speed)

    def turn(self, degrees):
        m = self.map
        if not m.clear_disk(m.pose.x, m.pose.y):
            return False
        direction = "l" if degrees < 0 else "r"
        rate = m.arena.left_rate if direction == "l" else m.arena.right_rate
        return self.move(direction, min(abs(degrees), 30), rate)

    def run(self):
        try:
            state = self.command("state")
            if state.get("autoVersion") != 1:
                return "Grave o firmware que oferece o protocolo autônomo."
            if not state["servoReady"] or not state["motorReady"] or state["motorBusy"]:
                return "Confira o servo e a ponte H e aguarde o robô parar."
            self.command("auto_begin")
            self.owns_session = True
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
