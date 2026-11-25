from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import unittest
from unittest.mock import patch

from ControleAutonomo import Navigator
from Navegacao import Arena, FREE, Map, OCCUPIED, Pose, cell, localize, relative

ARENA = Arena(200, 200, 80, 50, 20, 20, 30, 100, 100)


def wall_readings(arena, real_pose):
    """Retornos analíticos de um retângulo, com o mesmo referencial da placa."""
    xmin, xmax, ymin, ymax = arena.walls
    points = []
    for angle in range(0, 181, 10):
        dx, dy = relative(angle, 1, real_pose.heading)
        distances = []
        if abs(dx) > 1e-8:
            distances.append(((xmax if dx > 0 else xmin) - real_pose.x) / dx)
        if abs(dy) > 1e-8:
            distances.append(((ymax if dy > 0 else ymin) - real_pose.y) / dy)
        points.append((angle, min(distances)))
    return points


class MapaTest(unittest.TestCase):
    def test_orientacao_local_e_global(self):
        self.assertAlmostEqual(relative(0, 60)[0], -60)
        self.assertAlmostEqual(relative(90, 60)[1], 60)
        self.assertAlmostEqual(relative(180, 60)[0], 60)
        self.assertAlmostEqual(relative(90, 60, 90)[0], 60)

    def test_corrige_pose_e_rejeita_uma_parede(self):
        real = Pose(4, -4, 3)
        fixed = localize(ARENA, Pose(), wall_readings(ARENA, real))
        self.assertIsNotNone(fixed)
        for actual, expected in zip((fixed.x, fixed.y, fixed.heading), (real.x, real.y, real.heading)):
            self.assertAlmostEqual(actual, expected, delta=1)
        self.assertIsNone(localize(ARENA, Pose(), [(80,151),(90,150),(100,151)]))

    def test_remove_apenas_com_duas_varreduras_localizadas(self):
        model = Map(ARENA)
        old = cell(0, 42)
        with patch("Navegacao.localize", return_value=Pose()):
            model.scan([(90, 42)])
            self.assertEqual(model.cells[old], OCCUPIED)
            model.scan([(90, None), (90, 28)])
            self.assertNotIn(old, model.clearing)
            model.scan([(90, 90), (90, 91)])
            self.assertEqual(model.cells[old], OCCUPIED)
            self.assertEqual(model.clearing[old], 1)
        with patch("Navegacao.localize", return_value=None):
            model.scan([(90, 91)])
            self.assertEqual(model.cells[old], OCCUPIED)
        with patch("Navegacao.localize", return_value=Pose()):
            model.scan([(90, 90)])
        self.assertEqual(model.cells[old], FREE)

    def test_saida_exige_bordas_corredor_e_travessia(self):
        model = Map(ARENA)
        for i in range(-4, 5):
            for j in range(10, 22):
                model.cells[i,j] = FREE
        self.assertIsNone(model.find_exit())
        model.cells[-5,15] = model.cells[5,15] = OCCUPIED
        model.exit = model.find_exit()
        self.assertIsNotNone(model.exit)
        model.pose = Pose(0, 150 + model.arena.radius + 2, 0)
        self.assertFalse(model.past_exit())
        model.localized = True
        self.assertTrue(model.past_exit())
        model.cells[0,16] = OCCUPIED
        self.assertIsNone(model.find_exit())

    def test_margem_e_rota_pelo_espaco_livre(self):
        model = Map(ARENA)
        for i in range(-4, 5):
            for j in range(-4, 8):
                model.cells[i,j] = FREE
        path = model.route()
        self.assertTrue(path)
        self.assertTrue(all(model.clear_disk(x, y) for x, y in path))
        model.cells[1,0] = OCCUPIED
        self.assertFalse(model.clear_disk(0, 0))

    def test_salva_e_reabre(self):
        model = Map(ARENA)
        model.integrate("f", 200)
        model.integrate("r", 300)
        self.assertAlmostEqual(model.pose.y, 6)
        self.assertAlmostEqual(model.pose.heading, 30)
        with TemporaryDirectory() as folder:
            path = Path(folder) / "mapa.json"
            model.save(path)
            loaded = Map.load(path)
        self.assertEqual(loaded.pose, model.pose)
        self.assertEqual(loaded.trail, model.trail)
        self.assertEqual(loaded.cells, model.cells)
        self.assertFalse(loaded.localized)


class Bridge:
    def __init__(self, points, fail_action=None):
        self.points = points
        self.requests = []
        self.fail_action = fail_action

    def request(self, action):
        self.requests.append(action)
        if action == self.fail_action:
            raise OSError("USB desconectado")
        return 200, {"state": "done", "motorBusy": False, "motorReady": True, "servoReady": True,
                     "autoVersion": 1, "autoActive": True, "autoEnd": "e", "autoElapsedMs": 0, "points": self.points}


class ControleTest(unittest.TestCase):
    def make(self, bridge):
        self.stop = Event()
        return Navigator(bridge, Map(ARENA), self.stop, lambda _: None)

    def test_para_se_localizacao_falhar(self):
        bridge = Bridge([(90,None)])
        self.assertIn("Duas paredes", self.make(bridge).run())
        self.assertEqual(bridge.requests, ["state","auto_begin","start","state","stop"])

    def test_para_apos_erro_usb(self):
        bridge = Bridge([], fail_action="start")
        self.assertIn("USB desconectado", self.make(bridge).run())
        self.assertEqual(bridge.requests[-1], "stop")

    def test_cancelamento_antes_de_acionar(self):
        bridge = Bridge(wall_readings(ARENA, Pose()))
        control = self.make(bridge)
        self.stop.set()
        self.assertIn("operador", control.run())
        self.assertEqual(bridge.requests, ["stop"])

    def test_sem_eco_interrompe_pulso(self):
        bridge = Bridge(wall_readings(ARENA, Pose()))
        control = self.make(bridge)
        with patch.object(control, "next_move", side_effect=lambda: control.move("f", 9, 30)):
            self.assertIn("Sem eco frontal", control.run())
        self.assertIn("auto_f_300", bridge.requests)
        self.assertEqual(bridge.requests[-1], "stop")


if __name__ == "__main__":
    unittest.main()
