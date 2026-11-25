from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import unittest
from unittest.mock import patch

from ControleAutonomo import Navigator
from Navegacao import FREE, OCCUPIED, MapaAutomatico, Map, Pose, cell


class BuscaAutomaticaTest(unittest.TestCase):
    def test_inicio_sem_medidas_e_expansao(self):
        model = MapaAutomatico()
        self.assertEqual(model.pose, Pose())
        model.scan([(0,80),(90,120),(180,100)])
        self.assertEqual(model.cells[cell(0,120)], OCCUPIED)
        self.assertEqual(model.cells[cell(0,70)], FREE)
        self.assertGreater(model.bounds[3], 120)

    def test_duas_varreduras_confiaveis_removem_marca(self):
        model = MapaAutomatico()
        target = cell(0,45)
        model.scan([(90,45)])
        model.scan([(90,None)])
        model.scan([(90,120)])
        self.assertEqual(model.cells[target], OCCUPIED)
        with patch("Navegacao.reconhecer_pose", return_value=Pose()):
            model.scan([(90,120), (90,121)])
            self.assertEqual(model.cells[target], OCCUPIED)
            model.scan([(90,120)])
        self.assertEqual(model.cells[target], FREE)

    def test_passagem_atravessada_e_salva(self):
        model = MapaAutomatico()
        for x in range(-6,7):
            for y in range(-5,9):
                model.cells[x,y] = FREE
        # Duas bordas deixam passagem de largura suficiente diante do chassi.
        borders = {(-4,2),(3,2)}
        for key in borders:
            model.cells[key] = OCCUPIED
        model.observar_passagens(borders)
        self.assertTrue(model.passages)
        model.pose = Pose(0,23,0)
        model.integrate("f", 200)
        self.assertTrue(any(p["crossed"] for p in model.passages.values()))
        with TemporaryDirectory() as directory:
            path = Path(directory) / "sessao.json"
            model.save(path)
            saved = Map.load(path)
            self.assertEqual(saved.trail, model.trail)
            self.assertEqual(saved.passages, model.passages)

    def test_rota_contorna_direcao_bloqueada(self):
        model = MapaAutomatico()
        for x in range(-8,9):
            for y in range(-8,9):
                model.cells[x,y] = FREE
        model.blocked.add(model.chave_direcao(0))
        path = model.route()
        self.assertTrue(path)
        self.assertTrue(all(model.clear_disk(*p) for p in path))
        self.assertNotEqual(path[0], (5,15))

    def test_volta_pela_grade_quando_a_fronteira_acaba(self):
        model = MapaAutomatico()
        for i in range(-3,4):
            for j in range(-3,4):
                model.cells[i,j] = FREE
        for sector in range(12):
            model.visited[(0,0,sector)] = 1
        for i,j in model.cells:
            model.visited.setdefault((i,j,0),1)
        path = model.route()
        self.assertTrue(path)
        self.assertTrue(all(model.clear_disk(*p) for p in path))

    def test_falta_de_eco_bloqueia_direcao_sem_encerrar(self):
        model = MapaAutomatico()
        nav = Navigator(None,model,Event(),lambda _: None)
        with patch.object(nav,"command"), patch.object(nav,"wait_for",
                return_value={"autoElapsedMs": 0,"autoEnd": "e"}):
            self.assertFalse(nav.move("f",8,25))
        self.assertTrue(model.direction_blocked(0))
        self.assertEqual(model.pose,Pose())

    def test_busca_fica_ativa_apos_passagem_ou_falta_de_rota(self):
        model = MapaAutomatico()
        model.passages[(0,0,1)] = {"at":[0,0],"axis":1,"crossed":True}
        stop = Event()
        messages = []
        nav = Navigator(None,model,stop,messages.append)
        def finish(_):
            stop.set()
            return True
        with patch.object(nav,"observe") as observe, patch.object(nav,"next_move",return_value=False), patch.object(stop,"wait",side_effect=finish):
            from ControleAutonomo import StopRequested
            with self.assertRaises(StopRequested):
                nav.explore()
        observe.assert_called_once()
        self.assertIn("Aguardando passagem segura", messages)


if __name__ == "__main__":
    unittest.main()
