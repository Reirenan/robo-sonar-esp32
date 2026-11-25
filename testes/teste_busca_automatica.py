from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
from time import monotonic
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
        self.assertNotIn(target,model.clearing)
        model.scan([(90,120),(90,121)])
        self.assertEqual(model.cells[target],OCCUPIED)
        self.assertEqual(model.clearing[target],1)
        model.scan([(90,None)])
        self.assertEqual(model.cells[target],OCCUPIED)
        model.scan([(90,120)])
        self.assertEqual(model.cells[target],FREE)

    def test_roda_em_movimento_invalida_referencia_para_remover(self):
        model = MapaAutomatico()
        target = cell(0,45)
        model.scan([(90,45)])
        model.integrate("f",200)
        with patch("Navegacao.reconhecer_pose",return_value=None):
            model.scan([(90,110)])
            model.scan([(90,110)])
        self.assertEqual(model.cells[target],OCCUPIED)
        self.assertNotIn(target,model.clearing)

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

    def test_re_sem_trajeto_na_origem(self):
        model = MapaAutomatico()
        nav = Navigator(None,model,Event(),lambda _: None)
        nav.reverse_limit = 120
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.backtrack())
            move.assert_not_called()

    def test_re_curta_respeita_tempo_e_limite_do_trecho(self):
        model = MapaAutomatico()
        for i in range(-8,9):
            for j in range(-8,9):
                model.cells[i,j] = FREE
        model.pose = Pose(0,10,0)
        model.localized = True
        nav = Navigator(None,model,Event(),lambda _: None)
        nav.reverse_limit = 120
        nav.rear_origin = Pose()
        nav.rear_heading = 0
        nav.forward_at = monotonic()
        with patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.backtrack())
            move.assert_called_once_with("b",3,25)
            move.reset_mock()
            model.pose = Pose(0,1,0)
            self.assertFalse(nav.backtrack())
            move.assert_not_called()

    def test_re_exige_folga_e_posicao_confirmada(self):
        model = MapaAutomatico()
        model.pose = Pose(0,10,0)
        nav = Navigator(None,model,Event(),lambda _: None)
        nav.reverse_limit = 120
        nav.rear_origin = Pose()
        nav.rear_heading = 0
        nav.forward_at = monotonic()
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.backtrack())
            model.localized = True
            model.cells.pop((0,0),None)
            self.assertFalse(nav.backtrack())
            move.assert_not_called()

    def test_integracao_de_re_mantem_rumo(self):
        model = MapaAutomatico()
        model.pose = Pose(0,10,0)
        model.integrate("b",120)
        self.assertEqual(model.pose,Pose(0,7,0))
        self.assertFalse(model.stationary_reference)

    def direcoes(self,left=90,right=90):
        model = MapaAutomatico()
        for i in range(-7,8):
            for j in range(-7,8):
                model.cells[i,j] = FREE
        nav = Navigator(None,model,Event(),lambda _: None)
        nav.scan_points = [(i,left if i<90 else right if i>90 else 30)
                           for i in range(0,181,10)]
        nav.side_readings_current = True
        return model,nav

    def test_frente_tem_prioridade(self):
        model,nav = self.direcoes()
        with patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.decision(80,allow_turn=True))
        self.assertEqual(model.selected_direction,"f")
        move.assert_called_once_with("f",8.75,25)

    def test_direita_quando_a_abertura_e_maior(self):
        model,nav = self.direcoes(left=70,right=130)
        with patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.decision(32,allow_turn=True))
        self.assertEqual(model.selected_direction,"r")
        turn.assert_called_once_with(25)

    def test_tenta_direita_quando_esquerda_esta_ocupada(self):
        model,nav = self.direcoes(left=30,right=100)
        with patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.decision(30,allow_turn=True))
        self.assertEqual(model.selected_direction,"r")
        turn.assert_called_once_with(25)

    def test_giro_parcial_nao_exige_disco_inteiro(self):
        model,nav = self.direcoes()
        full_disk = set(model.footprint(0,0))
        swept = {k for angle in range(0,26,1)
                 for k in model.body_cells(0,0,angle,5)}
        obstruction = next(iter(full_disk-swept))
        model.cells[obstruction] = OCCUPIED
        self.assertFalse(model.clear_disk(0,0))
        self.assertTrue(model.swept_clear(turn=25))
        model.cells[0,0] = OCCUPIED
        self.assertFalse(model.swept_clear(turn=25))

    def test_sem_eco_na_frente_nao_dispara_avanco_rapido(self):
        model,nav = self.direcoes()
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.decision(None,allow_turn=False))
        self.assertIsNone(model.selected_direction)
        move.assert_not_called()

    def test_avanco_rapido_respeita_25_cm_entre_varreduras(self):
        _,nav = self.direcoes()
        nav.since_scan = 21
        with patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.decision(80,allow_turn=False))
        move.assert_called_once_with("f",4,25)

    def test_ecos_incertos_de_varreduras_antigas_nao_acumulam(self):
        model = MapaAutomatico()
        model.scan([(90,45)])
        model.integrate("l",100)
        old_mark = cell(0,45)
        with patch("Navegacao.reconhecer_pose",return_value=None):
            model.scan([(90,120)])
            first = set(model.tentative)
            model.scan([(90,150)])
        self.assertTrue(first)
        self.assertFalse(first & model.tentative)
        self.assertEqual(model.cells[old_mark],OCCUPIED)

    def test_sondagem_nao_altera_mapa_nem_votos(self):
        model = MapaAutomatico()
        model.scan([(90,45)])
        target = cell(0,45)
        model.scan([(90,120)])
        self.assertEqual(model.clearing[target],1)
        before = dict(model.cells)
        model.quick_observation([(90,120),(30,40)])
        self.assertEqual(model.cells,before)
        self.assertEqual(model.clearing[target],1)
        self.assertTrue(model.quick_hits)

    def test_sondagem_curta_escolhe_lateral_sem_varredura_completa(self):
        model,nav = self.direcoes()
        result = {"state":"done","scanKind":"quick","angle":90,
                  "points":[[30,70],[60,65],[90,32],[120,80],[150,85]]}
        with patch.object(nav,"command") as command, patch.object(nav,"wait_for",return_value=result):
            front = nav.quick_observe()
        command.assert_called_once_with("probe")
        with patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.decision(front, allow_turn=True))
        turn.assert_called_once_with(25)
        self.assertEqual(model.selected_direction,"r")

    def test_sondagem_curta_recusa_resultado_incompleto(self):
        _,nav = self.direcoes()
        with patch.object(nav,"command"), patch.object(nav,"wait_for",
                 return_value={"state":"done","scanKind":"quick","angle":120,"points":[]}):
            with self.assertRaisesRegex(RuntimeError,"incompleta"):
                nav.quick_observe()

    def test_ciclo_pega_lateral_rapida_antes_de_atualizar_mapa(self):
        from ControleAutonomo import StopRequested
        model,nav = self.direcoes()
        nav.quick_supported = True
        calls = []

        def observe():
            calls.append("completa")
            if len(calls) > 1:
                raise StopRequested

        def decide(front,allow_turn):
            calls.append((front,allow_turn))
            if front == 99:
                nav.since_scan = 25
            return front in {30,99}

        with patch.object(nav,"observe",side_effect=observe), \
             patch.object(nav,"front_probe",return_value=None), \
             patch.object(nav,"quick_observe",return_value=99) as quick, \
             patch.object(nav,"decision",side_effect=decide):
            with self.assertRaises(StopRequested):
                nav.explore()
        quick.assert_called_once()
        self.assertEqual(calls,["completa",(30,True),(None,False),(99,True),"completa"])

    def test_busca_fica_ativa_apos_passagem_ou_falta_de_rota(self):
        model = MapaAutomatico()
        model.passages[(0,0,1)] = {"at":[0,0],"axis":1,"crossed":True}
        stop = Event()
        messages = []
        nav = Navigator(None,model,stop,messages.append)
        def finish(_):
            stop.set()
            return True
        with patch.object(nav,"observe") as observe, patch.object(nav,"decision",return_value=False), patch.object(stop,"wait",side_effect=finish):
            from ControleAutonomo import StopRequested
            with self.assertRaises(StopRequested):
                nav.explore()
        observe.assert_called_once()
        self.assertTrue(any(m.startswith("Aguardando passagem segura") for m in messages))


if __name__ == "__main__":
    unittest.main()
