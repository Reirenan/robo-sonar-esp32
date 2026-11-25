import unittest
from threading import Event
from unittest.mock import patch
from ControleAutonomo import Navigator
from Navegacao import FREE, MapaAutomatico, OCCUPIED

class RecuperacaoTest(unittest.TestCase):
    def test_releitura_libera_incerteza_sem_apagar_ocupacao(self):
        mapa = MapaAutomatico()
        mapa.tentative = {(0,2),(0,3)}
        mapa.cells[0,3] = OCCUPIED
        mapa.quick_observation([(90,None)])
        self.assertIn((0,2),mapa.tentative)
        mapa.quick_observation([(90,80)])
        self.assertNotIn((0,2),mapa.tentative)
        self.assertEqual(mapa.cells[0,3],OCCUPIED)
        self.assertFalse(mapa.clearing)

    def test_giro_menor_e_tentativas_registradas(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _: None)
        nav.side_readings_current = True
        nav.scan_points = [(angle,80 if angle < 90 else 20) for angle in range(0,181,10)]
        with patch.object(mapa,"swept_blockers",
                          side_effect=lambda **kw: {(0,0)} if abs(kw["turn"])>10 else set()):
            self.assertEqual(nav.gap_turns()[0][0][2],-10)
            nav.turn_attempts.add(nav.turn_key(-10))
            self.assertEqual(nav.gap_turns()[0][0][2],-5)
            nav.turn_attempts.add(nav.turn_key(-5))
            self.assertFalse(nav.gap_turns()[0])
            self.assertEqual(mapa.maneuver_blockers["l"],{(0,0)})

    def test_sem_eco_preserva_veto_na_mesma_pose(self):
        mapa = MapaAutomatico()
        mapa.stationary_reference = False
        mapa.has_scan = True
        mapa.tentative_reference = mapa.pose
        mapa.tentative = {(0,2)}
        mapa.maneuver_blockers = {"f": {(0,2)}}
        mapa.scan([(90,None)])
        self.assertIn((0,2),mapa.tentative)

    def test_tres_pulsos_em_trecho_livre_nao_forcam_varredura(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _: None)
        nav.quick_supported = nav.valid_scan = True
        nav.since_scan = 26.25
        with patch.object(mapa,"swept_blockers",return_value=set()), \
             patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.decision(80,allow_turn=False))
            move.assert_called_once_with("f",8.75,25)
            mapa.uncertainty = 9
            move.reset_mock()
            self.assertFalse(nav.decision(80,allow_turn=False))
            move.assert_not_called()

    def test_marca_em_revisao_antecipa_atualizacao(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _: None)
        nav.quick_supported = nav.valid_scan = True
        self.assertEqual(nav.scan_budget(),(50,4))
        mapa.clearing[0,2] = 1
        self.assertEqual(nav.scan_budget(),(25,2))
    def test_giro_de_observacao_esquerda_e_direita(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _:None)
        nav.valid_scan = nav.quick_supported = True
        with patch.object(mapa,"swept_blockers",side_effect=lambda *,turn: set() if turn<0 else {(1,0)}), patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.survey_turn())
            turn.assert_called_once_with(-10)
            self.assertTrue(nav.survey_turn_pending)
        with patch.object(mapa,"swept_blockers",side_effect=lambda *,turn: set() if turn>0 else {(-1,0)}), patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.survey_turn())
            turn.assert_called_once_with(10)

    def test_todas_as_manobras_bloqueadas(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _:None)
        nav.valid_scan = True
        with patch.object(mapa,"swept_blockers",return_value={(0,0)}), patch.object(nav,"turn") as turn, patch.object(nav,"move") as move:
            self.assertFalse(nav.decision(50,allow_turn=True))
            turn.assert_not_called()
            move.assert_not_called()

    def test_giro_de_observacao_pede_sondagem_antes_do_avanco(self):
        from ControleAutonomo import StopRequested
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _:None)
        nav.valid_scan = nav.quick_supported = True
        nav.since_scan = 25

        def pursue(front,allow_turn):
            if front is None:
                nav.survey_turn_pending = True
                return True
            self.assertEqual(front,80)
            raise StopRequested

        with patch.object(nav,"observe") as observe, patch.object(nav,"plan_frontier"), patch.object(nav,"pursue_frontier",side_effect=pursue), patch.object(nav,"quick_observe",return_value=80) as quick:
            with self.assertRaises(StopRequested):
                nav.explore()
        observe.assert_called_once()
        quick.assert_called_once()

    def test_rota_recalculada_executada_na_mesma_rodada(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _:None)
        mapa.frontier_target = (0,2)
        mapa.frontier_path = [(10,0)]

        def replan():
            mapa.frontier_path = [(0,10)]

        with patch.object(mapa,"swept_clear",side_effect=lambda distance=0,turn=0: turn==0), patch.object(nav,"plan_frontier",side_effect=replan) as plan, patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.pursue_frontier(80,allow_turn=True))
        plan.assert_called_once()
        move.assert_called_once_with("f",8.75,25)

    def test_rota_sem_trecho_ainda_permite_avanco_local_com_eco(self):
        mapa = MapaAutomatico()
        nav = Navigator(None, mapa, Event(), lambda _: None)
        mapa.frontier_target = (0, 2)
        with patch.object(mapa, "swept_blockers", return_value=set()), \
             patch.object(nav, "move", return_value=True) as move:
            self.assertTrue(nav.pursue_frontier(80, allow_turn=True))
        move.assert_called_once_with("f", 8.75, 25)

    def test_rota_lateral_sem_giro_permitido_nao_move_robo(self):
        mapa = MapaAutomatico()
        nav = Navigator(None, mapa, Event(), lambda _: None)
        mapa.frontier_target = (2, 0)
        mapa.frontier_path = [(10, 0)]
        with patch.object(mapa, "swept_clear", return_value=True), \
             patch.object(nav, "move") as move:
            self.assertFalse(nav.pursue_frontier(80, allow_turn=False))
        move.assert_not_called()

    def test_rota_lateral_bloqueada_reaproveita_eco_frontal_na_decisao_local(self):
        mapa = MapaAutomatico()
        nav = Navigator(None, mapa, Event(), lambda _: None)
        mapa.frontier_target = (2, 0)
        mapa.frontier_path = [(10, 0)]
        with patch.object(mapa, "swept_clear", return_value=False), \
             patch.object(mapa, "swept_blockers", return_value=set()), \
             patch.object(nav, "plan_frontier"), \
             patch.object(nav, "move", return_value=True) as move:
            self.assertTrue(nav.pursue_frontier(80, allow_turn=True))
        move.assert_called_once_with("f", 8.75, 25)

    def test_varredura_informa_progresso_e_retorno_ao_centro(self):
        msgs = []
        nav = Navigator(None, MapaAutomatico(), Event(), msgs.append)
        points = [(a, 80) for a in range(0, 181, 10)]

        def wait_for(check, timeout):
            self.assertFalse(check({"state": "running", "points": points[:4]}))
            self.assertFalse(check({"state": "running", "points": points[:4]}))
            self.assertFalse(check({"state": "returning", "points": points}))
            state = {"state": "done", "points": points}
            self.assertTrue(check(state))
            return state

        with patch.object(nav, "command"), patch.object(nav, "wait_for", side_effect=wait_for):
            nav.observe()
        self.assertEqual(sum("4/19" in msg for msg in msgs), 1)
        self.assertTrue(any("voltando ao centro" in msg for msg in msgs))
        self.assertEqual(nav.scan_points, points)

    def test_preferir_lateral_mais_larga_e_tentar_o_outro_lado(self):
        mapa = MapaAutomatico()
        nav = Navigator(None, mapa, Event(), lambda _: None)
        nav.side_readings_current = True
        nav.scan_points = [(a, 33 if a < 90 else 42 if a > 90 else 22.6)
                           for a in range(0, 181, 10)]
        mapa.frontier_target, mapa.frontier_path = (-1, 1), [(-5, 15)]
        with patch.object(mapa, "swept_blockers", return_value=set()), \
             patch.object(nav, "turn", side_effect=[False, True]) as turn, \
             patch.object(nav, "move") as move:
            self.assertTrue(nav.pursue_frontier(22.6, allow_turn=True))
        self.assertEqual([args.args[0] for args in turn.call_args_list], [25, -25])
        self.assertEqual(mapa.selected_direction, "l")
        move.assert_not_called()

    def test_reaproveitar_trecho_da_re_apos_girar_e_realinhar(self):
        mapa = MapaAutomatico()
        mapa.cells.update({(x, y): FREE for x in range(-4, 5) for y in range(-4, 9)})
        nav = Navigator(None, mapa, Event(), lambda _: None)
        nav.reverse_limit = 120
        nav.valid_scan = True
        with patch.object(nav, "command") as command, \
             patch.object(nav, "wait_for", side_effect=[
                 {"autoElapsedMs": ms, "autoEnd": "t"} for ms in [350, 100, 100, 120]]):
            self.assertTrue(nav.move("f", 8.75, 25))
            self.assertTrue(nav.move("l", 10, 100))
            mapa.localized = True
            self.assertFalse(nav.backtrack())
            self.assertTrue(nav.move("r", 10, 100))
            mapa.localized = True
            with patch.object(nav, "gap_turns", return_value=([], {})), \
                 patch.object(nav, "survey_turn") as survey:
                self.assertTrue(nav.decision(22.6, allow_turn=True))
                survey.assert_not_called()
        self.assertEqual([args.args[0] for args in command.call_args_list],
                         ["auto_f_350", "auto_l_100", "auto_r_100", "auto_b_120"])
        self.assertAlmostEqual(mapa.pose.y, 5.75)

    def test_reproduzir_varredura_22_cm_sem_folga(self):
        ecos = [22.3, 21.4, 25.2, 35, 35, 20.8, 20.8, 20.8, 21.7, 22.6,
                21.4, 39.6, 42.1, 37, 40.3, 39.1, 38.9, 37.5, 37.3]
        mapa = MapaAutomatico()
        nav = Navigator(None, mapa, Event(), lambda _: None)
        nav.valid_scan = nav.side_readings_current = True
        nav.reverse_limit = 120
        nav.scan_points = list(zip(range(0, 181, 10), ecos))
        mapa.scan(nav.scan_points)
        nav.plan_frontier()
        with patch.object(nav, "move") as move:
            self.assertFalse(nav.pursue_frontier(22.6, allow_turn=True))
        move.assert_not_called()
        self.assertEqual(mapa.directions["f"], (22.6, "Obstáculo até 45 cm"))
        self.assertEqual(mapa.directions["b"], (None, "Sem trecho recente atrás"))
        self.assertIn("sem folga", mapa.directions["r"][1])

    def test_retoma_avanco_apos_duas_varreduras_sem_obstaculo(self):
        mapa = MapaAutomatico()
        mapa.cells.update({(x, y): FREE for x in range(-4, 5) for y in range(-4, 9)})
        nav = Navigator(None, mapa, Event(), lambda _: None)
        mapa.scan([(90, 26)])
        self.assertEqual(mapa.cells[0, 2], OCCUPIED)
        with patch.object(nav, "move", return_value=True) as move:
            self.assertFalse(nav.decision(26, allow_turn=False))
            mapa.scan([(90, 80)])
            self.assertFalse(nav.decision(80, allow_turn=False))
            move.assert_not_called()
            mapa.scan([(90, 80)])
            self.assertTrue(nav.decision(80, allow_turn=False))
        move.assert_called_once_with("f", 8.75, 25)
