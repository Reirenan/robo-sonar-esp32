import unittest
from threading import Event
from unittest.mock import patch
from ControleAutonomo import Navigator
from Navegacao import MapaAutomatico, OCCUPIED

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
        nav.scan_points = [(angle,80 if angle < 90 else 30) for angle in range(0,181,10)]
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
