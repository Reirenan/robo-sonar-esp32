import unittest
from threading import Event
from unittest.mock import patch

from ControleAutonomo import Navigator, encontrar_aberturas
from Navegacao import MapaAutomatico


class FollowTheGapTest(unittest.TestCase):
    def test_fecho_por_eco_invalido_ou_angulo_ausente(self):
        self.assertEqual(encontrar_aberturas([(30,80),(60,None),(90,100),(150,100)],30),[])
        self.assertEqual(encontrar_aberturas([(30,80),(60,45),(90,100)],30),[])

    def test_empate_escolhe_esquerda(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _: None)
        nav.scan_kind = "quick"
        nav.side_readings_current = True
        nav.scan_points = [(30,90),(60,90),(90,30),(120,90),(150,90)]
        with patch.object(mapa,"swept_blockers",return_value=set()), patch.object(nav,"turn",return_value=True) as turn:
            self.assertTrue(nav.decision(30,allow_turn=True))
        turn.assert_called_once_with(-25)
        self.assertEqual(mapa.selected_gap[1].center,45)

    def test_sondagem_insuficiente_adianta_varredura_antes_da_re(self):
        mapa = MapaAutomatico()
        nav = Navigator(None,mapa,Event(),lambda _: None)
        nav.scan_kind = "quick"
        nav.side_readings_current = True
        nav.scan_points = [(30,55),(60,55),(90,None),(120,None),(150,100)]
        with patch.object(nav,"backtrack",return_value=True) as reverse:
            self.assertFalse(nav.decision(None,allow_turn=True))
            reverse.assert_not_called()
            nav.scan_kind = "full"
            self.assertTrue(nav.decision(None,allow_turn=True))
            reverse.assert_called_once()
