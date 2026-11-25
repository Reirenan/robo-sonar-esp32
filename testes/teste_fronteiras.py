from threading import Event
import unittest
from unittest.mock import patch

from ControleAutonomo import Navigator
from Navegacao import FREE, OCCUPIED, MapaAutomatico, Pose, midpoint


class FronteirasTest(unittest.TestCase):
    def test_retorna_pelo_tronco_e_acha_outro_ramo(self):
        m = MapaAutomatico()
        # Cruzamento de corredores; a célula do fim esquerdo é o beco.
        graph = {(-2,0),(-1,0),(0,0),(1,0),(2,0),(3,0)}
        m.cells.update(dict.fromkeys(graph,FREE))
        nav = Navigator(None,m,Event(),lambda _: None)
        m.pose = Pose(*midpoint((-1,0)))
        m.frontier_target = (-2,0)
        with patch.object(m,"clear_disk",side_effect=lambda x,y: (-15<=x<=35 and y==5)), patch.object(
                m,"frontier_evidence",side_effect=lambda k: {(k[0],1)} if k in {(-2,0),(3,0)} else set()):
            nav.plan_frontier()
            self.assertEqual(m.frontier_target,(-2,0))
            m.pose = Pose(*midpoint((-2,0)))
            nav.plan_frontier()
            self.assertEqual(m.frontier_path,[])
            nav.plan_frontier()
            self.assertEqual(m.frontier_target,(3,0))
            self.assertIn(midpoint((0,0)),m.frontier_path)

    def test_frente_sem_eco_na_rota_nao_avanca(self):
        m = MapaAutomatico()
        m.frontier_target = (0,2)
        m.frontier_path = [(0,25)]
        nav = Navigator(None,m,Event(),lambda _: None)
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.pursue_frontier(None,allow_turn=False))
            move.assert_not_called()

    def test_alvo_adiado_reabre_quando_mapa_muda(self):
        m = MapaAutomatico()
        target = (0,1)
        signature = m.frontier_signature(target)
        m.cells[0,3] = OCCUPIED
        self.assertNotEqual(signature,m.frontier_signature(target))
        with patch.object(m,"clear_disk",side_effect=lambda x,y: 0<=x<=5 and 0<=y<=15), patch.object(
                m,"frontier_evidence",side_effect=lambda k: {(0,4)} if k==target else set()):
            chosen,path = m.plan_frontier(None,{target:signature})
        self.assertEqual(chosen,target)
        self.assertEqual(path,[midpoint(target)])
