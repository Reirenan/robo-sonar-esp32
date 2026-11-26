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

    def box(self):
        m = MapaAutomatico()
        m.cells = {(x,y): (OCCUPIED if x in {-1,15} or y in {-1,15} else FREE)
                   for x in range(-1,16) for y in range(-1,16)}
        m.pose = Pose(75,75)
        return m

    def test_sem_fronteira_planeja_rota_livre_para_observar_borda(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.plan_frontier()
        self.assertIsNone(m.frontier_target)
        self.assertIsNotNone(m.edge_target)
        self.assertTrue(m.frontier_path)
        self.assertTrue(all(m.clear_disk(*point) for point in m.frontier_path))
        self.assertTrue(nav.edge_visible)
        self.assertFalse(m.passages)

    def test_duas_varreduras_distintas_adiam_o_ponto_e_tentam_outro(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.plan_frontier()
        first = m.edge_target
        m.pose = Pose(*midpoint(first))
        nav.scan_sequence, nav.side_readings_current = 1, True
        nav.plan_frontier()
        nav.plan_frontier()
        self.assertEqual(nav.edge_observations,1)
        self.assertEqual(m.edge_target,first)
        nav.scan_sequence = 2
        nav.plan_frontier()
        self.assertIn(first,nav.deferred_edge_points)
        self.assertIsNotNone(m.edge_target)
        self.assertNotEqual(m.edge_target,first)
        self.assertFalse(nav.inspected_edges)

    def test_so_eco_da_varredura_no_ponto_confirma_borda(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.plan_frontier()
        hit = min(nav.edge_visible)
        m.pose = Pose(*midpoint(m.edge_target))
        m.last_scan_hits = {hit}
        nav.scan_sequence, nav.side_readings_current, nav.scan_kind = 1, True, "quick"
        nav.plan_frontier()
        self.assertFalse(nav.inspected_edges)
        nav.scan_sequence, nav.scan_kind = 2, "full"
        nav.plan_frontier()
        self.assertEqual(nav.inspected_edges,{hit})
        self.assertNotIn(hit,nav.edge_visible)

    def test_volta_para_fronteira_quando_mapa_mostra_abertura(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.plan_frontier()
        self.assertIsNotNone(m.edge_target)
        for x in range(5,11):
            for y in range(15,20):
                m.cells[x,y] = FREE
        nav.plan_frontier()
        self.assertIsNotNone(m.frontier_target)
        self.assertIsNone(m.edge_target)

    def test_sem_rota_de_borda_nao_tenta_avanco_avulso(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        with patch.object(m,"clear_disk",return_value=False):
            nav.plan_frontier()
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.pursue_frontier(80,allow_turn=True))
            move.assert_not_called()
        self.assertEqual(nav.wait_reason,"Bordas restantes sem rota segura")

    def test_ponto_de_borda_atras_alinha_sem_avancar(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        m.edge_target = (7,7)
        m.pose = Pose(*midpoint(m.edge_target))
        nav.edge_visible = {(7,-1)}
        nav.valid_scan = nav.side_readings_current = True
        with patch.object(m,"swept_blockers",return_value=set()), \
             patch.object(nav,"move",return_value=True) as move:
            self.assertTrue(nav.pursue_frontier(65,allow_turn=True))
        move.assert_called_once_with("l",25,100)

    def test_perimetro_verificado_aguarda_em_vez_de_repetir_avanco(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.inspected_edges.update(m.edge_cells())
        nav.plan_frontier()
        with patch.object(nav,"move") as move:
            self.assertFalse(nav.pursue_frontier(80,allow_turn=True))
            move.assert_not_called()
        self.assertEqual(nav.wait_reason,"Bordas alcançáveis verificadas")

    def test_ponto_adiado_volta_a_ser_elegivel_com_mudanca_no_mapa(self):
        m = self.box()
        nav = Navigator(None,m,Event(),lambda _: None)
        nav.plan_frontier()
        first = m.edge_target
        nav.deferred_edge_points[first] = m.frontier_signature(first)
        m.edge_target = None
        nav.plan_frontier()
        self.assertNotEqual(m.edge_target,first)
        m.edge_target = None
        m.cells[first[0],first[1]+5] = OCCUPIED
        self.assertNotEqual(nav.deferred_edge_points[first],m.frontier_signature(first))
        target,_,_ = m.plan_edge(first,m.edge_cells(),nav.deferred_edge_points,())
        self.assertEqual(target,first)
