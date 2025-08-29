'''
We are AI eng and Statistics students respectively.
This module provides :
    Graph Data Structure
    Haversine Formula
    Djkistra algorithm implementation
We used Data Structures & Algorithms in Python by Michael T. Goodrich , Wiley and ChatGPT.
The most of code structure below is implemented from this book and we tried to modify some of methods or attributes in order to use our project.
'''



import math
import heapq
from pykml import parser
from lxml import etree
import json
from typing import Dict, Tuple, Any , List
import os
import argparse
import sys
from io import StringIO
#import rclpy
#from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import json




def haversine(point1, point2):
    R = 6371.0  # Earth's diameter as kilometer
    # converts coordinates to radians
    lat1_rad = math.radians(point1[0])
    lon1_rad = math.radians(point1[1])
    lat2_rad = math.radians(point2[0])
    lon2_rad = math.radians(point2[1])
    # Haversine Formula
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * \
        math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance*1000  # converts to meter

def euclidean_distance(point1,point2):
    vectors_zipped=zip(point1,point2)
    return (sum([(x[0]-x[1])**2 for x in vectors_zipped]))**(1/2)

def manhattan_distance(point1,point2):
    vectors_zipped=zip(point1,point2)
    return sum([abs(x[0]-x[1]) for x in vectors_zipped])

class Graph:

    class Vertex:
        """Lightweight vertex structure for a graph."""
        __slots__ = '_element'

        def __init__(self, x):
            """Do not call constructor directly. Use Graph's insert_vertex(x)."""
            self._element = x

        def element(self):
            """Return element associated with this vertex."""
            return self._element

        def __hash__(self):
            '''will allow vertex to be a map/set key'''
            return hash(id(self))

        def __repr__(self):
            ''' Returns:
            str: A string representation of the Vertex object in the format 'Vertex(element)'.
            '''
            return f"Vertex({self._element})"

        def __str__(self):
            '''   Returns:
            str: A string representation of the element contained in the Vertex object.
            '''
            return str(self._element)

        def __lt__(self, other):
            """Less than operator based on the element of the vertex."""
            return self._element < other._element

    class Edge:
        '''Edge class'''
        __slots__ = '_origin', '_destination', '_weight'

        def __init__(self, u, v, x):
            '''Do not call this constructor . use insert_edge(u,v,x)'''
            self._origin = u
            self._destination = v
            self._weight = x

        def endpoints(self):
            '''returns endpoints of edge such that (u,v) as tuple'''
            return (self._origin, self._destination)

        def opposite(self, v):
            return self._destination if v is self._origin else self._origin

        def weight(self):
            '''returns weight of edge.'''
            return self._weight

        def __hash__(self):
            '''will allow vertex to be a map/set key'''
            return hash((self._origin, self._destination, self._weight))

        def __repr__(self):
            '''
            Returns:
            str: A string representation of the Edge object in the format 'Edge(origin, destination, weight)'.
            '''
            return f"Edge({self._origin}, {self._destination}, {self._weight})"

        def __str__(self):
            '''
            Returns:
            str: A string representation of the Edge object in the format '(origin -> destination, weight)'.
            '''
            return f"({self._origin} -> {self._destination}, {self._weight})"
    
    def __init__(self):
        '''
        _path stores what shortest_path function returns (actually it returns lenght and path so we store only path)
        _visited is used for storing visited vertices in memory.
        '''
        self.vertex_G_edges = {}
        self.stop_list = list()
        self._path = list()
        self._visited = list()
        self._VertexDict = dict()




    def vertex_count(self):
        """returns number of vertices."""
        return len(self.vertex_G_edges)

    def vertices(self):
        """returns all vertices as dict keys"""
        return self.vertex_G_edges.keys()
    
    def edge_count(self):
        """returns number of edges"""
        total = sum(len(self.vertex_G_edges[v]) for v in self.vertex_G_edges)
        return total
    
    def edges(self):
        """returns all edges as set"""
        result = set()
        for secondary_map in self.vertex_G_edges.values():
            result.update(secondary_map.values())
        return result

    def get_edge(self, u: Vertex, v: Vertex):
        """Return the edge from u to v, or None if not adjacent."""
        return self.vertex_G_edges[u].get(v)

    def insert_edge(self, u, v, x=float('inf')):
        """Insert and return a new Edge from u to v with auxiliary element x."""
        e1 = self.Edge(u, v, x)  # u->v instantiate edge from u to v
        e2 = self.Edge(v, u, x)  # v->u instantiate edge from v to u
        if u not in self.vertex_G_edges:
            self.vertex_G_edges[u] = {}
        if v not in self.vertex_G_edges:
            self.vertex_G_edges[v] = {}
        self.vertex_G_edges[u][v] = e1
        self.vertex_G_edges[v][u] = e2
        return e1
    
    def insert_vertex(self, x=None):
        """Insert and return a new Vertex with element x."""
        v = self.Vertex(x)
        self.vertex_G_edges[v] = {}
        return v

    def weights(self):
        """Returns sum of all unique edge weights."""
        total = 0
        for edge in self.edges():
            total += edge._weight
        return total/2

    def remove_edge(self, v1: Vertex = None, v2: Vertex = None) -> None:
        '''removes edge between v1 and v2.'''
        if v1 in self.vertex_G_edges[v2]:
            del self.vertex_G_edges[v2][v1]
            del self.vertex_G_edges[v1][v2]
        return None

    def remove_vertex(self, v1: Vertex = None) -> None:
        """Removes given vertex and all its incident edges."""
        if v1 is None:
            return None
        if v1 in self.vertex_G_edges:
            neighbors = list(self.vertex_G_edges[v1].keys())
            for v2 in neighbors:
                del self.vertex_G_edges[v1][v2]
                del self.vertex_G_edges[v2][v1]
            del self.vertex_G_edges[v1]

    def dijkstra(self, start_vertex: Vertex):
        """Dijkstra's algorithm to find shortest paths from start_vertex to all other vertices."""
        distances = {v: float('inf') for v in self.vertices()}
        previous_vertices = {v: None for v in self.vertices()}
        distances[start_vertex] = 0
        priority_queue = [(0, start_vertex)]  # (distance, vertex)
        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)
            if current_distance > distances[current_vertex]:
                continue
            for edge in self.vertex_G_edges[current_vertex].values():
                neighbor = edge.opposite(current_vertex)
                new_distance = current_distance + edge.weight()
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous_vertices[neighbor] = current_vertex
                    heapq.heappush(priority_queue, (new_distance, neighbor))
        return distances, previous_vertices

    def shortest_path(self, start_vertex: Vertex, end_vertex: Vertex) -> tuple:
        """Returns the shortest path from start_vertex to end_vertex using Dijkstra's algorithm."""
        distances, previous_vertices = self.dijkstra(start_vertex)
        path = []
        current_vertex = end_vertex
        while current_vertex is not None:
            path.append(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        path.reverse()
        self._path = path
        return distances[end_vertex], path
    
    def visited_vertex(self, current_vertex: tuple, target_vertex: Vertex = None, R: float = 5, metric_func=haversine) -> tuple:
        if target_vertex == None:
            return self._path[0]
        distance = metric_func(current_vertex, target_vertex._element)
        if distance <= R:
            self._visited.append(target_vertex)
            if target_vertex == self._path[-1]:
                return True
            else:
                new_target = self._path[len(self._visited)]
                return target_vertex, new_target
        return None
    
    def add_stops(self, *vertices ,end:bool=True) -> None:
        """Adds vertices to the stop list in the order they are provided."""
        if end:
            for vertex in vertices:
                self.stop_list.append(vertex)
        else :
            for vertex in vertices:
                self.stop_list.insert(0,vertex)

    def stop_pairs(self) -> tuple:
        if len(self.stop_list)<2:
            return None
        pair = (self.stop_list[0], self.stop_list[1])
        self.stop_list = self.stop_list[1:]
        return pair

    def not_include_edge(self,v1:Vertex=None,v2:Vertex=None) ->None:
        if v1==None or v2==None :
            v1,v2 =self._path[-2:]
        weight=self.get_edge(v1,v2).weight()
        self.insert_edge(v1,v2,float("inf"))
        return v1,v2,weight

    def set_default(self,stop_list:bool=False,_path:bool=True,_visited:bool=True,_VertexDict:bool=False) -> None:
        if stop_list:
            self.stop_list = list()
        if _path:
            self._path = list()
        if _visited:
            self._visited = list()
        if _VertexDict:
            self._VertexDict = list()




def gps_graph_creater(g: Graph, vertexDict: dict = {}, kml_file_path="Map.kml",metric=haversine) -> None:
    '''
    This function creates vertex and edge from given kml file then saves VertexDict....
    VertexDict : 
            Keys : Names
            Values : Coordinates like (40.78982305690907, 29.50933038732505)
            first elements of values represent latitude and second altitude..... 
    '''
    with open(kml_file_path, 'r', encoding='utf-8') as f:
        doc = parser.parse(f)
        root = doc.getroot()
    coords_dict: dict[str, tuple[float, float]] = {}
    for placemark in root.Document.Placemark:
        name = placemark.name if hasattr(placemark, 'name') else "No Name"
        if hasattr(placemark, 'Point'):
            # coords1 longtitude(boylam)
            # coords2 latitude(enlem)
            # coords3 altitude(yukseklik)
            coords = placemark.Point.coordinates.text.strip()
            coord_pairs = coords.split()
            for coord in coord_pairs:
                # creates vertex
                lon, lat, alt = map(float, coord.split(','))
                vertex = g.insert_vertex(((lat), (lon)))
                vertexDict[f"{name}"] = vertex
                coords_dict[name] = (lat, lon)
        elif hasattr(placemark, 'LineString'):
            coords = placemark.LineString.coordinates.text.strip()
        elif hasattr(placemark, 'Polygon'):
            coords = placemark.Polygon.outerBoundaryIs.LinearRing.coordinates.text.strip()
    for placemark in root.Document.Placemark:
        # creates edges
        name = placemark.name if hasattr(placemark, 'name') else "No Name"
        if hasattr(placemark, 'LineString'):
            points = name.text.split("-")
            distance = metric(vertexDict[points[0]]._element,vertexDict[points[1]]._element)
            g.insert_edge(vertexDict[points[0]],
                          vertexDict[points[1]], distance)
        elif hasattr(placemark, 'Polygon'):
            coords = placemark.Polygon.outerBoundaryIs.LinearRing.coordinates.text.strip()
    return coords_dict

def cripto(vertexDict: dict, element: Graph.Vertex):
    '''
    VertexDict ->Dict element -> element._element use this way
    '''
    for key, value in (vertexDict.items()):
        if (value._element == element):
            return key
    return None

def interactive_shortest_path(g: Graph, start_name, end_name, blocked_edges: List[Tuple[str, str]] = None) -> Tuple[
        float, List[Graph.Vertex]]:

    list_vertex_names(g)

    if blocked_edges:
        for u_name, v_name in blocked_edges:
            u = g._VertexDict[u_name]
            v = g._VertexDict[v_name]
            g.insert_edge(u, v, float("inf"))  # geçici engelleme

    print("\n--- Sonuç ---")
    start_vertex = g._VertexDict[start_name]
    end_vertex = g._VertexDict[end_name]





    distance, path = g.shortest_path(start_vertex, end_vertex)

    if blocked_edges:
        for u_name, v_name in blocked_edges:
            u = g._VertexDict[u_name]
            v = g._VertexDict[v_name]
            real_weight = haversine(u._element, v._element)
            g.insert_edge(u, v, real_weight)
    print(f"En kısa mesafe: {distance:.2f} metre")
    print("İzlenen yol:")
    for v in path:
        print(" -", cripto(g._VertexDict, v._element))
    return distance, path









def list_vertex_names(g: Graph):
    print("\nMevcut Noktalar:")
    for name in g._VertexDict.keys():
        print(" -", name)
def read_geojson_features(json_file: str) -> Dict[str, Dict[str, Any]]:
    """
    Reads a GeoJSON FeatureCollection from the given file and returns a dict mapping
    feature names to their properties:
    {
        name: {
            "description": str,
            "local": (local_x, local_y),
            "heading": float,
            "coord": (lat, lon)
        },
        ...
    }
    """
    result: Dict[str, Dict[str, Any]] = {}
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"File not found: {json_file}")
        return result
    except json.JSONDecodeError as e:
        print(f"Invalid JSON in file {json_file}: {e}")
        return result

    features = data.get("features", [])
    for feat in features:
        props = feat.get("properties", {})
        name = props.get("name")
        if not name:
            continue

        geom = feat.get("geometry", {})
        if geom.get("type") != "Point":
            continue

        coords = geom.get("coordinates", [])
        if len(coords) < 2:
            continue

        lon, lat = coords[0], coords[1]
        result[name] = {
            "description": props.get("description"),
            "local": (props.get("local_x"), props.get("local_y")),
            "heading": props.get("heading"),
            "coord": (lat, lon),
        }
    return result

from typing import Dict, Tuple, Any
import math

def match_geojson_to_kml(
    data: Dict[str, Dict[str, Any]],
    coords_dict: Dict[str, Tuple[float, float]]
) -> List[str] :
    """
    Her GeoJSON özelliğini ('data') KML sözlüğündeki en yakın koordinat ('coords_dict')
    ile eşleştirir ve çıkış olarak birleştirilmiş bir KML string’i döner.

    Args:
        data: {
            feature_name: {
                "coord": (lat, lon),
                ...
            },
            ...
        }
        coords_dict: {
            placemark_name: (lat, lon),
            ...
        }

    Returns:
        Tek bir KML dosyası olarak string. İçinde her çift için bir <Placemark> var:
        <name>GeoJSONName(KMLName)</name>
        <Point><coordinates>lon,lat</coordinates></Point>
    """
    matches: List[str] = []
    def haversine(a: Tuple[float,float], b: Tuple[float,float]) -> float:
        # Dünya yarıçapı km
        R = 6371.0
        lat1, lon1 = a
        lat2, lon2 = b
        φ1, φ2 = math.radians(lat1), math.radians(lat2)
        Δφ = math.radians(lat2 - lat1)
        Δλ = math.radians(lon2 - lon1)
        x = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
        return 2 * R * math.atan2(math.sqrt(x), math.sqrt(1-x))

    # KML başlık ve kuyruk şablonları
    kml_header = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
"""
    kml_footer = """  </Document>
</kml>
"""

    body = ""
    for feature_name, feat in data.items():
        lat_lon = feat.get("coord")
        if not lat_lon:
            continue

        # en yakın KML noktasını bul
        best_name = None
        best_dist = float("inf")
        for kml_name, kml_latlon in coords_dict.items():
            d = haversine(lat_lon, kml_latlon)
            if d < best_dist:
                best_dist = d
                best_name = kml_name

        if best_name is not None:
            lat, lon = lat_lon
            print(feature_name,best_name,  lat, lon)
            matches.append(f"{best_name}")
    return matches
def get_name_by_coord(coord: Tuple[float, float], coords_dict: Dict[str, Tuple[float, float]]) -> str:
    for name, c in coords_dict.items():
        if c == coord:
            return name
    return str(coord)
def create_full_route(g, merged_kml) -> List[str]:
    stop_names = [name for name in g._VertexDict.keys() if "STOP" in name]

    pairs = []
    for i in range(1, len(merged_kml) - 1):
        pairs.append((merged_kml[i], merged_kml[i + 1]))

    full_route = []
    total_dist = 0.0

    for i, (u_name, v_name) in enumerate(pairs):
        distance, path = interactive_shortest_path(g, u_name, v_name)
        total_dist += distance

        for j, v in enumerate(path):
            name = cripto(g._VertexDict, v._element)

            # Geri dönüş kontrolü
            if len(full_route) >= 2 and name == full_route[-2]:
                print(f"🔁 Geri dönüş tespit edildi: {full_route[-2]} → {full_route[-1]} → {name}")

                # Geri dönülen vertex'i geçici olarak inf yap
                prev_v = g._VertexDict[full_route[-1]]
                back_v = g._VertexDict[full_route[-2]]

                # Eski ağırlığı kaydet
                edge = g.get_edge(prev_v, back_v)
                old_weight = edge.weight()

                # Bağlantıyı inf yap
                g.insert_edge(prev_v, back_v, float("inf"))

                # Yeni path hesapla
                alt_distance, alt_path = g.shortest_path(prev_v, g._VertexDict[v_name])
                print(f"➡️ Alternatif rota bulundu ({cripto(g._VertexDict, prev_v._element)} → {v_name})")

                # Edge’i eski haline getir
                g.insert_edge(prev_v, back_v, old_weight)

                # Yeni yolu ekle
                for alt_v in alt_path[1:]:
                    alt_name = cripto(g._VertexDict, alt_v._element)
                    if full_route[-1] != alt_name:
                        full_route.append(alt_name)

                break  # Bu path tamamlandı, diğerine geç
            else:
                if not full_route or full_route[-1] != name:
                    full_route.append(name)

    # Aynı noktaları art arda ekleme
    i = 1
    while i < len(full_route):
        if full_route[i] == full_route[i - 1]:
            del full_route[i]
        else:
            i += 1

    # STOP işlem bloğu
    stack_data = []
    processed_stops = []

    i = 0
    while i < len(full_route):
        current = full_route[i]
        current_vertex = g._VertexDict[current]
        neighbors = g.vertex_G_edges[current_vertex].keys()

        stop_inserted = False

        for neighbor in neighbors:
            stop_name = cripto(g._VertexDict, neighbor._element)

            if "STOP" in stop_name:
                # 🔒 Daha önce eklendiyse geç
                if stop_name in processed_stops:
                    continue

                # Stack’e ilk kez giriyorsa
                if stop_name not in [item[1] for item in stack_data]:
                    stack_data.append((current, stop_name))
                    print(f"Stack'e eklendi: ({current} → {stop_name})")
                else:
                    # Aynı STOP ikinci kez görüldü, araya ekle
                    first_vertex = None
                    for item in stack_data:
                        if item[1] == stop_name:
                            first_vertex = item[0]
                            break

                    if first_vertex is None:
                        continue

                    try:
                        first_index = full_route.index(first_vertex)
                        second_index = i
                        if first_index > second_index:
                            first_index, second_index = second_index, first_index
                    except ValueError:
                        continue

                    # STOP araya yerleştiriliyor
                    full_route = (
                        full_route[:first_index + 1] +
                        [stop_name] +
                        full_route[second_index:]
                    )

                    print(f"✅ STOP eklendi: {first_vertex} → {stop_name} → {full_route[second_index]}")
                    stack_data = [s for s in stack_data if s[1] != stop_name]
                    processed_stops.append(stop_name)  # ✅ tekrar işlememek için
                    stop_inserted = True
                    break

        if stop_inserted:
            i = 0  # reset
        else:
            i += 1

    print("\n📌 Final Rota:")
    print(full_route)
    return full_route

def gps_callback(self, msg: NavSatFix):
        """
        GPS düzeltme verisini aldığında çağrılır.
        msg.latitude ve msg.longitude değerlerini günceller.
        """
        self.current_gps = (msg.latitude, msg.longitude)
        #self.get_logger().info(
        #    f"[GPS] lat: {msg.latitude:.6f}, lon: {msg.longitude:.6f}")

def plan_path(self, start_name: str, end_name: str):
        """
        Belirtilen başlangıç ve bitiş noktaları arasında rota planlar.
        """
        _, vertex_path = interactive_shortest_path(self.g, start_name, end_name)
        # Vertex listesinden isim listesine dönüştür
        self.current_path = [cripto(self.g._VertexDict, v._element) for v in vertex_path]
        self.next_index = 0






if __name__ == "__main__":
    g = Graph()
    coords_dict = gps_graph_creater(g, g._VertexDict, kml_file_path="Coordinates (1).kml")


    parser = argparse.ArgumentParser(description="Read GeoJSON files and extract features.")
    parser.add_argument("path", help="File or directory to read GeoJSON from.")
    args = parser.parse_args()

    data = read_geojson_features(args.path)
    merged_kml = match_geojson_to_kml(data, coords_dict)
    print(merged_kml)
    full_route = create_full_route(g, merged_kml)



