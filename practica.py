#Contreras Matla Luis Fernando.
#S21020225.
# Librerías de RoboDK
from robodk.robolink import * 
from robodk.robomath import *  
import time
import numpy as np
import heapq
import Pyro4

RDK = Robolink()
robot = RDK.Item('UR3e') #Nombre del robot
gripper = RDK.Item('Gripper') #Nombre del gripper acoplado
robot_inicio = RDK.Item('UR3e_Inicio')

#Escenario 1.
Home = [5,-112,-94,-64,90,0]
goal = [125,-96,-112,-60,90,0]

#Escenario 2.
Home = [0,-110,-90,-70,90,0] 
goal = [180,-116,-70,-36,90,0]

#Escenario 3.
Home = [0,-90,-90,-90,90,0]
goal = [-156,-90,-104,-74,90,0]

#Escenario 4.
Home = [0,-140,-90,8,90,0]
goal = [-200,-98,-112,-60,90,0]

open = [50, 0, 0, 0, 0, 0]
close = [0, 0, 0, 0, 0, 0]

#Limites de velocidad y aceleracion
gripper.setSpeedJoints(10)
gripper.setAccelerationJoints(10)
robot.setSpeedJoints(10)
robot.setAcceleration(10)

#Mandar el robot a Home
#Home es su posicion inicial
robot.setJoints(Home)
robot_inicio.setJoints(goal)


#Verificador de colisión
def collition(j1,j2):
    col = robot.MoveJ_Test(j1, j2)
    if col != 0:
        return True
    return False

#Funcion para verificar una trayectoria
def VerEjc(tra): #tra = trayectoria a ejecutar([[q1,q2,q3...q6], [q1,q2,q3...q6], [q1,q2,q3...q6]........[q1,q2,q3...q6]])
    for i in range(len(tra)-1):
        p1 = tra[i]
        p2 = tra[i+1]
        com = collition(p1, p2)
        time.sleep(2)
        print("--------------------------------------")
        if com != 0:
            print('\033[91m'+"COLICION DETECTADA\n"+'\033[0m'+"La trayectoria de:\n",p1,"\n",p2, "\nes invalida")
            a = True
            break
        else:
            print("La trayectoria de:\n",p1,"\n",p2, "\nes correcta")
            a = False

    #Ejecucion de trayectoria valida
    if a == False:
        robot.setJoints(Home)
        print("EJECUTANDO TRAYECTORIA...")
        time.sleep(5)
        for i in range(len(tra)):
            robot.MoveJ(tra[i])   

#Funcion para verificar si existe colision entre 2 configuraciones articulares 
def VerPun(p1,p2): #P1 = Configuracion actual(i), P2 = Siguiente configuracion(i+1)
    com = collition(p1, p2)
    time.sleep(2)
    print("--------------------------------------")
    if com != 0:
        print ('\033[91m'+"COLICION DETECTADA\n"+'\033[0m'+"La trayectoria de:\n",p1,"\n",p2, "\nes invalida")
        robot.setJoints(p1)
        return (True)
    else:
        print ("La trayectoria de:\n",p1,"\n",p2, "\nes correcta")
        return (False)



#----------------------------------------------------------------------------------------------------------------
#Programa principal

'''
VerEjec valida la trayecria dada y la ejecuta en caso de no tener colisiones.
Esta debe de ser mandada mediante una lista de listas, ejemplo:
Trayectoria = [[0,-90,-90,-90,90,0],[-98,-93,-108,-68,90,0]]
'''
#VerEjc(Trayectoria)

'''
VerPun valida la trayectoria entre 2 puntos, los cuales deben de ser proporcionados de la siguiente manera:
p1 = [0,-90,-90,-90,90,0]   p2 = [-98,-93,-108,-68,90,0]
'''
#VerPun(p1, p2)

#defino una clase que contiene los métodos estáticos relacionados con la cinemática directa y la detección de colisiones.
class RoboDK_Server:

    @staticmethod
    #calcula la posición cartesiana de un robot dado una configuración.
    def Cin_Dir(conf):
        pose = robot.SolveFK(conf) #este método devuelve la pose del robot en el espacio cartesiano
        xyz = pose.Pos() #extrae las coordenadas x, y, z de la pose calculada.
        return round(xyz[0], 3), round(xyz[1], 3), round(xyz[2], 3) #redondea las coordenadas a res decimales y las devuelve como una tupla.

    @staticmethod
    #determina si existe la colisión en un trayecto lineal entre dos puntos, lo cual divide el trayecto en pasos intermedios.
    def detectacolisiones(p1, p2, npasos=5): #calcula los puntos intermedios entre q1 y q2. para cada paso i, se interpola linealmente en cada coordenada j.
        pintermedios = [
            [p1[j] + (i / npasos) * (p2[j] - p1[j]) for j in range(len(p1))]
            for i in range(npasos + 1)
        ]

        for i in range(len(pintermedios) - 1): #itera sobre los pares consecutivos de puntos intermedios.
            if collition(pintermedios[i], pintermedios[i + 1]): #comprueba si hay colisión entre dos puntos.
                return True, i, pintermedios[i] #si llega a detectar una colisión devuelve tru, su índice del punto donde ocurrió la colisión y las coordenadas.
        return False, -1, None #si no llega a detectar la colisión devuelve false, un índice -1 y none para las coordenadas.

#defino una clase que representa un nodo en el algoritmo.
class Nodo:
    def __init__(self, pos, configuracion, papa=None): #inicializa las propiedades del nodo.
        self.pos = pos   #posición cartesiana del nodo.
        self.configuracion = configuracion  #configuración articular asociada al nodo.
        self.hijos = []  #lista de gijos generados desde este nodo.
        self.papa = papa  #referencia al nodo padre el cual se utiliza para reconstruir el camino del objetivo.
        self.h = 0  #heurística
        self.g = 0  #costo acumulado desde el nodo inicial. 
        self.f = 0  #costo total de g + h.
    
    #heurística que combina la distancia euclidiana cartesiana y la distancia de Manhattan en los puntos clave.
    def heuristica(self, metaconf):
        posmeta = RoboDK_Server.Cin_Dir(metaconf) #obtiene la posición cartesiana del objetivo a partir de su configuración.
        distcart = sum(abs(self.pos[i] - posmeta[i]) for i in range(3)) #calcula la distancia cartesiana entre la posición actual y la del objetibvo.
        distart = sum(abs(self.configuracion[i] - metaconf[i]) for i in range(6)) #calcula la distancia articular entre la configuración actual y la del objetivo.
        self.h = distart * 0.3 + distcart * 0.7 #combina la distancias con pesos .3 para articular y .7 para cartesiana.
        self.f = self.g + self.h #calcula el costo total de dicho nodo.

    #genera los nodos hijos aplicando movimientos en las articulaciones.
    def generarhijos(self, metaconf, visitados, basedelta=5):
        distanciaobjetivo = sum(abs(self.configuracion[i] - metaconf[i]) for i in range(6)) #calcula la distancia total del objetivo en el espacio articular.
        delta = max(1, basedelta if distanciaobjetivo > 20 else 2) #llega a determinar el paso de ajuste el cual se basa en la distancia del objetivo.
        for i in range(6): #itera sobre cada articulación del robot.
            if abs(self.configuracion[i] - metaconf[i]) < delta: #omite las articulaciones cuya diferencia al objetivo sea menor al paso.
                continue
            for cambio in [-delta, delta]: #genera dos nuevos valores para la articulación que son positivo y negativo.
                confignueva = self.configuracion[:] #llega a crear una copia de la configuración actual.
                confignueva[i] += cambio #aplica el cambio a la articulación seleccionada.
                if not (-360 <= confignueva[i] <= 360): #omite las configuraciones fuera del rango permitido -360° a 360.
                    continue  
                configtupla = tuple(confignueva) #evita generar nodos que ya fueron visitados.
                if configtupla in visitados:
                    continue 
                colision, _, _ = RoboDK_Server.detectacolisiones(self.configuracion, confignueva) #verifica si el movimiento genera colisión sino lo genera continúa.
                if not colision:
                    pos = RoboDK_Server.Cin_Dir(confignueva) #calcula la posición cartesiana de la nueva configuración.
                    hijo = Nodo(pos, confignueva, self) #crea un nuevo nodo hijo.   
                    hijo.g = self.g + 1 #actualiza el costo acumulado
                    hijo.heuristica(metaconf) #calcula la heurística del hijo.
                    self.hijos.append(hijo) #agrega el hijo a la lista de hijos.
                    visitados.add(configtupla) #marca la configuración como visitada.
    
    #compara los nodos en la base al costo total.
    def __lt__(self, other):
        return self.f < other.f

#implemento el algoritmo a estrella.
def a_estrella(config, metaconf):
    posini = RoboDK_Server.Cin_Dir(config) #se obtiene la posición cartesiana inicial a partir de la configuración inicial.
    nodoini = Nodo(posini, config) #crea el nodo inicial.
    nodoini.heuristica(metaconf) #dalcula la heurística del nodo inicial.
    listabierta = [] #lista de nodos por explorar.
    listacerrada = set()#nodos ya explorados.
    visitados = set() #configuraciones ya visitadas.
    heapq.heappush(listabierta, nodoini) #agrega el nodo inicial a la lista abierta (cola de prioridad).
    while listabierta: #mientras hay nodos por explorar.
        nodoact = heapq.heappop(listabierta) #obtiene el nodo de la lista abierta.
        if tuple(nodoact.configuracion) == tuple(metaconf): #si la configuración es igual al objetivo.
            path = [] #inicializa el camino.
            while nodoact: #reconstruye el camino desde el nodo actual al nodo inicial.
                path.append(nodoact.configuracion)
                nodoact = nodoact.papa
            return path[::-1] #devuelve el camino desde el inicio al objetivo.
        listacerrada.add(tuple(nodoact.configuracion)) #marca el nodo actual como explorado.
        nodoact.generarhijos(metaconf, visitados=visitados) #genera los hijos del nodo actual.
        for hijo in nodoact.hijos:
            if tuple(hijo.configuracion) in listacerrada: #omite nodos ya explorados.
                continue
            heapq.heappush(listabierta, hijo) #agrega el hijo a la lista abierta.
    return None

if __name__ == "__main__":
    tiempoini = time.time() #registra el tiempo.
    print("Verificando configuración inicial...")
    if VerPun(Home, Home): #verifica si la configuración inicial está en colisión consigo misma.
        print("Posición inicial en colisión")
        exit()
    print("Verificando configuración objetivo...")
    if VerPun(goal, goal): #verifica si la configuración objetivo está en colisión consigo misma.
        print("Posición objetivo en colisión")
        exit()
    print("Iniciando búsqueda con A*")
    trayectoria = a_estrella(Home, goal) #calcula la trayectoria desde el inicio hasta la meta.
    if trayectoria:
        print("Camino encontrado. Verificando trayectoria completa...")
        if VerEjc(trayectoria): #verifica sila trayectproa encontrada es ejecutable.
            print("Trayectoria ejecutada exitosamente")
        else:
            print("La trayectoria encontrada no es válida")
    else:
        print("No se encontró un camino válido")
    tiempofin = time.time() #registra el tiempo de finalización.
    print(f"Tiempo total de ejecución: {tiempofin - tiempoini:.2f} segundos") #calcula y muestra el tiempo total de ejecución del programa.