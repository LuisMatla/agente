# 🤖 Planificación de Trayectorias con A* para Robot UR3e

Sistema de planificación de trayectorias para robot UR3e utilizando el algoritmo A* (A Estrella) con detección de colisiones en RoboDK.

## 📋 Descripción

Este proyecto implementa un sistema completo de planificación de trayectorias para un robot UR3e utilizando el algoritmo de búsqueda A*. El sistema calcula trayectorias libres de colisiones desde una configuración inicial hasta una configuración objetivo, considerando tanto el espacio cartesiano como el espacio articular del robot.

### Características Principales

- ✅ Planificación de trayectorias usando algoritmo A*
- ✅ Detección de colisiones en tiempo real con RoboDK
- ✅ Cinemática directa para cálculo de posiciones cartesianas
- ✅ Heurística híbrida (cartesiana + articular)
- ✅ Validación completa de trayectorias antes de ejecución
- ✅ Soporte para múltiples escenarios de prueba
- ✅ Integración con RoboDK para simulación y control

## 🔧 Tecnologías Utilizadas

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![RoboDK](https://img.shields.io/badge/RoboDK-FF6B35?style=for-the-badge&logo=robot&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white)
![Pyro4](https://img.shields.io/badge/Pyro4-3776AB?style=for-the-badge&logo=python&logoColor=white)

---

## 📦 Requisitos

### Software
- 🐍 Python 3.x
- 📊 RoboDK (versión compatible)
- 📈 numpy
- 🔧 Pyro4
- ⏱️ time (biblioteca estándar)
- 📚 heapq (biblioteca estándar)

### Hardware/Simulación
- Robot UR3e configurado en RoboDK
- Gripper configurado y acoplado al robot
- Escenario con obstáculos (opcional)

## 🚀 Instalación

1. Clona el repositorio:
```bash
git clone https://github.com/LuisMatla/agente.git
cd agente
```

2. Instala las dependencias:
```bash
pip install numpy Pyro4
```

3. Instala RoboDK:
   - Descarga RoboDK desde [robodk.com](https://www.robodk.com)
   - Configura el robot UR3e en el entorno
   - Asegúrate de que el gripper esté correctamente acoplado

4. Configura el entorno:
   - Abre RoboDK
   - Carga el archivo de configuración del robot UR3e
   - Verifica que los nombres de los items coincidan:
     - `UR3e` (robot)
     - `Gripper` (gripper)
     - `UR3e_Inicio` (robot de referencia)

## 💻 Uso

### Configuración de Escenarios

El código incluye 4 escenarios predefinidos. Para cambiar de escenario, comenta/descomenta las líneas correspondientes:

```python
# Escenario 1
Home = [5,-112,-94,-64,90,0]
goal = [125,-96,-112,-60,90,0]

# Escenario 2
Home = [0,-110,-90,-70,90,0] 
goal = [180,-116,-70,-36,90,0]

# Escenario 3
Home = [0,-90,-90,-90,90,0]
goal = [-156,-90,-104,-74,90,0]

# Escenario 4
Home = [0,-140,-90,8,90,0]
goal = [-200,-98,-112,-60,90,0]
```

### Ejecución

1. Abre RoboDK y carga tu escenario
2. Ejecuta el script:
```bash
python practica.py
```

El programa realizará:
1. ✅ Verificación de configuración inicial
2. ✅ Verificación de configuración objetivo
3. 🔍 Búsqueda de trayectoria con A*
4. ✅ Validación de la trayectoria completa
5. 🤖 Ejecución de la trayectoria (si es válida)

## 🏗️ Estructura del Código

### Clases Principales

#### `RoboDK_Server`
Clase estática que contiene métodos para interactuar con RoboDK:

- **`Cin_Dir(conf)`**: Calcula la cinemática directa
  - Recibe: Configuración articular `[q1, q2, q3, q4, q5, q6]`
  - Retorna: Posición cartesiana `(x, y, z)` redondeada a 3 decimales
  - Utiliza `robot.SolveFK()` para obtener la pose del robot

- **`detectacolisiones(p1, p2, npasos=5)`**: Detecta colisiones en un trayecto
  - Divide el trayecto en `npasos` puntos intermedios
  - Verifica colisiones entre puntos consecutivos
  - Retorna: `(colision: bool, indice: int, punto: list)`

#### `Nodo`
Representa un nodo en el grafo de búsqueda:

**Atributos:**
- `pos`: Posición cartesiana `(x, y, z)`
- `configuracion`: Configuración articular `[q1...q6]`
- `papa`: Referencia al nodo padre
- `hijos`: Lista de nodos hijos generados
- `g`: Coste acumulado desde el inicio
- `h`: Valor heurístico
- `f`: Coste total `f = g + h`

**Métodos:**
- **`heuristica(metaconf)`**: Calcula heurística híbrida
  - Distancia cartesiana (peso 0.7)
  - Distancia articular (peso 0.3)
  - `h = 0.3 * dist_articular + 0.7 * dist_cartesiana`

- **`generarhijos(metaconf, visitados, basedelta=5)`**: Genera nodos hijos
  - Ajusta `delta` según distancia al objetivo
  - Para cada articulación, genera movimientos `±delta`
  - Verifica límites articulares (-360° a 360°)
  - Detecta colisiones antes de crear nodos
  - Calcula posición cartesiana del hijo

- **`__lt__(other)`**: Comparación para cola de prioridad
  - Ordena por coste total `f`

### Funciones Principales

#### `collition(j1, j2)`
Verifica colisión entre dos configuraciones articulares:
- Utiliza `robot.MoveJ_Test()` de RoboDK
- Retorna `True` si hay colisión, `False` en caso contrario

#### `VerEjc(tra)`
Valida y ejecuta una trayectoria completa:
- Verifica cada segmento de la trayectoria
- Ejecuta la trayectoria si es válida
- Muestra mensajes de estado con colores

#### `VerPun(p1, p2)`
Valida un segmento entre dos puntos:
- Verifica colisión entre `p1` y `p2`
- Retorna `True` si hay colisión

#### `a_estrella(config, metaconf)`
Implementa el algoritmo A*:

**Algoritmo:**
1. Crea nodo inicial con configuración de inicio
2. Inicializa lista abierta (cola de prioridad) y lista cerrada
3. Mientras haya nodos en la lista abierta:
   - Extrae el nodo con menor `f`
   - Si es el objetivo, reconstruye y retorna el camino
   - Marca como explorado
   - Genera hijos del nodo actual
   - Agrega hijos no explorados a la lista abierta
4. Retorna `None` si no encuentra solución

**Estructuras de datos:**
- `listabierta`: Heap (cola de prioridad) ordenada por `f`
- `listacerrada`: Set de configuraciones exploradas
- `visitados`: Set de configuraciones visitadas

## 🔬 Algoritmo A* - Detalles Técnicos

### Heurística Híbrida

El algoritmo utiliza una heurística que combina dos métricas:

1. **Distancia Cartesiana** (peso 0.7):
   ```python
   distcart = sum(abs(pos[i] - posmeta[i]) for i in range(3))
   ```
   - Mide la distancia euclidiana en el espacio cartesiano
   - Más peso porque el objetivo final es una posición espacial

2. **Distancia Articular** (peso 0.3):
   ```python
   distart = sum(abs(config[i] - metaconf[i]) for i in range(6))
   ```
   - Mide la diferencia en el espacio articular
   - Menos peso pero importante para guiar la búsqueda

**Fórmula final:**
```python
h = 0.3 * dist_articular + 0.7 * dist_cartesiana
f = g + h
```

### Generación de Hijos

El algoritmo genera nodos hijos de manera inteligente:

1. **Cálculo de Delta Adaptativo**:
   ```python
   delta = max(1, basedelta if distanciaobjetivo > 20 else 2)
   ```
   - Si está lejos del objetivo: `delta = 5` (pasos grandes)
   - Si está cerca: `delta = 2` (pasos pequeños para precisión)

2. **Movimientos por Articulación**:
   - Para cada articulación `i`:
     - Omite si la diferencia al objetivo es menor que `delta`
     - Genera dos movimientos: `+delta` y `-delta`
     - Verifica límites articulares (-360° a 360°)

3. **Detección de Colisiones**:
   - Verifica colisión antes de crear el nodo hijo
   - Solo crea nodos libres de colisiones

### Optimizaciones

- **Evita nodos duplicados**: Usa `visitados` set para evitar explorar la misma configuración
- **Cola de prioridad**: Usa `heapq` para eficiencia O(log n) en inserción/extracción
- **Detección temprana**: Verifica colisiones antes de agregar a la lista abierta

## 📊 Flujo de Ejecución

```
1. Inicialización
   ├── Conexión con RoboDK
   ├── Configuración de robot y gripper
   └── Establecimiento de velocidades

2. Validación Inicial
   ├── Verificar configuración Home (sin colisiones)
   └── Verificar configuración goal (sin colisiones)

3. Búsqueda A*
   ├── Crear nodo inicial
   ├── Inicializar estructuras de datos
   └── Bucle principal:
       ├── Extraer nodo con menor f
       ├── Verificar si es objetivo
       ├── Generar hijos
       └── Agregar a lista abierta

4. Validación de Trayectoria
   ├── Verificar cada segmento
   └── Detectar colisiones intermedias

5. Ejecución
   ├── Mover robot a Home
   └── Ejecutar trayectoria completa
```

## 🎯 Escenarios de Prueba

### Escenario 1
- **Home**: `[5,-112,-94,-64,90,0]`
- **Goal**: `[125,-96,-112,-60,90,0]`
- Movimiento moderado en múltiples articulaciones

### Escenario 2
- **Home**: `[0,-110,-90,-70,90,0]`
- **Goal**: `[180,-116,-70,-36,90,0]`
- Movimiento grande en articulación 1

### Escenario 3
- **Home**: `[0,-90,-90,-90,90,0]`
- **Goal**: `[-156,-90,-104,-74,90,0]`
- Movimiento negativo en articulación 1

### Escenario 4
- **Home**: `[0,-140,-90,8,90,0]`
- **Goal**: `[-200,-98,-112,-60,90,0]`
- Movimiento extremo en múltiples articulaciones

## ⚙️ Configuración de Velocidades

```python
gripper.setSpeedJoints(10)      # Velocidad del gripper
gripper.setAccelerationJoints(10)  # Aceleración del gripper
robot.setSpeedJoints(10)        # Velocidad del robot
robot.setAcceleration(10)       # Aceleración del robot
```

Ajusta estos valores según tus necesidades de seguridad y velocidad.

## 🔍 Detección de Colisiones

El sistema utiliza dos métodos de detección:

1. **`collition(j1, j2)`**: Verificación directa entre dos configuraciones
   - Usa `robot.MoveJ_Test()` de RoboDK
   - Retorna inmediatamente si hay colisión

2. **`detectacolisiones(p1, p2, npasos=5)`**: Verificación con puntos intermedios
   - Divide el trayecto en `npasos + 1` puntos
   - Interpola linealmente entre configuraciones
   - Verifica cada segmento intermedio
   - Más preciso pero más lento

## 📈 Métricas y Rendimiento

El programa muestra:
- ⏱️ Tiempo total de ejecución
- ✅ Estado de cada verificación
- 🔍 Progreso de la búsqueda
- 📊 Validación de trayectoria

## ⚠️ Consideraciones Importantes

1. **Límites Articulares**: El robot debe estar dentro del rango -360° a 360°
2. **Colisiones**: El algoritmo no puede encontrar solución si el objetivo está en colisión
3. **Tiempo de Ejecución**: Puede variar según la complejidad del escenario
4. **RoboDK**: Debe estar ejecutándose y con el escenario correctamente configurado

## 🐛 Solución de Problemas

### Error: "Posición inicial en colisión"
- Verifica que la configuración `Home` sea válida
- Asegúrate de que no haya obstáculos en esa posición

### Error: "Posición objetivo en colisión"
- Verifica que la configuración `goal` sea alcanzable
- Revisa los límites articulares

### "No se encontró un camino válido"
- El algoritmo A* no pudo encontrar una trayectoria
- Intenta ajustar `basedelta` en `generarhijos()`
- Verifica que exista un camino libre de colisiones

### RoboDK no responde
- Verifica la conexión con RoboDK
- Asegúrate de que los nombres de los items sean correctos
- Revisa que el robot esté correctamente cargado

## 👨‍💻 Autor

**Luis Fernando Contreras Matla**

## 📚 Información Académica

Esta práctica fue desarrollada como parte de la Experiencia Educativa:

**Materia:** Introducción a la Inteligencia Artificial

**Universidad:** Universidad Veracruzana

**Facultad:** Ingeniería Eléctrica y Electrónica

**Docente:** Luis Felipe Marín Urias

## 📄 Licencia

Este proyecto es de código abierto y está disponible para uso educativo.

