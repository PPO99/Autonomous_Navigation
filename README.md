# Autonomous Navigation

Este repositorio contiene un nodo ROS2 desarrollado para la navegación autónoma de robots móviles dentro de un entorno conocido y mapeado previamente. El nodo implementa un pipeline que prueba diferentes técnicas de navegación global y local, extrayendo datos de navegación (éxito, tiempo empleado (s), distancia teórica proporcionada por el navegador global (m), distancia real recorrida por el robot (m)) para cada técnica.

->	Bucle 1: Para cada técnica de navegación global:
	->	Bucle 2: Para cada técnica de navegación local:
		->	Bucle 3: - Lanzamiento simulaciones
			         	 - Guardar datos de navegación
-> 	Fin de los bucles: Mostrar medias de los resultados.


El pipeline del nodo consta de varios pasos:

1. Configuraciones Iniciales: Se establecen los valores de las distintas técnicas de navegación global y local a probar, así como la cantidad de poses objetivo para cada configuración.

2. Obtención de los Límites del Mapa: Se obtienen los límites del mapa a partir de los datos publicados en el topic /map.
Publicación de la Pose Inicial: Se publica la pose inicial del robot en el topic /amcl_pose.

3. Selección de la Técnica de Navegación Global: Se selecciona la técnica de navegación global apropiada para cada iteración del bucle 1 (En este caso están comprobandose A* y Dijkstra).

4. Selección de la Técnica de Navegación Reactiva: Se selecciona la técnica de navegación local o reactiva apropiada para cada iteración del bucle 2 (diferentes velocidades lineales (m/s) y angulares (rad/s).

5. Teletransportación a una Nueva Pose Inicial: Se teletransporta al robot a una nueva pose inicial para iniciar el bucle 3.

6. Limpiar Mapa de Costos Global: Se limpia el mapa de costos global y se genera nuevamente desde la nueva pose inicial.

7. Lanzamiento de Acción de Pose Objetivo: Se envía una acción del tipo NavigateToPose al servidor de acción navigate_to_pose y se espera a recibir un camino a seguir.

8. Acción Completada: Se maneja la acción hasta su completación, guardando los datos de navegación en el registro para la tupla de técnicas ejecutadas.

9. Última Pose Objetivo: Se verifica si es la última pose objetivo para el conjunto de técnicas global-reactivas.

10. Última Técnica de Navegación Local: Se verifica si es la última técnica de navegación reactiva para la técnica global 
propuesta.

11. Última Técnica de Navegación Global: Se verifica si es la última técnica de navegación global.

12. Final: Se finaliza la ejecución del código, interpretando los resultados y seleccionando la mejor tupla de técnicas utilizadas. Se muestra una tabla con los valores medios de los datos de navegación obtenidos y se crea un archivo csv para poder leer los datos de cada navegación individualmente.

Este repositorio proporciona una base sólida para la experimentación y evaluación de diferentes técnicas de navegación para robots móviles en entornos conocidos.






