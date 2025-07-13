# Documentación CP4: reconocimiento de colores y números
Podemos obtener las dependencias necesarias del script instalando pipreqs en el entorno virtual de Python (o en el propio sistema):
```bash
pip install pipreqs
```

Vamos al directorio donde está SOLO el script del que queremos extraer las dependencias:
```bash
pipreqs . --force
```

Y obtenemos el ``requirements.txt``:
```
numpy==2.2.3
opencv_python==4.11.0.86
tensorflow==2.19.0
```

CUIDADO! La Raspberry Pi tiene una arquitectura ARM64, y no todas las versiones de las librerías son compatibles. Para ejecutarlo en la Raspberry Pi del rover se han utilizado el ``requirements_aarch64.txt`` (librería playsound para reproducir los audios, opcional):
```
numpy==1.21.5
opencv_python==4.11.0.86
playsound==1.3.0
tensorflow==2.19.0
```

Para instalar las dependencias, copiamos el requirements.txt al directorio en donde vamos a ejecutar la instalación, e instalamos:
```bash
pip install -r requirements.txt
```

Recuerda que si se utiliza ``pip freeze`` se obtienen TODAS las dependencias del entorno. Como mi entorno tiene muchas más dependencias, solo me interesan las de mi script, por lo que he utilizado ``pipreqs``.

<div style="page-break-after: always;"></div>

# Reconocimiento de una cartulina de color rojo o azul
**Objetivo**: detectar el color de la cartulina. Hay tres posibilidades: rojo, azul, color distractorio

**Condiciones**: los colores pueden no ser puros, y las dimensiones de la cartulina son las especificadas por el concurso, así como la distancia a la que debe leerse.

## Análisis
Se ha decidido por utlizar el modelo de color HSV ([https://es.wikipedia.org/wiki/Modelo_de_color_HSV](https://es.wikipedia.org/wiki/Modelo_de_color_HSV)) - Matiz, Saturación, Valor-, porque permite (Bing Copilot):

+ separar la información de color: en el modelo HSV, el matiz (Hue) representa el color en sí (rojo, azul, verde, etc.), mientras que la saturación (Saturation) mide la intensidad del color y el valor (Value) indica su brillo. Esto permite centrarte únicamente en el "matiz" para detectar colores, sin que los cambios en iluminación (brillo) afecten demasiado.

+ estabilidad frente a cambios de iluminación: a diferencia del modelo RGB, donde los colores son más sensibles a variaciones de luz, HSV facilita la detección consistente de colores porque los componentes de brillo (Value) y saturación se separan del componente de color (Hue).

+ facilidad de umbralización: los rangos de colores (como los límites que definiste para azul y rojo) son más intuitivos y precisos en el espacio HSV, ya que el matiz se mide en grados (de 0 a 180 en OpenCV) y cada color tiene un intervalo bien definido.

Además, hay que tener en cuenta que habría que eliminar las detecciones de color del fondo o aquellas que no sean de la propia cartulina. Por ejemplo, podemos poner una cartulina, pero en  el fondo haber puntitos de color rojo o azul, y este mediante este modelo detectaríamos todo, fondo incluído.

Para ello, se ha introducido al código que únicamente marcaremos como detectado si cuando se detecta el color cumple con los requisitos de la forma de la cartulina y distancia. Aunque depende del fondo puede seguir detectando ruido, reduce significativamente este hecho y facilita lograr el objetivo.

## Solución
Script ``color_a_distancia.py``:

```python
# Este codigo está configurado para poner un folio din a4 a 78.75 cm 
# de la cámara, siendo el equivalente de un cuadrado de 40cm de lado a 
# 1'5 metros de distancia (que son las especificaciones de la prueba)

# Junto con los scripts hay un pdf con la conversación con Microsoft 
# Copilot para más detalle. 

# Si se quiere probar con el tamaño del cuadrado de la prueba modificar 
# if 150 <= w <= 300 and 200 <= h <= 350:
# por
# if 300 <= w <= 400 and 300 <= h <= 400:

# Si se quiere otro tamaño (por ejemplo, cartulina), también hay que adaptarlo.
# Preguntar a Copilot, está el pdf de muestra de cómo le he preguntado.

# Si se muestran dos colores a la vez, pintara el cuadro pero por pantalla
# solo mostrará "Rojo", porque en la programación está después de la lógica
# del color azul, por lo que es lo último que escribe.

# El trozo de código que está comentado estaba pensado para marcar en gris
# los colores distractorios, pero no tiene el comportamiento esperado, por 
# lo que se ha comentado.

# En este script:
#   - el azul está configurado para azul clarito y oscuro
#   - el rojo detecta también un folio naranja

# Tutorial base: https://www.youtube.com/watch?v=Cb1zw7lmnmA

# Buena suerte!

import cv2
import numpy as np

cap = cv2.VideoCapture(2)

cap.set(3, 640)
cap.set(4, 480)

while True:
    success, frame = cap.read()

    if success:
        frame = cv2.flip(frame, 1)

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define los límites de los colores azul y rojo en el espacio HSV
        blue_lower_bound = np.array([80, 50, 50])
        blue_upper_bound = np.array([130, 255, 255])
        red_lower_bound1 = np.array([0, 100, 100])
        red_upper_bound1 = np.array([10, 255, 255])
        red_lower_bound2 = np.array([160, 100, 100])
        red_upper_bound2 = np.array([180, 255, 255])

        # Crea máscaras para los colores azul y rojo
        blue_mask = cv2.inRange(frame_hsv, blue_lower_bound, blue_upper_bound)
        red_mask1 = cv2.inRange(frame_hsv, red_lower_bound1, red_upper_bound1)
        red_mask2 = cv2.inRange(frame_hsv, red_lower_bound2, red_upper_bound2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Encuentra los contornos para los colores azul y rojo
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detecta y dibuja los rectángulos
        detected = False
        color_detected = "Color distractorio"

        for contour in blue_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 150 <= w <= 300 and 200 <= h <= 350:  # Tamaño ajustado para folios a 1.5 metros
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                detected = True
                color_detected = "Azul"

        for contour in red_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 150 <= w <= 300 and 200 <= h <= 350:  # Tamaño ajustado para folios a 1.5 metros
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                detected = True
                color_detected = "Rojo"

        # Si no es ni azul ni roja, dibuja el marco en gris
        """if not detected:
            gray_mask = cv2.inRange(frame_hsv, np.array([0, 0, 0]), np.array([179, 255, 100]))  # Rango para otros colores
            gray_contours, _ = cv2.findContours(gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in gray_contours:
                x, y, w, h = cv2.boundingRect(contour)
                if 150 <= w <= 300 and 200 <= h <= 350:  # Tamaño ajustado para folios a 1.5 metros
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 128, 128), 3)
        """
        # Añade el texto en el marco
        cv2.putText(frame, color_detected, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

<div style="page-break-after: always;"></div>

# Reconocimiento de dígitos mediante un modelo CNN (Convolutional Neural Network)
**Objetivo**: reconocer los dígitos del 1 al 9 en tiempo real con una cámara. También debe identificar cuando no hay ningún dígito.

**Condiciones**: el número estará sobre una cartulina o cuadrado de color rojo o azul, y las dimensiones del dígito serán la mitad que el fondo sobre el que está.

## Análisis
La cámara verá un escenario en el que aparecerá una cartulina con un dígito (o sin dígito). Detrás de la cartulina podrá habrá un fondo, que compone el escenario. Es decir, el protagonista principal de la imagen será la cartulina con el dígito, pero a su alrededor podrá haber cualquier cosa. 


Este fondo supone un problema para el reconocimiento tanto del color de la cartulina como del dígito. 

Para solucionarlo, se ha decidido programar que únicamente detecte los colores rojo y azul en formas cuadradas o rectangulares a una distancia determinada. Estas especificaicones son las del concurso. De esta manera, si hubiera un objeto de color rojo o azul que no cumpla esos requisitos, la cámara lo detectará pero no lo marcará, eliminando de esta manera el ruido para la tarea de detección del dígito. (Más detalles en la documentación de percepción de colores).

Por tanto, la imagen sobre la que se tiene que identificar el dígito será aproximadamente la cartulina o el cuadrado.

Para discernir la cartulina y el dígito, se ha aplicado el método o algoritmo de Canny (Canny Edge Detections). Obtenemos la silueta del número, pero cuidado, también la silueta de la cartulina y siluetas que pudieran quedar del fondo (al limitar la detección a la cartulina, se reduce significativamente el fondo que aparece en la imagen, pero puede seguir apareciendo un trozo).
La silueta del fondo hace que la imagen sea menos pura, pero es muy importante porque simula en mejor grado una imagen real, y permite entrenar al modelo para reconocer los números incluso con esas interferencias.

Al modelo se le pasan las imágenes de bordes Canny procesadas tanto para entrenar como para predecir.

<div style="display: flex; justify-content: space-between;">
  <img src="images_documentacion/real_3.png" alt="Imagen real capturada" style="width: 48%;"/>
  <img src="images_documentacion/canny_3.png" alt="Bordes obtenidos con el algoritmo Canny y tras preprocesado" style="width: 48%;"/>
</div>
<div style="page-break-after: always;"></div>
<div style="display: flex; justify-content: space-between;">
  <img src="images_documentacion/real_7.png" alt="Imagen real capturada" style="width: 48%;"/>
  <img src="images_documentacion/canny_7.png" alt="Bordes obtenidos con el algoritmo Canny y tras preprocesado" style="width: 48%;"/>
</div>
<br>
<center><p>Imágenes reales capturadas y sus procesados Canny.</p></center>

## Generación del dataset
### Imágenes reales
Se ha amplaido el script de detección y marcado de las cartulinas de color para que además guarde en imágenes las detecciones. De esta forma, cuando mostramos la cartulina con el dígito a la cámara, el script está continuamente detectando el color, y a cada ciclo guarda una imagen. Se detendrá cuando no detecte ninguna cartulina o se detenga el programa.

El script utlizado para este fin es ``colores_numeros_guardarRegiones.py``. Dado que el color de la cartulina no afecta para el procesado de bordes Canny, se ha simplificado y facilitado el proceso obteniendo todas las imágenes sobre la cartulina de color rojo.

```python
import cv2
import numpy as np
import os

cap = cv2.VideoCapture(2)

cap.set(3, 640)
cap.set(4, 480)

numero = "8"

if not os.path.exists(f'data/{numero}'):
    os.makedirs(f'data/{numero}')

def save_region(image, x, y, w, h, color, count):
    region = image[y:y+h, x:x+w]
    filename = f"data/{numero}/{color}_{count}.png"
    cv2.imwrite(filename, region)

blue_count = 0
red_count = 0

while True:
    success, frame = cap.read()

    if success:
        frame = cv2.flip(frame, 1)
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue_lower_bound = np.array([80, 50, 50])
        blue_upper_bound = np.array([130, 255, 255])
        red_lower_bound1 = np.array([0, 100, 100])
        red_upper_bound1 = np.array([10, 255, 255])
        red_lower_bound2 = np.array([160, 100, 100])
        red_upper_bound2 = np.array([180, 255, 255])

        blue_mask = cv2.inRange(frame_hsv, blue_lower_bound, blue_upper_bound)
        red_mask1 = cv2.inRange(frame_hsv, red_lower_bound1, red_upper_bound1)
        red_mask2 = cv2.inRange(frame_hsv, red_lower_bound2, red_upper_bound2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        color_detected = "Color distractorio"

        """for contour in blue_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 50 <= w <= 500 and 100 <= h <= 600:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                detected = True
                color_detected = "Azul"
                save_region(frame, x, y, w, h, 'azul', blue_count)
                blue_count += 1"""

        for contour in red_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 50 <= w <= 500 and 100 <= h <= 600:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                detected = True
                color_detected = "Rojo"
                save_region(frame, x, y, w, h, 'rojo', red_count)
                red_count += 1

        cv2.putText(frame, color_detected, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

Las imágenes se han capturado desde diferentes distancias, ángulos y orientaciones, y en total se han obtenido unas 1000 imágenes por dígito (incluyedo la clase 'sin dígito' o 'NaN' (Not a Number)).

<div style="display: flex; justify-content: space-between;">
  <img src="images_documentacion/ejemplo_captura.png" alt="Imagen real capturada" style="width: 50%;"/>
  <img src="images_documentacion/ejemplo_captura_2.png" alt="Imagen real capturada" style="width: 50%;"/>

</div>
<br>
<div style="display: flex; justify-content: space-between;">
    <img src="images_documentacion/ejemplo_captura_3.png" alt="Imagen real capturada" style="width: 50%;"/>
    <img src="images_documentacion/ejemplo_captura_4.png" alt="Imagen real capturada" style="width: 50%;"/>
</div>
<br>
<div style="display: flex; justify-content: space-between;">
<img src="images_documentacion/ejemplo_captura_5.png" alt="Imagen real capturada" style="width: 50%;"/>
    <img src="images_documentacion/ejemplo_captura_6.png" alt="Imagen real capturada" style="width: 50%;"/>
</div>
<br>
<center><p>Imágenes reales capturadas con diferentes distancias, ángulos y orientaciones</p></center>

Después, se han filtrado para eliminar aquellas que solo aportan ruido tanto al entrenamiento como a la clasificación:
<div style="display: flex; justify-content: space-between;">
<img src="images_documentacion/ruido.png" alt="Imagen real capturada" style="width: 100%;"/>
</div>
<br>

Y finalmente se ha aplicado el procesado de bordes Canny (las imágenes originales estaban volteadas horizontalmente, por lo que en el código también se voltean). El script es ``extraer_canny_relleno.py``.

```python
import cv2
import os

# Directorios
input_dir = "data_cleaned"
output_dir = "data_canny"

# Recortar un 15% por todos los lados
def recortar_imagen(imagen, porcentaje_recorte=0.20):
    altura, ancho = imagen.shape[:2]
    recorte_vertical = int(altura * porcentaje_recorte)
    recorte_horizontal = int(ancho * porcentaje_recorte)
    return imagen[recorte_vertical:altura - recorte_vertical, recorte_horizontal:ancho - recorte_horizontal]

# Crear el directorio de salida si no existe
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Recorrer el árbol de directorios y archivos
for root, dirs, files in os.walk(input_dir):
    for file in files:
        if file.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
            # Ruta completa del archivo de entrada
            input_path = os.path.join(root, file)
            
            # Leer la imagen
            img = cv2.imread(input_path, 0)  # Leer en escala de grises

            if img is None:
                print(f"Error: No se pudo leer la imagen {input_path}")
                continue

            # Aplicar flip horizontal
            img = cv2.flip(img, 1)

            # Recortar la imagen
            img = recortar_imagen(img)
            
            # Aplicar detección de bordes con Canny
            canny = cv2.Canny(img, 20, 110)
            
            # Engrosar los contornos usando dilatación
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))  # Tamaño del kernel
            thickened_canny = cv2.dilate(canny, kernel, iterations=1)  # Aumentar iteraciones para engrosar más
            
            # Crear la estructura de carpetas en el directorio de salida
            relative_path = os.path.relpath(root, input_dir)  # Ruta relativa dentro del directorio de entrada
            output_subdir = os.path.join(output_dir, relative_path)
            if not os.path.exists(output_subdir):
                os.makedirs(output_subdir)
            
            # Ruta completa del archivo de salida
            output_path = os.path.join(output_subdir, file)
            
            # Guardar la imagen procesada
            cv2.imwrite(output_path, thickened_canny)
            print(f"Imagen procesada y guardada en: {output_path}")
```

### Imágenes sintéticas
Para completar el dataset se han generado imágenes sintéticas que simulan las reales. Para entrenar al modelo se han generado únicamente con la misma fuente (tipografía) que las especificaciones: TrebuchetMs Bold (generación de 100 imágenes por clase o dígito).

Es importante destacar que no se han generado imágenes sintéticas para la clase de 'sin número' o 'NaN'.

Se ha utilizado el script ``generarTrebuchet.py``.

```python
import cv2
import numpy as np
import os
import random
from PIL import ImageFont, ImageDraw, Image

# Ruta a la fuente TrebuchetMS Bold
fuente_path = "TrebuchetMS-Bold.ttf"  # Reemplaza esto con la ruta real a tu archivo de fuente

# Crear carpetas para almacenar los dígitos
def crear_carpetas(base_dir="digitos_trebuchet"):
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    for dígito in range(1, 10):
        carpeta = os.path.join(base_dir, str(dígito))
        os.makedirs(carpeta, exist_ok=True)

# Generar ruido en el fondo del cuadrado
def añadir_ruido_al_fondo(imagen, color_cuadrado):
    mask = (imagen[:, :, 0] == color_cuadrado[0]) & \
           (imagen[:, :, 1] == color_cuadrado[1]) & \
           (imagen[:, :, 2] == color_cuadrado[2])
    ruido = np.random.randint(0, 50, imagen.shape, dtype=np.uint8)
    imagen_con_ruido = imagen.copy()
    imagen_con_ruido[~mask] = cv2.add(imagen[~mask], ruido[~mask])
    return imagen_con_ruido

# Dibujar un cuadrado de color con un número blanco usando TrebuchetMS Bold
def dibujar_cuadrado_con_numero(color_cuadrado, dígito, fuente_path):
    imagen = np.zeros((250, 250, 3), dtype=np.uint8)
    cv2.rectangle(imagen, (25, 25), (210, 210), color_cuadrado, -1)
    texto = str(dígito)
    font = ImageFont.truetype(fuente_path, 100)
    img_pil = Image.fromarray(imagen)
    draw = ImageDraw.Draw(img_pil)
    bbox = font.getbbox(texto)
    texto_ancho, texto_alto = bbox[2] - bbox[0], bbox[3] - bbox[1]
    texto_x = 25 + (210 - 25 - texto_ancho) // 2
    texto_y = 25 + (210 - 25 - texto_alto) // 2
    
    if random.random() > 0.7:
        offset_x = random.randint(-10, 10)
        offset_y = random.randint(-10, 10)
        texto_x = max(0, texto_x + offset_x)
        texto_y = max(texto_alto, texto_y + offset_y)
    
    draw.text((texto_x, texto_y), texto, font=font, fill=(255, 255, 255, 0))
    imagen = np.array(img_pil)
    return imagen

def escalar_imagen(imagen, tamaño_min=63, tamaño_max=250):
    nuevo_tamaño = random.randint(tamaño_min, tamaño_max)
    imagen_escalada = cv2.resize(imagen, (nuevo_tamaño, nuevo_tamaño), interpolation=cv2.INTER_LINEAR)
    lienzo = np.zeros((250, 250, 3), dtype=np.uint8)
    y_offset = (250 - nuevo_tamaño) // 2
    x_offset = (250 - nuevo_tamaño) // 2
    lienzo[y_offset:y_offset + nuevo_tamaño, x_offset:x_offset + nuevo_tamaño] = imagen_escalada
    return lienzo

def añadir_bordes_canny_fuera(imagen, color_cuadrado):
    mask = (imagen[:, :, 0] == color_cuadrado[0]) & \
           (imagen[:, :, 1] == color_cuadrado[1]) & \
           (imagen[:, :, 2] == color_cuadrado[2])
    imagen_grises = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    bordes = cv2.Canny(imagen_grises, 50, 150)
    imagen_resultante = imagen.copy()
    imagen_resultante[~mask] = cv2.add(imagen[~mask], cv2.merge([bordes, bordes, bordes])[~mask])
    return imagen_resultante

def aplicar_perspectiva(imagen):
    filas, columnas = imagen.shape[:2]
    puntos_originales = np.float32([
        [0, 0], [columnas, 0], [0, filas], [columnas, filas]
    ])
    margen = 40
    puntos_transformados = np.float32([
        [random.randint(0, margen), random.randint(0, margen)],
        [columnas - random.randint(0, margen), random.randint(0, margen)],
        [random.randint(0, margen), filas - random.randint(0, margen)],
        [columnas - random.randint(0, margen), filas - random.randint(0, margen)]
    ])
    matriz_perspectiva = cv2.getPerspectiveTransform(puntos_originales, puntos_transformados)
    imagen_perspectiva = cv2.warpPerspective(imagen, matriz_perspectiva, (columnas, filas))
    return imagen_perspectiva

def recortar_imagen(imagen, porcentaje_recorte=0.20):
    altura, ancho = imagen.shape[:2]
    recorte_vertical = int(altura * porcentaje_recorte)
    recorte_horizontal = int(ancho * porcentaje_recorte)
    return imagen[recorte_vertical:altura - recorte_vertical, recorte_horizontal:ancho - recorte_horizontal]

def generar_imagenes(base_dir="digitos_trebuchet", num_imagenes_por_fuente=50):
    colores = [(255, 0, 0), (0, 0, 255)]
    for dígito in range(1, 10):
        for i in range(num_imagenes_por_fuente):
            color_cuadrado = random.choice(colores)
            imagen = dibujar_cuadrado_con_numero(color_cuadrado, dígito, fuente_path)
            imagen_con_bordes = añadir_bordes_canny_fuera(imagen, color_cuadrado)
            imagen_grises = cv2.cvtColor(imagen_con_bordes, cv2.COLOR_BGR2GRAY)
            imagen_canny = cv2.Canny(imagen_grises, 100, 200)
            imagen_recortada = recortar_imagen(imagen_canny)
            guardar_imagen(imagen_recortada, base_dir, dígito, i, perspectiva=False)
            imagen_con_perspectiva = aplicar_perspectiva(imagen_con_bordes)
            imagen_grises_perspectiva = cv2.cvtColor(imagen_con_perspectiva, cv2.COLOR_BGR2GRAY)
            imagen_canny_perspectiva = cv2.Canny(imagen_grises_perspectiva, 100, 200)
            imagen_recortada_perspectiva = recortar_imagen(imagen_canny_perspectiva)
            guardar_imagen(imagen_recortada_perspectiva, base_dir, dígito, i, perspectiva=True)

def guardar_imagen(imagen, base_dir, dígito, indice, perspectiva):
    sufijo = "_perspectiva" if perspectiva else "_sin_perspectiva"
    carpeta = os.path.join(base_dir, str(dígito))
    os.makedirs(carpeta, exist_ok=True)
    nombre_archivo = f"{dígito}_{indice}{sufijo}.png"
    cv2.imwrite(os.path.join(carpeta, nombre_archivo), imagen)

crear_carpetas()
generar_imagenes()
print("Conjunto de datos generado con éxito.")
```

Incluyo también en esta documentación el script para generar las imágenes con todas las fuentes disponibles en OpenCV (generación de 800 imágenes por clase o dígito), por si fuera de interés. Es el script ``generarCanny.py``:

```python
import cv2
import numpy as np
import os
import random

# Fuentes disponibles en OpenCV
fuentes = [
    cv2.FONT_HERSHEY_SIMPLEX,
    cv2.FONT_HERSHEY_PLAIN,
    cv2.FONT_HERSHEY_DUPLEX,
    cv2.FONT_HERSHEY_COMPLEX,
    cv2.FONT_HERSHEY_TRIPLEX,
    cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
    cv2.FONT_HERSHEY_SCRIPT_COMPLEX,
    cv2.FONT_ITALIC
]

# Crear carpetas para almacenar los dígitos
def crear_carpetas(base_dir="digitos"):
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    for dígito in range(1, 10):
        carpeta = os.path.join(base_dir, str(dígito))
        os.makedirs(carpeta, exist_ok=True)

# Generar ruido en el fondo del cuadrado
def añadir_ruido_al_fondo(imagen, color_cuadrado):
    # Crear una máscara para el cuadrado (color_cuadrado)
    mask = (imagen[:, :, 0] == color_cuadrado[0]) & \
           (imagen[:, :, 1] == color_cuadrado[1]) & \
           (imagen[:, :, 2] == color_cuadrado[2])
    
    # Crear ruido aleatorio solo en las áreas fuera del cuadrado
    ruido = np.random.randint(0, 50, imagen.shape, dtype=np.uint8)
    imagen_con_ruido = imagen.copy()
    imagen_con_ruido[~mask] = cv2.add(imagen[~mask], ruido[~mask])
    return imagen_con_ruido

# Dibujar un cuadrado de color con un número blanco
def dibujar_cuadrado_con_numero(color_cuadrado, dígito, fuente):
    
    # Crear una imagen negra
    imagen = np.zeros((250, 250, 3), dtype=np.uint8)
    # Dibujar un cuadrado más grande y centrado
    cv2.rectangle(imagen, (25, 25), (210, 210), color_cuadrado, -1)
    
    # Calcular el tamaño del texto para centrarlo dentro del cuadrado
    texto = str(dígito)
    texto_tamaño = cv2.getTextSize(texto, fuente, 3.0, 8)[0]
    texto_ancho, texto_alto = texto_tamaño
    texto_x = 25 + (210 - 25 - texto_ancho) // 2  # Centramos horizontalmente
    texto_y = 25 + (210 - 25 + texto_alto) // 2  # Centramos verticalmente
    
    # Introducir un desplazamiento aleatorio en algunas ocasiones (opcional)
    if random.random() > 0.7:  # 30% de las veces el número estará descentrado
        offset_x = random.randint(-10, 10)
        offset_y = random.randint(-10, 10)
        texto_x = max(0, texto_x + offset_x)
        texto_y = max(texto_alto, texto_y + offset_y)  # Asegurar que no salga del borde superior
    
    # Dibujar el número en la posición calculada
    cv2.putText(imagen, texto, (texto_x, texto_y), fuente, 3.0, (255, 255, 255), 8, cv2.LINE_AA)
    return imagen

# Escalar imagen a un tamaño aleatorio
def escalar_imagen(imagen, tamaño_min=63, tamaño_max=250):  # Aumentamos tamaños para ajustarse al 25% más grande
    nuevo_tamaño = random.randint(tamaño_min, tamaño_max)
    imagen_escalada = cv2.resize(imagen, (nuevo_tamaño, nuevo_tamaño), interpolation=cv2.INTER_LINEAR)

    # Crear un lienzo negro de tamaño fijo (250x250) y centrar la imagen escalada
    lienzo = np.zeros((250, 250, 3), dtype=np.uint8)
    # Crear un fondo con ruido para rellenar el espacio
    #lienzo = np.random.randint(0, 50, (250, 250, 3), dtype=np.uint8)
    y_offset = (250 - nuevo_tamaño) // 2
    x_offset = (250 - nuevo_tamaño) // 2
    lienzo[y_offset:y_offset + nuevo_tamaño, x_offset:x_offset + nuevo_tamaño] = imagen_escalada

    return lienzo

# Añadir bordes Canny fuera del cuadrado
def añadir_bordes_canny_fuera(imagen, color_cuadrado):
    mask = (imagen[:, :, 0] == color_cuadrado[0]) & \
           (imagen[:, :, 1] == color_cuadrado[1]) & \
           (imagen[:, :, 2] == color_cuadrado[2])
    imagen_grises = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    bordes = cv2.Canny(imagen_grises, 50, 150)

    # Añadir los bordes Canny donde no está el cuadrado
    imagen_resultante = imagen.copy()
    imagen_resultante[~mask] = cv2.add(imagen[~mask], cv2.merge([bordes, bordes, bordes])[~mask])
    return imagen_resultante

# Aplicar transformaciones de perspectiva (ángulos inclinados)
def aplicar_perspectiva(imagen):
    filas, columnas = imagen.shape[:2]
    puntos_originales = np.float32([
        [0, 0], [columnas, 0], [0, filas], [columnas, filas]
    ])
    margen = 40  # Más margen para transformaciones más notorias
    puntos_transformados = np.float32([
        [random.randint(0, margen), random.randint(0, margen)],
        [columnas - random.randint(0, margen), random.randint(0, margen)],
        [random.randint(0, margen), filas - random.randint(0, margen)],
        [columnas - random.randint(0, margen), filas - random.randint(0, margen)]
    ])
    matriz_perspectiva = cv2.getPerspectiveTransform(puntos_originales, puntos_transformados)
    imagen_perspectiva = cv2.warpPerspective(imagen, matriz_perspectiva, (columnas, filas))
    return imagen_perspectiva

# Recortar un 15% por todos los lados
def recortar_imagen(imagen, porcentaje_recorte=0.20):
    altura, ancho = imagen.shape[:2]
    recorte_vertical = int(altura * porcentaje_recorte)
    recorte_horizontal = int(ancho * porcentaje_recorte)
    return imagen[recorte_vertical:altura - recorte_vertical, recorte_horizontal:ancho - recorte_horizontal]

# Crear imágenes con dígitos destacados, ruido, escalado, bordes y perspectivas
def generar_imagenes(base_dir="digitos", num_imagenes_por_fuente=50):
    colores = [(255, 0, 0), (0, 0, 255)]  # Rojo y azul
    for dígito in range(1, 10):
        for fuente in fuentes:
            for i in range(num_imagenes_por_fuente):
                # Elegir un color aleatorio para el cuadrado (rojo o azul)
                color_cuadrado = random.choice(colores)
                
                # Dibujar el cuadrado con el número destacado
                imagen = dibujar_cuadrado_con_numero(color_cuadrado, dígito, fuente)
                
                # Añadir bordes Canny fuera del cuadrado
                imagen_con_bordes = añadir_bordes_canny_fuera(imagen, color_cuadrado)
                
                # Generar imagen sin perspectiva y aplicar Canny
                imagen_grises = cv2.cvtColor(imagen_con_bordes, cv2.COLOR_BGR2GRAY)
                imagen_canny = cv2.Canny(imagen_grises, 100, 200)
                imagen_recortada = recortar_imagen(imagen_canny)  # Recortar la imagen
                guardar_imagen(imagen_recortada, base_dir, dígito, fuente, i, perspectiva=False)
                
                # Generar imagen con perspectiva, y después aplicar Canny
                imagen_con_perspectiva = aplicar_perspectiva(imagen_con_bordes)
                imagen_grises_perspectiva = cv2.cvtColor(imagen_con_perspectiva, cv2.COLOR_BGR2GRAY)
                imagen_canny_perspectiva = cv2.Canny(imagen_grises_perspectiva, 100, 200)
                imagen_recortada_perspectiva = recortar_imagen(imagen_canny_perspectiva)  # Recortar la imagen
                guardar_imagen(imagen_recortada_perspectiva, base_dir, dígito, fuente, i, perspectiva=True)

# Guardar imágenes generadas
def guardar_imagen(imagen, base_dir, dígito, fuente, indice, perspectiva):
    sufijo = "_perspectiva" if perspectiva else "_sin_perspectiva"
    carpeta = os.path.join(base_dir, str(dígito))
    os.makedirs(carpeta, exist_ok=True)
    nombre_archivo = f"{dígito}_fuente{fuentes.index(fuente)}_{indice}{sufijo}.png"
    cv2.imwrite(os.path.join(carpeta, nombre_archivo), imagen)

# Ejecutar todo el flujo
crear_carpetas()
generar_imagenes()
print("Conjunto de datos generado con éxito.")
```

### Dataset de entrenamiento final
Se han juntado las imágenes reales y las sintéticas para generar el dataset, y se ha separado en tres grupos: train, validation, test.

Destacar que el modelo ha sido entrenado en Google Colab, por lo que para subir el dataset completo se ha subido en formato .zip, y desde Colab se ha extraído y dividido en los tres sets:

#### Extraer .zip en Colab:
```python
import zipfile
import os

# Ruta a tu archivo .zip de entrenamiento
zip_path = "drive/My Drive/bilbotiks/canny/DATA/numeros.zip"
# Ruta a la carpeta donde quieres extraer el contenido
extract_to_dir = "drive/My Drive/bilbotiks/canny/DATA/sinSeparar"

# Crear la carpeta si no existe
if not os.path.exists(extract_to_dir):
    os.makedirs(extract_to_dir)

# Extraer archivos .zip en la carpeta de destino
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    zip_ref.extractall(extract_to_dir)

print(f"Contenido del archivo .zip extraído en {extract_to_dir}")
```

#### Generar train, validation y test:
```python
import os
import shutil
import numpy as np

# Directorios de entrada y salida
input_dir = "drive/My Drive/bilbotiks/canny/DATA/sinSeparar"
train_output_dir = "drive/My Drive/bilbotiks/canny/DATA/train"
validation_output_dir = "drive/My Drive/bilbotiks/canny/DATA/validation"
test_output_dir = "drive/My Drive/bilbotiks/canny/DATA/test"

for dir in [train_output_dir, validation_output_dir, test_output_dir]:
    if not os.path.exists(dir):
        os.makedirs(dir)

# Proporciones para dividir los datos
train_ratio = 0.7
validation_ratio = 0.15
test_ratio = 0.15

# Recorrer el árbol de directorios y archivos
for class_name in os.listdir(input_dir):
    class_path = os.path.join(input_dir, class_name)
    if os.path.isdir(class_path):
        # Crear subcarpetas en los directorios de salida para cada clase
        train_class_dir = os.path.join(train_output_dir, class_name)
        validation_class_dir = os.path.join(validation_output_dir, class_name)
        test_class_dir = os.path.join(test_output_dir, class_name)

        for dir in [train_class_dir, validation_class_dir, test_class_dir]:
            if not os.path.exists(dir):
                os.makedirs(dir)

        # Obtener todos los archivos de la clase actual
        files = [file for file in os.listdir(class_path) if file.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]

        # Mezclar archivos de forma aleatoria
        np.random.shuffle(files)

        # Calcular el número de archivos para cada conjunto
        total_files = len(files)
        train_count = int(total_files * train_ratio)
        validation_count = int(total_files * validation_ratio)
        test_count = total_files - train_count - validation_count

        # Dividir los archivos en train, validation y test
        train_files = files[:train_count]
        validation_files = files[train_count:train_count + validation_count]
        test_files = files[train_count + validation_count:]

        # Copiar archivos a las carpetas correspondientes
        for file in train_files:
            shutil.copy(os.path.join(class_path, file), os.path.join(train_class_dir, file))
        for file in validation_files:
            shutil.copy(os.path.join(class_path, file), os.path.join(validation_class_dir, file))
        for file in test_files:
            shutil.copy(os.path.join(class_path, file), os.path.join(test_class_dir, file))

print("Archivos divididos en train, validation y test, manteniendo las clases.")
```

El dataset completo y preparado por grupos puede encontrarse en ``dataset_numeros_bilbotiks.zip``:
* Train: Found 7193 images belonging to 10 classes.
* Validation: Found 1538 images belonging to 10 classes.
* Test: Found 1551 images belonging to 10 classes.

## Generación y entrenamiento del modelo
El modelo se ha entrenado en diferentes etapas (por falta de tiempo). Por ello, tenemos varios modelos intermedios y el final.

* Modelo de 7 epochs (``modelo_7epochs.keras``), con learning_rate constante:
    + accuracy: 0.7344 
    + loss: 1.3368
    + val_accuracy: 0.7809
    + val_loss: 1.2431
    + learning_rate: 0.0010

* Modelo de 50 epochs, 7+43, (``modelo_50epochs.keras``), con learning_rate variable (adaptativa):
    + accuracy: 0.9671 
    + loss: 0.2836
    + val_accuracy: 0.9831
    + val_loss: 0.2235
    + learning_rate: 2.0000e-04

* Modelo de 65 epochs, 50+15, (``modelo_65epochs.keras``), con learning_rate variable (adaptativa) y últimas 15 epochs con batch_size=32:
    + accuracy: 0.9646 
    + loss: 0.277
    + val_accuracy: 0.9818
    + val_loss: 0.2181
    + learning_rate: 1.0000e-04

* Modelo de 75 epochs, 50+25, (``modelo_75epochs.keras``), con learning_rate variable (adaptativa):
    + accuracy: 0.9740
    + loss: 0.2100
    + val_accuracy: 0.9824
    + val_loss: 0.1688
    + learning_rate: 1.0000e-04

* Modelo de 100 epochs, 75+25, (``modelo_100epochs.keras``), con learning_rate variable (adaptativa):
    + accuracy: 0.9773
    + loss: 0.1976
    + val_accuracy: 0.9896
    + val_loss: 0.1514
    + learning_rate: 1.0000e-04

Por tanto, tras analizar las epochs en las que la leraning_rate se ha ido adaptando, el modelo final es el de las 100 epoch, que puede obtenerse directamente de la siguiente manera (si se dispone de tiempo suficiente para no ir parando y continuando). Cuaderno ``modelo.ipynb``:

```python
from google.colab import drive
drive.mount('/content/drive')
```

```python
import os
import numpy as np
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout, BatchNormalization
from keras.optimizers import Adam
from keras.callbacks import ReduceLROnPlateau, EarlyStopping, ModelCheckpoint
from tensorflow.keras.models import load_model
```


```python
# Definir las rutas a tus carpetas de datos
train_dir = "drive/My Drive/bilbotiks/canny/DATA/train"
validation_dir = "drive/My Drive/bilbotiks/canny/DATA/validation"
test_dir = "drive/My Drive/bilbotiks/canny/DATA/test"

# Definir el tamaño de las imágenes y la normalización
img_size = (64, 64)
batch_size = 64

# Creación de generadores de datos
train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=False,
    fill_mode='nearest'
)

validation_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=False,
    fill_mode='nearest'
)

test_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=20,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    horizontal_flip=False,
    fill_mode='nearest'
)

# Generadores de datos para entrenamiento, validación y prueba
train_generator = train_datagen.flow_from_directory(
    train_dir,
    target_size=img_size,
    batch_size=batch_size,
    class_mode='categorical'
)

validation_generator = validation_datagen.flow_from_directory(
    validation_dir,
    target_size=img_size,
    batch_size=batch_size,
    class_mode='categorical'
)

test_generator = test_datagen.flow_from_directory(
    test_dir,
    target_size=img_size,
    batch_size=batch_size,
    class_mode='categorical'
)
```

```python
model = Sequential()

# Capa Convolucional 1
model.add(Conv2D(32, (3, 3), activation='relu', input_shape=(64, 64, 3)))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.3))  # Ajustado a 0.3

# Capa Convolucional 2
model.add(Conv2D(64, (3, 3), activation='relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.3))  # Ajustado a 0.3

# Capa Convolucional 3 (Nueva)
model.add(Conv2D(128, (3, 3), activation='relu'))
model.add(BatchNormalization())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.3))  # Ajustado a 0.3

# Aplanar las características extraídas
model.add(Flatten())

# Capa Densa
model.add(Dense(128, activation='relu', kernel_regularizer='l2'))  # Añadida L2 Regularization
model.add(Dropout(0.3))  # Ajustado a 0.3

# Capa Densa adicional (Nueva)
model.add(Dense(64, activation='relu', kernel_regularizer='l2'))  # Añadida L2 Regularization
model.add(Dropout(0.3))  # Ajustado a 0.3

# Capa de salida para 9 clases
model.add(Dense(10, activation='softmax'))

# Compilación del modelo
model.compile(
    optimizer=Adam(learning_rate=0.001),
    loss='categorical_crossentropy',
    metrics=['accuracy']
)

# Callbacks
reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.2, patience=5, min_lr=0.0001)
early_stopping = EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)
model_checkpoint = ModelCheckpoint("drive/My Drive/bilbotiks/canny/model_0.3.keras", monitor='val_loss', save_best_only=True)

# Entrenamiento del modelo
history = model.fit(
    train_generator,
    epochs=100,
    validation_data=validation_generator,
    callbacks=[reduce_lr, early_stopping, model_checkpoint]
)
```
Para probar el modelo con los test, tenemos el notebook ``testing.ipynb``:

```python
from google.colab import drive
drive.mount('/content/drive')
```

```python
import os
from tensorflow.keras.models import load_model
import numpy as np
import cv2
from tensorflow.keras.preprocessing.image import img_to_array

# Ruta del directorio con las imágenes para predecir
predict_dir = 'drive/My Drive/bilbotiks/canny/DATA/test'

# Cargar el modelo desde el archivo .h5
model = load_model('drive/My Drive/bilbotiks/canny/model_0.3.keras')

# Define los nombres de tus clases
class_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "NaN"]

def load_and_preprocess_image(image_path, target_size=(64, 64)):
    """Carga y preprocesa una imagen"""
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, target_size)
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    image = image / 255.0
    return image

def predict_all_images_in_directory(model, directory, max_images_per_class=50):
    filenames = []
    predictions = []
    true_classes = []

    # Crear un diccionario para contar las imágenes por clase
    class_counts = {label: 0 for label in class_labels}

    for root, dirs, files in os.walk(directory):
        class_name = os.path.basename(root)  # Obtener el nombre de la carpeta como la clase real
        if class_name in class_labels:  # Asegurarse de que sea una clase válida
            for file in files:
                if file.endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
                    # Verificar si la clase ya alcanzó el límite de imágenes
                    if class_counts[class_name] >= max_images_per_class:
                        continue
                    file_path = os.path.join(root, file)
                    image = load_and_preprocess_image(file_path)
                    prediction = model.predict(image)
                    predicted_class = np.argmax(prediction)
                    filenames.append(file_path)
                    predictions.append(predicted_class)
                    true_classes.append(class_name)
                    class_counts[class_name] += 1

    return filenames, predictions, true_classes


# Llamar a la función para hacer predicciones
filenames, predictions, true_classes = predict_all_images_in_directory(model, predict_dir, max_images_per_class=50)

for filename, prediction, true_class in zip(filenames, predictions, true_classes):
    print(f"Imagen: {filename} -> Clase Real: {true_class} -> Predicción: {class_labels[prediction]}")

```

Importante destacar que si no se tiene Tensorflow correctamente configurado en local para ejeuctarlo con GPU, se puede intentar ejecutar en CPU con la siguiente línea:
```python
tf.config.set_visible_devices([], 'GPU')  # Desactivar el uso de la GPU
```

Se incluyen también los cuadernos ``binaria.ipynb``, ``multiclase.ipynb``, ``multiclase_v2.ipynb``, ``multiclase_v2_continuar.ipynb`` con el proceso sin ordenar.

## Integración en tiempo real con la identificación de los colores
Desde que se captura la imagen hasta que se le mete al modelo, hay que aplicar el mismo preproceso que durante el entrenamiento.

Otro aspecto importante es el retardo introducido en ``prediction_interval``, para no sobrecargar la CPU y evitar reducir significativamente la latencia de la visualización de la imagen de la cámara. Se puede poner en 1 segundo, 0.1, o en 0 (como se muestra en el código).

Este script es ``colores_numeros.py``:

```python
import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
import time
import tensorflow as tf

# Desactivar el uso de la GPU
tf.config.set_visible_devices([], 'GPU')

# Cargar el modelo previamente entrenado
model = load_model('modelo_100epochs.keras')

# Define los nombres de tus clases
class_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "NaN"]

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Definir el intervalo de tiempo entre las predicciones (en segundos)
prediction_interval = 0  # Realizar la predicción cada 1 segundo
last_prediction_time = 0  # Tiempo de la última predicción

count=0
numero = ""

# Recortar un 15% por todos los lados
def recortar_imagen(imagen, porcentaje_recorte=0.20):
    altura, ancho = imagen.shape[:2]
    recorte_vertical = int(altura * porcentaje_recorte)
    recorte_horizontal = int(ancho * porcentaje_recorte)
    return imagen[recorte_vertical:altura - recorte_vertical, recorte_horizontal:ancho - recorte_horizontal]


def load_and_preprocess_image(image, count, target_size=(64, 64)):
    # Aplicar flip horizontal
    image = cv2.flip(image, 1)

    # Recortar la imagen
    image = recortar_imagen(image)
    
    # Aplicar detección de bordes con Canny
    canny = cv2.Canny(image, 20, 110)
    
    # Engrosar los contornos usando dilatación
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))  # Tamaño del kernel
    thickened_canny = cv2.dilate(canny, kernel, iterations=1)  # Aumentar iteraciones para engrosar más
    
    #filename = f"new/{count}.png"
    #cv2.imwrite(filename, thickened_canny)
    
    """Carga y preprocesa una imagen"""
    image = cv2.cvtColor(thickened_canny, cv2.COLOR_BGR2RGB)  # Convertir a RGB
    image = cv2.resize(image, target_size)  # Redimensionar
    image = img_to_array(image)  # Convertir a array
    image = np.expand_dims(image, axis=0)  # Añadir una dimensión extra
    image = image / 255.0  # Normalizar
    return image

def predic_digit(image):
    prediction = model.predict(image, verbose=0)
    predicted_class = np.argmax(prediction)
    print(class_labels[predicted_class])
    return class_labels[predicted_class]

while True:
    success, frame = cap.read()
    if success:
        frame = cv2.flip(frame, 1)
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define los límites de los colores azul y rojo en el espacio HSV
        blue_lower_bound = np.array([80, 50, 50])
        blue_upper_bound = np.array([130, 255, 255])
        red_lower_bound1 = np.array([0, 100, 100])
        red_upper_bound1 = np.array([10, 255, 255])
        red_lower_bound2 = np.array([160, 100, 100])
        red_upper_bound2 = np.array([180, 255, 255])

        # Crea máscaras para los colores azul y rojo
        blue_mask = cv2.inRange(frame_hsv, blue_lower_bound, blue_upper_bound)
        red_mask1 = cv2.inRange(frame_hsv, red_lower_bound1, red_upper_bound1)
        red_mask2 = cv2.inRange(frame_hsv, red_lower_bound2, red_upper_bound2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Encuentra los contornos para los colores azul y rojo
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detecta y dibuja los rectángulos
        detected = False
        color_detected = "Color distractorio"

        for contour in blue_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 150 <= w <= 300 and 200 <= h <= 350:  # Tamaño ajustado para folios a 1.5 metros
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                detected = True
                color_detected = "Azul"
                # Recortar la imagen de la cartulina detectada
                cropped_image = frame[y:y + h, x:x + w]
                
                # Realizar la predicción solo si ha pasado el intervalo
                current_time = time.time()
                if current_time - last_prediction_time >= prediction_interval:
                    # Preprocesar la imagen recortada
                    count+=1
                    preprocessed_img = load_and_preprocess_image(cropped_image, count)
                    # Realizar la predicción
                    numero = predic_digit(preprocessed_img)
                    # Actualizar el tiempo de la última predicción
                    last_prediction_time = current_time

        for contour in red_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if 150 <= w <= 300 and 200 <= h <= 350:  # Tamaño ajustado para folios a 1.5 metros
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                detected = True
                color_detected = "Rojo"
                # Recortar la imagen de la cartulina detectada
                cropped_image = frame[y:y + h, x:x + w]
                
                # Realizar la predicción solo si ha pasado el intervalo
                current_time = time.time()
                if current_time - last_prediction_time >= prediction_interval:
                    # Preprocesar la imagen recortada
                    count+=1
                    preprocessed_img = load_and_preprocess_image(cropped_image, count)
                    # Realizar la predicción
                    numero = predic_digit(preprocessed_img)
                    # Actualizar el tiempo de la última predicción
                    last_prediction_time = current_time

        # Mostrar el color detectado
        cv2.putText(frame, color_detected, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Ult. Prediccion: {numero}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    

        # Mostrar la imagen en la ventana de la cámara
        cv2.imshow('Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

```

Ejemplos de resultados (recordar que las imágenes se voltean horizontalmente durante el preproceso):
<div style="display: flex; justify-content: space-between;">
  <img src="images_documentacion/pred_1_azul.png" alt="Imagen real capturada" style="width: 48%;"/>
  <img src="images_documentacion/pred_6_rojo.png" alt="Bordes obtenidos con el algoritmo Canny y tras preprocesado" style="width: 48%;"/>
</div>
<br>
<div style="display: flex; justify-content: space-between;">
  <img src="images_documentacion/pred_8_azul.png" alt="Imagen real capturada" style="width: 48%;"/>
  <img src="images_documentacion/pred_8_rojo.png" alt="Bordes obtenidos con el algoritmo Canny y tras preprocesado" style="width: 48%;"/>
</div>
<div style="page-break-after: always;"></div>
<div style="display: flex; justify-content: space-between;">
<img src="images_documentacion/pred_rojo_nan.png" alt="Imagen real capturada" style="width: 100%;"/>
</div>

## Resumen de documentos adjuntos a la documentación
En las siguientes carpetas están los siguientes ficheros

+ modelo:
    + DATA:
        + sinSeparar: dataset en bruto
        + train: dataset de entrenamiento
        + validation: dataset de validación de entrenamiento
        + test: dataset de pruebas
    + modelo.ipynb: notebook de desarrollo y entrenamiento del modelo
    + testing.ipynb
    + modelo_100epochs.keras: fichero del modelo.

# Firmado: JAVI :)