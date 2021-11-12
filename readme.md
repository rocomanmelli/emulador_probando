
# Contenido
Este repositorio contiene el firmware para la controlcard F28335 utilizado para mover el motor de 30 HP, necesario para hacer funcionar el rectificador del proyecto Morzán-Quinteros. Constituye una versión modificada del firmware original del proyecto Asad-Dórdolo, cuya finalidad era emular una turbina eólica y que se desarrolló para un motor más pequeño, de 2 HP. Las principales modificaciones en relación a este son las siguientes:
- se implementa debilitamiento de campo (función `_iq FieldWeakening(_iq);`) para poder subir las RPMs de la máquina por encima del valor nominal,
- se ajustaron parámetros ligados a las máquinas eléctricas (tanto para el motor de 30 HP como para el de 2 HP) y a módulos de software,
- la medición del ángulo de la máquina se hace como `1 - qep1.ElecTheta` en lugar de como `qep1.ElecTheta`,
- se añadió una protección por software para evitar sobremodulación, que satura el módulo del índice de modulación en 1 (manteniendo su fase), y por último,
- el programa no empieza ejecutándose en un modo (`ModoAD`) que emula el comportamiento de una turbina eólica sino en uno de prueba.

Luego hay otras modificaciones menores que incluyen:
- la implementación de un "datalogger" que funciona almacenando valores de ciertas variables en un buffer,
- la adquisición de algunas señales adicionales para hacer debugging, y
- la inhabilitación de la interrupción por desaturación (la máquina está protegida porque se usa la interrupción por trip zone que se dispara también por desaturación; notar que igualmente el trip zone no está configurado para funcionar por hardware sino que la parada se realiza por software habiendo entrado en dicha interrupción).

**(Esto es lo que entendí de cuando revisé el código, podría haber algún error.)**

# Procedimiento

## Descarga

Para tener disponible el contenido de este repositorio en su computadora, puede o bien descargar un archivo comprimido (en la interfaz gráfica de Github: Code&#8594;Download ZIP) o directamente clonarlo, por ejemplo por ssh:

```
git clone git@github.com:rocomanmelli/emulador_probando.git 
```
**(Actualizar este path de clonado.)**

Tenga en cuenta que la idea de este repositorio es que sea usado como punto de partida para otros proyectos. Eso requerirá armar un nuevo repositorio en Github para controlar las versiones que vaya desarrollando. Para esto y otras recomendaciones, se recomienda dirigirse al documento interno del LAC "Buenas Prácticas de Organización del Software".

Una vez que tenga los archivos en su computadora, asumiendo que tanto el Code Composer Studio como la controlSUITE se instalaron en sus paths por defecto, compilar el proyecto debería ser tan fácil como importarlo (Import&#8594;CCS Projects) y darle al martillito. Esto también se explica con más detalle en el documento mencionado anteriormente.

## Utilización

El procedimiento para hacer andar el equipo (al menos como se lo usó para que funcionara el rectificador del proyecto Morzán-Quinteros) involucraba los siguientes pasos:
1. setear `EnableFlag` en 1,
2. setear `ModoAD` en 3 para hacer control de velocidad,
3. setear `enable_weakening` en 1 para habilitar el debilitamiento de campo y poder subir la velocidad hasta el punto requerido, y finalmente,
4. incrementar escalonadamente `SpeedRef` hasta alcanzar el valor 0.5 (que sería lo necesario para llegar a 1800 RPM).

Tener en cuenta además que seteando la variable `CLEAR_FAULT` en 1 se pueden limpiar las fallas si es que saltase alguna protección. De más está decir que habrá que verificar y entender por qué está dándose la falla para solucionar el problema de fondo.

**(El procedimiento mencionado debe ser chequeado en los cuadernillos ubicados próximos a los inversores Allen-Bradley, en la estantería del aula pequeña del LAC que da contra la pared del fondo (respecto de la puerta de entrada).)**

# Contacto

Ante cualquier problema, ~~no~~ dirigirse a rcomelli@fceia.unr.edu.ar.
