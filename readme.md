Este repositorio contiene el firmware para la controlcard F28335 utilizado para mover el motor de 30 HP, necesario para hacer funcionar el rectificador del proyecto Morzán-Quinteros. Constituye una versión modificada del firmware original del proyecto Asad-Dórdolo, cuya finalidad era emular una turbina eólica y que se desarrolló para un motor más pequeño, de 2 HP. Las principales modificaciones en relación a este son las siguientes:
    -tiene protección de sobremodulación que satura el índice en 1.
    -implementan debilitamiento de campo (función "_iq FieldWeakening(_iq);")
    -Algunos parámetros distintos (no solo en los 30 HP sino también en los 2 HP). En los comentarios indican que se ajustó empíricamente, así que me suena a que son los parámetros que ajustaron Renzo y Nico para el 30 HP. Hay una cte de tiempo rotórica más grande.
    -tiene implementado un "capturador" de señal
    -empieza en modo (modoAD) 0 en lugar de en modo 8 como la otra implementación. Modo 0 es para controlar 
    -Tiene rc1.RampDelayMax seteado en 10 en lugar de en 2. Supongo que hacerlo más lento puede ser también por el motor más grande.
    -Deshabilitan la interrupción por desaturación porque usan directamente la interrupción de trip zone?
    -no configura el trip zone por hardware sino que ponen interrupción y ahí cortan la operación del inversor.
    -adquieren un par de señales más para registrar datos.
    -la medición del ángulo de la máquina se hace como 1-eqep en lugar de eqep directamente.

(cómo clonar y ¿compilar?, hacer referencia a un documento interno)