![Static Badge](https://img.shields.io/badge/stability-v3.0-3314333)

# <p align="center"> Turtle-Bot NEO EVO </p>

<div align="center">

_Programado en_

[![made-in-ArduinoIDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)

_Servicios en la nube ofrecidos por_
<div align="center">
  <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQEBDzjaVjjedkNWruWr53P-XmtGd00nFSa8Q&s" width="90"  style="margin: 10px;"/>

</div>
<br/>

## Mini rover WiFi con sensor climático y de material particulado, GPS, panel OLED, luces LED y bocina controlado desde Telegram y NodeRED Dashboard a los que se envían los datos recogidos en tiempo real

</div>

<div align="justify">

___

## Modo de funcionamiento
1. Se conecta el robot a una WiFi accediendo a su hotspot desde otro dispositivo (PC o smartphone)
2. En caso de querer usarse el I/O desde Telegram, se deberá crear un bot desde BotFather para acceder a su token y deshabilitar la privacidad en grupos para poder explotar el máximo potencial con el comando `/setprivacy`
3. Se accede al panel de control en la nube para su pilotaje de forma remota
    - ALTERNATIVAMENTE: Si no se ha accedido aún a NodeRED, se debe importar el flujo (más info a continuación) y configurar los nodos relacionados con MQTT, bots de Telegram e InfluxDB.


___

</div>

## <p align="justify">  Lista de componentes </p>

<div align="center">

| Componente | Unidades |
| ------------- | ------------- |
| LilyGO T3 S3 | 1 |
| Protoboard | 1 |
| Chasis | 1 |
| LED | 1 |
| Resistencia 220ohm | 1 |
| Zumbador | 1 |
| BME280 | 1 |
| GPS NEO-6M | 1 |
| HMC5883L | 1 |
| SDS011 | 1 |
| Servo de rotación continua | 2 |
| Ruedas | 2 |
| Jumpers | Los que hagan falta |

</div>

<div align="justify">

___

## Diagrama de conexiones

A continuación, se muestra el diagrama de conexiones del robot.

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/TURTLE-BOT-NEOEVO_schematic_1.jpg" width="900"  style="margin: 10px;"/>
  
  <em>Circuito del Turtle-Bot NEO EVO</em>
</div>
<br/>

La tarjeta de prototipado específica empleada para el desarrollo ha sido la LilyGO T3-S3 v1.2:

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/LILYGO_T3_S3_V1_2.jpg" width="600"  style="margin: 10px;"/>
  
  <em>Pinout de la tarjeta de prototipado LilyGO T3-S3 v1.2</em>
</div>
<br/>

> [!CAUTION]
> **OJO**, el microcontrolador usado en el dibujo es el TTGO LoRa32 OLED, antecesor del LilyGO T3-S3 v1.2 que se emplea en este ejemplo. Por ello, el pinout del esquema es distinto, **sólo se debe seguir la ubicación de los pines**.

<div align="justify">

___

## Modo de implementación

Toda la explicación del código de Arduino viene dada en el propio código, [`99danirmoya/Turtle-Bot-NEO-EVO/blob/main/tbne/tbne.ino`](https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/tbne/tbne.ino), en formato de comentarios al lado de cada línea.

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/rover2.jpeg" width="900"  style="margin: 10px;"/>

  <em>Diagrama de bloques de la implementación en freeRTOS</em>
</div>
<br/>

Por otra parte, se debe importar el flujo que se muestra a continuación desde el archivo [`flow_tbne.json`](https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/nodered_flow/flow_tbne.json):

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/NodeRED_flow_1_1.jpeg" width="900"  style="margin: 10px;"/>

  <em>Flujo de NodeRED para el control de Turtle-Bot NEO EVO</em>
</div>
<br/>

Con el flujo importado y habiéndose configurado los nodos de MQTT, Telegram, función e InfluxDB, se puede acceder al siguiente panel de control y monitoreo del sensor:

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/DASHBOARD_1_1_1.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard de NodeRED para el control y monitoreo climático a borde de Turtle-Bot NEO EVO</em>
</div>
<br/>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/DASHBOARD_2_1_1.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard de NodeRED para la visualización historica del sensor climático a bordo</em>
</div>
<br/>

En el siguiente enlace, se puede consultar el [panel a distancia](https://e047be1273f7b77c71d4d02783f546a4.balena-devices.com/ui/#!/0?socketid=kMCTOBbX9VSPFcfIAAAF) original. **Está configurado para el Turtle-Bot NEO EVO original**.

Por otra parte, se puede hacer uso de un bot de Telegram para recibir notificaciones de eventos importantes de la batería (batería baja o batería cargada), así como invocar un menú, creado por [Alex Trostle](https://flows.nodered.org/flow/c8194f9d056455018d2da8ef7e109733), en el que encender o apagar el panel OLED y la luz OLED al hacer uso del comando `/neo2`:

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/telegrambot1.jpg" width="550"  style="margin: 10px;"/>

  <em>Menú del bot de Telegram para escoger dispositivo al que modificar su estado</em>
</div>
<br/>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/telegrambot2.jpg" width="550"  style="margin: 10px;"/>

  <em>Menú del bot de Telegram para encender o apagar el dispositivo elegido</em>
</div>
<br/>

___
___

## Ejemplo de montaje

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/photo_5789838300280900015_y.jpg" width="900"  style="margin: 10px;"/>

  <em>Muestra de montaje y cableado</em>
</div>
<br/>

> [!NOTE]
> Esta imagen es únicamente una muestra orientativa. Lo razonable sería montar tanto el Arduino UNO como la protoboard sobre el chasis y distanciar el switch de tres posiciones y el joystick a modo de mando. El chasis está hecho en impresión 3D y vale cualquiera que venga preparado para servos y los sensores elegidos para cada modo

___

___

## <p align="justify"> Licencia </p>

Este proyecto está licenciado bajo la [GPL-3.0 license](https://github.com/99danirmoya/turtle-bot-neo?tab=GPL-3.0-1-ov-file).

___

## <p align="justify"> Contacto </p>

> [!IMPORTANT]
> Respondo amablemente a dudas y leo sugerencias: [![Gmail Badge](https://img.shields.io/badge/-Gmail-c14438?style=for-the-badge&logo=Gmail&logoColor=white&link=mailto:daniel.rodriguezm99@gmail.com)](mailto:daniel.rodriguezm99@gmail.com)
> 
> Más información sobre mis proyectos: [![Linkedin Badge](https://img.shields.io/badge/-LinkedIn-blue?style=for-the-badge&logo=Linkedin&logoColor=white&link=https://www.linkedin.com/in/daniel-rodr%C3%ADguez-moya-510a35167)](https://www.linkedin.com/in/daniel-rodr%C3%ADguez-moya-510a35167)

_<p align="justify"> Autor: Daniel Rodríguez Moya :shipit: </p>_
