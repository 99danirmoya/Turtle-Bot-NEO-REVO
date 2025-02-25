![Static Badge](https://img.shields.io/badge/stability-v4.0-3314333)

# <p align="center"> Turtle-Bot NEO REVO (rover2) </p>

<div align="center">

_Desarrollado por_

`César Rubio Sánchez`, `Alejandro Martín Sánchez`, `Daniel Rodríguez Moya` y `Álvaro Rodríguez Piñeiro`

_Programado en_

[![made-in-ArduinoIDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)

_Servicios en la nube ofrecidos por_
<div align="center">
  <img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQEBDzjaVjjedkNWruWr53P-XmtGd00nFSa8Q&s" width="90"  style="margin: 10px;"/>

</div>
<br/>

## Mini rover WiFi con camara digital, sensor climático y de material particulado, GPS, mangnetómetro, panel OLED, luces LED y bocina controlado desde Telegram y NodeRED Dashboard/Thingsboard a los que se envían los datos recogidos en tiempo real. Cuenta con dos modos de operación: manual y auto-pilotado por coordenadas GPS

</div>

<div align="justify">

___

## Demo del proyecto

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/photo_5789838300280900015_y.jpg" width="900"  style="margin: 10px;"/>

  <em>Muestra de montaje y cableado</em>
</div>
<br/>

> [!NOTE]
> Esta imagen es únicamente una muestra orientativa. El chasis está hecho en impresión 3D/metacrilato y vale cualquiera que venga preparado para servos y los sensores elegidos para cada modo

___

## Índice
### 1. [Modo de funcionamiento](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#modo-de-funcionamiento)
### 2. [Lista de componentes](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#--lista-de-componentes-)
### 3. [Diagrama de conexiones](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#diagrama-de-conexiones)
### 4. [Modo de implementación](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#modo-de-implementaci%C3%B3n)
### 5. [Licencia](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#-licencia-)
### 6. [Contacto](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#-contacto-)

___
___

## Modo de funcionamiento [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice)
1. Se conecta el robot a una WiFi accediendo a su hotspot desde otro dispositivo (PC o smartphone)
2. En caso de querer usarse el I/O desde Telegram, se deberá crear un bot desde BotFather para acceder a su token y deshabilitar la privacidad en grupos para poder explotar el máximo potencial con el comando `/setprivacy`
3. Se accede al panel de control en la nube para su pilotaje de forma remota
    - ALTERNATIVAMENTE: Si no se ha accedido aún a NodeRED, se debe importar el flujo (más info a continuación) y configurar los nodos relacionados con MQTT, bots de Telegram e InfluxDB.

___

</div>

## <p align="justify">  Lista de componentes [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice) </p>

<div align="center">

| Componente | Unidades |
| ------------- | ------------- |
| LilyGO T3 S3 | 1 |
| ESP32 CAM | 1 |
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

## Diagrama de conexiones [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice)

La tarjeta de prototipado específica empleada para el desarrollo ha sido la LilyGO T3-S3 v1.2:

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/Screenshot%202025-01-18%20221859.png" width="900"  style="margin: 10px;"/>
  
  <em>Pinout de la tarjeta de prototipado LilyGO T3-S3 v1.2</em>
</div>
<br/>

A continuación, se muestra el diagrama de conexiones del robot.

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/TURTLE-BOT-NEOEVO_schematic_1.jpg" width="900"  style="margin: 10px;"/>
  
  <em>Circuito del Turtle-Bot NEO EVO</em>
</div>
<br/>

> [!CAUTION]
> **OJO**, el microcontrolador usado en el dibujo es el TTGO LoRa32 OLED, antecesor del LilyGO T3-S3 v1.2 que se emplea en este ejemplo. Por ello, el pinout del esquema es distinto, **sólo se debe seguir la ubicación de los pines. Por parte de la ESP32 CAM, no se incluye en el diagrama ya que se puede considerar un SoC**.

<div align="justify">

___

## Modo de implementación [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice)

Toda la explicación del código de Arduino viene dada en el propio código, [`99danirmoya/Turtle-Bot-NEO-EVO/blob/main/tbne/tbne.ino`](https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/tbne/tbne.ino), en formato de comentarios al lado de cada línea. Cabe destacar la implementación de freeRTOS para fragmentar en tareas ejecutadas en paralelo para mejorar los tiempos de respuesta.

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/rover2_2.jpeg" width="900"  style="margin: 10px;"/>

  <em>Diagrama de bloques de la implementación en freeRTOS</em>
</div>
<br/>

De esta manera, las funcionalidades principales son:
1. Monitoreo de variables climáticas y de contaminacion
2. Sistema de alarmas remoto y local (LED, OLED y sirena) en caso de riesgo para ganado/cosecha
3. Piloto manual por joystick virtual con información sobre geolocalización, orientación, velocidad e imagen
4. Piloto automático por ruta de coordenadas GPS

### Servicios en NodeRED

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

### Servicios en ThingsBoard

Se ha creado, además, un dashboard analogo al de NodeRED, pero en ThingsBoard.

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/dashboard-general.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard general para acceder al resto de paneles</em>
</div>
<br/>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/dashboard-control.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard para el control del robot</em>
</div>
<br/>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/dashboard-weather.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard para acceder a los datos actualizados de meteorología</em>
</div>
<br/>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/dashboard-airquality.jpeg" width="900"  style="margin: 10px;"/>

  <em>Dashboard para acceder a los datos actualizados de calidad de aire</em>
</div>
<br/>

Como servicio adicional al dashboard de Thingsboard, se ha creado una aplicacion en Android Studio que permite consultar todos los sensores, la cámara, controlar el rover con un joystick y recibir notificaciones de alarmas. La conexión de la app con Thingsboard ha sido gracias a la RESTful API que provee.

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/Screenshot%202025-02-10%20113812.png" width="900"  style="margin: 10px;"/>

  <em>Aplicación para Android con conexión a Thingsboard</em>
</div>
<br/>

La arquitectura de red se puede consultar en el diagrama a continuación:

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/pics/Screenshot%202025-02-10%20114958.png" width="900"  style="margin: 10px;"/>

  <em>Arquitectura de red con Thingsboard</em>
</div>
<br/>

___
___

## <p align="justify"> Licencia [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice) </p>

Este proyecto está licenciado bajo la [GPL-3.0 license](https://github.com/99danirmoya/turtle-bot-neo?tab=GPL-3.0-1-ov-file).

</div>

___

## <p align="justify"> Contacto [:leftwards_arrow_with_hook:](https://github.com/99danirmoya/Turtle-Bot-NEO-REVO/blob/main/README.md#%C3%ADndice) </p>

> [!IMPORTANT]
> Respondo amablemente a dudas y leo sugerencias: [![Gmail Badge](https://img.shields.io/badge/-Gmail-c14438?style=for-the-badge&logo=Gmail&logoColor=white&link=mailto:daniel.rodriguezm99@gmail.com)](mailto:daniel.rodriguezm99@gmail.com)
> 
> Más información sobre mis proyectos: [![Linkedin Badge](https://img.shields.io/badge/-LinkedIn-blue?style=for-the-badge&logo=Linkedin&logoColor=white&link=https://www.linkedin.com/in/daniel-rodr%C3%ADguez-moya-510a35167)](https://www.linkedin.com/in/daniel-rodr%C3%ADguez-moya-510a35167)

_<p align="justify"> Autores: César Rubio Sánchez, Alejandro Martín Sánchez, Daniel Rodríguez Moya y Álvaro Rodríguez Piñeiro :shipit: </p>_
