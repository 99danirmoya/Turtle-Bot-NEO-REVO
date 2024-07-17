![Static Badge](https://img.shields.io/badge/stability-v1.0-3314333)

# <p align="center"> Turtle-Bot NEO EVO </p>

<div align="center">

_Hecho con_

[![made-in-ArduinoIDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)

## Mini rover WiFi con sensor climático, panel OLED, LED y bocina controlado desde NodeRED Dashboard al que se envían los datos recogidos en tiempo real

</div>

<div align="justify">

___

## Modo de funcionamiento
1. Se conecta el robot a una WiFi desde otro dispositivo (PC o smartphone)
1. Se accede al panel de control en la nube para su pilotaje de forma remota

___

</div>

## <p align="justify">  Lista de componentes </p>

<div align="center">

| Componente | Unidades |
| ------------- | ------------- |
| ESP32 | 1 |
| Protoboard | 1 |
| Chasis | 1 |
| LED | 2 |
| Resistencia 220ohm | 2 |
| Zumbador | 1 |
| BME280 | 1 |
| Servo de rotación continua | 2 |
| Ruedas | 2 |
| Jumpers | Los que hagan falta |

</div>

<div align="justify">

___

## Diagrama de conexiones

A continuación, se muestra el diagrama de conexiones del robot. Hacer clic en el botón morado de la imagen para acceder a la simulación de TinkerCAD.

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/turtle-bot-neo/blob/main/pics/t725.png" width="750"  style="margin: 10px;"/>
  
  <em>Circuito del Turtle-Bot NEO</em>

  [![Static Badge](https://img.shields.io/badge/TINKER_THIS!-8A2BE2?logo=autodesk)](https://www.tinkercad.com/things/kZLjRCSkkIt-turtle-bot-neo)
</div>
<br/>

> [!CAUTION]
> Debido al número limitado de I/O de TinkerCAD, algunos elementos han sido sustituidos por las alternativas más parecidas, como el joystick por un botón y dos potenciómetros, y los sensores de infrarrojos, por sensores de proximidad (PIR)

<div align="justify">

___

## Modo de implementación

Toda la explicación del código de Arduino viene dada en el propio código, [`99danirmoya/turtle-bot-neo/turtle_bot_neo/turtle_bot_neo.ino`](https://github.com/99danirmoya/turtle-bot-neo/blob/main/turtle_bot_neo/turtle_bot_neo.ino), en formato de comentarios al lado de cada línea.

Por otra parte, se debe importar el flujo que se muestra a continuación desde el archivo [`flow_tbne`](https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/nodered/flow_tbne):

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/NodeRED_flow.jpeg" width="750"  style="margin: 10px;"/>

  <em>Flujo de NodeRED para el control de Turtle-Bot NEO EVO</em>
</div>
<br/>

[`PANEL A DISTANCIA`](https://4f566df1fed52c6e7fd5f661f64ae3eb.balena-devices.com/ui/#!/1?socketid=BDvahPaq06OiHTluAAA4)

___
___

## Ejemplo de montaje

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Turtle-Bot-NEO-EVO/blob/main/pics/BQ_RENACUAJO.jpg" width="750"  style="margin: 10px;"/>

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
