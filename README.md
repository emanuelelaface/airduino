# airduino
Air Quality Monitor for Arduino Nano IoT 33

A small project to monitor the air quality at home using an Arduino Nano IoT 33
(https://store.arduino.cc/arduino-nano-33-iot)
as web server to create a JSON file on demand and Freeboard
(https://github.com/Freeboard/freeboard)
to display the values in a web browser.

The project uses two sensors, a BME280 to monitor temperature, pressure and humidity and an optical sensor MH-Z14A to read the CO2. A simple RGB LED is used to show the status of the sensors and wifi.

The hardware layout is:
<img src=https://github.com/emanuelelaface/airduino/blob/master/airduino.jpeg></img>

The final result in the browser is:
<img src=https://github.com/emanuelelaface/airduino/blob/master/website-screenshot.png></img>
