This project displays the current CO2 levels (ppm) and is made with only minimal components.

This code is based off the complex LibreCO2 project by Daniel Bernalb (<a href="https://github.com/danielbernalb/LibreCO2">LibreCO2</a>).

  

Fresh air contains CO2 levels slightly above 400 ppm. Indoor air quality is deemed acceptable if 1000 ppm or lower. Long term (>24 hours) of exposure to CO2 levels over 1000 ppm can result in respiratory symptoms (runny or congested nose, sore or dry throat, sneezing, coughing), decreased test performance (decision making, task performance), neurophysiological symptoms (headache, tiredness, fatigue, difficulty concentrating) <sup>[1]</sup>.  
  
What is concerning is the decreased in mental performance, decision making, tiredness, and difficulty concentrating when exposed to high levels of CO2. Particularly for those who work from home or in a crowded office or classroom.  
  
![CO2 Monitor Project](https://github.com/ZPaulWeleschuk/CO2_Monitor_Arduino/blob/main/img/CO2_Monitor_Stand_cropped_446.jpg)

**This project uses:**
- Arduino Nano  
- Sensair S8  
- TM1637 seven segment display
- button
- wire
- 50mm x 70mm prototype board

  
![CO2 Monitor](https://github.com/ZPaulWeleschuk/CO2_Monitor_Arduino/blob/main/img/CO2_Monitor_cropped_446.jpg)

![Wiring Diagram](https://github.com/ZPaulWeleschuk/CO2_Monitor_Arduino/blob/main/img/wiringDiagram_CO2_monitor.png)
  

The CO2 sensor may or may not need to be calibrated. Run the project outside for several minutes in a place protected from the wind. Press and hold the button down till ‘CAL’ appears on the display. The sensor will then begin a count down, where it will calibrate the outside air to 400 ppm.

  

The stand to hold the project can be 3D printed (<a href="https://www.printables.com/model/645775-prototyping-board-display-stand-50mm-x-70mm">Prototyping Board Display Stand (50mm x 70mm)</a>)

  

<sup>[1]</sup> Water and Air Quality Bureau, Health Canada, (2020), “Consultation: Proposed Residential Indoor Air Quality Guidelines for Carbon Dioxide”

