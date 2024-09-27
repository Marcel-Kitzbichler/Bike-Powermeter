# Bike-Powermeter
Crank powermeter based on strain gauges read by a HX711 ADC. The Seeed XIAO NRF52840 SENSE 
is used to collect the torque data from the HX711 and it's internal gyroscope is used,
to get rotational data, which is needed to calculate power data.
After some seconds of collecting data the average is calculated and 
transmitted wirelessly to a standard bike computer.

This project is still very much a work in progress. It currently lacks any kind of data filtering
or power saving.
