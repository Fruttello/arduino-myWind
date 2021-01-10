# arduino-myWind

Wind meter and safeguard for remote-controlled sun awnings: when the wind is too strong, rolls up the awnings to prevent damage

Tutorial for Arduino self-learning purposes only, by Fruttello 2021; feel free to use and copy 

Released under the Creative Commons Zero CC0 V1.0 Universal license, see LICENSE or https://creativecommons.org/publicdomain/zero/1.0/legalcode

Requirements:

1) Measure wind speed through a wind mill with pulse sensor (a Reed switch that generates 1 pulse per revolution)
2) Compute average and max wind speeds over a sliding window of measurements
3) Display instantaneous, average and max speeds on LCD with the selected speed unit
4) Cycle selected speed display unit between m/s, km/h, mph and knots on short keypresses of a button (with debounce)
5) Adjust LCD brightess on long keypresses of the same button used for display mode (ramps brightness progressively up to max then down to zero; repeat until button is released)
6) If either average speed or max speed are above safety limits, light an alarm LED and send command to roll up the awnings

This prototype cannot sense the current awning status (up, down or in transit), so the safeguard will keep sending "roll up" commands at set intervals until the alarm condition ends.
