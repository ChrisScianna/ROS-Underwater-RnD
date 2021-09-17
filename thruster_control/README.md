# Thruster control.
The thruster control manages the velocity of the thruster. It also reports:
- Thruster velocity.
- Motor temperature.
- Battery state.

The thruster control node diagnoses:
  - The thruster temperature threshold has been reached. (Error)
  - The thruster motor rpm report has stalled. (Warning)
  - The thruster temperature report has stalled. (Warning)
  - The thruster motor velocity threshold has been reached. (Warning)
  - The thruster temperature readings have stagnated. (Warning)
  - Battery current threshold has been reached. (Warning)
  - Battery voltage threshold has been reached. (Warning)
  - Battery report has stalled. (Warning)
  - Battery report readings have stagnated. (Warning)
