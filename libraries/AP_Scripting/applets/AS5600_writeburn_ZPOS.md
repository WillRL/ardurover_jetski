# AS5600 Write and Burn ZPOS

Writes the current position - 180 as ZPOS and burns it. This effectively sets the current position as halfway point (180 degrees).

## How to use

Install this script on the flight controller
Move AS5600 to midpoint position, run script.

This script is only ran once and does not loop, it also disables scripting after being ran to prevent accidentally burning multiple times on reboot.
You can only burn ZPOS, MPOS three times, and MANG once IF ZPOS and MPOS has not been burnt previously.